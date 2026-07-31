#include <iostream>
#include <fstream>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/videodev2.h>
#include <sys/mman.h>
#include <string.h>
#include <errno.h>

#define ENABLE_DVPP_INTERFACE
#include "acl/acl.h"
#include "acl/ops/acl_dvpp.h"

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"
#include <opencv2/opencv.hpp>

#define V4L2_DEVICE "/dev/video0"
#define BUFFER_COUNT 4
#define WIDTH 1280
#define HEIGHT 720

using namespace std;
using namespace std::chrono_literals;

class DvppCameraNode : public rclcpp::Node {
public:
    DvppCameraNode() : Node("dvpp_camera_node") {
        // Publisher
        pub_ = this->create_publisher<sensor_msgs::msg::Image>("/camera/image_raw", 10);

        // Init ACL
        aclInit(nullptr);
        aclrtSetDevice(0);
        aclrtCreateContext(&context_, 0);

        // Init V4L2
        init_v4l2();

        // Timer
        timer_ = this->create_wall_timer(33ms, std::bind(&DvppCameraNode::timer_callback, this));
        RCLCPP_INFO(this->get_logger(), "DVPP Camera Node started");
    }

    ~DvppCameraNode() {
        if (fd_ >= 0) {
            int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
            ioctl(fd_, VIDIOC_STREAMOFF, &type);
            for (int i = 0; i < BUFFER_COUNT; i++) {
                munmap(buffers_[i], buf_length_[i]);
            }
            close(fd_);
        }
        aclrtDestroyContext(context_);
        aclFinalize();
    }

private:
    void init_v4l2() {
        fd_ = open(V4L2_DEVICE, O_RDWR);
        if (fd_ < 0) {
            RCLCPP_ERROR(this->get_logger(), "Cannot open %s: %s", V4L2_DEVICE, strerror(errno));
            return;
        }

        struct v4l2_format fmt = {};
        fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        fmt.fmt.pix.width = WIDTH;
        fmt.fmt.pix.height = HEIGHT;
        fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_MJPEG;
        fmt.fmt.pix.field = V4L2_FIELD_NONE;
        if (ioctl(fd_, VIDIOC_S_FMT, &fmt) < 0) {
            RCLCPP_ERROR(this->get_logger(), "VIDIOC_S_FMT failed: %s", strerror(errno));
            return;
        }

        struct v4l2_requestbuffers req = {};
        req.count = BUFFER_COUNT;
        req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        req.memory = V4L2_MEMORY_MMAP;
        if (ioctl(fd_, VIDIOC_REQBUFS, &req) < 0) {
            RCLCPP_ERROR(this->get_logger(), "VIDIOC_REQBUFS failed: %s", strerror(errno));
            return;
        }

        struct v4l2_buffer buf = {};
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        for (int i = 0; i < BUFFER_COUNT; i++) {
            buf.index = i;
            ioctl(fd_, VIDIOC_QUERYBUF, &buf);
            buf_length_[i] = buf.length;
            buffers_[i] = (unsigned char*)mmap(NULL, buf.length, PROT_READ | PROT_WRITE, MAP_SHARED, fd_, buf.m.offset);
            ioctl(fd_, VIDIOC_QBUF, &buf);
        }

        int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        ioctl(fd_, VIDIOC_STREAMON, &type);
    }

    void timer_callback() {
        struct v4l2_buffer buf = {};
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        if (ioctl(fd_, VIDIOC_DQBUF, &buf) < 0) {
            return;
        }

        unsigned char* mjpg_data = buffers_[buf.index];
        size_t mjpg_len = buf.bytesused;

        // DVPP decode
        cv::Mat bgr = decode_dvpp(mjpg_data, mjpg_len);

        if (!bgr.empty()) {
            sensor_msgs::msg::Image msg;
            cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", bgr).toImageMsg(msg);
            pub_->publish(msg);
        }

        ioctl(fd_, VIDIOC_QBUF, &buf);
    }

    cv::Mat decode_dvpp(unsigned char* data, size_t len) {
        cv::Mat empty;
        if (len < 100) return empty;

        acldvppChannelDesc* channelDesc = acldvppCreateChannelDesc();
        if (acldvppCreateChannel(channelDesc) != 0) return empty;

        uint32_t w = 0, h = 0;
        int32_t comp = 0;
        if (acldvppJpegGetImageInfoV2(data, len, &w, &h, &comp, nullptr) != 0) {
            acldvppDestroyChannel(channelDesc);
            return empty;
        }

        uint32_t predictSize = 0;
        acldvppJpegPredictDecSize(data, len, PIXEL_FORMAT_YUV_SEMIPLANAR_420, &predictSize);

        void* out_buf = nullptr;
        acldvppMalloc(&out_buf, predictSize);

        acldvppPicDesc* picDesc = acldvppCreatePicDesc();
        acldvppSetPicDescData(picDesc, out_buf);
        acldvppSetPicDescSize(picDesc, predictSize);
        acldvppSetPicDescFormat(picDesc, PIXEL_FORMAT_YUV_SEMIPLANAR_420);
        acldvppSetPicDescWidth(picDesc, w);
        acldvppSetPicDescHeight(picDesc, h);
        acldvppSetPicDescWidthStride(picDesc, ((w + 15) / 16) * 16);
        acldvppSetPicDescHeightStride(picDesc, ((h + 15) / 16) * 16);

        aclrtStream stream = nullptr;
        aclrtCreateStream(&stream);
        if (acldvppJpegDecodeAsync(channelDesc, data, len, picDesc, stream) == 0) {
            aclrtSynchronizeStream(stream);
            cv::Mat yuv(h * 3 / 2, w, CV_8UC1);
            aclrtMemcpy(yuv.data, predictSize, out_buf, predictSize, 0);
            cv::Mat bgr;
            cv::cvtColor(yuv, bgr, cv::COLOR_YUV2BGR_NV12);
            aclrtDestroyStream(stream);
            acldvppFree(out_buf);
            acldvppDestroyPicDesc(picDesc);
            acldvppDestroyChannel(channelDesc);
            return bgr;
        }

        aclrtDestroyStream(stream);
        acldvppFree(out_buf);
        acldvppDestroyPicDesc(picDesc);
        acldvppDestroyChannel(channelDesc);
        return empty;
    }

    int fd_ = -1;
    unsigned char* buffers_[BUFFER_COUNT];
    size_t buf_length_[BUFFER_COUNT];
    aclrtContext context_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DvppCameraNode>());
    rclcpp::shutdown();
    return 0;
}
