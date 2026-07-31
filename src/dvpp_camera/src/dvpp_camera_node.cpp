#include <iostream>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/videodev2.h>
#include <sys/mman.h>
#include <string.h>
#include <errno.h>
#include <stdlib.h>
#include <atomic>

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

using namespace std::chrono_literals;

std::atomic<int> malloc_count(0);
std::atomic<int> free_count(0);

class DvppCameraNode : public rclcpp::Node {
public:
    DvppCameraNode() : Node("dvpp_camera_node") {
        pub_ = this->create_publisher<sensor_msgs::msg::Image>("/camera/image_raw", 10);

        aclInit(nullptr);
        aclrtSetDevice(0);
        aclrtCreateContext(&context_, 0);

        init_v4l2();

        timer_ = this->create_wall_timer(16ms, std::bind(&DvppCameraNode::timer_callback, this));
        RCLCPP_INFO(this->get_logger(), "DvppCameraNode started");
        RCLCPP_INFO(this->get_logger(), "Memory tracking enabled: malloc=%d, free=%d", malloc_count.load(), free_count.load());
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
        RCLCPP_INFO(this->get_logger(), "Final memory stats: malloc=%d, free=%d, diff=%d",
            malloc_count.load(), free_count.load(), malloc_count.load() - free_count.load());
        aclrtDestroyContext(context_);
        aclFinalize();
    }

private:
    void init_v4l2() {
        fd_ = open(V4L2_DEVICE, O_RDWR | O_NONBLOCK);
        if (fd_ < 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open camera /dev/video0");
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

        struct v4l2_streamparm parm = {};
        parm.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        parm.parm.capture.timeperframe.numerator = 1;
        parm.parm.capture.timeperframe.denominator = 60;
        if (ioctl(fd_, VIDIOC_S_PARM, &parm) < 0) {
            RCLCPP_WARN(this->get_logger(), "VIDIOC_S_PARM failed");
        }

        system("v4l2-ctl -d /dev/video0 --set-ctrl=exposure_auto=1");
        system("v4l2-ctl -d /dev/video0 --set-ctrl=white_balance_temperature_auto=0");
        system("v4l2-ctl -d /dev/video0 --set-ctrl=white_balance_temperature=4000");
        system("v4l2-ctl -d /dev/video0 --set-ctrl=backlight_compensation=0");

        struct v4l2_requestbuffers req = {};
        req.count = BUFFER_COUNT;
        req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        req.memory = V4L2_MEMORY_MMAP;
        if (ioctl(fd_, VIDIOC_REQBUFS, &req) < 0) {
            RCLCPP_ERROR(this->get_logger(), "VIDIOC_REQBUFS failed");
            return;
        }

        struct v4l2_buffer buf = {};
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        for (int i = 0; i < BUFFER_COUNT; i++) {
            buf.index = i;
            if (ioctl(fd_, VIDIOC_QUERYBUF, &buf) < 0) {
                RCLCPP_ERROR(this->get_logger(), "VIDIOC_QUERYBUF failed");
                return;
            }
            buf_length_[i] = buf.length;
            buffers_[i] = (unsigned char*)mmap(NULL, buf.length, PROT_READ | PROT_WRITE, MAP_SHARED, fd_, buf.m.offset);
            if (buffers_[i] == MAP_FAILED) {
                RCLCPP_ERROR(this->get_logger(), "mmap failed");
                return;
            }
            ioctl(fd_, VIDIOC_QBUF, &buf);
        }

        int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        if (ioctl(fd_, VIDIOC_STREAMON, &type) < 0) {
            RCLCPP_ERROR(this->get_logger(), "VIDIOC_STREAMON failed");
            return;
        }
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

        cv::Mat bgr = decode_dvpp(mjpg_data, mjpg_len);

        if (!bgr.empty()) {
            auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", bgr).toImageMsg();
            pub_->publish(*msg);
        }

        ioctl(fd_, VIDIOC_QBUF, &buf);
    }

    cv::Mat decode_dvpp(unsigned char* data, size_t len) {
        cv::Mat empty;
        if (len < 100) {
            RCLCPP_WARN(this->get_logger(), "decode_dvpp: data too small (%zu bytes)", len);
            return empty;
        }

        acldvppChannelDesc* channelDesc = acldvppCreateChannelDesc();
        if (!channelDesc) {
            RCLCPP_ERROR(this->get_logger(), "decode_dvpp: acldvppCreateChannelDesc failed");
            return empty;
        }
        if (acldvppCreateChannel(channelDesc) != 0) {
            RCLCPP_ERROR(this->get_logger(), "decode_dvpp: acldvppCreateChannel failed");
            return empty;
        }

        uint32_t w = 0, h = 0;
        int32_t comp = 0;
        if (acldvppJpegGetImageInfoV2(data, len, &w, &h, &comp, nullptr) != 0) {
            RCLCPP_ERROR(this->get_logger(), "decode_dvpp: acldvppJpegGetImageInfoV2 failed");
            acldvppDestroyChannel(channelDesc);
            return empty;
        }

        uint32_t predictSize = 0;
        if (acldvppJpegPredictDecSize(data, len, PIXEL_FORMAT_YUV_SEMIPLANAR_420, &predictSize) != 0) {
            RCLCPP_ERROR(this->get_logger(), "decode_dvpp: acldvppJpegPredictDecSize failed");
            acldvppDestroyChannel(channelDesc);
            return empty;
        }

        void* out_buf = nullptr;
        if (acldvppMalloc(&out_buf, predictSize) != 0) {
            RCLCPP_ERROR(this->get_logger(), "decode_dvpp: acldvppMalloc failed");
            acldvppDestroyChannel(channelDesc);
            return empty;
        }
        malloc_count++;
        RCLCPP_DEBUG(this->get_logger(), "malloc_count: %d", malloc_count.load());

        acldvppPicDesc* picDesc = acldvppCreatePicDesc();
        if (!picDesc) {
            RCLCPP_ERROR(this->get_logger(), "decode_dvpp: acldvppCreatePicDesc failed");
            acldvppFree(out_buf);
            free_count++;
            acldvppDestroyChannel(channelDesc);
            return empty;
        }

        acldvppSetPicDescData(picDesc, out_buf);
        acldvppSetPicDescSize(picDesc, predictSize);
        acldvppSetPicDescFormat(picDesc, PIXEL_FORMAT_YUV_SEMIPLANAR_420);
        acldvppSetPicDescWidth(picDesc, w);
        acldvppSetPicDescHeight(picDesc, h);
        int aligned_w = ((w + 15) / 16) * 16;
        int aligned_h = ((h + 15) / 16) * 16;
        acldvppSetPicDescWidthStride(picDesc, aligned_w);
        acldvppSetPicDescHeightStride(picDesc, aligned_h);

        aclrtStream stream = nullptr;
        aclrtCreateStream(&stream);

        cv::Mat bgr;
        if (acldvppJpegDecodeAsync(channelDesc, data, len, picDesc, stream) == 0) {
            aclrtSynchronizeStream(stream);

            cv::Mat yuv(h * 3 / 2, w, CV_8UC1);
            aclrtMemcpy(yuv.data, predictSize, out_buf, predictSize, ACL_MEMCPY_DEVICE_TO_HOST);
            cv::cvtColor(yuv, bgr, cv::COLOR_YUV2BGR_NV12);
        } else {
            RCLCPP_ERROR(this->get_logger(), "decode_dvpp: acldvppJpegDecodeAsync failed");
        }

        aclrtDestroyStream(stream);
        acldvppFree(out_buf);
        free_count++;
        RCLCPP_DEBUG(this->get_logger(), "free_count: %d, diff: %d", free_count.load(), malloc_count.load() - free_count.load());

        acldvppDestroyPicDesc(picDesc);
        acldvppDestroyChannel(channelDesc);

        return bgr.clone();
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
    auto node = std::make_shared<DvppCameraNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
