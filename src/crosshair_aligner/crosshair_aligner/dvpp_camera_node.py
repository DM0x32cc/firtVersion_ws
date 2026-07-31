#!/usr/bin/env python3
import os
import sys
import time
import fcntl
import struct
import mmap
import ctypes
import threading
from collections import deque

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

# ========== V4L2 Constants ==========
V4L2_BUF_TYPE_VIDEO_CAPTURE = 1
V4L2_FIELD_NONE = 1
V4L2_PIX_FMT_MJPEG = 0x47504A4D
V4L2_MEMORY_MMAP = 1

VIDIOC_S_FMT = 0x40285604
VIDIOC_REQBUFS = 0x40085608
VIDIOC_QUERYBUF = 0x40285609
VIDIOC_QBUF = 0x40285610
VIDIOC_DQBUF = 0x40285611
VIDIOC_STREAMON = 0x40085612
VIDIOC_STREAMOFF = 0x40085613

# ========== ACL / DVPP ==========
acl = ctypes.CDLL("libascendcl.so")
acl_dvpp = ctypes.CDLL("libacl_dvpp.so")

acl.aclInit.argtypes = [ctypes.c_void_p]
acl.aclInit.restype = ctypes.c_int
acl.aclrtSetDevice.argtypes = [ctypes.c_int]
acl.aclrtSetDevice.restype = ctypes.c_int
acl.aclrtCreateContext.argtypes = [ctypes.POINTER(ctypes.c_void_p), ctypes.c_int]
acl.aclrtCreateContext.restype = ctypes.c_int
acl.aclrtDestroyContext.argtypes = [ctypes.c_void_p]
acl.aclrtDestroyContext.restype = ctypes.c_int
acl.aclFinalize.argtypes = []
acl.aclFinalize.restype = None
acl.aclrtCreateStream.argtypes = [ctypes.POINTER(ctypes.c_void_p)]
acl.aclrtCreateStream.restype = ctypes.c_int
acl.aclrtDestroyStream.argtypes = [ctypes.c_void_p]
acl.aclrtDestroyStream.restype = ctypes.c_int
acl.aclrtSynchronizeStream.argtypes = [ctypes.c_void_p]
acl.aclrtSynchronizeStream.restype = ctypes.c_int
acl.aclrtMemcpy.argtypes = [ctypes.c_void_p, ctypes.c_size_t, ctypes.c_void_p, ctypes.c_size_t, ctypes.c_int]
acl.aclrtMemcpy.restype = ctypes.c_int

acl_dvpp.acldvppCreateChannelDesc.argtypes = []
acl_dvpp.acldvppCreateChannelDesc.restype = ctypes.c_void_p
acl_dvpp.acldvppCreateChannel.argtypes = [ctypes.c_void_p]
acl_dvpp.acldvppCreateChannel.restype = ctypes.c_int
acl_dvpp.acldvppDestroyChannel.argtypes = [ctypes.c_void_p]
acl_dvpp.acldvppDestroyChannel.restype = ctypes.c_int
acl_dvpp.acldvppCreatePicDesc.argtypes = []
acl_dvpp.acldvppCreatePicDesc.restype = ctypes.c_void_p
acl_dvpp.acldvppDestroyPicDesc.argtypes = [ctypes.c_void_p]
acl_dvpp.acldvppDestroyPicDesc.restype = ctypes.c_int
acl_dvpp.acldvppSetPicDescData.argtypes = [ctypes.c_void_p, ctypes.c_void_p]
acl_dvpp.acldvppSetPicDescData.restype = None
acl_dvpp.acldvppSetPicDescSize.argtypes = [ctypes.c_void_p, ctypes.c_uint32]
acl_dvpp.acldvppSetPicDescSize.restype = None
acl_dvpp.acldvppSetPicDescFormat.argtypes = [ctypes.c_void_p, ctypes.c_int]
acl_dvpp.acldvppSetPicDescFormat.restype = None
acl_dvpp.acldvppSetPicDescWidth.argtypes = [ctypes.c_void_p, ctypes.c_uint32]
acl_dvpp.acldvppSetPicDescWidth.restype = None
acl_dvpp.acldvppSetPicDescHeight.argtypes = [ctypes.c_void_p, ctypes.c_uint32]
acl_dvpp.acldvppSetPicDescHeight.restype = None
acl_dvpp.acldvppSetPicDescWidthStride.argtypes = [ctypes.c_void_p, ctypes.c_uint32]
acl_dvpp.acldvppSetPicDescWidthStride.restype = None
acl_dvpp.acldvppSetPicDescHeightStride.argtypes = [ctypes.c_void_p, ctypes.c_uint32]
acl_dvpp.acldvppSetPicDescHeightStride.restype = None
acl_dvpp.acldvppJpegGetImageInfoV2.argtypes = [ctypes.c_void_p, ctypes.c_uint32, ctypes.POINTER(ctypes.c_uint32), ctypes.POINTER(ctypes.c_uint32), ctypes.POINTER(ctypes.c_int32), ctypes.c_void_p]
acl_dvpp.acldvppJpegGetImageInfoV2.restype = ctypes.c_int
acl_dvpp.acldvppJpegPredictDecSize.argtypes = [ctypes.c_void_p, ctypes.c_uint32, ctypes.c_int, ctypes.POINTER(ctypes.c_uint32)]
acl_dvpp.acldvppJpegPredictDecSize.restype = ctypes.c_int
acl_dvpp.acldvppJpegDecodeAsync.argtypes = [ctypes.c_void_p, ctypes.c_void_p, ctypes.c_uint32, ctypes.c_void_p, ctypes.c_void_p]
acl_dvpp.acldvppJpegDecodeAsync.restype = ctypes.c_int
acl_dvpp.acldvppMalloc.argtypes = [ctypes.POINTER(ctypes.c_void_p), ctypes.c_size_t]
acl_dvpp.acldvppMalloc.restype = ctypes.c_int
acl_dvpp.acldvppFree.argtypes = [ctypes.c_void_p]
acl_dvpp.acldvppFree.restype = None

PIXEL_FORMAT_YUV_SEMIPLANAR_420 = 1

def yuv420_to_bgr(yuv_data, width, height):
    y_size = width * height
    uv_size = y_size // 2
    yuv = np.zeros((height * 3 // 2, width), dtype=np.uint8)
    yuv[0:height, 0:width] = yuv_data[0:y_size].reshape(height, width)
    yuv[height:height + uv_size // 2, 0:width] = yuv_data[y_size:y_size + uv_size].reshape(uv_size // 2, width)
    yuv[height + uv_size // 2:, 0:width] = yuv_data[y_size + uv_size:].reshape(uv_size // 2, width)
    return cv2.cvtColor(yuv, cv2.COLOR_YUV2BGR_NV12)


class DvppCameraNode(Node):
    def __init__(self, device="/dev/video0"):
        super().__init__('dvpp_camera_node')
        self.device = device
        self.running = True
        self.fd = None

        self.pub = self.create_publisher(Image, '/camera/image_raw', 10)
        self.bridge = CvBridge()

        self._init_acl()
        self._init_v4l2()

        self.frame_queue = deque(maxlen=2)
        self.thread = threading.Thread(target=self._camera_loop, daemon=True)
        self.thread.start()

        self.timer = self.create_timer(1.0 / 30.0, self._publish_frame)
        self.get_logger().info("DvppCameraNode started")

    def _init_acl(self):
        if acl.aclInit(None) != 0:
            raise RuntimeError("aclInit failed")
        if acl.aclrtSetDevice(0) != 0:
            raise RuntimeError("aclrtSetDevice failed")
        self.context_ptr = ctypes.c_void_p()
        if acl.aclrtCreateContext(ctypes.byref(self.context_ptr), 0) != 0:
            raise RuntimeError("aclrtCreateContext failed")

    def _init_v4l2(self):
        self.fd = os.open(self.device, os.O_RDWR | os.O_NONBLOCK)
        if self.fd < 0:
            raise RuntimeError(f"Cannot open {self.device}")

        # Set format
        fmt = struct.pack('IHHIIIIIIII', V4L2_BUF_TYPE_VIDEO_CAPTURE, 1280, 720,
                         V4L2_PIX_FMT_MJPEG, V4L2_FIELD_NONE, 0, 0, 0, 0, 0, 0)
        fcntl.ioctl(self.fd, VIDIOC_S_FMT, fmt)

        # Request buffers
        req = struct.pack('IIII', V4L2_BUF_TYPE_VIDEO_CAPTURE, 4, V4L2_MEMORY_MMAP, 0)
        fcntl.ioctl(self.fd, VIDIOC_REQBUFS, req)

        self.buffers = []
        self.buf_lengths = []
        for i in range(4):
            buf = struct.pack('IIIIIIIIII', V4L2_BUF_TYPE_VIDEO_CAPTURE, i,
                             V4L2_MEMORY_MMAP, 0, 0, 0, 0, 0, 0, 0)
            fcntl.ioctl(self.fd, VIDIOC_QUERYBUF, buf)
            length = struct.unpack('I', buf[8:12])[0]
            offset = struct.unpack('I', buf[12:16])[0]
            mem = mmap.mmap(self.fd, length, mmap.MAP_SHARED, mmap.PROT_READ | mmap.PROT_WRITE, offset=offset)
            self.buffers.append(mem)
            self.buf_lengths.append(length)
            qbuf = struct.pack('IIIIIIIIII', V4L2_BUF_TYPE_VIDEO_CAPTURE, i,
                              V4L2_MEMORY_MMAP, 0, 0, 0, 0, 0, 0, 0)
            fcntl.ioctl(self.fd, VIDIOC_QBUF, qbuf)

        val = struct.pack('I', V4L2_BUF_TYPE_VIDEO_CAPTURE)
        fcntl.ioctl(self.fd, VIDIOC_STREAMON, val)

    def _camera_loop(self):
        while self.running and self.fd is not None:
            try:
                buf = struct.pack('IIIIIIIIII', V4L2_BUF_TYPE_VIDEO_CAPTURE, 0,
                                 V4L2_MEMORY_MMAP, 0, 0, 0, 0, 0, 0, 0)
                fcntl.ioctl(self.fd, VIDIOC_DQBUF, buf)
                idx = struct.unpack('I', buf[4:8])[0]
                bytesused = struct.unpack('I', buf[8:12])[0]

                mjpg_data = self.buffers[idx][:bytesused]
                bgr = self._decode_dvpp(mjpg_data)
                if bgr is not None:
                    self.frame_queue.append(bgr)

                qbuf = struct.pack('IIIIIIIIII', V4L2_BUF_TYPE_VIDEO_CAPTURE, idx,
                                  V4L2_MEMORY_MMAP, 0, 0, 0, 0, 0, 0, 0)
                fcntl.ioctl(self.fd, VIDIOC_QBUF, qbuf)

            except Exception as e:
                self.get_logger().error(f"Camera loop error: {e}")
                time.sleep(0.01)

    def _decode_dvpp(self, mjpg_data):
        if len(mjpg_data) < 100:
            return None

        channel_desc = acl_dvpp.acldvppCreateChannelDesc()
        if acl_dvpp.acldvppCreateChannel(channel_desc) != 0:
            return None

        w = ctypes.c_uint32(0)
        h = ctypes.c_uint32(0)
        comp = ctypes.c_int32(0)
        if acl_dvpp.acldvppJpegGetImageInfoV2(mjpg_data, len(mjpg_data), ctypes.byref(w), ctypes.byref(h), ctypes.byref(comp), None) != 0:
            acl_dvpp.acldvppDestroyChannel(channel_desc)
            return None

        predict_size = ctypes.c_uint32(0)
        if acl_dvpp.acldvppJpegPredictDecSize(mjpg_data, len(mjpg_data), PIXEL_FORMAT_YUV_SEMIPLANAR_420, ctypes.byref(predict_size)) != 0:
            acl_dvpp.acldvppDestroyChannel(channel_desc)
            return None

        out_buf = ctypes.c_void_p()
        if acl_dvpp.acldvppMalloc(ctypes.byref(out_buf), predict_size.value) != 0:
            acl_dvpp.acldvppDestroyChannel(channel_desc)
            return None

        pic_desc = acl_dvpp.acldvppCreatePicDesc()
        acl_dvpp.acldvppSetPicDescData(pic_desc, out_buf)
        acl_dvpp.acldvppSetPicDescSize(pic_desc, predict_size.value)
        acl_dvpp.acldvppSetPicDescFormat(pic_desc, PIXEL_FORMAT_YUV_SEMIPLANAR_420)
        acl_dvpp.acldvppSetPicDescWidth(pic_desc, w.value)
        acl_dvpp.acldvppSetPicDescHeight(pic_desc, h.value)
        acl_dvpp.acldvppSetPicDescWidthStride(pic_desc, ((w.value + 15) // 16) * 16)
        acl_dvpp.acldvppSetPicDescHeightStride(pic_desc, ((h.value + 15) // 16) * 16)

        stream = ctypes.c_void_p()
        acl.aclrtCreateStream(ctypes.byref(stream))
        if acl_dvpp.acldvppJpegDecodeAsync(channel_desc, mjpg_data, len(mjpg_data), pic_desc, stream) != 0:
            acl.aclrtDestroyStream(stream)
            acl_dvpp.acldvppFree(out_buf)
            acl_dvpp.acldvppDestroyPicDesc(pic_desc)
            acl_dvpp.acldvppDestroyChannel(channel_desc)
            return None

        acl.aclrtSynchronizeStream(stream)

        host_buf = ctypes.create_string_buffer(predict_size.value)
        acl.aclrtMemcpy(host_buf, predict_size.value, out_buf, predict_size.value, 0)

        yuv_data = np.frombuffer(host_buf.raw, dtype=np.uint8)
        bgr = yuv420_to_bgr(yuv_data, w.value, h.value)

        acl.aclrtDestroyStream(stream)
        acl_dvpp.acldvppFree(out_buf)
        acl_dvpp.acldvppDestroyPicDesc(pic_desc)
        acl_dvpp.acldvppDestroyChannel(channel_desc)

        return bgr

    def _publish_frame(self):
        if len(self.frame_queue) > 0:
            frame = self.frame_queue.pop()
            msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            self.pub.publish(msg)

    def __del__(self):
        self.running = False
        if self.fd is not None:
            val = struct.pack('I', V4L2_BUF_TYPE_VIDEO_CAPTURE)
            try:
                fcntl.ioctl(self.fd, VIDIOC_STREAMOFF, val)
            except:
                pass
            os.close(self.fd)
        if hasattr(self, 'context_ptr') and self.context_ptr:
            acl.aclrtDestroyContext(self.context_ptr)
            acl.aclFinalize()


def main(args=None):
    rclpy.init(args=args)
    node = DvppCameraNode("/dev/video0")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
