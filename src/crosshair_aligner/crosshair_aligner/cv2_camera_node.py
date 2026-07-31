#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import threading
import os
from collections import deque

class Cv2CameraNode(Node):
    def __init__(self):
        super().__init__('cv2_camera_node')

        self.pub = self.create_publisher(Image, '/camera/image_raw', 10)
        self.bridge = CvBridge()
        self.running = True

        self.cap = cv2.VideoCapture("/dev/video100", cv2.CAP_V4L2)
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M','J','P','G'))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
        self.cap.set(cv2.CAP_PROP_FPS, 60)

        if not self.cap.isOpened():
            self.get_logger().error('Failed to open camera /dev/video100')
            return

        os.system('v4l2-ctl -d /dev/video100 --set-ctrl=exposure_auto=1')
        os.system('v4l2-ctl -d /dev/video100 --set-ctrl=white_balance_temperature_auto=0')
        os.system('v4l2-ctl -d /dev/video100 --set-ctrl=white_balance_temperature=4000')
        os.system('v4l2-ctl -d /dev/video100 --set-ctrl=backlight_compensation=0')

        self.frame_queue = deque(maxlen=2)

        self.thread = threading.Thread(target=self._reader_thread, daemon=True)
        self.thread.start()

        self.timer = self.create_timer(1.0 / 60.0, self.timer_callback)
        self.get_logger().info('Cv2CameraNode started (async), publishing /camera/image_raw from /dev/video100 at 1280x720 60fps')

    def _reader_thread(self):
        while self.running:
            ret, frame = self.cap.read()
            if ret:
                self.frame_queue.append(frame)

    def timer_callback(self):
        if len(self.frame_queue) > 0:
            frame = self.frame_queue.pop()
            msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            self.pub.publish(msg)

    def __del__(self):
        self.running = False
        if hasattr(self, 'cap'):
            self.cap.release()

def main(args=None):
    rclpy.init(args=args)
    node = Cv2CameraNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
