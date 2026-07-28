#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from msg_tool.msg import Color
from .detect_animal import Net


class AnimalTrackerNode(Node):
    def __init__(self):
        super().__init__('animal_tracker')

        self.declare_parameter('model_path', '/home/HwHiAiUser/best_animal.om')
        model_path = self.get_parameter('model_path').value

        self.get_logger().info(f'Loading model: {model_path}')
        self.net = Net(model_path)
        self.get_logger().info('Model loaded')

        self.bridge = CvBridge()
        self.latest_frame = None
        self.latest_header = None
        self.img_w = 0
        self.img_h = 0

        self.sub = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10)

        self.pub = self.create_publisher(Color, '/animal_tracker', 10)
        self.timer = self.create_timer(1.0 / 30.0, self.publish_result)

    def image_callback(self, msg):
        self.latest_header = msg.header
        self.latest_frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        if self.img_w == 0:
            self.img_h, self.img_w = self.latest_frame.shape[:2]
            self.get_logger().info(f'Image size: {self.img_w}x{self.img_h}')

    def publish_result(self):
        if self.latest_frame is None:
            return

        detections = self.net.infer(self.latest_frame)

        msg = Color()

        if len(detections) > 0:
            cx = self.img_w / 2.0
            cy = self.img_h / 2.0
            best_det = None
            best_dist = float('inf')

            for det in detections:
                bx = (det[0] + det[2]) / 2.0
                by = (det[1] + det[3]) / 2.0
                dist = math.hypot(bx - cx, by - cy)
                if dist < best_dist:
                    best_dist = dist
                    best_det = det

            if best_det is not None:
                bx = (best_det[0] + best_det[2]) / 2.0
                by = (best_det[1] + best_det[3]) / 2.0
                msg.detected = True
                msg.delta_x = (bx - cx) / (self.img_w / 2.0)
                msg.delta_y = (cy - by) / (self.img_h / 2.0)
                msg.color = 'animal'
            else:
                msg.detected = False
        else:
            msg.detected = False
            msg.delta_x = 0.0
            msg.delta_y = 0.0
            msg.color = ''

        self.pub.publish(msg)

    def destroy_node(self):
        if hasattr(self.net, 'release'):
            self.net.release()
        super().destroy_node()


def main():
    rclpy.init()
    node = AnimalTrackerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
