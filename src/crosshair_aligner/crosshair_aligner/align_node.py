#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from msg_tool.msg import Color
from cv_bridge import CvBridge
from crosshair_aligner.detect_crosshair import detect_crosshair


class AlignNode(Node):
    def __init__(self):
        super().__init__('align_node')

        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )

        self.cmd_publisher = self.create_publisher(
            Color,
            '/target',
            rclpy.qos.QoSProfile(
                depth=10,
                reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
                durability=rclpy.qos.DurabilityPolicy.VOLATILE
            )
        )
        self.bridge = CvBridge()
        self.img_width = None
        self.img_height = None

        self.get_logger().info('AlignNode started, waiting for images...')

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'Failed to convert image: {e}')
            return

        if self.img_width is None:
            self.img_height, self.img_width = cv_image.shape[:2]
            self.get_logger().info(f'Image size: {self.img_width}x{self.img_height}')

        cx, cy, detected = detect_crosshair(cv_image)

        cmd_msg = Color()
        cmd_msg.delta_x = 0.0
        cmd_msg.delta_y = 0.0
        cmd_msg.detected = detected

        if detected:
            center_x = self.img_width / 2.0
            center_y = self.img_height / 2.0
            cmd_msg.delta_x = cx - center_x
            cmd_msg.delta_y = cy - center_y

        self.cmd_publisher.publish(cmd_msg)

        if detected:
            self.get_logger().info(
                f'Target at ({cx:.1f}, {cy:.1f}), '
                f'offset ({cmd_msg.delta_x:.1f}, {cmd_msg.delta_y:.1f})',
                throttle_duration_sec=1.0
            )
        else:
            self.get_logger().info('No target detected', throttle_duration_sec=1.0)


def main(args=None):
    rclpy.init(args=args)
    node = AlignNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
