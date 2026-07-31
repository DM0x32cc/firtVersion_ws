import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool


class GStationLaunchNode(Node):
    def __init__(self, name):
        super().__init__(name)
        self.pub = self.create_publisher(Bool, "launch", 10)
        self.start_time = self.get_clock().now()
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.get_logger().info("地面站模拟: 起飞信号已发送 (持续5秒)")

    def timer_callback(self):
        elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        msg = Bool()

        if elapsed < 5.0:
            msg.data = True
            self.pub.publish(msg)
        else:
            self.get_logger().info("起飞信号结束，退出")
            rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = GStationLaunchNode("gstation_launch")
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
