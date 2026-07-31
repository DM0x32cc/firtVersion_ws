import sys
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32


class GStationTaskNode(Node):
    def __init__(self, name, task_id):
        super().__init__(name)
        self.pub = self.create_publisher(Int32, "task", 10)
        self.task_id = task_id
        self.count = 0
        self.timer = self.create_timer(0.5, self.timer_callback)
        self.get_logger().info(f"地面站模拟: 任务 {task_id} 已发送")

    def timer_callback(self):
        msg = Int32()
        msg.data = self.task_id
        self.pub.publish(msg)
        self.count += 1
        if self.count >= 6:
            self.get_logger().info(f"任务 {self.task_id} 发送完毕，退出")
            rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)

    task_id = 1
    if len(sys.argv) > 1:
        try:
            task_id = int(sys.argv[1])
        except ValueError:
            pass

    if task_id not in (1, 2):
        print(f"用法: ros2 run launch_flag gstation_task [1|2]")
        rclpy.shutdown()
        return

    node = GStationTaskNode("gstation_task", task_id)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
