import rclpy
from rclpy.node import Node
import time
from std_msgs.msg import Bool


class LaunchFlagNode(Node):
    def __init__(self,name):
        super().__init__(name)
        self.pub=self.create_publisher(Bool,"launch",10)
        self.timer=self.create_timer(0.1,self.timer_callback)

    def timer_callback(self):
        msg=Bool()
        msg.data=False
        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args) #初始化rclpy
    node = LaunchFlagNode("node_launch_flag_false") #创建节点对象
    rclpy.spin(node) #进入自旋状态，如果没有这个函数，程序会直接退出，这个发布者节点还没有发布信息，他就被销毁了
    node.destroy_node()                              # 销毁节点对象
    rclpy.shutdown()   
