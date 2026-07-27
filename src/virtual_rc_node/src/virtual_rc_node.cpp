#include "rclcpp/rclcpp.hpp"
#include "mavros_msgs/msg/manual_control.hpp"

class VirtualRCNode : public rclcpp::Node
{
public:
    VirtualRCNode()
    : Node("virtual_rc_node")
    {
        // 创建发布者
        publisher_ = this->create_publisher<mavros_msgs::msg::ManualControl>(
            "/mavros/manual_control/control", 10);
        
        // 创建定时器 10Hz (100ms)
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&VirtualRCNode::publish_manual_control, this));
        
        RCLCPP_INFO(this->get_logger(), "Virtual RC Node Started");
    }

private:
    void publish_manual_control()
    {
        auto msg = mavros_msgs::msg::ManualControl();
        
        // ✅ 核心修改：使用 x, y, z, r 而不是 roll, pitch...
        msg.x = 0;      // 左右 (Roll)
        msg.y = 0;      // 前后 (Pitch)
        msg.z = 0;      // 油门 (Throttle)
        msg.r = 0;      // 偏航 (Yaw)
        
        // // ✅ 核心修改：初始化按钮和开关
        // msg.buttons = 0;
        // msg.switches = 0;
        
        publisher_->publish(msg);
    }
    
    rclcpp::Publisher<mavros_msgs::msg::ManualControl>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VirtualRCNode>());
    rclcpp::shutdown();
    return 0;
}