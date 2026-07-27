#include "rclcpp/rclcpp.hpp"
#include "mavros_msgs/msg/manual_control.hpp"

class VirtualRCNode : public rclcpp::Node
{
public:
    VirtualRCNode()
    : Node("virtual_rc_node")
    {
        // 创建 MANUAL_CONTROL 发布者
        publisher_ = this->create_publisher<mavros_msgs::msg::ManualControl>(
            "/mavros/manual_control/control", 10);
        
        // 创建定时器，10-50Hz 均可，推荐 20Hz
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),  // 20Hz
            std::bind(&VirtualRCNode::publish_manual_control, this));
        
        RCLCPP_INFO(this->get_logger(), "Virtual RC Node Started");
    }

private:
    void publish_manual_control()
    {
        auto msg = mavros_msgs::msg::ManualControl();
        
        // 所有通道设为 0，表示"有控制源但无操作"
        msg.roll = 0.0;      // -1.0 ~ 1.0
        msg.pitch = 0.0;     // -1.0 ~ 1.0
        msg.yaw = 0.0;       // -1.0 ~ 1.0
        msg.throttle = 0.0;  // -1.0 ~ 1.0
        
        // 辅助通道（如需要）
        msg.aux1 = 0.0;
        msg.aux2 = 0.0;
        msg.aux3 = 0.0;
        msg.aux4 = 0.0;
        
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