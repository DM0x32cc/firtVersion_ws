#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/utils.h"

class OdomToPoseNode : public rclcpp::Node
{
public:
    OdomToPoseNode() : Node("odom_to_pose_node")
    {
        offset_ = 0.0;
        lidar_height_ = 0.0;
        checktime_ = 0;
        sum_offset_ = 0.0;
        data_ready_ = false;

        radar_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/Odometry", 10, std::bind(&OdomToPoseNode::radar_cb, this, std::placeholders::_1));//回调函数radar_cb

        lidar_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
            "/mavros/vision_pose/pose", 10);//这个数字10处，在另一个地方填的是qos:比如：qos_best_effort

        send_data_.header.frame_id = "map";

        // 创建定时器，10Hz = 100ms
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&OdomToPoseNode::timer_cb, this));
    }

private:
    void radar_cb(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        receive_data_ = *msg;

        if (checktime_ < 10)
        {
            checktime_++;
            sum_offset_ += receive_data_.pose.pose.position.z;
            RCLCPP_INFO(this->get_logger(), "Collecting offset sample %d: %.4lf", 
                        checktime_, receive_data_.pose.pose.position.z);
            return;
        }

        if (checktime_ == 10)
        {
            offset_ = sum_offset_ / 10.0;
            checktime_++;  // prevent repeated averaging
            RCLCPP_INFO(this->get_logger(), "Calculated offset: %.4lf", offset_);
        }

        // 更新处理后的数据
        lidar_height_ = receive_data_.pose.pose.position.z - offset_;
        send_data_.pose.position.x = receive_data_.pose.pose.position.x;
        send_data_.pose.position.y = receive_data_.pose.pose.position.y;
        send_data_.pose.position.z = lidar_height_;
        send_data_.pose.orientation = receive_data_.pose.pose.orientation;
        //x,y与yaw我们可以不用官，因为这是相对位置，我们不用那么精确。
        // 此外，雷达发布的
        data_ready_ = true;
    }

    void timer_cb()
    {
        if (data_ready_ && checktime_ > 10)
        {
            send_data_.header.stamp = this->now();
            send_data_.header.frame_id = "map";

            lidar_pub_->publish(send_data_);
        }
    }

    // 成员变量
    nav_msgs::msg::Odometry receive_data_;
    geometry_msgs::msg::PoseStamped send_data_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr radar_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr lidar_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    float offset_;
    float lidar_height_;
    int checktime_;
    float sum_offset_;
    bool data_ready_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<OdomToPoseNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
