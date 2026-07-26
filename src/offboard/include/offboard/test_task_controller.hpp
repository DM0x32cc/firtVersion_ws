#ifndef OFFBOARD_TEST_TASK_CONTROLLER_HPP
#define OFFBOARD_TEST_TASK_CONTROLLER_HPP


#include "offboard/base_controller.hpp"
#include "msg_tool/msg/color.hpp"
#include "msg_tool/msg/line.hpp"
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/point32.hpp>
#include <geometry_msgs/msg/polygon.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/int32.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <memory>



namespace offboard
{
    class TaskController : public BaseController
{
public:
    TaskController();//不需要初始化列表么？ 这里只是声明

protected:
    void execute_waypoint_mission();
    bool check_task_switch_conditions();
    bool tilt_land();
    bool waypoint_generate();//临时代替
    void switch_task(FlightState new_state);
    bool is_at_point(const double x, const double y, const double z) const ;
    void check_position_stuck_protection();
    bool check_emergency_condition();
    void reset_position_stuck_detection();
   


    // 回调函数
    void timer_callback()override;
    
    // 工具函数
    void compress_waypoints(std::vector<std::vector<double>>& waypoints);//航点压缩非常好的一个工具



    // bool approach();

    // 运行参数以及数据
    msg_tool::msg::Color target_msg_;
    struct ProcessedTarget 
    {
        std::string color;
        double x;
        double y;
    };
    
    geometry_msgs::msg::Polygon path_msg;

    
    
    //目标处理情况相关
    std::vector<ProcessedTarget> processed_targets_;
    double processed_target_radius_ = 0.08;  // 认为已处理目标的判定半径
    int approach_success_count_ = 0; // 连续成功帧计数
    const int APPROACH_THRESHOLD = 3; // 连续成功帧阈值
    //飞机运行相关，防卡位监测相关参数
    rclcpp::Time last_position_time_;
    double last_position_x_ = 0.0;
    double last_position_y_ = 0.0;
    double last_position_z_ = 0.0;
    bool position_stuck_detected_ = false;
    rclcpp::Time stuck_detection_time_;
    rclcpp::Time ignore_target_until_;
    bool ignoring_targets_ = false;
    // 航点相关
    std::vector<std::vector<double>> waypoints_;//这是航点，我们要事先把他计算出来存着
    size_t current_waypoint_index_ = 0;//这是索引，我们用他来记录我们已经打到的航点
    double takeoff_height_;
    double waypoint_threshold_=0.1;//距离阈值（单位：米），用于判断飞行器是否“到达”了目标航点。注意，这个用于不单单是水平座标点，包括高度上也是由这个来判断。也就是说判定其实是一个立方体
    // 起飞之前的检查，确保我们连续发送5s的点的消息，下面这些就是相关变量
    bool setpoint_ready_ = false;
    int offboard_setpoint_counter_ = 0;
};

}

#endif