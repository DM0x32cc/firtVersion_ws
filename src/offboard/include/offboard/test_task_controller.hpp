#ifndef OFFBOARD_TEST_TASK_CONTROLLER_HPP
#define OFFBOARD_TEST_TASK_CONTROLLER_HPP


#include "offboard/base_controller.hpp" 
#include "msg_tool/msg/color.hpp"
#include "msg_tool/msg/line.hpp"
#include "msg_tool/msg/car_state.hpp"
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
    // bool waypoint_generate();//临时代替
    // 通用任务函数
    void execute_waypoint_mission();
    bool tilt_land();
    void return_home();

    // task1 任务函数
    void hover_3s();
    void fly_to_point();
    void companion_fly();
    void do_drop();
    void trigger_drop_servo();
    void search_car();
    void approach_car();
    void land_on_car();
    void stay_on_car();
    void takeoff_from_car();


    void switch_task(FlightState new_state);
    bool is_at_point(const double x, const double y, const double z) const ;
    bool check_emergency_condition();
    bool check_task_switch_conditions();
    // 回调函数
    void timer_callback()override;
    void target_callback(const msg_tool::msg::Color::ConstSharedPtr& msg);
    void car_state_callback(const msg_tool::msg::CarState::ConstSharedPtr& msg);
    // 工具函数
    void compress_waypoints(std::vector<std::vector<double>>& waypoints);//航点压缩非常好的一个工具
    double pixel_to_meter_x(double delta_pixel, double height);
    double pixel_to_meter_y(double delta_pixel, double height);

    // tilt_land()函数所需变量
    // tilt_land 阶段状态（非 static，每次任务重置）
    bool tilt_stage1_completed_ = false;// 第一阶段是否完成
    bool tilt_stage1_started_ = false;// 第一阶段是否已开始
    int tilt_stage2_stuck_counter_ = 0;//新增
    double tilt_stage2_last_z_ = -1.0;//新增

    // 订阅话题： target，就是摄像头看见的对象
    rclcpp::Subscription<msg_tool::msg::Color>::SharedPtr target_sub;
    double filtered_car_x= 0;
    double filtered_car_y= 0;
    double filter_param_company_ ;
    // 订阅话题: 获得小车状态
    rclcpp::Subscription<msg_tool::msg::CarState>::SharedPtr car_state_sub;

    // 订阅话题消息存储
        // 视觉识别小车位置
    msg_tool::msg::Color target_msg_;
        // 小车速度以及朝向
    // msg_tool::msg::CarState car_state_msg_;
    double car_speed_x_;
    double car_speed_y_;
    double deviation_angle_;


    // 伴飞相关
    bool target_data_ready_=false;  //help you check
    offboard::PIDController cpfly_pid_x_;   // 前后方向 PID，输入: e_filt_x，输出: 前后修正速度
    offboard::PIDController cpfly_pid_y_;   // 左右方向 PID，输入: e_filt_y，输出: 左右修正速度
    bool cpfly_takedown_ = false;

    // drop相关
    int drop_stable_count_ = 0;
    int drop_retry_count_ = 0;
    bool drop_sent_ = false;
    bool drop_confirmed_ = false;
    // 降落on car 上相关
    int approach_stable_count_ = 0;
    int touch_count_ = 0;
    double land_last_z_;
    offboard::PIDController land_pid_x_;
    offboard::PIDController land_pid_y_;
    rclcpp::Time stay_start_time_;
    // 运行参数以及数据
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
    void check_position_stuck_protection();
    void reset_position_stuck_detection();
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