// 这base——controller是我们构建节点的部分，这节点构建的好阿！！！
// 他定义的那些接口其实就是我们python搞属性的那一部分啊！！！
// 就是定义这些东西出来方便我们控制那些发布者，服务商的！
// 他们似乎已经把与mavros等等节点通信的部分做好了，我们无需官通信部分，只要使用他抽象出来的几个功能即可
#ifndef OFFBOARD_BASE_CONTROLLER_HPP
#define OFFBOARD_BASE_CONTROLLER_HPP

#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>

#include <mavros_msgs/msg/state.hpp>
#include <mavros_msgs/srv/command_bool.hpp>
#include <mavros_msgs/srv/set_mode.hpp>
#include <mavros_msgs/srv/command_long.hpp> // 新增

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <functional>
#include <memory>
#include <chrono>
#include "offboard/pid_controller.hpp"//very good!
#include "msg_tool/msg/flight_info.hpp"//这个找不到，似乎是因为我们没有编译
#include <cmath>


namespace offboard
{

enum class FlightState//枚举类型
{
    // 通用飞行状态
    INIT,//初始化阶段。等待起飞授权（launch_flag_ == true）（这是设计目标，但此时还没有在base_controller里面实现）与障碍物信息接收完毕。满足条件后进入 TAKEOFF。
    TAKEOFF,//起飞阶段。无人机从当前点垂直爬升到预设高度（如 1.2m），完成后转入 WAYPOINT。
    WAYPOINT,//进入航点模式，会根据你的点去飞行。
    APPROACH,//	回家阶段——从最后一个航点飞向目标降落点上空，调整位置和姿态，为倾斜降落做准备，
    TILTLAND,//倾斜降落阶段。无人机在指定区域执行前倾降落（可能是抓取或特殊降落），完成后转入 LAND。
    LAND,//	最终着陆锁桨，电机停转，任务结束
    // 特殊飞行状态
    HOVER_3S,//（悬停3秒）
    FLY_TO_MIDPOINT,//(飞到中点等小车)
    COMPANION_FLIGHT,//（伴飞）
    DROP,//（抛投）
    RETURN_HOME,//（返航回起降点）
    SEARCH_CAR,//（搜索小车）
    APPROACH_CAR,//（接近小车）
    LAND_ON_CAR,//（降落到小车平台）
    STAY_ON_CAR,//（停留5秒）
    TAKEOFF_FROM_CAR//（从小车平台起飞）
};

// 悬停状态枚举
enum class HoverState {
    IDLE,           // 未悬停
    POSITIONING,    // 正在移动到悬停位置
    HOVERING,       // 正在悬停
    COMPLETED       // 悬停完成
};

// 悬停配置结构体
struct HoverConfig {//为什么这构造函数不给传参？配置这个实现悬停。
    double x, y, z, yaw;                    // 悬停位置和姿态
    double duration;                        // 悬停时间（秒），-1表示无限时间
    double position_tolerance;              // 位置容差
    double yaw_tolerance;                   // 偏航角容差
    bool use_current_position;              // 是否使用当前位置作为悬停点
    std::function<bool()> exit_condition;   // 退出条件函数（可选）
    
    // HoverConfig() : 
    //     x(0.0), y(0.0), z(0.0), yaw(0.0), 
    //     duration(-1.0), 
    //     position_tolerance(0.05), 
    //     yaw_tolerance(0.05),
    //     use_current_position(true),
    //     exit_condition(nullptr) {}

    HoverConfig(
    double _x = 0.0,double _y=0.0,double _z=0.0,double _yaw=0.0,
    double _duration = -1.0,double _pt=0.05,double _yt=0.05,
    bool _ucp = true,std::function<bool()> _ec=nullptr) : 

    x(_x), y(_y), z(_z), yaw(_yaw), 
    duration(_duration), 
    position_tolerance(_pt), 
    yaw_tolerance(_yt),
    use_current_position(_ucp),
    exit_condition(_ec) {}
};

class BaseController : public rclcpp::Node//这是节点
{
public:
    explicit BaseController(const std::string& node_name);

    // 配置QoS the two variable will be used many times
    rclcpp::QoS qos_best_effort = rclcpp::QoS(10)//这个10是队列长度的意思，这个初始化Qos的方法是链式的
        .best_effort()
        .durability_volatile();// 最佳努力传输，一直发
    rclcpp::QoS qos_reliable = rclcpp::QoS(10)
        .reliable()
        .durability_volatile();// 可靠传输，单发
    
    // 基础控制功能
    void arm();//解锁用的
    void engage_offboard_mode() const;//const意思是说绝对不改变成元变量
    void land();//原地垂直降落
    void auto_land();//切换自动降落模式
    void task_state_pub() const;//发布任务话题
    void publish_position_setpoint(double x, double y, double z, double yaw);
    void publish_velocity_body(double v_x, double v_y, double v_z, double v_yaw);


    // 优化的悬停接口
    bool start_hover(const HoverConfig& config);
    bool start_hover(double duration);  // 简化版本：在当前位置悬停指定时间
    bool start_hover_at_position(double x,double y,double z,double yaw, double duration);
    bool start_hover_with_condition(const std::function<bool()>& exit_condition);  // 条件悬停
    void stop_hover();
    bool update_hover();  // 更新悬停状态，返回是否完成
    HoverState get_hover_state() const { return hover_state_; }
    bool is_hovering() const { return hover_state_ == HoverState::HOVERING; }
    
    // 工具函数
    double distance_to_target(double x, double y, double z) const;
    static std::vector<double> get_euler_from_pose(const geometry_msgs::msg::Pose& pose);
    void rotation(int reverse);
    static double shortest_angular_distance(double from, double to);
    static std::string flightStateToString(const FlightState& state);

protected:
    // 回调函数
    virtual void timer_callback() = 0;//这么写，是为了后续方便你继承之后重写
    void vehicle_state_callback(const mavros_msgs::msg::State::ConstSharedPtr& msg);
    void local_position_callback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr& msg);
    void arm_callback(std::shared_future<std::shared_ptr<mavros_msgs::srv::CommandBool::Response>> future);
    void set_mode_callback(std::shared_future<std::shared_ptr<mavros_msgs::srv::SetMode::Response>> future) const;
    void launch_callback(const std_msgs::msg::Bool::ConstSharedPtr& future);
    void task_callback(const std_msgs::msg::Int32::ConstSharedPtr& msg);
    
    // 悬停相关的私有方法
    void reset_hover_state();
    bool check_hover_position_reached();
    bool check_hover_exit_conditions();

    // ROS2 接口：这些就是你创建节点的那些功能之后返回给你的遥控器
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr trajectory_setpoint_pub_;//这个与下一个要不停的发
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr velocity_setpoint_pub_;
    rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr vehicle_state_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr local_position_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr launch_sub_;
    rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arming_client_;
    rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr set_mode_client_;
    rclcpp::Client<mavros_msgs::srv::CommandLong>::SharedPtr command_client_; // 修正类型
    rclcpp::TimerBase::SharedPtr timer_;//这是定时器的遥控器
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr task_sub_;
    rclcpp::Publisher<msg_tool::msg::FlightInfo>::SharedPtr task_pub_;//this is send to gstation

    // 申请offboard与解锁（arm）的标志位
    mutable bool apply_offboard_flag_=false;//家这个mutable代表在函数末尾加上const的函数也可以改变他
    bool apply_arm_flag_=false;
    bool apply_disarm_flag_=false;
    // 降落时切换状态标志位
    bool mode_switched_for_landing_ = false;
    // 状态变量
    FlightState flight_state_ = FlightState::INIT;
    mavros_msgs::msg::State current_state_;//mavros
    geometry_msgs::msg::PoseStamped local_position_;//这个获取当前位置点与四元数姿态，这个数据从谁那里获得的呢？
    bool launch_flag_ = false;
    int current_task_id_ = -1; // 存储当前任务ID
    // 当前姿态
    double current_roll_ = 0, current_pitch_ = 0, current_yaw_ = 0;//那这么说我摆放时要注意了，你当前的飞控朝向是认为角度0的。

    
    // 速度限制
    double max_linear_velocity_;
    double max_angular_velocity_;
    // PID 参数

    double kP_yaw_ = 1.0;             // yaw 比例增益
    double last_err_body_x_ = 0.0;
    double last_err_body_y_ = 0.0;
    double last_err_body_z_ = 0.0;
    double integral_err_body_x_ = 0.0;
    double integral_err_body_y_ = 0.0;
    double integral_err_body_z_ = 0.0;

    struct TrajectorySegment 
    {
    double start_x, start_y, start_z;
    double end_x, end_y, end_z;
    double length;
    double direction_x, direction_y, direction_z; // 单位方向向量
    };
    //pd控制函数。
    void publish_position_setpoint_trajectory(
        const double start_x, const double start_y, const double start_z,
        const double end_x, const double end_y, const double end_z,
        const double target_yaw) ;

    // 类成员变量
    double last_cross_err_x_ = 0.0;
    double last_cross_err_y_ = 0.0;
    rclcpp::Time last_cross_time_ = this->get_clock()->now();

    rclcpp::Time last_velo_pid_time_ = this->get_clock()->now();
    rclcpp::Time last_time_ = this->get_clock()->now();
    
    // 悬停相关状态
    HoverState hover_state_;
    HoverConfig hover_config_;
    rclcpp::Time hover_start_time_;
    
};

} // namespace offboard

#endif // OFFBOARD_BASE_CONTROLLER_HPP