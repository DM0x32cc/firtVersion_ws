#include "offboard/base_controller.hpp"//我猜，编译之后就找到了
// 这个是辅助文件

namespace offboard
{

BaseController::BaseController(const std::string& node_name)
    : Node(node_name),
    hover_state_(HoverState::IDLE)
{

    // 声明并获取速度限制参数（他使用this来说明了自己从父类那里调用函数）
    this->declare_parameter("max_linear_velocity", 1.0);  // 默认1.0 m/s
    this->declare_parameter("max_angular_velocity", 0.5); // 默认0.5 rad/s


    // 加载速度限制参数
    max_linear_velocity_ = this->get_parameter("max_linear_velocity").as_double();
    max_angular_velocity_ = this->get_parameter("max_angular_velocity").as_double();


    // 创建发布者
    trajectory_setpoint_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
        "/mavros/setpoint_position/local", qos_best_effort);
    velocity_setpoint_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>(
        "/mavros/setpoint_velocity/cmd_vel", qos_best_effort);
    task_pub_ = create_publisher<msg_tool::msg::FlightInfo>(
        "/task_reply",qos_best_effort);
    // 发布任务
    // 创建订阅者
    vehicle_state_sub_ = create_subscription<mavros_msgs::msg::State>(
        "/mavros/state",
        qos_best_effort,
        [this](const mavros_msgs::msg::State::ConstSharedPtr& msg) {
            vehicle_state_callback(msg);
        });

    local_position_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
        "/mavros/local_position/pose",
        qos_best_effort,
        [this](const geometry_msgs::msg::PoseStamped::ConstSharedPtr& msg){
            local_position_callback(msg);
        });

    launch_sub_ = create_subscription<std_msgs::msg::Bool>(
        "/launch",
        qos_reliable,
        [this](const std_msgs::msg::Bool::ConstSharedPtr& msg){
            launch_callback(msg);
        });

    task_sub_ = create_subscription<std_msgs::msg::Int32>(
        "/task", qos_best_effort,
    [this](const std_msgs::msg::Int32::ConstSharedPtr& msg){
            task_callback(msg);
        });
        
    // 创建服务客户端
    arming_client_ = create_client<mavros_msgs::srv::CommandBool>("/mavros/cmd/arming");
    set_mode_client_ = create_client<mavros_msgs::srv::SetMode>("/mavros/set_mode");
    command_client_ = create_client<mavros_msgs::srv::CommandLong>("/mavros/cmd/command"); // 新增

    // 创建定时器
    timer_ = create_wall_timer(
        std::chrono::milliseconds(100),//设定定时器触发时间
        // std::bind(&BaseController::timer_callback, this));//这行代码是与下面含义相同的，下面是lambda表达式
    [this]()   /*这是cpp传回调函数的方法，python里面是直接传的，这是定义了一个lambda函数，后续的毁掉函数都是这么传进去的！*/
            {
                timer_callback();//这似乎要我们自己去定义？对对对，这是唯一一个给我们操作空间的多态可改函数
            });

    RCLCPP_INFO(get_logger(), "Base controller initialized");
}

void BaseController::arm()
{
    if (!arming_client_->wait_for_service(std::chrono::seconds(1))) {
        RCLCPP_ERROR(get_logger(), "解锁服务不可用");
        return;
    }


    auto request = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
    request->value = true;
    
    // 下面这里实现了发送，而且一有response就会去调用arm_callback,future会接受这个response，async意思是异步
    auto result_future = arming_client_->async_send_request(
        request,
        [this](const std::shared_future<std::shared_ptr<mavros_msgs::srv::CommandBool::Response>> future)
        {
            arm_callback(future);
        });
    apply_arm_flag_ = true;//代表此次已经发送请求
}
// 这个response是这个service的回应，我们根据回应来判断是否切换成功
void BaseController::arm_callback(const std::shared_future<std::shared_ptr<mavros_msgs::srv::CommandBool::Response>> future)
{
    try {
        auto result = future.get();
        if (result->success) {
            RCLCPP_INFO(get_logger(), "无人机已解锁，hiahiahia！");
        } else {
            RCLCPP_ERROR(get_logger(), "无人机解锁失败，woc了，返回码: %d", result->result);
            apply_arm_flag_ = false;
        }
    } catch (const std::exception& e) {
        RCLCPP_ERROR(get_logger(), "服务调用失败: %s", e.what());
        apply_arm_flag_ = false;
    }

}
void BaseController::launch_callback(const std_msgs::msg::Bool::ConstSharedPtr& future)
{
    if(future->data) launch_flag_ = true;
    else launch_flag_ = false;
}

void BaseController::task_callback(const std_msgs::msg::Int32::ConstSharedPtr& msg)//信息填在回调函数里面，然后才可以去接受信息，这些订阅者的回调函数的参数就是对应的信息数据类型，他会把信息传入这个参数里面，你要接受这个信息
{
    int new_task_id = msg->data;//看msg里面拿到信息

    // 只有当任务ID有效且发生变化时才处理
    if (new_task_id == 1 || new_task_id == 2) {
        if (new_task_id != current_task_id_) {
            RCLCPP_INFO(get_logger(), "接收到任务指令: %d（任务已切换）", new_task_id);
            current_task_id_ = new_task_id;
        } else {
            // 数据没有变化，不处理
            RCLCPP_DEBUG(get_logger(), "接收到任务指令: %d（与当前任务相同，未切换）", new_task_id);
        }
    } else {
        RCLCPP_WARN(get_logger(), "无效的任务ID: %d", new_task_id);
    }

}

void BaseController::engage_offboard_mode() const
{
    if (!set_mode_client_->wait_for_service(std::chrono::seconds(1))) {
        RCLCPP_INFO(get_logger(), "设置模式服务不可用，等待中...");
        return;
    }
    
    auto request = std::make_shared<mavros_msgs::srv::SetMode::Request>();
    request->custom_mode = "OFFBOARD";
    
    auto result_future = set_mode_client_->async_send_request(
        request,
        [this](std::shared_future<std::shared_ptr<mavros_msgs::srv::SetMode::Response>> future){
            set_mode_callback(future);
        });
    apply_offboard_flag_=true;
}

void BaseController::set_mode_callback(std::shared_future<std::shared_ptr<mavros_msgs::srv::SetMode::Response>> future) const
{
    try {
        auto result = future.get();
        
        if (result->mode_sent) {
            RCLCPP_INFO(get_logger(), "已改变模式，当前模式：%s", current_state_.mode.c_str());
        } else {
            RCLCPP_ERROR(get_logger(), "改变模式失败，当前模式：%s", current_state_.mode.c_str());
            apply_offboard_flag_=false;
        }
    } catch (const std::exception& e) {
        RCLCPP_ERROR(get_logger(), "服务调用失败: %s", e.what());
        apply_offboard_flag_=false;
    }
    
}

void BaseController::land()//怎么有两个函数？一个被注释了？
{
    if(current_state_.armed==true)
    {
        publish_velocity_body(0.0, 0.0, -0.3, 0.0);
    }
    else
    {
        publish_velocity_body(0.0, 0.0, 0.0, 0.0);
        apply_disarm_flag_ = false;
        return;
    }

    double altitude = local_position_.pose.position.z;
//    RCLCPP_INFO(get_logger(), "当前高度: %.3f", altitude);
    if (altitude < 0.03) 
    {
        
        if (arming_client_->service_is_ready()) 
        {
            if (apply_disarm_flag_ == true) return;   // 标志位防重复
            apply_disarm_flag_ = true;                // 上锁请求已发出
            auto request = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
            request->value = false;                   // false = disarm
            arming_client_->async_send_request(
                request,
                [this](std::shared_future<std::shared_ptr<mavros_msgs::srv::CommandBool::Response>> future) 
                {
                    try 
                    {
                        auto result = future.get();
                        if (result->success)
                        {
                            RCLCPP_INFO(get_logger(), "无人机已上锁（通过 CommandBool）");
                            //庙在妙哉！成功之后不重置，因为成功了，那么就保持true状态，这样就算状态没有及时更新，也没事，不会再次申请，这样子就消除了延迟的影响
                        } 
                        else 
                        {
                            RCLCPP_ERROR(get_logger(), "无人机上锁失败（CommandBool），飞控拒绝");
                            apply_disarm_flag_ = false;
                        }
                    } 
                    catch (const std::exception& e) 
                    {
                        RCLCPP_ERROR(get_logger(), "上锁服务调用失败: %s", e.what());
                        apply_disarm_flag_ = false;
                    }
                    
                    
                }
            );
        } 
        else 
        {
            RCLCPP_WARN(get_logger(), "上锁服务不可用（CommandBool）");
        }
    }
}
// the version below seems only decrease the  threshold，我去，而且还用了commandbool
// void BaseController::land()
// {
//     publish_velocity_body(0.0, 0.0, -0.3, 0.0);

//     double altitude = local_position_.pose.position.z;
// //    RCLCPP_INFO(get_logger(), "当前高度: %.3f", altitude);

//     if (altitude < 0.1) {
//         if (arming_client_->wait_for_service(std::chrono::seconds(1))) {
//             auto request = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
//             request->value = false;  // false表示上锁

//             arming_client_->async_send_request(
//                 request,
//                 [this](rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedFuture future) {
//                     try {
//                         auto result = future.get();
//                         if (result->success) {
//                             RCLCPP_INFO(get_logger(), "无人机已上锁");
//                         } else {
//                             RCLCPP_ERROR(get_logger(), "无人机上锁失败，飞控拒绝");
//                         }
//                     } catch (const std::exception& e) {
//                         RCLCPP_ERROR(get_logger(), "上锁服务调用失败: %s", e.what());
//                     }
//                 });
//         } else {
//             RCLCPP_WARN(get_logger(), "上锁服务不可用");
//         }
//     }
// }

void BaseController::auto_land() {
    if (!set_mode_client_->wait_for_service(std::chrono::seconds(1))) {
        RCLCPP_ERROR(get_logger(), "设置模式服务不可用，无法进入自动降落模式");
        return;
    }
    
    auto request = std::make_shared<mavros_msgs::srv::SetMode::Request>();
    request->custom_mode = "AUTO.LAND";
    
    auto result_future = set_mode_client_->async_send_request(
        request,
        [this](std::shared_future<std::shared_ptr<mavros_msgs::srv::SetMode::Response>> future){
            try {
                auto result = future.get();
                if (result->mode_sent) {
                    RCLCPP_INFO(get_logger(), "已进入自动降落模式");
                } else {
                    RCLCPP_ERROR(get_logger(), "进入自动降落模式失败");
                }
            } catch (const std::exception& e) {
                RCLCPP_ERROR(get_logger(), "自动降落服务调用失败: %s", e.what());
            }
        });
}

void BaseController::task_state_pub() const
{
    msg_tool::msg::FlightInfo msg;
    msg.state = flightStateToString(flight_state_);
    msg.task_id = current_task_id_;
    task_pub_->publish(msg);
}


void BaseController::publish_position_setpoint(const double x, const double y, const double z, const double yaw)
{
    geometry_msgs::msg::PoseStamped msg;//为什么着某写，而不：geometry_msgs::msg::PoseStamped msg；
    msg.header.stamp = this->get_clock()->now();
    msg.header.frame_id = "map";

    msg.pose.position.x = x;
    msg.pose.position.y = y;
    msg.pose.position.z = z;

    tf2::Quaternion q;//这是欧拉角转四元数了
    q.setRPY(0, 0, yaw);
    msg.pose.orientation.x = q.x();
    msg.pose.orientation.y = q.y();
    msg.pose.orientation.z = q.z();
    msg.pose.orientation.w = q.w();

    trajectory_setpoint_pub_->publish(msg);

    RCLCPP_DEBUG(get_logger(), "发布位置设定点: [%.2f, %.2f, %.2f, %.2f]", x, y, z, yaw);
}
//这怎么也有两个，因为这个用不着了
// void BaseController::publish_position_setpoint(const double x, const double y, const double z, const double yaw)
// {
//     const auto& pos = local_position_.pose.position;
//     double err_x = x - pos.x;
//     double err_y = y - pos.y;
//     double err_z = z - pos.z;

//     double yaw_curr = current_yaw_;

//     // 将误差从map系转换到body系
//     double err_body_x = std::cos(yaw_curr) * err_x + std::sin(yaw_curr) * err_y;
//     double err_body_y = -std::sin(yaw_curr) * err_x + std::cos(yaw_curr) * err_y;
//     double err_body_z = err_z;

//     // 时间间隔
//     rclcpp::Time now = this->get_clock()->now();
//     double dt = (now - last_time_).seconds();
//     if (dt < 1e-6) dt = 1e-6;  // 防止除零

//     // PID参数
//     double kP = 5.0;
//     double kI = 0;//0.2
//     double kD = 0.8;
//     double i_limit = 1.0;  // 积分限幅，防止过大

//     // ----------- 积分项累加 ------------
//     integral_err_body_x_ += err_body_x * dt;
//     integral_err_body_y_ += err_body_y * dt;
//     integral_err_body_z_ += err_body_z * dt;

//     // 积分限幅
//     integral_err_body_x_ = std::clamp(integral_err_body_x_, -i_limit, i_limit);
//     integral_err_body_y_ = std::clamp(integral_err_body_y_, -i_limit, i_limit);
//     integral_err_body_z_ = std::clamp(integral_err_body_z_, -i_limit, i_limit);

//     // ----------- PID计算 ------------
//     double vx = kP * err_body_x
//               + kI * integral_err_body_x_
//               + kD * (err_body_x - last_err_body_x_) / dt;

//     double vy = kP * err_body_y
//               + kI * integral_err_body_y_
//               + kD * (err_body_y - last_err_body_y_) / dt;

//     double vz =  err_body_z;

//     // 更新上一次误差
//     last_err_body_x_ = err_body_x;
//     last_err_body_y_ = err_body_y;
//     last_err_body_z_ = err_body_z;
//     last_time_ = now;

//     // yaw 还是 PD 就够了（I项容易导致累积漂移）
//     double dyaw = yaw - yaw_curr;
//     while (dyaw > M_PI) dyaw -= 2 * M_PI;
//     while (dyaw < -M_PI) dyaw += 2 * M_PI;
//     double vyaw = kP_yaw_ * dyaw;  

//     publish_velocity_body(vx, vy, vz, vyaw);

//     RCLCPP_DEBUG(get_logger(),
//                  "PID控制 -> body误差: [%.2f, %.2f, %.2f], 积分: [%.2f, %.2f, %.2f], 发布速度: [%.2f, %.2f, %.2f, %.2f]",
//                  err_body_x, err_body_y, err_body_z,
//                  integral_err_body_x_, integral_err_body_y_, integral_err_body_z_,
//                  vx, vy, vz, vyaw);
// }


void BaseController::publish_position_setpoint_trajectory(//这函数怎么多出来一个？
    const double start_x, const double start_y, const double start_z,
    const double end_x, const double end_y, const double end_z,
    const double target_yaw) 
{
    const auto& pos = local_position_.pose.position;

    // 航线参数
    TrajectorySegment trajectory;
    trajectory.start_x = start_x;
    trajectory.start_y = start_y;
    trajectory.start_z = start_z;
    trajectory.end_x = end_x;
    trajectory.end_y = end_y;
    trajectory.end_z = end_z;

    double dx = end_x - start_x;
    double dy = end_y - start_y;
    double dz = end_z - start_z;
    trajectory.length = std::sqrt(dx*dx + dy*dy + dz*dz);

    if (trajectory.length < 1e-6) {
        publish_position_setpoint(end_x, end_y, end_z, target_yaw);
        return;
    }

    // 单位方向
    trajectory.direction_x = dx / trajectory.length;
    trajectory.direction_y = dy / trajectory.length;
    trajectory.direction_z = dz / trajectory.length;

    // 当前点相对起点
    double curr_rel_x = pos.x - start_x;
    double curr_rel_y = pos.y - start_y;
    double curr_rel_z = pos.z - start_z;

    // 沿线距离
    double along_track_distance = curr_rel_x * trajectory.direction_x + 
                                  curr_rel_y * trajectory.direction_y + 
                                  curr_rel_z * trajectory.direction_z;
    along_track_distance = std::clamp(along_track_distance, 0.0, trajectory.length);

    // 航线上最近点
    double closest_x = start_x + along_track_distance * trajectory.direction_x;
    double closest_y = start_y + along_track_distance * trajectory.direction_y;
    double closest_z = start_z + along_track_distance * trajectory.direction_z;

    // cross track 误差
    double cross_track_x = pos.x - closest_x;
    double cross_track_y = pos.y - closest_y;
    double cross_track_z = pos.z - closest_z;

    // along track 误差
    double along_track_error = trajectory.length - along_track_distance;

    double yaw_curr = current_yaw_;

    // 转换到机体系
    double cross_err_body_x = std::cos(yaw_curr) * cross_track_x + std::sin(yaw_curr) * cross_track_y;
    double cross_err_body_y = -std::sin(yaw_curr) * cross_track_x + std::cos(yaw_curr) * cross_track_y;

    double along_err_body_x = std::cos(yaw_curr) * (trajectory.direction_x * along_track_error) + 
                              std::sin(yaw_curr) * (trajectory.direction_y * along_track_error);
    double along_err_body_y = -std::sin(yaw_curr) * (trajectory.direction_x * along_track_error) + 
                              std::cos(yaw_curr) * (trajectory.direction_y * along_track_error);

    // PID参数
    double kP_cross_track = 2;
    double kD_cross_track = 0.8;   // 新增 D 项
    double kP_along_track = 1.0;
    double kP_altitude   = 1.0;

    // 时间差
    rclcpp::Time now = this->get_clock()->now();
    double dt = (now - last_cross_time_).seconds();
    if (dt < 1e-3) dt = 1e-3;  // 避免除零

    // cross track 误差变化率
    double d_err_x = (cross_err_body_x - last_cross_err_x_) / dt;
    double d_err_y = (cross_err_body_y - last_cross_err_y_) / dt;

    // 更新缓存
    last_cross_err_x_ = cross_err_body_x;
    last_cross_err_y_ = cross_err_body_y;
    last_cross_time_ = now;

    // 控制量
    double vx_cross = -(kP_cross_track * cross_err_body_x + kD_cross_track * d_err_x);
    double vy_cross = -(kP_cross_track * cross_err_body_y + kD_cross_track * d_err_y);

    double vx_along = kP_along_track * along_err_body_x;
    double vy_along = kP_along_track * along_err_body_y;

    double vx = vx_cross + vx_along;
    double vy = vy_cross + vy_along;
    double vz = -kP_altitude * cross_track_z;

    // yaw 控制
    double dyaw = target_yaw - yaw_curr;
    while (dyaw > M_PI) dyaw -= 2 * M_PI;
    while (dyaw < -M_PI) dyaw += 2 * M_PI;
    double vyaw = kP_yaw_ * dyaw;

    publish_velocity_body(vx, vy, vz, vyaw);

    RCLCPP_DEBUG(get_logger(), 
        "航线控制PD -> 沿线: %.2f/%.2f, cross误差: [%.2f, %.2f, %.2f], "
        "沿线误差: %.2f, d_err: [%.2f, %.2f], 速度: [%.2f, %.2f, %.2f, %.2f]",
        along_track_distance, trajectory.length,
        cross_track_x, cross_track_y, cross_track_z,
        along_track_error,
        d_err_x, d_err_y,
        vx, vy, vz, vyaw);
}



void BaseController::publish_velocity_body(double v_x, double v_y, double v_z, double v_yaw)
{
    // // 先限制线速度
    // const double linear_velocity = std::sqrt(v_x * v_x + v_y * v_y + v_z * v_z);
    // if (linear_velocity > max_linear_velocity_) {
    //     double scale = max_linear_velocity_ / linear_velocity;
    //     v_x *= scale;
    //     v_y *= scale;
    //     v_z *= scale;
    // }

    // 再限制角速度
    v_yaw = std::clamp(v_yaw, -max_angular_velocity_, max_angular_velocity_);

    geometry_msgs::msg::TwistStamped msg;
    msg.header.stamp = this->get_clock()->now();
    msg.header.frame_id = "map";

    msg.twist.linear.x = v_x;
    msg.twist.linear.y = v_y;
    msg.twist.linear.z = v_z;

    msg.twist.angular.x = 0.0;
    msg.twist.angular.y = 0.0;
    msg.twist.angular.z = v_yaw;

    velocity_setpoint_pub_->publish(msg);

    RCLCPP_DEBUG(get_logger(),
        "机体速度(vx=%.2f, vy=%.2f) ,vz=%.2f, yaw=%.2f",
        v_x, v_y, v_z, v_yaw);
}

void BaseController::rotation(const int reverse)
{
    // 保持当前位置不变，只改变 yaw
    publish_velocity_body(0 , 0, 0, 0.5*reverse);
}

double BaseController::distance_to_target(const double x,const double y,const double z) const
{
    const double dx = local_position_.pose.position.x - x;
    const double dy = local_position_.pose.position.y - y;
    const double dz = local_position_.pose.position.z - z;
    return std::sqrt(dx*dx + dy*dy + dz*dz);
}

std::vector<double> BaseController::get_euler_from_pose(const geometry_msgs::msg::Pose& pose)
{
    tf2::Quaternion q(
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z,
        pose.orientation.w);
    
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    
    return {roll, pitch, yaw};
}

void BaseController::vehicle_state_callback(const mavros_msgs::msg::State::ConstSharedPtr& msg)
{
    current_state_ = *msg;
}

void BaseController::local_position_callback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr& msg)
{
    local_position_ = *msg;
    
    // 更新当前姿态
    auto euler = get_euler_from_pose(msg->pose);
    current_roll_ = euler[0];
    current_pitch_ = euler[1];
    current_yaw_ = euler[2];
}

bool BaseController::start_hover(const HoverConfig& config)//状态机编程
{
    if (hover_state_ != HoverState::IDLE) {
        RCLCPP_WARN(get_logger(), "Hover already in progress, stopping current hover");
        stop_hover();
    }
    
    hover_config_ = config;
    
    // 如果使用当前位置作为悬停点
    if (hover_config_.use_current_position) {
        hover_config_.x = local_position_.pose.position.x;
        hover_config_.y = local_position_.pose.position.y;
        hover_config_.z = local_position_.pose.position.z;
        hover_config_.yaw = current_yaw_;
    }
    
    // 检查是否已经在目标位置
    if (check_hover_position_reached()) {
        hover_state_ = HoverState::HOVERING;
        hover_start_time_ = this->get_clock()->now();
        RCLCPP_INFO(get_logger(), "Started hovering at position [%.2f, %.2f, %.2f, %.2f]", 
                   hover_config_.x, hover_config_.y, hover_config_.z, hover_config_.yaw);
    } else {
        hover_state_ = HoverState::POSITIONING;
        RCLCPP_INFO(get_logger(), "Moving to hover position [%.2f, %.2f, %.2f, %.2f]", 
                   hover_config_.x, hover_config_.y, hover_config_.z, hover_config_.yaw);
    }
    
    return true;
}

// 简化版本：在当前位置悬停指定时间
bool BaseController::start_hover(const double duration)
{
    HoverConfig config;
    config.use_current_position = true;
    config.duration = duration;
    return start_hover(config);
}

// 在指定位置悬停
bool BaseController::start_hover_at_position(const double x,const double y,const double z,const double yaw,const double duration)
{
    HoverConfig config;
    config.x = x;
    config.y = y;
    config.z = z;
    config.yaw = yaw;
    config.duration = duration;
    config.use_current_position = false;
    return start_hover(config);
}

// 条件悬停（直到满足退出条件）
bool BaseController::start_hover_with_condition(const std::function<bool()>& exit_condition)
{
    HoverConfig config;
    config.use_current_position = true;
    config.duration = -1.0;  // 无限时间
    config.exit_condition = exit_condition;
    return start_hover(config);
}

// 停止悬停
void BaseController::stop_hover()
{
    if (hover_state_ != HoverState::IDLE) {
        RCLCPP_INFO(get_logger(), "Stopping hover");
        reset_hover_state();
    }
}

// 更新悬停状态（在timer_callback中调用）
bool BaseController::update_hover()
{
    switch (hover_state_) {
        case HoverState::IDLE:
            return true;  // 没有悬停任务
            
        case HoverState::POSITIONING:
            // 移动到悬停位置
            publish_position_setpoint(hover_config_.x, hover_config_.y, 
                                    hover_config_.z, hover_config_.yaw);
            
            // 检查是否到达位置
            if (check_hover_position_reached()) {
                hover_state_ = HoverState::HOVERING;
                hover_start_time_ = this->get_clock()->now();
                RCLCPP_INFO(get_logger(), "Reached hover position, starting hover");
            }
            return false;
            
        case HoverState::HOVERING:
            // 保持在悬停位置（使用固定的坐标，避免累积误差）
            publish_position_setpoint(hover_config_.x, hover_config_.y, 
                                    hover_config_.z, hover_config_.yaw);
            
            // 检查退出条件
            if (check_hover_exit_conditions()) {
                hover_state_ = HoverState::COMPLETED;
                RCLCPP_INFO(get_logger(), "Hover completed");
                return true;
            }
            return false;
            
        case HoverState::COMPLETED:
            reset_hover_state();
            return true;
    }
    
    return false;
}

// 私有辅助方法
void BaseController::reset_hover_state()
{
    hover_state_ = HoverState::IDLE;
    hover_config_ = HoverConfig();  // 重置配置
}

bool BaseController::check_hover_position_reached()
{
    // 检查位置容差
    double dx = local_position_.pose.position.x - hover_config_.x;
    double dy = local_position_.pose.position.y - hover_config_.y;
    double dz = local_position_.pose.position.z - hover_config_.z;
    double position_error = std::sqrt(dx*dx + dy*dy + dz*dz);
    
    // 检查偏航角容差
    double dyaw = current_yaw_ - hover_config_.yaw;
    if (dyaw > M_PI) dyaw -= 2 * M_PI;
    if (dyaw < -M_PI) dyaw += 2 * M_PI;
    double yaw_error = std::abs(dyaw);
    
    return (position_error < hover_config_.position_tolerance) && 
           (yaw_error < hover_config_.yaw_tolerance);
}

bool BaseController::check_hover_exit_conditions()
{
    // 检查时间条件
    if (hover_config_.duration > 0.0) {
        auto elapsed_time = (this->get_clock()->now() - hover_start_time_).seconds();
        if (elapsed_time >= hover_config_.duration) {
            return true;
        }
    }
    
    // 检查自定义退出条件
    if (hover_config_.exit_condition && hover_config_.exit_condition()) {
        return true;
    }
    
    return false;
}


//工具

double BaseController::shortest_angular_distance(const double from,const double to)
{
    double diff = to - from;
    while (diff > M_PI)  diff -= 2 * M_PI;
    while (diff < -M_PI) diff += 2 * M_PI;
    return diff;
}


std::string BaseController::flightStateToString(const FlightState& state) {
    switch (state) {
    case FlightState::INIT: return "INIT";
    case FlightState::TAKEOFF: return "TAKEOFF";
    case FlightState::WAYPOINT: return "WAYPOINT";
    case FlightState::TILTLAND: return "TITLELAND";
    case FlightState::LAND: return "LAND";
    default: return "UNKNOWN";
    }
}


} // namespace offboard