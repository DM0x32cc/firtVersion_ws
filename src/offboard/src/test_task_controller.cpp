//cpp里面似乎是传指针的给spin的
// 这是主流程，我们先打四个点，然后飞到每个点上面时，让他悬停个2.5秒再转一圈。
#include "offboard/test_task_controller.hpp"
#include <Eigen/Core>
namespace offboard
{

TaskController::TaskController() : BaseController("offb_node") /*, path_planner_(nullptr), obstacles_received_(false), path_planned_(false)*/
{
    
    this->declare_parameter("takeoff_height", 1.0);
    this->declare_parameter("waypoint_threshold", 0.1);//到点阈值
    HoverConfig wp_config_;

    takeoff_height_=this->get_parameter("takeoff_height").as_double();
    waypoint_threshold_=this->get_parameter("waypoint_threshold").as_double();
    // 初始化位置停留保护机制变量
    last_position_time_ = this->get_clock()->now();
    last_position_x_ = 0.0;
    last_position_y_ = 0.0;
    last_position_z_ = 0.0;
    position_stuck_detected_ = false;
    ignoring_targets_ = false;

    waypoint_generate();
    RCLCPP_INFO(get_logger(), "Task controller initialized");
}


void TaskController::timer_callback()
{
    // 无论什么状态，都先发 setpoint（维持 offboard 连接）
    // 这一部先来检查，我们要持续发布5s以上的稳定信息
    if (!setpoint_ready_) 
    {
        publish_position_setpoint(0.0, 0.0, 0.0, 0.0);
        offboard_setpoint_counter_++;
        if (offboard_setpoint_counter_ > 50) 
        {
            setpoint_ready_ = true;
        }
        return;
    }
    // 
    check_task_switch_conditions();//这个应该是可以用于辅助switch flight state
    
    task_state_pub();// 发布消息，这波应该由地面站来接收

    switch (flight_state_) 
    {
        case FlightState::INIT://这里后续设计路径规划时，可以增加下waypoints_是否生成完毕,让相关函数传一个标识。
            publish_position_setpoint(//就一直先发着，确保丝滑切换
                local_position_.pose.position.x,
                local_position_.pose.position.y,
                local_position_.pose.position.z,
                0.0);
            break;

        case FlightState::TAKEOFF:
            publish_position_setpoint(0,0,takeoff_height_,0);//这种消息就要不停的发送
            break;

        case FlightState::WAYPOINT:
            execute_waypoint_mission();
            break;
        // case FlightState::APPROACH://不需要这个状态
            
        case FlightState::TILTLAND:
            if(tilt_land()) 
            switch_task(FlightState::LAND);
            break;

        case FlightState::LAND:
            land();
            break;
        default:
            RCLCPP_INFO(get_logger(), "我去！未知飞行状态？！你干哪里来了？");
    }

}

bool TaskController::check_task_switch_conditions()//return true代表着是切换state了,没有切换时一律return false（默认也是如此）
{
    if(check_emergency_condition())
    {
        RCLCPP_INFO(get_logger(), "检测到紧急情况，自动降落");
        auto_land();//紧急时才让飞控自己来，不然都用land()
        return true;
    }
    if(launch_flag_==false && flight_state_ != FlightState::INIT)
    {
        RCLCPP_INFO(get_logger(), "飞行途中收到launch_flag_ = false，自动降落");
        auto_land();
        return true;
    }
    switch(flight_state_)
    {
        case FlightState::INIT:
            if(launch_flag_ == false ) break;
            if( current_state_.mode!="OFFBOARD")
            {
                if(apply_offboard_flag_ == true) break;
                engage_offboard_mode();
                RCLCPP_INFO(get_logger(), "申请进入OFFBOARD模式");
                return false;
            }
            if(current_state_.armed==false)
            {
                if(apply_arm_flag_ == true) break;
                arm();//这个如果解锁不成功会不断重试的，所以无需加上if判断
                return false;
            }
            else
            {
                switch_task(FlightState::TAKEOFF);
                return true;
            }
            break;

        case FlightState::TAKEOFF:
            if(std::abs(local_position_.pose.position.z-takeoff_height_)<=waypoint_threshold_)
            {
                switch_task(FlightState::WAYPOINT);
                return true;
            }   
            break;

        case FlightState::WAYPOINT:
            if(current_waypoint_index_ >= waypoints_.size())
            {
                switch_task(FlightState::TILTLAND);
                return true;
            }
            break;

        case FlightState::TILTLAND:
            break;
        default:
            break;
    }
    
    return false;
}

bool TaskController::waypoint_generate()
{
    int rows=3;
    int cols=4;
    waypoints_.resize(rows);
    waypoints_[0]={0.0,1.0,takeoff_height_,0.0};
    waypoints_[1]={1.0,-1.0,takeoff_height_,0.0};//这yaw角范围是【-pi，pi】，他单位是弧度不太好算，就一直保持0.0吧
    waypoints_[2]={0.0,-1.0,takeoff_height_,0.0};//这yaw角范围是【-pi，pi】，他单位是弧度不太好算，就一直保持0.0吧
    // for(auto& row : waypoints_)
    // {
    //     row.resize(cols);
    // }
    return true;
}
void TaskController::switch_task(FlightState new_state)
{
    if (new_state == flight_state_) {
        return;
    }
    RCLCPP_INFO(get_logger(), "切换任务: 从 %d 到 %d",
                static_cast<int>(flight_state_),
                static_cast<int>(new_state));
    flight_state_ = new_state;
}


bool TaskController::check_emergency_condition()
{
    if(std::abs(local_position_.pose.position.x) >= 7 ||
        std::abs(local_position_.pose.position.y) >= 7 ||
        std::abs(local_position_.pose.position.z) >= 2.5){
        RCLCPP_INFO(get_logger(), "检测到紧急情况，位置超出范围，自动降落");
        return true;
    }
    return false;
}


void TaskController::execute_waypoint_mission()//学习一下思路，我们只需在开始之前把航点计算出来就行了，
{/*为什么不让飞机自己飞，而是说我控制他去飞呢？*/
    if (current_waypoint_index_ >= waypoints_.size()) {//检查是否遍历完全
        return;
    }
    
    const auto& current_waypoint = waypoints_[current_waypoint_index_];//引用？
    
    // 检查位置停留保护机制
    check_position_stuck_protection();//这对于我们目前单纯航点飞行没有用，因为这绝对不会被卡住的
    
    // // 如果当前有目标并且未处理，且不在忽略目标状态，则优先处理目标
    // if (target_msg_.detected && !ignoring_targets_) {
    //     approach();
    //     return;  // 这return确保一直在approach这里，不执行下面的航点任务，approach()里面在一直发送消息，不用害怕，
    // }//我们暂时用不着
    
    // 确定航线的起点
    double start_x, start_y, start_z;
    
    if (current_waypoint_index_ == 0) {
        // 第一个航点，使用当前位置作为起点
        start_x = 0;
        start_y = 0;
        start_z = takeoff_height_;
    } else {
        // 使用上一个航点作为起点
        const auto& previous_waypoint = waypoints_[current_waypoint_index_ - 1];
        start_x = previous_waypoint[0];
        start_y = previous_waypoint[1];
        start_z = previous_waypoint[2];
    }
    
    // 当前航点作为终点
    double end_x = current_waypoint[0];
    double end_y = current_waypoint[1];
    double end_z = current_waypoint[2];
    double target_yaw = current_waypoint[3];
    
    // 使用基于航线的位置控制
    publish_position_setpoint_trajectory(start_x, start_y, start_z,
                                        end_x, end_y, end_z,
                                        target_yaw);//这里面会不停的发送信息的
    
    // 检查是否到达当前航点
    if (is_at_point(current_waypoint[0], current_waypoint[1], current_waypoint[2])) 
    {
        RCLCPP_INFO(get_logger(), "到达航点 %zu: [%.2f, %.2f, %.2f]",
                    current_waypoint_index_, current_waypoint[0], current_waypoint[1], current_waypoint[2]);
        current_waypoint_index_++;
        

        // 重置位置停留检测
        reset_position_stuck_detection();
    }
}

void TaskController::check_position_stuck_protection()//这个是不断检测你行动的代码，忽略的作用是防止你被一个目标吸引后粘住太久，于是主动忽略一秒的作用是使你脱离了approach(),然后你就放弃这个识别目标向前飞了
{                                                       /*你可以注意，执行航点时他想要approach(),必须没有在忽略状态*/
    double current_x = local_position_.pose.position.x;
    double current_y = local_position_.pose.position.y;
    double current_z = local_position_.pose.position.z;
    
    rclcpp::Time current_time = this->get_clock()->now();
    
    // 检查是否在同一位置（使用waypoint_threshold_作为阈值）
    double position_change = std::sqrt(
        std::pow(current_x - last_position_x_, 2) + 
        std::pow(current_y - last_position_y_, 2) + 
        std::pow(current_z - last_position_z_, 2)
    );
    
    if (position_change > waypoint_threshold_) {
        // 位置有显著变化，重置计时器
        last_position_time_ = current_time;
        last_position_x_ = current_x;
        last_position_y_ = current_y;
        last_position_z_ = current_z;
        
        // 如果之前检测到卡住状态，现在位置有变化了，重置状态
        if (position_stuck_detected_) {
            position_stuck_detected_ = false;
            RCLCPP_INFO(get_logger(), "位置恢复移动，重置卡住检测状态");
        }
    } else {
        // 位置变化很小，检查停留时间
        double stuck_duration = (current_time - last_position_time_).seconds();
        
        if (!position_stuck_detected_ && stuck_duration >= 8.0) {
            // 检测到在同一位置停留超过8秒
            position_stuck_detected_ = true;
            stuck_detection_time_ = current_time;
            ignore_target_until_ = current_time + rclcpp::Duration::from_nanoseconds(1000000000LL); // 1秒后
            ignoring_targets_ = true;
            
            RCLCPP_INFO(get_logger(), 
                       "检测到在位置 [%.2f, %.2f, %.2f] 停留超过8秒，开始忽略目标数据1秒",
                       current_x, current_y, current_z);
        }
    }
    
    // 检查是否应该停止忽略目标，为什么忽略
    if (ignoring_targets_ && current_time >= ignore_target_until_) {
        ignoring_targets_ = false;
        RCLCPP_INFO(get_logger(), "停止忽略目标数据");
    }
    
    // 打印调试信息（可选）
    if (position_stuck_detected_) {
        double total_stuck_time = (current_time - stuck_detection_time_).seconds();
        double remaining_ignore_time = std::max(0.0, (ignore_target_until_ - current_time).seconds());
        
        RCLCPP_DEBUG(get_logger(), 
                    "位置停留保护状态 - 卡住时间: %.1fs, 剩余忽略时间: %.1fs, 忽略状态: %s",
                    total_stuck_time, remaining_ignore_time, ignoring_targets_ ? "是" : "否");
    }
}

bool TaskController::is_at_point(const double x, const double y, const double z) const
{
    return distance_to_target(x, y, z) < waypoint_threshold_;
}


void TaskController::reset_position_stuck_detection()
{
    position_stuck_detected_ = false;
    ignoring_targets_ = false;
    last_position_time_ = this->get_clock()->now();//这是节点自带的一个获取时间的方法
    last_position_x_ = local_position_.pose.position.x;
    last_position_y_ = local_position_.pose.position.y;
    last_position_z_ = local_position_.pose.position.z;
}

bool TaskController::tilt_land()
{
    // 静态变量记录降落状态
    static bool stage1_completed = false;  // 第一阶段是否完成
    static bool stage1_started = false;    // 第一阶段是否已开始
    
    // 目标降落位置 (0, 0, 0)
    double target_x = 0.0;
    double target_y = 0.0;
    double target_z = 0.0;  // 地面高度
    
    // 获取当前位置
    double current_x = local_position_.pose.position.x;
    double current_y = local_position_.pose.position.y;
    double current_z = local_position_.pose.position.z;
    
    // 计算到目标点(0,0)的水平距离
    double horizontal_distance = sqrt(pow(current_x - target_x, 2) + pow(current_y - target_y, 2));
    
    // 检查是否已经降落完成
    if (current_z < 0.08) {
        RCLCPP_INFO(get_logger(), "降落完成！");
        // 重置状态变量以便下次使用
        stage1_completed = false;
        stage1_started = false;
        return true;
    }
    
    // 45度角降落的理想水平距离应该等于当前高度
    double ideal_horizontal_distance = current_z;  // 45度角条件
    
    // 第一阶段：飞到45度线起点（只执行一次）
    if (!stage1_completed) {
        double position_tolerance = 0.05;  // 位置容差
        
        // 如果还没开始第一阶段，或者距离45度线太远，继续第一阶段
        if (!stage1_started || std::abs(horizontal_distance - ideal_horizontal_distance) > position_tolerance) {
            stage1_started = true;
            
            // 计算45度线起点位置
            double start_point_distance = current_z;  // 45度线起点距离原点的距离等于当前高度
            
            // 计算移动方向
            double move_direction_x, move_direction_y;
            if (horizontal_distance > 0.1) {
                // 沿着当前到原点的方向
                move_direction_x = (target_x - current_x) / horizontal_distance;
                move_direction_y = (target_y - current_y) / horizontal_distance;
            } else {
                // 如果已经在原点正上方，选择一个默认方向
                move_direction_x = 1.0;
                move_direction_y = 0.0;
            }
            
            // 计算45度线起点的目标位置
            double target_start_x = target_x - move_direction_x * start_point_distance;
            double target_start_y = target_y - move_direction_y * start_point_distance;
            
            // 移动到45度线起点（保持当前高度）
            publish_position_setpoint(target_start_x, target_start_y, current_z, 0.0);
            
            RCLCPP_INFO(get_logger(), "第一阶段：移动到45度线起点 [%.2f, %.2f, %.2f] (距离原点: %.2f)",
                        target_start_x, target_start_y, current_z, start_point_distance);
            return false;
        } else {
            // 到达45度线起点，标记第一阶段完成
            stage1_completed = true;
            RCLCPP_INFO(get_logger(), "第一阶段完成！开始准备45度角降落");
        }
    }
    
    
    // 第二阶段：沿45度线降落
    RCLCPP_INFO(get_logger(), "第二阶段：开始45度角降落");
    
    // 计算位置偏差：当前水平距离与理想水平距离的差值
    double horizontal_error = horizontal_distance - ideal_horizontal_distance;
    
    // 固定的垂直下降速度
    double descent_speed = 0.25;  // m/s
    
    // ESP32信号发布，这为什么要esp32在这？不过，我们发消息而已不用管！
    std_msgs::msg::Int32 msg;
    msg.data = 1;
    // esp32_pub_->publish(msg);//我曹了这不知道是什么
    
    // 水平速度控制参数
    double kp_horizontal = 2;  // 比例增益，可根据实际情况调整
    double max_horizontal_speed = 1.0;  // 最大水平速度限制
    
    // 根据水平位置偏差计算水平速度
    double horizontal_speed = 0.0;
    double vel_x_body = 0.0;
    double vel_y_body = 0.0;
    
    if (horizontal_distance > 0.05) {
        // 计算期望的水平速度大小（基于位置偏差）
        horizontal_speed = kp_horizontal * std::abs(horizontal_error);
        
        // 限制最大水平速度
        horizontal_speed = std::min(horizontal_speed, max_horizontal_speed);
        
        // 计算移动方向
        double direction_x = (target_x - current_x) / horizontal_distance;
        double direction_y = (target_y - current_y) / horizontal_distance;
        
        // 如果当前距离大于理想距离，向原点移动（负偏差时也向原点移动）
        // 如果当前距离小于理想距离，远离原点移动
        if (horizontal_error > 0) {
            // 当前距离大于理想距离，需要向原点移动
            direction_x = (target_x - current_x) / horizontal_distance;
            direction_y = (target_y - current_y) / horizontal_distance;
        } else {
            // 当前距离小于理想距离，需要远离原点
            direction_x = (current_x - target_x) / horizontal_distance;
            direction_y = (current_y - target_y) / horizontal_distance;
        }
        
        // 计算世界坐标系下的水平速度
        double vel_x_world = direction_x * horizontal_speed;
        double vel_y_world = direction_y * horizontal_speed;
        
        // 转换到机体坐标系
        vel_x_body = vel_x_world * cos(-current_yaw_) - vel_y_world * sin(-current_yaw_);
        vel_y_body = vel_x_world * sin(-current_yaw_) + vel_y_world * cos(-current_yaw_);
    }
    
    // 发布速度指令（垂直速度始终向下）
    publish_velocity_body(vel_x_body, vel_y_body, -descent_speed, 0.0);
    
    RCLCPP_INFO(get_logger(), 
                "45度角降落控制: 位置[%.2f, %.2f, %.2f] "
                "水平距离: %.2f, 理想距离: %.2f, 偏差: %.2f "
                "速度[%.2f, %.2f, %.2f]",
                current_x, current_y, current_z,
                horizontal_distance, ideal_horizontal_distance, horizontal_error,
                vel_x_body, vel_y_body, -descent_speed);
    
    return false;
}
// 航点压缩函数：合并直线航点，保留拐角处的航点，长直线段添加中点
void TaskController::compress_waypoints(std::vector<std::vector<double>>& waypoints)
{
    if (waypoints.size() < 2) {
        RCLCPP_INFO(get_logger(), "航点数量少于2个，无需压缩，保持原有 %zu 个航点", waypoints.size());
        return;
    }
    
    // 如果只有2个航点，直接返回
    if (waypoints.size() == 2) {
        RCLCPP_INFO(get_logger(), "只有2个航点，无需压缩，保持原有航点");
        return;
    }

    // 记录原始航点数量
    size_t original_count = waypoints.size();
    std::vector<std::vector<double>> raw_waypoints = waypoints;  // 保存原始数据
    waypoints.clear();  // 清空原数组，重新填充

    // 总是保留第一个点
    waypoints.push_back(raw_waypoints[0]);
    RCLCPP_INFO(get_logger(), "保留起始点: (%.2f, %.2f, %.2f)", 
            raw_waypoints[0][0], raw_waypoints[0][1], raw_waypoints[0][2]);

    // 用于记录当前直线段的信息
    size_t line_start_idx = 0;  // 当前直线段的起始索引
    std::vector<size_t> current_line_points;  // 当前直线段包含的所有点索引

    for (size_t i = 1; i < raw_waypoints.size() - 1; ++i) {
        // 获取三个连续点
        const auto& prev = raw_waypoints[i - 1];
        const auto& curr = raw_waypoints[i];
        const auto& next = raw_waypoints[i + 1];
        
        // 计算两个方向向量
        double dx1 = curr[0] - prev[0];
        double dy1 = curr[1] - prev[1];
        double dx2 = next[0] - curr[0];
        double dy2 = next[1] - curr[1];
        
        // 判断是否在同一直线上（方向向量平行）
        // 使用叉积判断：如果叉积为0，则向量平行
        double cross_product = dx1 * dy2 - dy1 * dx2;
        const double epsilon = 1e-6;  // 数值误差容忍度
        
        // 首先检查是否为掉头点（同一直线上但方向相反）
        bool is_uturn = false;
        if (std::abs(cross_product) <= epsilon) {
            // 在同一直线上，检查是否为掉头
            double dot_product = dx1 * dx2 + dy1 * dy2;
            if (dot_product < -epsilon) {
                is_uturn = true;
            }
        }
        
        // 判断是否需要保留此点（拐角点或掉头点）
        if (std::abs(cross_product) > epsilon || is_uturn) {
            // 这是拐角点或掉头点，需要保留
            
            // 先处理之前累积的直线段
            if (!current_line_points.empty()) {
                size_t total_merged_points = current_line_points.size();
                if (total_merged_points >= 4) {
                    // 计算直线段的中点
                    const auto& line_start = raw_waypoints[line_start_idx];
                    const auto& line_end = raw_waypoints[i - 1];
                    
                    // 创建中点航点
                    std::vector<double> midpoint(4);  // x, y, z, yaw
                    midpoint[0] = (line_start[0] + line_end[0]) / 2.0;
                    midpoint[1] = (line_start[1] + line_end[1]) / 2.0;
                    midpoint[2] = (line_start[2] + line_end[2]) / 2.0;
                    midpoint[3] = line_start.size() > 3 ? line_start[3] : 0.0;  // yaw
                    
                    waypoints.push_back(midpoint);
                    RCLCPP_INFO(get_logger(), "添加长直线段中点: (%.2f, %.2f, %.2f), 合并了%zu个点", 
                            midpoint[0], midpoint[1], midpoint[2], total_merged_points);
                }
                
                RCLCPP_INFO(get_logger(), "完成直线段合并: 起始索引%zu, 合并了%zu个点", 
                        line_start_idx, total_merged_points);
                current_line_points.clear();
            }
            
            // 保留当前关键点
            waypoints.push_back(curr);
            if (is_uturn) {
                double dot_product = dx1 * dx2 + dy1 * dy2;
                RCLCPP_INFO(get_logger(), "保留掉头点: (%.2f, %.2f, %.2f), 点积: %.6f", 
                        curr[0], curr[1], curr[2], dot_product);
            } else {
                RCLCPP_INFO(get_logger(), "保留拐角点: (%.2f, %.2f, %.2f), 叉积: %.6f", 
                        curr[0], curr[1], curr[2], cross_product);
            }
            
            // 重置直线段记录
            line_start_idx = i;
            
        } else {
            // 普通直线点，添加到当前直线段
            if (current_line_points.empty()) {
                // 开始新的直线段
                line_start_idx = i - 1;  // 直线段从前一个点开始
            }
            current_line_points.push_back(i);
            
            RCLCPP_DEBUG(get_logger(), "跳过直线点: (%.2f, %.2f, %.2f), 叉积: %.6f", 
                    curr[0], curr[1], curr[2], cross_product);
        }
    }

    // 处理最后可能剩余的直线段
    if (!current_line_points.empty()) {
        size_t total_merged_points = current_line_points.size();
        if (total_merged_points >= 4) {
            // 计算直线段的中点
            const auto& line_start = raw_waypoints[line_start_idx];
            const auto& line_end = raw_waypoints[raw_waypoints.size() - 2];  // 倒数第二个点
            
            // 创建中点航点
            std::vector<double> midpoint(4);
            midpoint[0] = (line_start[0] + line_end[0]) / 2.0;
            midpoint[1] = (line_start[1] + line_end[1]) / 2.0;
            midpoint[2] = (line_start[2] + line_end[2]) / 2.0;
            midpoint[3] = line_start.size() > 3 ? line_start[3] : 0.0;
            
            waypoints.push_back(midpoint);
            RCLCPP_INFO(get_logger(), "添加最后直线段中点: (%.2f, %.2f, %.2f), 合并了%zu个点", 
                    midpoint[0], midpoint[1], midpoint[2], total_merged_points);
        }
        
        RCLCPP_INFO(get_logger(), "完成最后直线段合并: 起始索引%zu, 合并了%zu个点", 
                line_start_idx, total_merged_points);
    }

    // 总是保留最后一个点
    if (raw_waypoints.size() > 1) {
        waypoints.push_back(raw_waypoints.back());
        RCLCPP_INFO(get_logger(), "保留终点: (%.2f, %.2f, %.2f)", 
                raw_waypoints.back()[0], raw_waypoints.back()[1], raw_waypoints.back()[2]);
    }
    
    RCLCPP_INFO(get_logger(), "航点压缩完成：原始 %zu 个航点 -> 压缩后 %zu 个航点", 
                original_count, waypoints.size());
        // 输出合并后的航线
    std::stringstream compressed_path_ss;
    compressed_path_ss << "压缩后航线: ";
    for (size_t i = 0; i < waypoints.size(); ++i) {
        // 将坐标转换回航点名称
        double x = waypoints[i][0];
        double y = waypoints[i][1];
        
        // 逆向转换坐标到航点名称
        // x = (row - 1) * 0.5 -> row = x / 0.5 + 1
        // y = (9 - col) * 0.5 -> col = 9 - y / 0.5
        int row = static_cast<int>(std::round(x / 0.5 + 1));
        int col = static_cast<int>(std::round(9 - y / 0.5));
        
        // 验证范围并格式化输出
        if (row >= 1 && row <= 7 && col >= 1 && col <= 9) {
            compressed_path_ss << "A" << col << "B" << row;
        } else {
            // 对于中点等特殊情况，直接输出坐标
            compressed_path_ss << "(%.2f,%.2f)" << x << "," << y;
        }
        
        if (i < waypoints.size() - 1) {
            compressed_path_ss << " -> ";
        }
    }
    RCLCPP_INFO(get_logger(), "%s", compressed_path_ss.str().c_str());

}



}




int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<offboard::TaskController>());//cpp里面似乎是传指针的
    rclcpp::shutdown();
    return 0;
}