//cpp里面似乎是传指针的给spin的
// 这是主流程，我们先打四个点，然后飞到每个点上面时，让他悬停个2.5秒再转一圈。
#include "offboard/test_task_controller.hpp"
#include <Eigen/Core>
namespace offboard
{

TaskController::TaskController() : BaseController("offb_node"), cpfly_pid_x_(1.0, 0.3, 0.0, 0.5),  
    cpfly_pid_y_(1.0, 0.3, 0.0, 0.5),land_pid_x_(1.0, 0.3, 0.0, 0.5),land_pid_y_(1.0, 0.3, 0.0, 0.5)// kp=1, ki=0.3, kd=0, 积分上限 0.5m/s
            /*, path_planner_(nullptr), obstacles_received_(false), path_planned_(false)*/
{
    
    this->declare_parameter("takeoff_height", 1.5);
    this->declare_parameter("waypoint_threshold", 0.1);//到点阈值
    this->declare_parameter("filter_param_company",0.5);

    takeoff_height_=this->get_parameter("takeoff_height").as_double();
    waypoint_threshold_=this->get_parameter("waypoint_threshold").as_double();
    filter_param_company_ = this ->get_parameter("filter_param_company").as_double();

    // 初始化位置停留保护机制变量
    last_position_time_ = this->get_clock()->now();
    last_position_x_ = 0.0;
    last_position_y_ = 0.0;
    last_position_z_ = 0.0;
    position_stuck_detected_ = false;
    ignoring_targets_ = false;
    target_data_ready_ = false;
    cpfly_takedown_ = false;
    // is_drop = false;

    target_sub = create_subscription<msg_tool::msg::Color>(
        "/target", qos_best_effort,
        [this](const msg_tool::msg::Color::ConstSharedPtr& msg){
            target_callback(msg);
        });
    car_state_sub = create_subscription<msg_tool::msg::CarState>(
        "/car_state",qos_best_effort,
        [this](const msg_tool::msg::CarState::ConstSharedPtr& msg){
            car_state_callback(msg);
        });
    
    
    RCLCPP_INFO(get_logger(), "Task controller initialized");
}

void TaskController::target_callback(const msg_tool::msg::Color::ConstSharedPtr& msg)//先不用世界坐标系,先滤波
{
    target_msg_ = *msg;
    double height = local_position_.pose.position.z + 0.025;
    double dx_meter = pixel_to_meter_x(msg->delta_x, height);
    double dy_meter = pixel_to_meter_y(msg->delta_y, height);

    double world_dx = dx_meter * cos(current_yaw_) - dy_meter * sin(current_yaw_);
    double world_dy = dx_meter * sin(current_yaw_) + dy_meter * cos(current_yaw_);
    double car_world_x = local_position_.pose.position.x + world_dx;
    double car_world_y = local_position_.pose.position.y + world_dy;
    //错误数据不要直接跳过，全盘照收，如果中途丢数据，那么就用小车速度行驶吗？？？不行，还是留着县
    
    if (!target_data_ready_  ) 
    {
        target_data_ready_ = true;
        filtered_car_x_= car_world_x;
        filtered_car_y_ = car_world_y;
        RCLCPP_INFO(get_logger(), "首次收到目标检测数据");
        return;
    }
    // 如果中途扫见了，会不会拖慢我们的数据更新呢？我觉得如果转世界坐标系就不会出事了
    filtered_car_x_ = filter_param_company_ * filtered_car_x_
                       + (1.0 - filter_param_company_) * car_world_x;
    filtered_car_y_ = filter_param_company_ * filtered_car_y_
                       + (1.0 - filter_param_company_) * car_world_y;
    target_msg_.delta_x = static_cast<float>(filtered_car_x_ - local_position_.pose.position.x);
    target_msg_.delta_y = static_cast<float>(filtered_car_y_ - local_position_.pose.position.y);

    // 我想到一个东西，反正最终去参与PID的是距离差，那么如果我们想实在想把这个无人机的抖动与小车的抖动分离的话，
    // 那我们其实可以在回调函数里面直接进行分离，就接收到摄像头检查这的距离差之后，先把它转到10呃市里头报一下，
    // 然后进行滤波，滤波完之后再拿当前位置减去滤波之后结果就可以得到，依旧得到这个J绝差了。啊我们现在是直接拿到
    // 距离差滤波，然后这个只是多了一步，所以说想改的话是非常简单的，一点都不难
}

void TaskController::car_state_callback(const msg_tool::msg::CarState::ConstSharedPtr& msg)
{
    car_speed_x_=msg->speed*std::cos(msg->deviation_angle);//这是对的
    car_speed_y_=msg->speed*std::sin(msg->deviation_angle);
    // 机头不转可太方便了!,这样小车的速度矢量直接加上去即可!
    deviation_angle_ = msg->deviation_angle;
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
    
    task_state_pub();// 发布消息，这波应该由地面站来接收，这个会一直发布准确的的

    if (current_task_id_ == 1) 
    {
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
            case FlightState::HOVER_3S:     hover_3s(); break;      // 新增
            case FlightState::FLY_TO_MIDPOINT: fly_to_point(); break;
            case FlightState::COMPANION_FLIGHT:    companion_fly(); break; // 新增
            case FlightState::DROP:         do_drop(); break;       // 新增
            case FlightState::RETURN_HOME:  return_home(); break;   // 新增
            case FlightState::TILTLAND:     
                if(tilt_land()) 
                switch_task(FlightState::LAND);
                break;
            case FlightState::LAND:         
                land(); 
                if (!current_state_.armed) 
                {
                    switch_task(FlightState::INIT);//也就是说，在降落状态下，如果上锁，就进入这init模式，
                                                   //这个switch函数会让他全部恢复至刚上电状态
                }

                break;
            default:
            RCLCPP_INFO(get_logger(), "我去！未知飞行状态？！你干哪里来了？");
        }
    }
    else if(current_task_id_ == 2)
    {
        switch(flight_state_)
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
            case FlightState::SEARCH_CAR:   search_car(); break;       // 新增
            case FlightState::APPROACH_CAR: approach_car(); break;     // 新增
            case FlightState::LAND_ON_CAR:  land_on_car(); break;      // 新增
            case FlightState::STAY_ON_CAR:  stay_on_car(); break;      // 新增
            case FlightState::TAKEOFF_FROM_CAR:  takeoff_from_car(); break; // 新增
            // case FlightState::RETURN_HOME:  return_home(); break;//不必了
            case FlightState::TILTLAND:     
                if(tilt_land()) 
                switch_task(FlightState::LAND);
                break;
            case FlightState::LAND:         
                land(); 
                if (!current_state_.armed) 
                {
                    switch_task(FlightState::INIT);
                }
                break;
            default:
            RCLCPP_INFO(get_logger(), "我去！未知飞行状态？！你干哪里来了？");
        }
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

    if(current_task_id_==1)
    {
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
            if(std::abs(local_position_.pose.position.z - takeoff_height_)< waypoint_threshold_)
            {
                start_hover(3.0);
                switch_task(FlightState::HOVER_3S);   // 进入悬停3s
                return true;
            } 
            break;
        case FlightState::HOVER_3S:
            if (update_hover()) //这个随便调哟in，三s一道才会返回true
            {                         // 每帧检查，返回true=时间到
                switch_task(FlightState::FLY_TO_MIDPOINT);      // 进入伴飞
                return true;
            }
            break;
        case FlightState::FLY_TO_MIDPOINT :
            if(distance_to_target(0.875,-0.375,takeoff_height_) < 0.15)//如果太慢可以放大这个阈值,反正这不用太精确
            {
                switch_task(FlightState::COMPANION_FLIGHT);
                return true;
            }
            break;
        case FlightState::COMPANION_FLIGHT :
            if(std::hypot(local_position_.pose.position.x-2.375,local_position_.pose.position.y-1.875) < 0.15)//只算水平距离
            {
                // 只有到那个点才可以去切换
                switch_task(FlightState::DROP);
                return true;
            }
            break;
        case FlightState::DROP:
            // if(is_drop == true)
            // {
            //     switch_task(FlightState::RETURN_HOME);
            //     return true;
            // }
            // do_drop()函数自己会转
            break;
        case FlightState::RETURN_HOME:
            if(is_at_point(0.0,-1.875,1.5))
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
    }
    else if(current_task_id_==2)
    {
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
            if(std::abs(local_position_.pose.position.z - takeoff_height_)< waypoint_threshold_)
            {
                switch_task(FlightState::SEARCH_CAR);   // 进入悬停3s
                return true;
            } 
            break;
        case FlightState::SEARCH_CAR:
        // 万一没到点就，发现了，然后一到点就切换，怎么办呢？反正是不断更新的，这么来应该就没有问题了。
            if( target_msg_.detected == true && is_at_point(0.875,-0.375,takeoff_height_) == true) 
            {
                switch_task(FlightState::APPROACH_CAR);
                return true;
            }
            break;
        case FlightState::APPROACH_CAR:
            double error = std::hypot(target_msg_.delta_x , target_msg_.delta_y );
            if (error < 0.05 && target_msg_.detected)
            {
                approach_stable_count_++;
            }
            else
            {
                approach_stable_count_ = 0;
            }
            if (approach_stable_count_ >= 12)   // 对准了，开始降
            {
                approach_stable_count_ = 0;
                touch_count_ = 0;
                land_last_z_ = local_position_.pose.position.z;//这里是提前来一个定义，不然在函数里面
                // 总之定义在这里完全没有问题！！！
                switch_task(FlightState::LAND_ON_CAR);
                return true;
            }
            break;
        case FlightState::LAND_ON_CAR:
            // 这啥也不用，在timer_callbacj里面切换
            break;
        case FlightState::TAKEOFF_FROM_CAR:
            if(is_at_point(0.0,-1.875,1.5))
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
    }
    return false;
}

void  TaskController::hover_3s()
{
    update_hover();
}
void TaskController::fly_to_point()
{
    publish_position_setpoint(0.875,-0.375,takeoff_height_,0.0);
}

void TaskController::companion_fly()
{
    // 如果中途丢失目标怎么处理？？？？？？？？没想好我曹了，感觉应该预测一下
    if(target_msg_.detected == false)//原地悬停，不能回飞。这里飞到中点才切换这个模式，那么此时看到小车，则是有用的消息。
    {
        publish_velocity_body(car_speed_x_, car_speed_y_,
                          1.5 * (1.50 - local_position_.pose.position.z), 0.0);
        return;
    }
    static auto last_velo_pid_time_ = this->get_clock()->now();//这没有问题，这只会第一次调用的时候使得积分项为0,后续完全不影响了
    auto now = this->get_clock()->now();
    double dt = (now - last_velo_pid_time_).seconds();
    if (dt > 0.5) dt = 0.1; 
    // 这时候可以开始处理了，true，已经收到消息
    double vx_pid = cpfly_pid_x_.compute(target_msg_.delta_x, dt);   // 前后速度,这个方向可以直接用,因为机头方向不变
    double vy_pid = cpfly_pid_y_.compute(target_msg_.delta_y, dt);   // 左右速度
    // 这边需要提前降落，所以加上判断逻辑
    
    if(distance_to_target(3.125,-1.125,takeoff_height_) < 0.20) cpfly_takedown_ = true;
    double vz_cmd;
    if(cpfly_takedown_ == true)
    {
        vz_cmd = 1.5 * (1.10 - local_position_.pose.position.z );    // 高度纯 P
    }
    else
    {
        vz_cmd = 1.5 * (1.50 - local_position_.pose.position.z );   // 高度纯 P
    }
    last_velo_pid_time_ =now;
    // 加上前馈速度
    double vx_cmd = vx_pid + car_speed_x_;
    double vy_cmd = vy_pid + car_speed_y_;
    publish_velocity_body(vx_cmd,vy_cmd,vz_cmd,0.0);
}
void TaskController::do_drop()
{

    if(target_msg_.detected == false)//这里是为了等待吗？不是吧
    {
        publish_velocity_body(car_speed_x_, car_speed_y_,
                          1.5 * (1.10 - local_position_.pose.position.z), 0.0);
        return;
    }

    static auto last_velo_pid_time_ = this->get_clock()->now();//这没有问题，这只会第一次调用的时候使得积分项为0,后续完全不影响了
    auto now = this->get_clock()->now();
    double dt = (now - last_velo_pid_time_).seconds();
    if (dt > 0.5) dt = 0.1; 
    // 这时候可以开始处理了，true，已经收到消息
    double vx_pid = cpfly_pid_x_.compute(target_msg_.delta_x, dt);   // 前后速度,这个方向可以直接用,因为机头方向不变
    double vy_pid = cpfly_pid_y_.compute(target_msg_.delta_y, dt);   // 左右速度
    // 这边需要提前降落，所以加上判断逻辑
    double vz_cmd = 1.5 * (1.10 - local_position_.pose.position.z );   // 高度纯 P
    last_velo_pid_time_ =now;
    // 加上前馈速度
    double vx_cmd = vx_pid + car_speed_x_;
    double vy_cmd = vy_pid + car_speed_y_;
    publish_velocity_body(vx_cmd,vy_cmd,vz_cmd,0.0);
    // 这里投掷的时候，要判断距离小到一定程度才可以
        // 重试超过次数6次，放弃                         
    if (drop_retry_count_ >= 6)                    
    {                                              
        RCLCPP_INFO(get_logger(),"重试次数太多，提前结束这个阶段");           
        drop_sent_ = false;                       
        drop_confirmed_ = false;           
        drop_retry_count_ = 0;                    
        drop_stable_count_ = 0;         
        switch_task(FlightState::RETURN_HOME);    
        return;                                    
    }                               
    if (drop_sent_)
    {
        if (drop_confirmed_)
        {
            drop_sent_ = false;
            drop_confirmed_ = false;
            drop_retry_count_ = 0; 
            drop_stable_count_ = 0;
            RCLCPP_INFO(get_logger(), "抛投确认完成，返航");
            switch_task(FlightState::RETURN_HOME);
        }
        return;
    }
    double error = std::hypot(target_msg_.delta_x,target_msg_.delta_y);
    if (error < 0.07 && target_msg_.detected)
    {
        drop_stable_count_++;
    }
    else
    {
        drop_stable_count_ = 0;
    }
    if (drop_stable_count_ >= 10)
    {
        trigger_drop_servo();
        drop_sent_ = true;
    }
    
}

void TaskController::trigger_drop_servo()
{
    if (!command_client_->service_is_ready())
    {
        RCLCPP_WARN(get_logger(), "抛投: command 服务未就绪，跳过本帧");
        drop_sent_ = false;
        return;
    }
    auto request = std::make_shared<mavros_msgs::srv::CommandLong::Request>();
    request->command  = 183;    // MAV_CMD_DO_SET_SERVO
    request->param1   = 6;     // AUX 口编号，改成你舵机实际接的口，我们是六
    request->param2   = 2200;  // PWM 值（微秒），改成你舵机抛投动作对应的值，改为最大的2200
    request->param3   = 0.0;
    request->param4   = 0.0;
    request->param5   = 0.0;
    request->param6   = 0.0;
    request->param7   = 0.0;
    command_client_->async_send_request(
        request,
        [this](rclcpp::Client<mavros_msgs::srv::CommandLong>::SharedFuture future)
        {
            try
            {
                auto result = future.get();
                if (result->success)
                {
                    RCLCPP_INFO(get_logger(), "舵机抛投成功");
                    drop_confirmed_ = true;
                }
                else
                {
                    RCLCPP_ERROR(get_logger(), "舵机被飞控拒绝，返回码: %d", result->result);
                    drop_sent_ = false;
                    drop_retry_count_++;

                }
            }
            catch (const std::exception& e)
            {
                RCLCPP_ERROR(get_logger(), "舵机调用异常: %s", e.what());
                drop_sent_ = false;
                drop_retry_count_++;
            }
        });
}

void TaskController::return_home()
{
    const auto &pos = local_position_.pose.position;
    // 起点 = 当前位置
    double start_x = pos.x;
    double start_y = pos.y;
    double start_z = pos.z;
    if(!is_at_point(start_x,start_y,1.5))
    {
        publish_position_setpoint(start_x,start_y,1.5,0.0);
        return;
    }

    // 终点 = 你要飞去的固定点（例如从起飞点飞到前方 10m，高度 5m）
    double end_x = 0.0;
    double end_y = -1.875;
    double end_z = 1.5;   // NED 坐标系，负值 = 向上

    publish_position_setpoint_trajectory(
        start_x, start_y, 1.5,
        end_x, end_y, end_z,
        0);
}

void TaskController::search_car()  //暂时只会停在中点处
{
    // 如果中途丢失目标怎么处理？？？？？？？？没想好我曹了
    publish_position_setpoint(0.875,-0.375,takeoff_height_,0.0);
    return;
}
void TaskController::approach_car()//只是接近，至于切换逻辑，则在我的判断里面
{
    // 如果中途丢失目标怎么处理？？？？？？？？没想好我曹了
    if(target_msg_.detected == false)//原地悬停，不能回飞
    {
        publish_velocity_body(car_speed_x_, car_speed_y_,
                          1.5 * (1.50 - local_position_.pose.position.z), 0.0);
        return;
    }
    static auto last_velo_pid_time_ = this->get_clock()->now();//这没有问题，这只会第一次调用的时候使得积分项为0,后续完全不影响了
    auto now = this->get_clock()->now();
    double dt = (now - last_velo_pid_time_).seconds();
    if (dt > 0.5) dt = 0.11; 
    // 这时候可以开始处理了，true，已经收到消息
    double vx_pid = land_pid_x_.compute(target_msg_.delta_x, dt);   // 前后速度,这个方向可以直接用,因为机头方向不变
    double vy_pid = land_pid_y_.compute(target_msg_.delta_y, dt);   // 左右速度
    // 这边需要提前降落，所以加上判断逻辑
    double vz_cmd = 1.5 * (1.50 - local_position_.pose.position.z );   // 高度纯 P
    last_velo_pid_time_ =now;
    // 加上前馈速度
    double vx_cmd = vx_pid + car_speed_x_;
    double vy_cmd = vy_pid + car_speed_y_;
    publish_velocity_body(vx_cmd,vy_cmd,vz_cmd,0.0);
}
// pid 控制器可以混用吗？？？不可以！！！
void TaskController::land_on_car()//开始下降了
{
    // 如果中途丢失目标怎么处理？？？？？？？？没想好我曹了
    // 不行，这不可以有吧，不然太干扰，算了不知道怎么处理。。。。//这么处理好！！！
    publish_velocity_body(car_speed_x_, car_speed_y_,
                          1.5 * (1.50 - local_position_.pose.position.z), 0.0);
    // 只初始化一次，不用害怕，这函数只会被其中一个任务调用，但是同一个任务只可以执行一次，不能两次，不然要重启
    static auto last_velo_pid_time_ = this->get_clock()->now();
    auto now = this->get_clock()->now();
    double dt = (now - last_velo_pid_time_).seconds();
    if (dt > 0.5) dt = 0.1; 
    // 这时候可以开始处理了，true，已经收到消息
    double vx_pid = land_pid_x_.compute(target_msg_.delta_x, dt);   // 前后速度,这个方向可以直接用,因为机头方向不变
    double vy_pid = land_pid_y_.compute(target_msg_.delta_y, dt);   // 左右速度
    last_velo_pid_time_ =now;
    // 加上前馈速度
    double vx_cmd = vx_pid + car_speed_x_;
    double vy_cmd = vy_pid + car_speed_y_;
    double current_z = local_position_.pose.position.z;
    double dz = land_last_z_ - current_z;   // 正值 = 在下降
    land_last_z_ = current_z;
    if (std::abs(dz) < 0.003)
    {
        touch_count_++;
    }
    else
    {
        touch_count_ = 0;
    }
    if (touch_count_ >= 15)
    {
        // 确认停在车上
        // 不放零，放小车的速度
        publish_velocity_body(car_speed_x_, car_speed_y_, 0.0, 0.0);
        touch_count_ = 0;
        land_last_z_ = 0.0;
        stay_start_time_ = this->get_clock()->now();//这里开始计算时间
        switch_task(FlightState::STAY_ON_CAR);
        return;
    }
    // 还在下降中：水平修偏 + 极慢垂直速度
    if(current_z > 0.50)
    {
        publish_velocity_body(vx_cmd, vy_cmd, -0.35, 0.0);
    }
    else
    {
        publish_velocity_body(vx_cmd, vy_cmd, -0.06, 0.0);
    }
}

void TaskController::stay_on_car()
{
    publish_velocity_body(0.0, 0.0, 0.0, 0.0);
    auto elapsed = (this->get_clock()->now() - stay_start_time_).seconds();
    if (elapsed >= 5.0)
    {
        switch_task(FlightState::TAKEOFF_FROM_CAR);
    }
}

void TaskController::takeoff_from_car()//起飞，然后飞到倾斜降落点
{
    const auto &pos = local_position_.pose.position;
    // 起点 = 当前位置
    double start_x = pos.x;
    double start_y = pos.y;
    double start_z = pos.z;
    if(!is_at_point(start_x,start_y,1.5))
    {
        publish_position_setpoint(start_x,start_y,1.5,0.0);
        return;
    }

    // 终点 = 你要飞去的固定点（例如从起飞点飞到前方 10m，高度 5m）
    double end_x = 0.0;
    double end_y = -1.875;
    double end_z = 1.5;   // NED 坐标系，负值 = 向上

    publish_position_setpoint_trajectory(
        start_x, start_y, 1.5,
        end_x, end_y, end_z,
        0);
}

// 像素偏移 → 实际距离（米）
// delta_pixel: 像素偏移（align_node 发的 delta_x 或 delta_y）
// height:      离地高度（米）= local_position_.pose.position.z + 0.025
// 返回:        实际水平距离（米）

double TaskController::pixel_to_meter_x(double delta_pixel, double height)
{
    const double half_width = 320.0;                           // 640 / 2
    const double hfov_half  = 70.42 / 2.0 * M_PI / 180.0;     // 35.21°
    return delta_pixel / half_width * height * tan(hfov_half);
}

double TaskController::pixel_to_meter_y(double delta_pixel, double height)
{
    const double half_height = 240.0;                          // 480 / 2
    const double vfov_half   = 43.3 / 2.0 * M_PI / 180.0;     // 21.65°
    return delta_pixel / half_height * height * tan(vfov_half);
}

void TaskController::switch_task(FlightState new_state)
{
    if (new_state == flight_state_) 
    {
        return;
    }

    if (new_state == FlightState::INIT)
    {
        // 任务id重置
        current_task_id_ = -1 ;
        // === 父类变量 ===
        apply_disarm_flag_ = false;          // ✅ 已有
        mode_switched_for_landing_ = false;  // ✅ 已有
        launch_flag_ = false;                // ✅ 已有

        // === 伴飞/抛投 ===
        cpfly_takedown_ = false;             // ✅ 已有
        target_data_ready_ = false;          // ✅ 已有,目标消息
        drop_stable_count_ = 0;
        drop_retry_count_ = 0;
        drop_sent_ = false;
        drop_confirmed_ = false;             // ❌ 缺失
        

        // === 视觉滤波 ===
        filtered_car_x_ = 0;                  // ❌ 缺失
        filtered_car_y_ = 0;                  // ❌ 缺失

        // === 小车状态 ===
        car_speed_x_ = 0;                    // ❌ 缺失
        car_speed_y_ = 0;                    // ❌ 缺失

        // === 小车降落 ===
        approach_stable_count_ = 0;          // ❌ 缺失
        touch_count_ = 0;                    // ❌ 缺失（land_on_car 里手动重置了，但不保险）
        land_last_z_ = 0;                    // ❌ 缺失

        // === 倾斜降落 ===
        tilt_stage1_completed_ = false;      // ❌ 缺失
        tilt_stage1_started_ = false;        // ❌ 缺失
        tilt_stage2_stuck_counter_ = 0;      // ❌ 缺失
        tilt_stage2_last_z_ = -1.0;          // ❌ 缺失

        // === PID 控制器 ===
        cpfly_pid_x_.reset();                // ❌ 缺失（需要 PIDController 加 reset() 方法）
        cpfly_pid_y_.reset();                // ❌ 缺失
        land_pid_x_.reset();                 // ❌ 缺失
        land_pid_y_.reset();                 // ❌ 缺失

        // === 防卡位 ===
        reset_position_stuck_detection();    // ❌ 缺失（这个函数已写好，直接调用即可）
    }
    RCLCPP_INFO(get_logger(), "切换任务: 从 %s 到 %s",
                flightStateToString(flight_state_).c_str(),
                flightStateToString(new_state).c_str());
    flight_state_ = new_state;
}



bool TaskController::check_emergency_condition()
{
    if(std::abs(local_position_.pose.position.x) >= 9 ||
        std::abs(local_position_.pose.position.y) >= 9 ||
        std::abs(local_position_.pose.position.z) >= 2.5){
        RCLCPP_INFO(get_logger(), "检测到紧急情况，位置超出范围，自动降落");
        return true;
    }
    return false;
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
    if (current_z < 0.08 || (current_z < 0.15 && horizontal_distance < 0.08)) 
    {
        RCLCPP_INFO(get_logger(), "降落完成！");
        // 重置状态变量以便下次使用
        tilt_stage1_completed_ = false;
        tilt_stage1_started_ = false;
        return true;//这是唯一退出条件，但是太严苛了，后来已经放宽条件
    }
    
    
    // 新增整个 if 块，原始版本没有这段
    // 这是全新阶段，当高度小于20cm时不强行要求45度了
    if (current_z < 0.20) 
    {
        double near_vel_x = 0.0, near_vel_y = 0.0;
        if (horizontal_distance > 0.03) 
        {
            double dir_x = (target_x - current_x) / horizontal_distance;
            double dir_y = (target_y - current_y) / horizontal_distance;
            double near_speed = std::min(0.3, horizontal_distance * 1.5);
            near_vel_x = dir_x * near_speed;
            near_vel_y = dir_y * near_speed;
        }
        publish_velocity_body(near_vel_x, near_vel_y, -0.20, 0.0);
        return false;
    }

    // 45度角降落的理想水平距离应该等于当前高度
    double ideal_horizontal_distance = current_z;  // 45度角条件

    // 第一阶段：飞到45度线起点（只执行一次）
    if (!tilt_stage1_completed_) {
        double position_tolerance = 0.05;  // 位置容差
        
        // 如果还没开始第一阶段，或者距离45度线太远，继续第一阶段
        if (!tilt_stage1_started_ || std::abs(horizontal_distance - ideal_horizontal_distance) > position_tolerance) {
            tilt_stage1_started_ = true;
            
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
            tilt_stage1_completed_ = true;
            RCLCPP_INFO(get_logger(), "第一阶段完成！开始准备45度角降落");
        }
    }
    
    // 新增在第二阶段开头，我觉得这亨脊肋，没啥乱用，而且也不干扰代码，所以先留着把
    if (tilt_stage2_last_z_ > 0 && std::abs(current_z - tilt_stage2_last_z_) < 0.005) {
        tilt_stage2_stuck_counter_++;
    } else {
        tilt_stage2_stuck_counter_ = 0;
    }
    tilt_stage2_last_z_ = current_z;

    if (tilt_stage2_stuck_counter_ > 60) {
        publish_velocity_body(0.0, 0.0, -0.20, 0.0);
        return false;
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