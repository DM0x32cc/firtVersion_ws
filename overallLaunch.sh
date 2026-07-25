#!/bin/bash   
# shebang必须写绝对路jing
#this is gazebo's topic (optional.if you don't let them run you can't see them in ros2 topic list)
# /clock
# /gazebo/resource_paths
# /gui/camera/pose
# /gui/currently_tracked
# /gui/track
# /model/x500_0/command/motor_speed
# /model/x500_0/servo_0
# /model/x500_0/servo_1
# /model/x500_0/servo_2
# /model/x500_0/servo_3
# /model/x500_0/servo_4
# /model/x500_0/servo_5
# /model/x500_0/servo_6
# /model/x500_0/servo_7
# /stats
# /world/default/clock
# /world/default/dynamic_pose/info
# /world/default/model/x500_0/link/base_link/sensor/air_pressure_sensor/air_pressure
# /world/default/model/x500_0/link/base_link/sensor/imu_sensor/imu
# /world/default/model/x500_0/link/base_link/sensor/navsat_sensor/navsat
# /world/default/pose/info
# /world/default/scene/deletion
# /world/default/scene/info
# /world/default/state
# /world/default/stats
# /x500_0/command/motor_speed
# /model/x500/joint_states
# /model/x500_0/odometry_with_covariance
# /world/default/light_config
# /world/default/material_color
# /world/default/wrench
# /world/default/wrench/clear
# /world/default/wrench/persistent

#如何自己从gazebo可发布话题中自己添加到ros2话题之中？
#ros2 run ros_gz_bridge parameter_bridge \ <Gazebo话题>@<ROS2消息类型>[<Gazebo消息类型> \ <Gazebo话题>@<ROS2消息类型>[<Gazebo消息类型>

model=$1
cd ~/flyControl/PX4-Autopilot
# terminator -x LIBGL_ALWAYS_SOFTWARE=1 make px4_sitl $model &

terminator -e "bash -ic 'LIBGL_ALWAYS_SOFTWARE=1 make px4_sitl ${model:-gz_x500}; exec bash'" &

sleep 9

# terminator -x ros2 launch mavros px4.launch fcu_url:=udp://:14540@14557 &
terminator -e "bash -ic 'ros2 launch mavros px4.launch fcu_url:=udp://:14540@14557; exec bash'" &
sleep 2
#这里我们只使用一些常用的gazebo话题，其他gazebo话题可以自己添加到ros2话题中

ros2 run ros_gz_bridge parameter_bridge \
  /clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock \
  /world/default/model/x500_0/link/base_link/sensor/imu_sensor/imu@sensor_msgs/msg/Imu[gz.msgs.IMU \
  /world/default/model/x500_0/link/base_link/sensor/air_pressure_sensor/air_pressure@sensor_msgs/msg/FluidPressure[gz.msgs.FluidPressure \
  /world/default/model/x500_0/link/base_link/sensor/navsat_sensor/navsat@sensor_msgs/msg/NavSatFix[gz.msgs.NavSat \
  /model/x500_0/odometry_with_covariance@nav_msgs/msg/Odometry[gz.msgs.OdometryWithCovariance &

sleep 2
# terminator -x ros2 topic list &
# terminator -e "bash -ic 'ros2 topic list; exec bash'" &

# gazebo harmonic所有可以发布的话题
# ros2 run ros_gz_bridge parameter_bridge \
#   /clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock \
#   /world/default/model/x500_0/link/base_link/sensor/imu_sensor/imu@sensor_msgs/msg/Imu[gz.msgs.IMU \
#   /world/default/model/x500_0/link/base_link/sensor/air_pressure_sensor/air_pressure@sensor_msgs/msg/FluidPressure[gz.msgs.FluidPressure \
#   /world/default/model/x500_0/link/base_link/sensor/navsat_sensor/navsat@sensor_msgs/msg/NavSatFix[gz.msgs.NavSat \
#   /model/x500_0/odometry_with_covariance@nav_msgs/msg/Odometry[gz.msgs.OdometryWithCovariance \
#   ！！！ 以上为已经添加到ros2话题中的gazebo话题，下面为gazebo中其他可发布的话题
#   /stats@gz.msgs.Statistics \
#   /world/default/pose/info@gz.msgs.Pose_V \
#   /world/default/state@gz.msgs.WorldState \
#   /world/default/dynamic_pose/info@gz.msgs.Pose_V \
#   /world/default/scene/info@gz.msgs.Scene \
#   /world/default/scene/deletion@gz.msgs.Entity \
#   /world/default/light_config@gz.msgs.Light \
#   /world/default/material_color@gz.msgs.MaterialColor \
#   /world/default/wrench@gz.msgs.Wrench \
#   /world/default/wrench/clear@gz.msgs.Empty \
#   /world/default/wrench/persistent@gz.msgs.Boolean \
#   /gazebo/resource_paths@gz.msgs.StringMsg \
#   /gui/camera/pose@gz.msgs.GUI \
#   /gui/currently_tracked@gz.msgs.Int32 \
#   /gui/track@gz.msgs.GUI \
#   /model/x500_0/command/motor_speed@gz.msgs.Actuators \
#   /x500_0/command/motor_speed@gz.msgs.Actuators \
#   /model/x500_0/servo_0@gz.msgs.Double \
#   /model/x500_0/servo_1@gz.msgs.Double \
#   /model/x500_0/servo_2@gz.msgs.Double \
#   /model/x500_0/servo_3@gz.msgs.Double \
#   /model/x500_0/servo_4@gz.msgs.Double \
#   /model/x500_0/servo_5@gz.msgs.Double \
#   /model/x500_0/servo_6@gz.msgs.Double \
#   /model/x500_0/servo_7@gz.msgs.Double
#   注意，我们这个还可以指定gazebo发布的话题的细节情况
# 符号	  数据流向	        本质	           典型话题	                你现在需要用吗？
#  [ 	Gazebo → ROS 2	  读传感器	    IMU、GPS、相机、时钟	           ✅ 全部用这个
#  ] 	ROS 2 → Gazebo	  发控制指令	   电机转速、舵机角度	            ❌ 以后控制无人机时才用
#  @ 	      双向	       双向通信	      状态确认、参数同步	           ❌ 新手阶段忽略

# <Gazebo话题>@<ROS2消息类型>上述命令所在处<Gazebo消息类型>



# mavros配置文件在 /opt/ros/<你的ROS版本>/share/mavros/launch/px4_config.yaml  注意：我把他给改了，不让mavros同步了，ai说在模拟里面，这样没有问题
