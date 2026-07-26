


# ros2 launch livox_ros_driver2 msg_MID360_launch.py &

# sleep 10

# ros2 launch fast_lio mapping.launch.py rviz:=false &

# sleep 30

# ros2 run 转换桥节点 &

# sleep 5

# ros2 launch mavros px4.launch fcu_url:=/dev/ttyACM0:921600 gcs_url:=udp://@192.168.43.6 &

#ros2 topic pub --once /launch std_msgs/msg/Bool "{data: true}"
#ros2 topic pub --once /launch std_msgs/msg/Bool "{data: false}"


# #!/bin/bash
# source install/setup.bash
# # cd flyControl/test_ws
# ros2 run launch_flag node_launch_flag && sleep 3 &&  ros2 node kill /node_launch_flag
# ros2 run launch_flag node_launch_flag_false && sleep 3 &&  ros2 node kill /node_launch_flag__false
