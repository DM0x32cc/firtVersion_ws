#!/bin/bash
source install/setup.bash
# cd flyControl/test_ws
ros2 run launch_flag node_launch_flag

# ros2 run launch_flag node_launch_flag && sleep 3 &&  ros2 node kill /node_launch_flag
# ros2 run launch_flag node_launch_flag_false && sleep 3 &&  ros2 node kill /node_launch_flag__false