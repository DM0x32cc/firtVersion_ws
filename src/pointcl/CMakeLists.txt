cmake_minimum_required(VERSION 3.8)
project(pointcl)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(livox_ros_driver2 REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(msg_tool REQUIRED)

add_executable(livox_to_pcl src/livox_to_pcl.cpp)
add_executable(pole_detection src/pole_detection.cpp)

ament_target_dependencies(livox_to_pcl
    rclcpp
    sensor_msgs
    pcl_conversions
    pcl_ros
    livox_ros_driver2
)

ament_target_dependencies(pole_detection
  rclcpp
  sensor_msgs
  msg_tool
  pcl_conversions
)

# Install the executable，我曹，这杆子融合没写代码然后直接没有安装，这是神了
install(TARGETS 
  livox_to_pcl
  pole_detection
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
