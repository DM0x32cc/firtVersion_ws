# cmake_minimum_required(VERSION 3.8)
# project(offboard)
# set(CMAKE_CXX_STANDARD_REQUIRED ON)

# # 设置C++标准
# if(NOT CMAKE_CXX_STANDARD)
#   set(CMAKE_CXX_STANDARD 20)
# endif()

# if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#   add_compile_options(-Wall -Wextra -Wpedantic)
# endif()

# # 查找依赖包
# find_package(nav_msgs REQUIRED)
# find_package(Eigen3 REQUIRED)
# find_package(ament_cmake REQUIRED)
# find_package(rclcpp REQUIRED)
# find_package(rclpy REQUIRED)
# find_package(std_msgs REQUIRED)
# find_package(geometry_msgs REQUIRED)
# find_package(mavros_msgs REQUIRED)
# find_package(msg_tool REQUIRED)
# find_package(tf2 REQUIRED)
# find_package(tf2_ros REQUIRED)
# find_package(tf2_geometry_msgs REQUIRED)
# find_package(msg_tool REQUIRED)


# # 包含目录，这样方便你去找
# include_directories(
#   include
#   ${rclcpp_INCLUDE_DIRS}
#   ${geometry_msgs_INCLUDE_DIRS}
#   ${mavros_msgs_INCLUDE_DIRS}
#   ${std_msgs_INCLUDE_DIRS}
#   ${tf2_INCLUDE_DIRS}
#   ${tf2_ros_INCLUDE_DIRS}
#   ${tf2_geometry_msgs_INCLUDE_DIRS}
#   ${msg_tool_INCLUDE_DIRS}
#   ${CMAKE_CURRENT_BINARY_DIR}/include
# )

# # 添加源文件
# set(SOURCES
#   src/base_controller.cpp
#   # src/task_controller.cpp这里是改动，因为我们没有这个文件
#   src/ChessboardPathPlanner.cpp
# )

# # 添加可执行文件
# add_executable(offboard_node ${SOURCES})

# # add_executable(odom_to_pose_node src/odom_to_pose_node.cpp)这里是改动，这文件我们也没有。

# # 链接依赖库
# ament_target_dependencies(offboard_node
#   rclcpp
#   std_msgs
#   geometry_msgs
#   mavros_msgs
#   msg_tool
#   tf2
#   tf2_ros
#   tf2_geometry_msgs
#   nav_msgs
#   )
# ament_target_dependencies(odom_to_pose_node 
#   rclcpp 
#   nav_msgs 
#   geometry_msgs 
#   tf2_ros 
#   Eigen3)

# # 安装可执行文件
# install(TARGETS
#   offboard_node
#   DESTINATION lib/${PROJECT_NAME}
# )
# install(TARGETS 
#   odom_to_pose_node 
#   DESTINATION 
#   lib/${PROJECT_NAME})

# # 安装头文件
# install(DIRECTORY
#   include/
#   DESTINATION include/
# )

# # 安装启动文件
# install(DIRECTORY
#   launch
#   config
#   DESTINATION share/${PROJECT_NAME}
# )

# # 导出依赖
# ament_export_include_directories(include)
# ament_export_dependencies(
#   rclcpp
#   rclpy
#   std_msgs
#   geometry_msgs
#   mavros_msgs
#   msg_tool
#   tf2
#   tf2_ros
#   tf2_geometry_msgs
#   nav_msgs
#   Eigen3
# )

# ament_package()

# ai生成
cmake_minimum_required(VERSION 3.8)
project(offboard)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 设置 C++ 标准
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 查找依赖包
find_package(nav_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(msg_tool REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# 包含目录
include_directories(
  include
  ${rclcpp_INCLUDE_DIRS}
  ${geometry_msgs_INCLUDE_DIRS}
  ${mavros_msgs_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS}
  ${tf2_INCLUDE_DIRS}
  ${tf2_ros_INCLUDE_DIRS}
  ${tf2_geometry_msgs_INCLUDE_DIRS}
  ${msg_tool_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}
  ${CMAKE_CURRENT_BINARY_DIR}/include
)

# 添加源文件
set(BASE_SOURCES
  src/base_controller.cpp
)

# 添加可执行文件 - test_task_controller (主任务控制器)只有有main函数且有节点的文件才需要生成可执行文件，但他用到的文件都要加到后面的源文件列表中
add_executable(test_task_controller 
  ${BASE_SOURCES}
  src/test_task_controller.cpp
)



# 链接依赖库
ament_target_dependencies(test_task_controller
  rclcpp
  std_msgs
  geometry_msgs
  mavros_msgs
  msg_tool
  tf2
  tf2_ros
  tf2_geometry_msgs
  nav_msgs
  Eigen3
)


# 安装可执行文件
install(TARGETS
 test_task_controller
  DESTINATION lib/${PROJECT_NAME}
)
# targets 是你要安装的目标名，destination则是你要安装到的目录，不过这里省略了，默认还要加上install/功能包名

# 安装头文件
install(DIRECTORY
  include/
  DESTINATION include/
)

# 安装启动文件和配置文件（如果存在）
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/launch")#wc,他把我们自己写的launch文件搞上去了，使得我们可以ros2 launch
  install(DIRECTORY
    launch
    DESTINATION share/${PROJECT_NAME}
  )
endif()

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/config")
  install(DIRECTORY
    config
    DESTINATION share/${PROJECT_NAME}
  )
endif()

# 导出依赖
ament_export_include_directories(include)
ament_export_dependencies(
  rclcpp
  rclpy
  std_msgs
  geometry_msgs
  mavros_msgs
  msg_tool
  tf2
  tf2_ros
  tf2_geometry_msgs
  nav_msgs
  Eigen3
)

ament_package()
