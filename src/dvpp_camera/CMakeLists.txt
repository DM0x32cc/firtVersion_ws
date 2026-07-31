cmake_minimum_required(VERSION 3.8)
project(dvpp_camera)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

# CANN paths
set(CANN_ROOT /usr/local/Ascend/ascend-toolkit/latest)
include_directories(
  ${CANN_ROOT}/runtime/include
  ${CANN_ROOT}/aarch64-linux/include
)
link_directories(
  ${CANN_ROOT}/runtime/lib64
  ${CANN_ROOT}/aarch64-linux/lib64
)

add_executable(dvpp_camera_node src/dvpp_camera_node.cpp)
ament_target_dependencies(dvpp_camera_node
  rclcpp
  sensor_msgs
  cv_bridge
)
target_link_libraries(dvpp_camera_node
  ${OpenCV_LIBS}
  ascendcl
  acl_dvpp
)

install(TARGETS dvpp_camera_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
