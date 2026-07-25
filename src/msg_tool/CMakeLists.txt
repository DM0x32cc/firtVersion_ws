cmake_minimum_required(VERSION 3.8)
project(msg_tool)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}#这是我们定义自己的数据类型的关键！！！
  "msg/Color.msg"
  "msg/ColorDetections.msg"
  "msg/Line.msg"
  "msg/Pole.msg"
  "msg/PoleDetections.msg"
  "msg/Detect.msg"
  "msg/FrontDetection.msg"
  "msg/FlightInfo.msg"
  DEPENDENCIES std_msgs
)
# ament_export_dependencies(rosidl_default_runtime)#这是ai让我加的

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
