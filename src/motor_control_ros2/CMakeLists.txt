cmake_minimum_required(VERSION 3.8)
project(motor_control_ros2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ControlFrequency.msg"
  "msg/DJIMotorCommand.msg"
  "msg/DJIMotorCommandAdvanced.msg"
  "msg/DJIMotorState.msg"
  DEPENDENCIES std_msgs
)

include_directories(include)

add_library(motor_control_lib
  src/hardware/can_interface.cpp
  src/hardware/config_parser.cpp
  src/kinematics/omni_wheel_kinematics.cpp
  src/motors/dji_motor.cpp
)

target_link_libraries(motor_control_lib yaml-cpp)

rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_executable(motor_control_node
  src/nodes/motor_control_node.cpp
)
target_link_libraries(motor_control_node
  motor_control_lib
  yaml-cpp
  "${cpp_typesupport_target}"
)
ament_target_dependencies(motor_control_node
  ament_index_cpp
  rclcpp
  std_msgs
)

add_executable(omni_chassis_control_node
  src/nodes/omni_chassis_control_node.cpp
)
target_link_libraries(omni_chassis_control_node
  motor_control_lib
  yaml-cpp
  "${cpp_typesupport_target}"
)
ament_target_dependencies(omni_chassis_control_node
  ament_index_cpp
  geometry_msgs
  nav_msgs
  rcl_interfaces
  rclcpp
  std_msgs
)

add_executable(rc_usb_control_node
  src/nodes/rc_usb_control_node.cpp
)
ament_target_dependencies(rc_usb_control_node
  geometry_msgs
  rclcpp
  std_msgs
)

add_executable(vision_goal_tracker_node
  src/nodes/vision_goal_tracker_node.cpp
)
ament_target_dependencies(vision_goal_tracker_node
  geometry_msgs
  nav_msgs
  rclcpp
)

install(TARGETS
  motor_control_lib
  motor_control_node
  omni_chassis_control_node
  rc_usb_control_node
  vision_goal_tracker_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
