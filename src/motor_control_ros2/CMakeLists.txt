cmake_minimum_required(VERSION 3.8)
project(motor_control_ros2)

# 编译选项
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 查找依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(ament_index_cpp REQUIRED)

# 消息生成（不再使用服务）
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/MotorStateGeneric.msg"
  "msg/DJIMotorState.msg"
  "msg/DJIMotorCommand.msg"
  "msg/DJIMotorCommandAdvanced.msg"
  "msg/DamiaoMotorState.msg"
  "msg/DamiaoMotorCommand.msg"
  "msg/UnitreeMotorState.msg"
  "msg/UnitreeMotorCommand.msg"
  "msg/UnitreeGO8010State.msg"
  "msg/UnitreeGO8010Command.msg"
  "msg/ControlFrequency.msg"
  "msg/ChassisCommand.msg"
  "msg/ChassisState.msg"
  "msg/SteerWheelState.msg"
  DEPENDENCIES std_msgs sensor_msgs geometry_msgs
)

# 包含目录
include_directories(include)

# C++ 库 - 核心逻辑
add_library(motor_control_lib
  # 硬件层
  src/hardware/can_interface.cpp
  src/hardware/serial_interface.cpp
  src/hardware/hardware_manager.cpp
  # 电机驱动层
  src/dji_motor.cpp
  src/damiao_motor.cpp
  src/unitree_motor.cpp
  # 配置解析
  src/config_parser.cpp
  # 运动学
  src/steer_wheel_kinematics.cpp
  src/omni_wheel_kinematics.cpp
)

# 为库添加依赖
ament_target_dependencies(motor_control_lib
  rclcpp
  std_msgs
  sensor_msgs
)

# 导出库接口
rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(motor_control_lib "${cpp_typesupport_target}" yaml-cpp)

# C++ 可执行文件 - 实时控制节点
add_executable(motor_control_node
  src/motor_control_node.cpp
)

# 链接核心库
target_link_libraries(motor_control_node
  motor_control_lib
)

ament_target_dependencies(motor_control_node
  rclcpp
  std_msgs
  sensor_msgs
  ament_index_cpp
)

# C++ 可执行文件 - 电机监控节点
add_executable(motor_monitor_node
  src/motor_monitor_node.cpp
)

# 链接消息类型支持
target_link_libraries(motor_monitor_node
  "${cpp_typesupport_target}"
)

ament_target_dependencies(motor_monitor_node
  rclcpp
  std_msgs
  sensor_msgs
)

# C++ 可执行文件 - 底盘控制节点
add_executable(chassis_control_node
  src/chassis_control_node.cpp
)

# 链接核心库
target_link_libraries(chassis_control_node
  motor_control_lib
)

ament_target_dependencies(chassis_control_node
  rclcpp
  std_msgs
  geometry_msgs
  ament_index_cpp
)

# C++ 可执行文件 - 全向轮底盘控制节点
add_executable(omni_chassis_control_node
  src/omni_chassis_control_node.cpp
)

# 链接核心库
target_link_libraries(omni_chassis_control_node
  motor_control_lib
)

ament_target_dependencies(omni_chassis_control_node
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
  tf2
  tf2_geometry_msgs
  ament_index_cpp
)

# C++ 可执行文件 - 手柄控制节点
add_executable(joystick_control_node
  src/joystick_control_node.cpp
)

# 链接消息类型支持
target_link_libraries(joystick_control_node
  "${cpp_typesupport_target}"
)

ament_target_dependencies(joystick_control_node
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
)

# 安装目标
install(TARGETS
  motor_control_lib
  motor_control_node
  motor_monitor_node
  chassis_control_node
  omni_chassis_control_node
  joystick_control_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# 安装头文件
install(DIRECTORY include/
  DESTINATION include
)

# 安装配置文件
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
