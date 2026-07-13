cmake_minimum_required(VERSION 3.8)
project(wheel_imu_ekf)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(mcu_odom_bridge_node
  src/mcu_odom_bridge_node.cpp
)

target_compile_features(mcu_odom_bridge_node PUBLIC cxx_std_17)

ament_target_dependencies(mcu_odom_bridge_node
  rclcpp
  nav_msgs
  geometry_msgs
  tf2
  tf2_ros
)

install(TARGETS mcu_odom_bridge_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(DIRECTORY rviz/
  DESTINATION share/${PROJECT_NAME}/rviz
)

ament_package()
