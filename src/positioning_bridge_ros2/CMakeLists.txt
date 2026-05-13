cmake_minimum_required(VERSION 3.8)
project(positioning_bridge_ros2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(ament_index_cpp REQUIRED)

include_directories(include)

add_library(positioning_bridge_lib
  src/serial_transport.cpp
  src/protocol_decoder.cpp
  src/time_sync_estimator.cpp
  src/positioning_bridge_node.cpp
)

target_compile_features(positioning_bridge_lib PUBLIC cxx_std_17)

ament_target_dependencies(positioning_bridge_lib
  rclcpp
  nav_msgs
  geometry_msgs
  std_msgs
  diagnostic_msgs
  tf2
  tf2_ros
  std_srvs
  ament_index_cpp
)

target_link_libraries(positioning_bridge_lib
  yaml-cpp
)

add_executable(positioning_bridge_node
  src/main.cpp
)

target_compile_features(positioning_bridge_node PUBLIC cxx_std_17)

target_link_libraries(positioning_bridge_node
  positioning_bridge_lib
)

ament_target_dependencies(positioning_bridge_node
  rclcpp
)

install(TARGETS
  positioning_bridge_lib
  positioning_bridge_node
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

install(FILES resource/${PROJECT_NAME}
  DESTINATION share/ament_index/resource_index/packages
)

ament_export_include_directories(include)
ament_export_libraries(positioning_bridge_lib)
ament_export_dependencies(
  rclcpp
  nav_msgs
  geometry_msgs
  diagnostic_msgs
  tf2
  tf2_ros
  std_srvs
  ament_index_cpp
)

ament_package()
