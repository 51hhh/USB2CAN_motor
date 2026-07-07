cmake_minimum_required(VERSION 3.8)
project(rviz_bag_tools)

find_package(ament_cmake REQUIRED)

install(PROGRAMS
  scripts/bag_record.py
  scripts/interactive_bag_player.py
  scripts/live_odom_adapter.py
  scripts/start_live_rviz.sh
  scripts/start_replay_rviz.sh
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  config
  launch
  rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
