cmake_minimum_required(VERSION 3.5)
project(dummy_odom)
find_package(ament_cmake REQUIRED)
install(PROGRAMS
  dummy_odom/dummy_odom_node.py
  DESTINATION lib/${PROJECT_NAME}
)
ament_package()
install(PROGRAMS
  scripts/initial_pose_publisher.py
  DESTINATION lib/${PROJECT_NAME}
)
