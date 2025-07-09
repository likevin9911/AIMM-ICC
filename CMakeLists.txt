cmake_minimum_required(VERSION 3.8)
project(aimm_navigation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(robot_localization REQUIRED)
find_package(pointcloud_to_laserscan REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav2_core REQUIRED)
find_package(urdf REQUIRED)
find_package(xacro REQUIRED)
find_package(rclpy REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

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

# Install URDF files
install(
  DIRECTORY urdf/
  DESTINATION share/${PROJECT_NAME}/urdf
  FILES_MATCHING PATTERN "*.urdf"
)

# Install model meshes
install(
  DIRECTORY models/
  DESTINATION share/${PROJECT_NAME}/models
  FILES_MATCHING
    PATTERN "*.dae"
    PATTERN "*.stl"
    PATTERN "*.png"
    PATTERN "*.jpg"
)

# Install launch files
install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
  FILES_MATCHING PATTERN "*.launch.py"
)

# Install RViz config files
install(
  DIRECTORY rviz/
  DESTINATION share/${PROJECT_NAME}/rviz
  FILES_MATCHING PATTERN "*.rviz"
)

# Install any config or parameter YAML files
install(
  DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
  FILES_MATCHING
    PATTERN "*.yaml"
    PATTERN "*.yaml.in"
)

install(
  DIRECTORY maps/
  DESTINATION share/${PROJECT_NAME}/maps
  FILES_MATCHING PATTERN "*.yaml"
  PATTERN "*.pgm"
)

# If using Python scripts/nodes
# install(PROGRAMS
#   scripts/my_node.py
#   DESTINATION lib/${PROJECT_NAME}
# )



ament_package()
