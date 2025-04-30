cmake_minimum_required(VERSION 3.8)
project(camera_detector)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(yolo_msgs REQUIRED)
find_package(people_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

# Include Cpp "include" directory
include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
)

# Cpp executable
add_executable(projection_3D_node src/projection_3D_node.cpp)
ament_target_dependencies(projection_3D_node rclcpp tf2_ros tf2_geometry_msgs message_filters std_msgs geometry_msgs visualization_msgs yolo_msgs people_msgs cv_bridge)

add_executable(compute_human_pos_node src/compute_human_pos_node.cpp)
ament_target_dependencies(compute_human_pos_node rclcpp tf2_ros tf2_geometry_msgs message_filters std_msgs geometry_msgs visualization_msgs yolo_msgs people_msgs cv_bridge)

add_executable(visualize_human src/visualize_human.cpp)
ament_target_dependencies(visualize_human rclcpp tf2_ros tf2_geometry_msgs message_filters std_msgs geometry_msgs visualization_msgs yolo_msgs people_msgs cv_bridge)

# Install Cpp executables
install(TARGETS
  projection_3D_node
  compute_human_pos_node
  visualize_human
  DESTINATION lib/${PROJECT_NAME}
)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  scripts/track_2D_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install config file and launch file
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY
  params
  DESTINATION share/${PROJECT_NAME}
)

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
