cmake_minimum_required(VERSION 3.8)
project(double_pendulum_commander)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(double_pendulum_interfaces REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(test_moveit src/test_moveit.cpp)
add_executable(commander_template src/commander_template.cpp)

ament_target_dependencies(test_moveit
  rclcpp
  moveit_ros_planning_interface
)

ament_target_dependencies(commander_template
  rclcpp
  moveit_ros_planning_interface
  example_interfaces
  double_pendulum_interfaces
  std_msgs
)

install(TARGETS
  test_moveit
  commander_template
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()