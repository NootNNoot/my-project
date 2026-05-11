cmake_minimum_required(VERSION 3.8)
project(double_pendulum_gz_plugins)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake REQUIRED)

# Gazebo / GZ
find_package(gz-cmake3 REQUIRED)

gz_find_package(gz-plugin2 REQUIRED COMPONENTS register)
gz_find_package(gz-sim8 REQUIRED)
gz_find_package(gz-transport13 REQUIRED)
gz_find_package(gz-msgs10 REQUIRED)
gz_find_package(sdformat14 REQUIRED)

add_library(reset_joints_plugin SHARED
  src/reset_joints_plugin.cpp
)

target_include_directories(reset_joints_plugin PRIVATE
  ${SDFormat_INCLUDE_DIRS}
)

target_link_libraries(reset_joints_plugin
  gz-plugin2::gz-plugin2
  gz-sim8::gz-sim8
  gz-transport13::gz-transport13
  gz-msgs10::gz-msgs10
  ${SDFormat_LIBRARIES}
)

install(
  TARGETS reset_joints_plugin
  LIBRARY DESTINATION lib
)

# Optional but recommended: export plugin path when workspace is sourced
install(
  FILES hooks/${PROJECT_NAME}.dsv.in
  DESTINATION share/${PROJECT_NAME}/hook
)

ament_package()