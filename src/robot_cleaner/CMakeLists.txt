cmake_minimum_required(VERSION 3.8)
project(robot_cleaner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(turtlesim REQUIRED)

add_executable(move src/move.cpp)
ament_target_dependencies(move rclcpp geometry_msgs)

add_executable(rotate src/rotate.cpp)
ament_target_dependencies(rotate rclcpp geometry_msgs)

add_executable(goto src/gotogoal.cpp)
ament_target_dependencies(goto rclcpp geometry_msgs turtlesim)

install(TARGETS
  move
  rotate
  goto
  DESTINATION lib/${PROJECT_NAME})

ament_package()
