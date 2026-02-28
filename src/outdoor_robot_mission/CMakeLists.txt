cmake_minimum_required(VERSION 3.8)
project(outdoor_robot_mission)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(robot_localization REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(gps_waypoint_follower src/gps_waypoint_follower.cpp)
ament_target_dependencies(gps_waypoint_follower
  rclcpp
  rclcpp_action
  nav2_msgs
  robot_localization
  geometry_msgs
)

install(TARGETS
  gps_waypoint_follower
  DESTINATION lib/${PROJECT_NAME}
)


ament_package()
