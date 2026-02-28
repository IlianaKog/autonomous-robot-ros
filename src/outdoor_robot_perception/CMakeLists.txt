cmake_minimum_required(VERSION 3.8)
project(outdoor_robot_perception)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(OpenCV REQUIRED)

add_executable(inspection_behavior_node src/inspection_behavior_node.cpp)
target_include_directories(inspection_behavior_node PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(inspection_behavior_node ${OpenCV_LIBRARIES})
ament_target_dependencies(inspection_behavior_node
  rclcpp
  sensor_msgs
  cv_bridge
  image_transport
  geometry_msgs
)

install(TARGETS
  inspection_behavior_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
