cmake_minimum_required(VERSION 3.5)
project(action_output)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)

install(PROGRAMS
  src/action_output/action_output_node.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
