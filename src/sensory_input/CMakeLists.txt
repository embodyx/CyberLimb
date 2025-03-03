cmake_minimum_required(VERSION 3.5)
project(sensory_input)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

install(PROGRAMS
  src/sensory_input/sensory_input_node.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
