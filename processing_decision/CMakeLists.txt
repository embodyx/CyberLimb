cmake_minimum_required(VERSION 3.5)
project(processing_decision)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)

ament_python_install_package(
  src/processing_decision
)

install(PROGRAMS
  src/processing_decision/processing_decision_node.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
