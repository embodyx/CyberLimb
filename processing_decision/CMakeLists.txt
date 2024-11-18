cmake_minimum_required(VERSION 3.5)
project(processing_decision)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install the node script
install(PROGRAMS
  src/processing_decision/processing_decision_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install additional package resources (if needed)
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
