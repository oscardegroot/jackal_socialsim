cmake_minimum_required(VERSION 3.8)
project(jackal_socialsim)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)

# Install Python scripts
install(PROGRAMS
  scripts/ros2_condition_check.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})
  
install(DIRECTORY scenarios
  DESTINATION share/${PROJECT_NAME})
# Export package information
ament_export_dependencies()
ament_package()
