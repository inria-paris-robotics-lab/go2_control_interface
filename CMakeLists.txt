cmake_minimum_required(VERSION 3.8)
project(go2_control_interface)

# Required dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)

# Install Python modules
ament_python_install_package(${PROJECT_NAME}_py)
ament_python_install_package(${PROJECT_NAME}_common)

# Install package.xml
install(FILES
  package.xml
  DESTINATION share/${PROJECT_NAME}/
)

# Install configuration files
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}/
)

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

# Declare executables for the Python console scripts
file(GLOB COMMON_SCRIPTS ${CMAKE_CURRENT_SOURCE_DIR}/go2_control_interface_common/*.py)
install(PROGRAMS
  ${COMMON_SCRIPTS}
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()