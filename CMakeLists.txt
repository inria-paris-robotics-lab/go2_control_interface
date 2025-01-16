cmake_minimum_required(VERSION 3.8)
project(go2_control_interface)

# Required dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})
ament_python_install_package(go2_utils)

# Install package.xml
install(FILES
  package.xml
  DESTINATION share/${PROJECT_NAME}/
)

# Install Python scripts
file(GLOB SCRIPTS scripts/*.py)
install(PROGRAMS ${SCRIPTS}
  DESTINATION lib/${PROJECT_NAME}/
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
install(PROGRAMS
  ${CMAKE_CURRENT_SOURCE_DIR}/go2_utils/autodetect_network_if.py
  ${CMAKE_CURRENT_SOURCE_DIR}/go2_utils/autoset_environment_dds.py
  ${CMAKE_CURRENT_SOURCE_DIR}/go2_utils/shutdown_sportsmode.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()