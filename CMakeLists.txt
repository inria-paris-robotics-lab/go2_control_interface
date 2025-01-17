cmake_minimum_required(VERSION 3.8)
project(go2_control_interface)


###################
# Python Packages #
###################
set(PY_PROJECT_NAME ${PROJECT_NAME}_py)
set(COMMON_PROJECT_NAME ${PROJECT_NAME}_common)

# Required dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)

# Install Python modules
ament_python_install_package(${PY_PROJECT_NAME})
ament_python_install_package(${COMMON_PROJECT_NAME})

###############
# Cpp Package #
###############
set(CPP_PROJECT_NAME ${PROJECT_NAME}_cpp)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(unitree_go REQUIRED)
find_package(std_msgs REQUIRED)

file(GLOB CPP_SRC ${CMAKE_CURRENT_SOURCE_DIR}/${CPP_PROJECT_NAME}/src/*)
add_library(${CPP_PROJECT_NAME} STATIC ${CPP_SRC})
ament_target_dependencies(${CPP_PROJECT_NAME} rclcpp unitree_go std_msgs)

target_include_directories(${CPP_PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/${CPP_PROJECT_NAME}/include>
  $<INSTALL_INTERFACE:include>
)

install(TARGETS
  ${CPP_PROJECT_NAME}
  DESTINATION lib/${CPP_PROJECT_NAME}
)

install(
  DIRECTORY ${CPP_PROJECT_NAME}/include/
  DESTINATION include/${CPP_PROJECT_NAME}
)

########################
# Global package files #
########################

install(FILES
  package.xml
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# Declare executables for the Python console scripts
file(GLOB COMMON_SCRIPTS ${CMAKE_CURRENT_SOURCE_DIR}/${COMMON_PROJECT_NAME}/*.py)
install(PROGRAMS
  ${COMMON_SCRIPTS}
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()