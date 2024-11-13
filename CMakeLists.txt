cmake_minimum_required(VERSION 3.8)
project(go2_control_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
find_package(Eigen3 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(unitree_go REQUIRED)

include(GNUInstallDirs)

# Main library
add_library(${PROJECT_NAME} SHARED
  src/robot_controller.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)
target_link_libraries(${PROJECT_NAME} Eigen3::Eigen)
ament_target_dependencies(${PROJECT_NAME} unitree_go rclcpp)

install(TARGETS
  ${PROJECT_NAME}
  DESTINATION ${CMAKE_INSTALL_LIBDIR}
)

# Examples
add_executable(custom_damping_mode examples/custom_damping_mode.cpp)
target_link_libraries(custom_damping_mode Eigen3::Eigen ${PROJECT_NAME} rclcpp)

install(TARGETS
  custom_damping_mode
  DESTINATION ${CMAKE_INSTALL_EXECDIR}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
