cmake_minimum_required(VERSION 3.8)
project(cpp_pubsub)

# Default to C++20
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pupper_interfaces REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(gflags)
include_directories(${EIGEN3_INCLUDE_DIR})

# Libraries
include_directories(${CMAKE_SOURCE_DIR}/lib)

add_library(pupperv3 lib/motor_interface_generic.cpp lib/motor_controller_generic.cpp)
target_link_libraries(pupperv3 spdlog::spdlog_header_only)

# Executables
add_executable(interface_test src/interface_test.cpp)
target_link_libraries(interface_test pupperv3 gflags)

add_executable(serial_test src/serial_test.cc)
add_executable(serial_test_syscall src/serial_test_syscall.cc)


add_executable(motor_controller_generic_calibration_test src/motor_controller_generic_calibration_test.cpp)
target_link_libraries(motor_controller_generic_calibration_test pupperv3 gflags)

add_executable(motor_controller_generic_calibration_sequence_test src/motor_controller_generic_calibration_sequence_test.cpp)
target_link_libraries(motor_controller_generic_calibration_sequence_test pupperv3 gflags)

# Must update for generic interface
# add_executable(talker src/joint_state_publisher.cpp)
# target_link_libraries(talker motor_interface)
# ament_target_dependencies(talker rclcpp std_msgs sensor_msgs)

add_executable(motor_controller_generic_node_test src/motor_controller_generic_node_test.cpp lib/motor_controller_generic_node.cpp)
target_link_libraries(motor_controller_generic_node_test pupperv3 gflags)
ament_target_dependencies(motor_controller_generic_node_test rclcpp pupper_interfaces std_msgs sensor_msgs)

add_executable(test_rig src/motor_controller_generic_node_test_rig.cpp lib/motor_controller_generic_node.cpp)
target_link_libraries(test_rig pupperv3 gflags)
ament_target_dependencies(test_rig rclcpp pupper_interfaces std_msgs sensor_msgs)

add_executable(latency_test src/latency_test.cpp)
ament_target_dependencies(latency_test rclcpp sensor_msgs std_msgs pupper_interfaces)

# install(TARGETS talker DESTINATION lib/${PROJECT_NAME})
install(TARGETS motor_controller_generic_node_test DESTINATION lib/${PROJECT_NAME})
install(TARGETS test_rig DESTINATION lib/${PROJECT_NAME})
install(TARGETS interface_test DESTINATION lib/${PROJECT_NAME})
install(TARGETS motor_controller_generic_calibration_test DESTINATION lib/${PROJECT_NAME})
install(TARGETS motor_controller_generic_calibration_sequence_test DESTINATION lib/${PROJECT_NAME})
install(TARGETS latency_test DESTINATION lib/${PROJECT_NAME})
install(TARGETS serial_test DESTINATION lib/${PROJECT_NAME})
install(TARGETS serial_test_syscall DESTINATION lib/${PROJECT_NAME})


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  # the following line skips the linter which checks for copyrights comment the
  # line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo) comment the line
  # when this package is in a git repo and when a copyright and license is added
  # to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_cppcheck_FOUND TRUE)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_flake8_FOUND TRUE)
  set(ament_cmake_uncrustify_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  # Reporting failure still super broken
  ament_add_gtest_executable(basic_test test/test_motor_controller_node.cpp)

endif()

ament_package()
