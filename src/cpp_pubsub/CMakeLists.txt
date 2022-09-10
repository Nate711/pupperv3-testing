cmake_minimum_required(VERSION 3.8)
project(cpp_pubsub)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# Libraries
include_directories(${CMAKE_SOURCE_DIR}/lib)
add_library(motor_interface lib/motor_interface.cpp)

add_library(motor_controller lib/motor_controller.cpp)
target_link_libraries(motor_controller motor_interface)

# Executables
add_executable(talker src/joint_state_publisher.cpp)
target_link_libraries(talker motor_interface)
ament_target_dependencies(talker rclcpp std_msgs sensor_msgs)

add_executable(motor_controller_node src/motor_controller_node.cpp)
target_link_libraries(motor_controller_node motor_controller)
ament_target_dependencies(motor_controller_node rclcpp std_msgs sensor_msgs)

add_executable(can_test src/can_test.cpp)
target_link_libraries(can_test motor_interface)

add_executable(pd_test src/pd_test.cpp)
target_link_libraries(pd_test motor_interface)

add_executable(motor_controller_test src/motor_controller_test.cpp)
target_link_libraries(motor_controller_test motor_controller)

install(TARGETS
	talker
	DESTINATION lib/${PROJECT_NAME})

install(TARGETS
    can_test
    DESTINATION lib/${PROJECT_NAME})

install(TARGETS
    pd_test
    DESTINATION lib/${PROJECT_NAME})
  
install(TARGETS
    motor_controller_test
    DESTINATION lib/${PROJECT_NAME})
  
install(TARGETS
    motor_controller_node
    DESTINATION lib/${PROJECT_NAME})

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
