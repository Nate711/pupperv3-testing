cmake_minimum_required(VERSION 3.8)
project(mock_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pupper_interfaces)

# uncomment the following section in order to fill in further dependencies
# manually. find_package(<dependency> REQUIRED)

add_executable(mock_pub src/mock_pub.cpp)
ament_target_dependencies(mock_pub rclcpp std_msgs pupper_interfaces)
install(TARGETS mock_pub DESTINATION lib/${PROJECT_NAME})

add_executable(zero_motors src/zero_motors.cpp)
ament_target_dependencies(zero_motors rclcpp std_msgs pupper_interfaces)
install(TARGETS zero_motors DESTINATION lib/${PROJECT_NAME})


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights comment the
  # line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo) comment the line
  # when this package is in a git repo and when a copyright and license is added
  # to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
