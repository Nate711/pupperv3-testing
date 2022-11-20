cmake_minimum_required(VERSION 3.8)
project(pupper_mujoco)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

add_compile_options(-g)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(mujoco REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(pupper_interfaces)
find_package(builtin_interfaces REQUIRED)

include_directories(${CMAKE_SOURCE_DIR}/lib)

# DOES NOT WORK LINKER CANNOT FIND libmujoco.so.2.2.1 add_library(mujoco SHARED
# IMPORTED) set_target_properties( mujoco PROPERTIES IMPORTED_LOCATION
# "$ENV{HOME}/.mujoco/mujoco-2.2.1/lib/libmujoco.so.2.2.1"
# INTERFACE_INCLUDE_DIRECTORIES "$ENV{HOME}/.mujoco/mujoco-2.2.1/include")
# target_link_libraries(basic mujoco glfw)

# uncomment the following section in order to fill in further dependencies
# manually. find_package(<dependency> REQUIRED)

add_executable(simulate src/simulate.cc src/uitools.c)
target_link_libraries(simulate mujoco::mujoco glfw)

add_executable(basic src/basic.cc)
target_link_libraries(basic mujoco::mujoco glfw)

add_library(mujoco_core lib/mujoco_core.cpp)
target_link_libraries(mujoco_core mujoco::mujoco glfw)

add_library(mujoco_node lib/mujoco_node.cpp)
target_link_libraries(mujoco_node mujoco_core)
ament_target_dependencies(
  mujoco_node
  rclcpp
  std_msgs
  sensor_msgs
  pupper_interfaces
  tf2
  tf2_ros
  builtin_interfaces)

add_library(mujoco_interactive_node lib/mujoco_interactive_node.cpp)
target_link_libraries(mujoco_interactive_node mujoco::mujoco glfw)
ament_target_dependencies(
  mujoco_interactive_node
  rclcpp
  std_msgs
  sensor_msgs
  pupper_interfaces
  tf2
  tf2_ros
  builtin_interfaces
)

add_executable(mujoco_core_test src/mujoco_core_test.cpp)
target_link_libraries(mujoco_core_test mujoco_core)

add_executable(mujoco_node_test src/mujoco_node_test.cpp)
target_link_libraries(mujoco_node_test mujoco_node)

add_executable(mujoco_interactive_test src/mujoco_interactive_test.cpp)
target_link_libraries(mujoco_interactive_test mujoco_interactive_node)

install(TARGETS basic DESTINATION lib/${PROJECT_NAME})
install(TARGETS simulate DESTINATION lib/${PROJECT_NAME})
install(TARGETS mujoco_core_test DESTINATION lib/${PROJECT_NAME})
install(TARGETS mujoco_node_test DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY test/launch DESTINATION share/${PROJECT_NAME}/)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  find_package(ros_testing REQUIRED)

  # the following lines skip linters
  set(ament_cmake_cppcheck_FOUND TRUE)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_flake8_FOUND TRUE)
  set(ament_cmake_uncrustify_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  # Run all lint tests in package.xml except those listed above
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest_executable(basic_test test/basic_test.cpp)
  ament_target_dependencies(basic_test rclcpp)
  add_ros_test(test/launch/basic_test.test.py TIMEOUT 120 ARGS
               "test_binary_dir:=${CMAKE_CURRENT_BINARY_DIR}")
endif()

ament_package()
