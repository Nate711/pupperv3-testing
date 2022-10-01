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

find_package(pupper_interfaces)

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

add_library(basic_sim lib/basic_sim.cpp)
target_link_libraries(basic_sim mujoco::mujoco glfw)

add_library(mujoco_core lib/mujoco_core.cpp)
target_link_libraries(mujoco_core mujoco::mujoco glfw)

add_library(mujoco_node lib/mujoco_node.cpp)
target_link_libraries(mujoco_node mujoco_core)
ament_target_dependencies(mujoco_node rclcpp std_msgs sensor_msgs
                          pupper_interfaces)

add_executable(basic_sim_test src/basic_sim_test.cpp)
target_link_libraries(basic_sim_test basic_sim)

add_library(robot_emulator lib/robot_emulator.cpp)
target_link_libraries(robot_emulator basic_sim)

add_library(basic_sim_node lib/basic_sim_node.cpp)
target_link_libraries(basic_sim_node robot_emulator)
ament_target_dependencies(basic_sim_node rclcpp std_msgs sensor_msgs
                          rosgraph_msgs pupper_interfaces)

add_executable(basic_sim_node_test src/basic_sim_node_test.cpp)
target_link_libraries(basic_sim_node_test basic_sim_node)

add_executable(mujoco_node_test src/mujoco_node_test.cpp)
target_link_libraries(mujoco_node_test mujoco_node)

install(TARGETS basic DESTINATION lib/${PROJECT_NAME})
install(TARGETS basic_sim_test DESTINATION lib/${PROJECT_NAME})
install(TARGETS simulate DESTINATION lib/${PROJECT_NAME})
install(TARGETS basic_sim_node_test DESTINATION lib/${PROJECT_NAME})
install(TARGETS mujoco_node_test DESTINATION lib/${PROJECT_NAME})

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
