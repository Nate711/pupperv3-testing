cmake_minimum_required(VERSION 3.8)
project(pupper_mujoco)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(mujoco REQUIRED)
find_package(glfw3 REQUIRED)


# DOES NOT WORK
# LINKER CANNOT FIND libmujoco.so.2.2.1
# add_library(mujoco SHARED IMPORTED)
# set_target_properties(
#   mujoco
#   PROPERTIES IMPORTED_LOCATION
#              "$ENV{HOME}/.mujoco/mujoco-2.2.1/lib/libmujoco.so.2.2.1"
#              INTERFACE_INCLUDE_DIRECTORIES
#              "$ENV{HOME}/.mujoco/mujoco-2.2.1/include")
# target_link_libraries(basic mujoco glfw)


# uncomment the following section in order to fill in further dependencies
# manually. find_package(<dependency> REQUIRED)

add_executable(simulate src/simulate.cc src/uitools.c)
target_link_libraries(simulate mujoco::mujoco glfw)

add_executable(basic src/basic.cc)
target_link_libraries(basic mujoco::mujoco glfw)

add_executable(basic_sim src/mujoco_basic_sim.cpp)
target_link_libraries(basic_sim mujoco::mujoco glfw)

install(TARGETS basic DESTINATION lib/${PROJECT_NAME})
install(TARGETS basic_sim DESTINATION lib/${PROJECT_NAME})
install(TARGETS simulate DESTINATION lib/${PROJECT_NAME})

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
