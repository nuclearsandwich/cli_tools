cmake_minimum_required(VERSION 3.5)

project(cli_tools)

if(NOT WIN32)
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -Wextra")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -Wall -Wextra -Wpedantic")
endif()

find_package(ament_cmake REQUIRED)
find_package(c_utilities REQUIRED)
find_package(rclcpp REQUIRED)

add_executable(rostopic_list src/rostopic_list.cpp)
ament_target_dependencies(rostopic_list c_utilities rclcpp)

add_executable(rosnode_list src/rosnode_list.cpp)
ament_target_dependencies(rosnode_list c_utilities rclcpp)

install(TARGETS
  rostopic_list
  rosnode_list
  DESTINATION bin)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
