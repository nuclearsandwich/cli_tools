cmake_minimum_required(VERSION 3.5)

project(cli_tools)

if(NOT WIN32)
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -Wextra")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -Wall -Wextra")
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

add_executable(rostopic_list src/rostopic_list.cpp)
ament_target_dependencies(rostopic_list rclcpp)

install(TARGETS
  rostopic_list
  DESTINATION bin)

ament_package()
