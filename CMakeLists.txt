cmake_minimum_required(VERSION 2.8.3)
project(force_control)

add_definitions(-std=c++17)

include_directories(
  include
  /usr/include/eigen3
)

find_library(TIMER_LIB TimerLinux HINTS ${CMAKE_INSTALL_PREFIX}/lib/RobotUtilities)
find_library(RUT Utilities HINTS ${CMAKE_INSTALL_PREFIX}/lib/RobotUtilities)

add_library(force_control_controller src/force_control_controller.cpp)
target_link_libraries(force_control_controller ${RUT})

install(DIRECTORY include/
  DESTINATION ${CMAKE_INSTALL_PREFIX}/include/
)

install(TARGETS force_control_controller
  DESTINATION ${CMAKE_INSTALL_PREFIX}/lib/
)