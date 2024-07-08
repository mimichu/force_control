cmake_minimum_required(VERSION 2.8.3)
project(force_control)

add_definitions(-std=c++17)

include_directories(
  include
  /usr/include/eigen3
)

find_library(YAMLLib yaml-cpp HINTS /usr/local/lib)
find_library(TIMER_LIB TimerLinux HINTS ${CMAKE_INSTALL_PREFIX}/lib/RobotUtilities)
find_library(RUT Utilities HINTS ${CMAKE_INSTALL_PREFIX}/lib/RobotUtilities)

add_library(FORCE_CONTROLLERS src/admittance_controller.cpp src/config_deserialize.cpp)
  target_link_libraries(FORCE_CONTROLLERS ${RUT} ${YAMLLib})

install(DIRECTORY include/
  DESTINATION ${CMAKE_INSTALL_PREFIX}/include/
)

install(TARGETS FORCE_CONTROLLERS
  DESTINATION ${CMAKE_INSTALL_PREFIX}/lib/
)