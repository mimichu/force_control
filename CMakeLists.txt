cmake_minimum_required(VERSION 2.8.3)
project(force_control)

add_definitions(-std=c++17)

# find_package(catkin REQUIRED COMPONENTS
#   roscpp
#   hardware_interfaces
# )

# find_package(Boost REQUIRED COMPONENTS
#   system
#   thread
#   program_options
# )

include_directories(
  include
  /usr/include/eigen3
#   ${Boost_INCLUDE_DIRS}
)

find_library(TIMER_LIB TimerLinux HINTS ${CMAKE_INSTALL_PREFIX}/lib/RobotUtilities)
find_library(RUT Utilities HINTS ${CMAKE_INSTALL_PREFIX}/lib/RobotUtilities)

###########
## Build ##
###########

add_library(force_control_controller src/force_control_controller.cpp)
target_link_libraries(force_control_controller ${RUT})


##
## Test (optional)
##
# add_subdirectory(test)
