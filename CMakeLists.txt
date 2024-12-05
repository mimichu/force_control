cmake_minimum_required(VERSION 3.5)
project(force_control)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Add paths for dependencies
set(yaml-cpp_DIR "/usr/local/lib/cmake/yaml-cpp")

# Find required packages
find_package(Eigen3 3.4 REQUIRED)
find_package(yaml-cpp REQUIRED)

# Find RobotUtilities
find_library(RUT Utilities HINTS 
    /usr/local/lib/RobotUtilities
    ${CMAKE_INSTALL_PREFIX}/lib/RobotUtilities
)
if(NOT RUT)
    message(FATAL_ERROR "RobotUtilities library not found. Please install cpplibrary first.")
endif()

# Include directories
include_directories(
    include
    ${EIGEN3_INCLUDE_DIRS}
    /usr/local/include  # For RobotUtilities headers
)

# Build library
add_library(FORCE_CONTROLLERS SHARED 
    src/admittance_controller.cpp 
    src/config_deserialize.cpp
)

# Link dependencies
target_link_libraries(FORCE_CONTROLLERS
    ${RUT}
    yaml-cpp::yaml-cpp
    Eigen3::Eigen
)

# Installation
install(TARGETS FORCE_CONTROLLERS
    LIBRARY DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
    ARCHIVE DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
)

install(DIRECTORY include/
    DESTINATION ${CMAKE_INSTALL_PREFIX}/include
)
