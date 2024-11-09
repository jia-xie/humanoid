cmake_minimum_required(VERSION 3.10)

# Project name and version
project(CANMotorControl VERSION 1.0)

# Set C++ standard
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Source files
set(SOURCES
    can_communication.cpp
    DaMiaoMotor.cpp
)

# Include directories
include_directories(${PROJECT_SOURCE_DIR})

# Add executable target
add_executable(CANMotorControl ${SOURCES})

# Link libraries
# This links to the socketcan library required for Linux CAN communication
target_link_libraries(CANMotorControl PRIVATE pthread rt)

# # Include additional Linux libraries needed for CAN
# target_link_libraries(CANMotorControl PRIVATE can)