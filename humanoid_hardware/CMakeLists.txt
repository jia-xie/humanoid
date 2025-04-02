cmake_minimum_required(VERSION 3.5)
project(humanoid_hardware)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(humanoid_interfaces REQUIRED)
find_package(std_msgs REQUIRED)

include_directories(include)

add_executable(MotorCtrlNode
    src/MotorCtrlNode.cpp
    src/DaMiaoMotor.cpp
)

add_executable(SerialNode
    src/SerialNode.cpp
)

add_executable(MotorAbstractionNode
    src/MotorAbstractionNode.cpp
)

ament_target_dependencies(MotorCtrlNode
    rclcpp
    sensor_msgs
    humanoid_interfaces
)

ament_target_dependencies(SerialNode
    rclcpp
    humanoid_interfaces
    std_msgs
)

ament_target_dependencies(MotorAbstractionNode
    rclcpp
    humanoid_interfaces
    sensor_msgs
)

# ---------------------------
# ROS 2 Control Plugin
# ---------------------------
add_library(humanoid_system_hardware SHARED
	src/HumanoidSystemHardware.cpp
)

ament_target_dependencies(humanoid_system_hardware
    rclcpp
    hardware_interface
    pluginlib
)

target_include_directories(humanoid_system_hardware PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

pluginlib_export_plugin_description_file(hardware_interface humanoid_hardware_plugins.xml)

install(TARGETS
    humanoid_system_hardware
    MotorCtrlNode
    SerialNode
    MotorAbstractionNode
    DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY script/
  DESTINATION share/${PROJECT_NAME}/script
)

ament_package()
