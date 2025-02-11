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
find_package(sensor_msgs)
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

install (
    TARGETS
    MotorCtrlNode
    SerialNode
    MotorAbstractionNode
    DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY script/
  DESTINATION share/${PROJECT_NAME}/script
)

ament_package()
