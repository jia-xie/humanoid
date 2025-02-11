from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os

def generate_launch_description():
    ld = LaunchDescription()

    # Declare the parameter file argument
    param_file_arg = DeclareLaunchArgument(
        'param_file',
        default_value='humanoid_hardware/config/motor_params.yaml',  # Update this with the correct path to your YAML file
        description='Path to the parameter file for the motor control node'
    )


    # Node to control the motor
    motor_ctrl_node = Node(
        package="humanoid_hardware",
        executable="MotorCtrlNode",
        name="motor_ctrl_node",
        parameters=[LaunchConfiguration('param_file')]
    )

    # Node to read serial data from STM32
    serial_node = Node(
        package="humanoid_hardware",
        executable="SerialNode",
        name="serial_node",
    )

    # Node to send motor commands to humanoid_hardware and form joint_states for visualization
    control_node = Node(
        package="humanoid_control",
        executable="ControlNode",
        name="control_node",
    )

    # Add actions to the launch description
    ld.add_action(param_file_arg)
    ld.add_action(motor_ctrl_node)
    ld.add_action(serial_node)
    # ld.add_action(control_node)
    
    return ld
