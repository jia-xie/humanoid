from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    # Declare the parameter file argument
    param_file_arg = DeclareLaunchArgument(
        'param_file',
        default_value='humanoid_hardware/config/motor_params.yaml',  # Update this with the default path to your YAML file
        description='Path to the parameter file for the motor control node'
    )

    # Node to control the motor
    motor_ctrl_node = Node(
        package="humanoid_hardware",
        executable="MotorCtrlNode",
        name="motor_ctrl_node",
        parameters=[LaunchConfiguration('param_file')]
    )

    # Add the argument and node to the launch description
    ld.add_action(param_file_arg)
    ld.add_action(motor_ctrl_node)

    return ld
