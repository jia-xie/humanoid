from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os

def generate_launch_description():
    ld = LaunchDescription()

    # Declare the parameter file argument
    param_file_arg = DeclareLaunchArgument(
        'param_file',
        default_value='humanoid_hardware/config/motor_params.yaml',  # Update with correct path to YAML file
        description='Path to the parameter file for the motor control node'
    )

    # # Path to the fdcan_setup.sh script (commented out for now)
    # script_path = os.path.join(
    #     get_package_share_directory('humanoid_hardware'),
    #     'script',
    #     'fdcan_setup.sh'
    # )
    
    # # Execute the fdcan_setup.sh script before starting nodes
    # fdcan_setup_process = ExecuteProcess(
    #     cmd=['bash', script_path],
    #     name='fdcan_setup',
    #     output='screen'
    # )

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

    # Node to send motor commands and form joint_states for visualization
    control_node = Node(
        package="humanoid_hardware",
        executable="MotorAbstractionNode",
        name="motor_abstraction_node",
    )

    motor_abstraction_node = Node(
        package="humanoid_hardware",
        executable="MotorAbstractionNode",
        name="motor_abstraction_node",
    )

    # URDF Path for robot_state_publisher
    urdf_path = os.path.join(
        get_package_share_directory('humanoid_description'),
        'urdf',
        'humanoid.urdf'
    )

    # Node to publish the robot state
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        parameters=[{'robot_description': Command(['xacro ', urdf_path])}]
    )

    # Node for joint state publishing (for visualization)
    # joint_state_publisher_node = Node(
    #     package="joint_state_publisher",
    #     executable="joint_state_publisher",
    #     name="joint_state_publisher"
    # )

    # Add actions to the launch description
    ld.add_action(param_file_arg)
    ld.add_action(motor_ctrl_node)
    ld.add_action(motor_abstraction_node)
    ld.add_action(serial_node)
    ld.add_action(control_node)
    ld.add_action(robot_state_publisher_node)
    # ld.add_action(joint_state_publisher_node)
    
    return ld
