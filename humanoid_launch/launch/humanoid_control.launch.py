from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    humanoid_description_pkg = FindPackageShare("humanoid_description")

    return LaunchDescription([
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            parameters=[
                {
                    "robot_description": Command([
                        FindExecutable(name="xacro"),
                        " ",
                        PathJoinSubstitution([humanoid_description_pkg, "urdf", "humanoid.urdf"])
                    ])
                },
                PathJoinSubstitution([humanoid_description_pkg, "config", "motor_params.yaml"])
            ],
            output="screen"
        )
    ])
