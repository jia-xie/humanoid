import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import math

class JointStatePublisher(Node):
    def __init__(self):
        super().__init__('joint_state_publisher')
        self.publisher_ = self.create_publisher(JointState, '/joint_states', 10)
        self.timer = self.create_timer(0.1, self.publish_joint_state)  # Publish at 10 Hz

        # Initialize yaw positions for spinning
        self.yaw_angle = 0.0
        self.yaw_increment = 0.1  # Adjust increment for spin speed (in radians per update)

    def publish_joint_state(self):
        joint_state = JointState()
        joint_state.header.stamp = self.get_clock().now().to_msg()

        # Increment yaw angles for a spinning effect
        self.yaw_angle += self.yaw_increment
        self.yaw_angle = self.yaw_angle % (2 * math.pi)  # Keep angle within [0, 2π]

        # Define joint names and positions
        joint_state.name = [
            'left_hip_yaw_joint', 'left_hip_roll_joint', 'left_hip_pitch_joint',
            'left_knee_joint', 'left_ankle_joint',
            'right_hip_yaw_joint', 'right_hip_roll_joint', 'right_hip_pitch_joint',
            'right_knee_joint', 'right_ankle_joint'
        ]
        joint_state.position = [
            self.yaw_angle, 0.0, 0.0, 0.0, 0.0,   # Left side joints
            self.yaw_angle, 0.0, 0.0, 0.0, 0.0    # Right side joints
        ]
        joint_state.velocity = []
        joint_state.effort = []

        # Publish the joint states
        self.publisher_.publish(joint_state)
        self.get_logger().info(f"Published joint states with yaw angle: {self.yaw_angle:.2f}")

def main(args=None):
    rclpy.init(args=args)
    node = JointStatePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
