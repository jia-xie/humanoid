#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import torch
import numpy as np
import os
from rclpy.qos import QoSProfile


class PolicyNode(Node):
    def __init__(self):
        super().__init__('policy_node')

        # Load policy model
        model_path = os.path.join('/home/purduerm/dev/humanoid/humanoid_control/models/policy.pt')
        self.policy = torch.load(model_path)
        self.policy.eval()  # set to evaluation mode
        self.get_logger().info('Policy loaded')

        # Initialize publisher
        self.publisher_ = self.create_publisher(Float64MultiArray, 'policy_output', 10)
        self.gravity_publisher_ = self.create_publisher(Float64MultiArray, 'projected_gravity', 10)
        qos_profile = QoSProfile(depth=1)
        self.imu_subscriber = self.create_subscription(
            Float64MultiArray,
            '/imu',
            self.imu_callback,
            1
        )
            # joint_pos_publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
        # "/processed_joint_positions", rclcpp::SensorDataQoS());   
    # joint_vel_publisher_ = this->create_publisher<std_msgs::msg   ::Float64MultiArray>(
        # "/processed_joint_velocities", rclcpp::SensorDataQoS());  
        self.joint_positions_subscriber = self.create_subscription(
            Float64MultiArray,
            '/processed_joint_positions',
            self.joint_positions_callback,
            1
        )
        self.joint_velocity_subscriber = self.create_subscription(
            Float64MultiArray,
            '/processed_joint_velocities',
            self.joint_velocities_callback,
            1
        )

        # Set timer to run policy loop at 100Hz
        self.timer = self.create_timer(0.01, self.policy_loop)

        # Initialize observation placeholder
        self.observation_dim = 33
        self.base_ang_vel = np.zeros(3)
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.last_actions = np.zeros(8)
        self.joint_positions = np.zeros(8)
        self.joint_velocities = np.zeros(8)

    def imu_callback(self, msg):
        # Update base angular velocity
        self.base_ang_vel = np.array(msg.data[:3])

        # Update roll and pitch
        self.roll = msg.data[6]
        self.pitch = msg.data[7]
        self.yaw = msg.data[8]

    def joint_positions_callback(self, msg):
        self.joint_positions = np.array(msg.data)
        # self.get_logger().info(f'joint_positions: {self.joint_positions}')

    
    def joint_velocities_callback(self, msg):
        self.joint_velocities = np.array(msg.data)


    def compute_projected_gravity(self,roll, pitch, yaw):
        R_x = np.array([
            [1, 0, 0],
            [0, np.cos(roll), -np.sin(roll)],
            [0, np.sin(roll), np.cos(roll)]
        ])

        R_y = np.array([
            [np.cos(pitch), 0, np.sin(pitch)],
            [0, 1, 0],
            [-np.sin(pitch), 0, np.cos(pitch)]
        ])

        R_z = np.array([
            [np.cos(yaw), -np.sin(yaw), 0],
            [np.sin(yaw), np.cos(yaw), 0],
            [0, 0, 1]
        ])

        # Final rotation matrix from body to world
        R = R_z @ R_y @ R_x
        # Gravity vector in world frame
        g_world = np.array([0, 0, -1])

        # Project into body frame
        g_body = R.T @ g_world

        # self.get_logger().info(f'Projected gravity: {g_body}')
        return g_body


    def get_observation(self):
        # TODO: Replace this placeholder with actual sensor data acquisition logic
        projected_gravity = self.compute_projected_gravity(self.roll, self.pitch, self.yaw)
        observation = np.concatenate([
            self.base_ang_vel,  # base_ang_vel
            projected_gravity,  # projected_gravity
            np.zeros(3),  # velocity_commands
            self.joint_positions,  # joint_pos
            self.joint_velocities,  # joint_vel
            self.last_actions  # last_actions
        ])
        
        return observation

    def policy_loop(self):
        # Get current observation
        observation = self.get_observation()

        # Convert observation to torch tensor
        obs_tensor = torch.tensor(observation, dtype=torch.float32).unsqueeze(0)

        # Get action from policy model
        with torch.no_grad():
            action_tensor = self.policy(obs_tensor)

        action = action_tensor.cpu().numpy().flatten()

        # Update last actions
        self.last_actions = action

        # Publish action
        msg = Float64MultiArray()
        msg.data = action.tolist()
        self.publisher_.publish(msg)

        # Log published action
        self.get_logger().debug(f'Published action: {msg.data}')


def main(args=None):
    rclpy.init(args=args)
    policy_node = PolicyNode()
    rclpy.spin(policy_node)
    policy_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()