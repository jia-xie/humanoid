#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import torch
import numpy as np
import os


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

        # Set timer to run policy loop at 100Hz
        self.timer = self.create_timer(0.01, self.policy_loop)

        # Initialize observation placeholder
        self.observation_dim = 33
        self.last_actions = np.zeros(8)

    def get_observation(self):
        # TODO: Replace this placeholder with actual sensor data acquisition logic
        observation = np.concatenate([
            np.zeros(3),  # base_ang_vel
            np.zeros(3),  # projected_gravity
            np.zeros(3),  # velocity_commands
            np.zeros(8),  # joint_pos
            np.zeros(8),  # joint_vel
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