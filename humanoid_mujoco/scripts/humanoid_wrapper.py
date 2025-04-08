import mujoco
import numpy as np
import torch
import os

class Humanoid_Wrapper:
    def __init__(self):
        self.model = mujoco.MjModel.from_xml_path('humanoid_mujoco/urdf/humanoid_scene.xml')
        self.data = mujoco.MjData(self.model)
        
        # Load policy model
        model_path = os.path.join('humanoid_control/models/policy.pt')
        self.policy = torch.jit.load(model_path)
        self.policy.eval()  # set to evaluation mode
        print('Policy loaded')

        # Initialize joint positions and velocities
        self.qpos_des = np.zeros_like(self.data.qpos)
        self.qvel_des = np.zeros_like(self.data.qvel)

        # Placeholder for last actions (to be updated during the simulation)
        self.last_actions = np.zeros(8)
        
        # Initialize the angular velocity, joint positions, velocities, and other state variables
        self.base_ang_vel = np.zeros(3)  # Assuming 3D angular velocity (roll, pitch, yaw)
        self.joint_positions = self.data.qpos
        self.joint_velocities = self.data.qvel

    def compute_projected_gravity(self, roll, pitch, yaw):
        """
        Compute the projected gravity based on the roll, pitch, and yaw.
        This is a simplified version assuming gravity is in the z-direction.
        """
        R = self.rotation_matrix(roll, pitch, yaw)
        gravity = np.array([0, 0, -9.81])  # Gravity vector
        projected_gravity = R @ gravity  # Project gravity onto the robot frame
        return projected_gravity

    def rotation_matrix(self, roll, pitch, yaw):
        """
        Compute a rotation matrix from roll, pitch, and yaw (ZYX convention).
        """
        # Rotation matrices for each axis (assuming standard ZYX rotation order)
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
        # Combined rotation matrix
        return R_z @ R_y @ R_x

    def get_observation(self):
        # Extract the base angular velocity from the simulation data
        self.base_ang_vel = self.data.qvel[3:6]  # Assuming angular velocity is in qvel[3:6]

        # Extract joint positions and velocities
        self.joint_positions = self.data.qpos[0:8]
        self.joint_velocities = self.data.qvel[0:8]

        # TODO: You need to define roll, pitch, yaw, based on the robot's body orientation
        # For simplicity, we assume the roll, pitch, yaw come from the orientation of the base
        # e.g., from the first 3 positions of qpos
        roll, pitch, yaw = self.data.qpos[3], self.data.qpos[4], self.data.qpos[5]

        # Compute the projected gravity
        projected_gravity = self.compute_projected_gravity(roll, pitch, yaw)

        # Create the observation vector
        observation = np.concatenate([
            self.base_ang_vel,       # Base angular velocity (3D)
            projected_gravity,       # Projected gravity (3D)
            np.zeros(3),             # Placeholder for velocity commands (3D)
            self.joint_positions,    # Joint positions
            self.joint_velocities,   # Joint velocities
            self.last_actions        # Last actions (control inputs)
        ])
        
        return observation