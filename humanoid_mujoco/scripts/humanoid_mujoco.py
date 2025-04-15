#!/usr/bin/env python3

import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import mujoco
import mujoco.viewer
import numpy as np
import torch
import os
from scipy.spatial.transform import Rotation as R


# import humanoid_control.scripts.policy_node
class Humanoid_Wrapper(Node):
    def __init__(self):
        super().__init__('mujoco_node')
        self.model = mujoco.MjModel.from_xml_path('humanoid_mujoco/urdf/humanoid_scene.xml')
        self.data = mujoco.MjData(self.model)
        self.get_logger().info('init')
        # Load policy model
        model_path = os.path.join('humanoid_control/models/policy.pt')
        self.policy = torch.jit.load(model_path)
        self.policy.eval()  # set to evaluation mode
        self.get_logger().info('Policy loaded')

        # Initialize joint positions and velocities
        self.qpos_des = np.zeros_like(self.data.qpos)
        self.qvel_des = np.zeros_like(self.data.qvel)

        # Placeholder for last actions (to be updated during the simulation)
        self.actions = np.zeros(8)
        self.last_actions = np.zeros(8)
        # initial position of all joints are 0 0 0.5 1 0 0 0.5 1
        self.initial_joint_pos = np.array([0, 0, 0.5, 1, 0, 0, 0.5, 1])
        self.action_scale = 0.5

        # Initialize the angular velocity, joint positions, velocities, and other state variables
        self.base_ang_vel = np.zeros(3)  # Assuming 3D angular velocity (roll, pitch, yaw)
        self.joint_positions = self.data.qpos
        self.joint_velocities = self.data.qvel

        self.action_publisher = self.create_publisher(Float64MultiArray, 'action', 10)
        self.observation_publisher = self.create_publisher(Float64MultiArray, 'observation', 10)
        # Timer to update policy
        self.timer_period = 0.02  # 20 ms = 50 Hz
        self.timer = self.create_timer(self.timer_period, self.update_policy)

    def compute_projected_gravity_direction(self, quat):
        """
        Compute the projected gravity based on the roll, pitch, and yaw.
        This is a simplified version assuming gravity is in the z-direction.
        """
        # Convert quaternion to rotation matrix
        rot_world_to_body = R.from_quat(quat).as_matrix().T  # Inverse rotation

        # Use gravity from model (usually [0, 0, -9.81])
        gravity_world = np.array([0, 0, -1])  # Gravity vector in world frame

        # Project gravity into the robot base frame
        gravity_body = rot_world_to_body @ gravity_world
        return gravity_body

    def get_observation(self):
        # Extract the base angular velocity from the simulation data
        self.base_ang_vel = self.data.qvel[3:6]
        print(f"Base angular velocity: {self.data.qvel[3]:.3f}, {self.data.qvel[4]:.3f}, {self.data.qvel[5]:.3f}\n")

        # Extract joint positions and velocities
        self.joint_positions = np.concatenate([self.data.qpos[7:7+4], self.data.qpos[7+5:7+5+4]])
        self.joint_velocities = np.concatenate([self.data.qvel[6:6+4], self.data.qvel[6+5:6+5+4]])

        quat = self.data.qpos[3:7]

        # Compute the projected gravity
        projected_gravity = self.compute_projected_gravity_direction(quat)

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
    
    def update_policy(self):
        observation = self.get_observation()
            
        obs_tensor = torch.tensor(observation, dtype=torch.float32).unsqueeze(0)

        # Get action from policy model
        with torch.no_grad():
            action_tensor = self.policy(obs_tensor)
    
        self.actions = action_tensor.cpu().numpy().flatten()
    
        # # Update last actions
        self.last_actions = self.actions
        joint_ang = self.initial_joint_pos + self.action_scale * self.actions
        self.data.ctrl[0:4] = joint_ang[0:4]
        self.data.ctrl[5:9] = joint_ang[4:8]
        obs_msg = Float64MultiArray(data=observation.tolist())
        act_msg = Float64MultiArray(data=self.actions.tolist())
        self.observation_publisher.publish(obs_msg)
        self.action_publisher.publish(act_msg)
        self.get_logger().info('updating')

    def reset(self):
        self.data.qpos[0:3] = np.array([0, 0, 0.345])  # Initial position of the humanoid
        self.data.qpos[7:7+4] = self.initial_joint_pos[0:4]
        self.data.qpos[7+4] = -0.5
        self.data.qpos[7+5:7+5+4] = self.initial_joint_pos[4:8]
        self.data.qpos[7+5+4] = -0.5


        self.data.ctrl[0:0+4] = self.initial_joint_pos[0:4]
        self.data.ctrl[0+4] = -0.5
        self.data.ctrl[0+5:0+5+4] = self.initial_joint_pos[4:8]
        self.data.ctrl[0+5+4] = -0.5

        self.data.qvel[6:16] = np.zeros(10)

    
    def simulate(self):
        
        self.reset()
        mujoco.mj_forward(self.model, self.data)  # Recompute derived quantities

        with mujoco.viewer.launch_passive(self.model, self.data) as viewer:
            viewer.opt.frame = mujoco.mjtFrame.mjFRAME_BODY
            start_time = time.time()


            while viewer.is_running() and time.time() - start_time < 300:
                rclpy.spin_once(self, timeout_sec=0.00)
                step_start = time.time()
                
                mujoco.mj_step(self.model, self.data)

                with viewer.lock():
                    viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTPOINT] = int(self.data.time % 2)
                viewer.sync()
                time_until_next_step = self.model.opt.timestep - (time.time() - step_start)
                if time_until_next_step > 0:
                    time.sleep(time_until_next_step)

def main(args=None):
    rclpy.init(args=args)
    mujoco_node = Humanoid_Wrapper()
    mujoco_node.simulate()
    rclpy.spin(mujoco_node)
    mujoco_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()