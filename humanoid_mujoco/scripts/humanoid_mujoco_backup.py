#!/usr/bin/env python3

import time

import mujoco
import mujoco.viewer
import numpy as np
import matplotlib.pyplot as plt
import torch
import os

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import Float64MultiArray
from humanoid_mujoco.scripts.humanoid_wrapper import Humanoid_Wrapper 

class MujocoNode(Node):
    def __init__(self):
        super().__init__('mujoco_node')

        # Load policy model
        model_path = os.path.join('/home/purduerm/dev/self.humanoid/humanoid_control/models/policy.pt')
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
        self.self.humanoid = Humanoid_Wrapper()
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
        self.timer = self.create_timer(0.01, self.mujoco_loop)


    def mujoco_loop(self):
        # Initialize the viewer
        with mujoco.viewer.launch_passive(self.humanoid.model, self.humanoid.data) as viewer:
            
            start_time = time.time()
            step_count = 0
            print_interval = 1  # e.g., print every ~1 second
            print("MuJoCo simulation time step:", self.humanoid.model.opt.timestep)
        
            while viewer.is_running() and time.time() - start_time < 300:
                step_start = time.time()

                # # --- Sine Wave Desired Position and Velocity ---
                # t = time.time() - start_time
                # qpos_des = amplitude * np.sin(2 * np.pi * frequency * t)
                # qpos_des[1] = sinestuff
                # qpos_des[6] = sinestuff
                # qpos_des_history.append(qpos_des.copy())
                # time_history.append(t)

                # # qvel_des = 2 * np.pi * frequency * amplitude * np.cos(2 * np.pi * frequency * t + phases)


                # # --- PD Control ---
                # pos_error = qpos_des - self.humanoid.data.qpos
                # vel_error = qvel_des - self.humanoid.data.qvel
                # torque = Kp * pos_error + Kd * vel_error

                # # Apply torque (must match number of actuators)
                # self.humanoid.data.ctrl[:] = torque[:self.humanoid.model.nu]  # self.humanoid.model.nu is number of actuators
                observation = self.humanoid.get_observation()
                
                obs_tensor = torch.tensor(observation, dtype=torch.float32).unsqueeze(0)

                # Get action from policy model
                with torch.no_grad():
                    action_tensor = self.humanoid.policy(obs_tensor)
        
                action = action_tensor.cpu().numpy().flatten()
        
                # # Update last actions
                self.humanoid.last_actions = action

                self.humanoid.data.ctrl[0:8] = action
                mujoco.mj_step(self.humanoid.model, self.humanoid.data)

                if step_count % print_interval == 0:
                    print("Joint states at t =", self.humanoid.data.time)
                    for i in range(1):
                        name = self.humanoid.model.joint(i).name
                        pos = self.humanoid.data.qpos[i]
                        vel = self.humanoid.data.qvel[i]
                        print(f"Joint {name}: pos = {pos}, vel = {vel}")

                step_count += 1

                with viewer.lock():
                    viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTPOINT] = int(self.humanoid.data.time % 2)
                viewer.sync()
                time_until_next_step = self.humanoid.model.opt.timestep - (time.time() - step_start)
                if time_until_next_step > 0:
                    time.sleep(time_until_next_step)


def main(args=None):
    rclpy.init(args=args)
    mujoco_node = MujocoNode()
    rclpy.spin(mujoco_node)
    mujoco_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()