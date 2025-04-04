import time

import mujoco
import mujoco.viewer
import numpy as np
import matplotlib.pyplot as plt
import torch
import os

class Humanoid_Wrapper:
    def __init__(self):
        self.model = mujoco.MjModel.from_xml_path('humanoid_mujoco/urdf/humanoid.xml')
        self.data = mujoco.MjData(self.model)
        # Load policy model
        model_path = os.path.join('humanoid_control/models/policy.pt')
        self.policy = torch.jit.load(model_path)
        self.policy.eval()  # set to evaluation mode
        print('Policy loaded')

        # --- PD Controller Parameters ---
        self.Kp = 20.0
        self.Kd = 0.05
        
        # Target joint positions (same size as qpos)
        self.qpos_des = np.zeros_like(self.data.qpos)
        self.qvel_des = np.zeros_like(self.data.qvel)


def main():
    # Initialize the humanoid wrapper
    humanoid = Humanoid_Wrapper()
    # Initialize the viewer
    with mujoco.viewer.launch_passive(humanoid.model, humanoid.data) as viewer:
        
        start_time = time.time()
        step_count = 0
        print_interval = 1  # e.g., print every ~1 second
        print("MuJoCo simulation time step:", humanoid.model.opt.timestep)
        while viewer.is_running() and time.time() - start_time < 300:
            step_start = time.time()

            # # --- Sine Wave Desired Position and Velocity ---
            # t = time.time() - start_time
            # qpos_des = amplitude * np.sin(2 * np.pi * frequency * t)
            # qpos_des_history.append(qpos_des.copy())
            # time_history.append(t)

            # # qvel_des = 2 * np.pi * frequency * amplitude * np.cos(2 * np.pi * frequency * t + phases)


            # # --- PD Control ---
            # pos_error = qpos_des - humanoid.data.qpos
            # vel_error = qvel_des - humanoid.data.qvel
            # torque = Kp * pos_error + Kd * vel_error

            # # Apply torque (must match number of actuators)
            # humanoid.data.ctrl[:] = torque[:humanoid.model.nu]  # humanoid.model.nu is number of actuators


            mujoco.mj_step(humanoid.model, humanoid.data)

            if step_count % print_interval == 0:
                print("Joint states at t =", humanoid.data.time)
                for i in range(1):
                    name = humanoid.model.joint(i).name
                    pos = humanoid.data.qpos[i]
                    vel = humanoid.data.qvel[i]
                    print(f"Joint {name}: pos = {pos}, vel = {vel}")

            step_count += 1

            with viewer.lock():
                viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTPOINT] = int(humanoid.data.time % 2)
            viewer.sync()
            time_until_next_step = humanoid.model.opt.timestep - (time.time() - step_start)
            if time_until_next_step > 0:
                time.sleep(time_until_next_step)

if __name__ == "__main__":
    main()