import time

import mujoco
import mujoco.viewer
import numpy as np
import matplotlib.pyplot as plt
qpos_des_history = []
time_history = []



m = mujoco.MjModel.from_xml_path('humanoid_mujoco/urdf/humanoid.xml')
d = mujoco.MjData(m)

# --- PD Controller Parameters ---
Kp = 20.0
Kd = 0.01

# Target joint positions (same size as qpos)
qpos_des = np.zeros_like(d.qpos)
qvel_des = np.zeros_like(d.qvel)

# Sine wave parameters
amplitude = 0.5  # radians
frequency = 0.5  # Hz

# Phase offset for each joint (optional: make it a traveling wave)
phases = np.linspace(0, 2 * np.pi, m.nq, endpoint=False)

with mujoco.viewer.launch_passive(m, d) as viewer:
    start_time = time.time()
    step_count = 0
    print_interval = 1  # e.g., print every ~1 second
    print("MuJoCo simulation time step:", m.opt.timestep)
    while viewer.is_running() and time.time() - start_time < 300:
        step_start = time.time()

        # --- Sine Wave Desired Position and Velocity ---
        t = time.time() - start_time
        qpos_des = amplitude * np.sin(2 * np.pi * frequency * t)
        qpos_des_history.append(qpos_des.copy())
        time_history.append(t)

        # qvel_des = 2 * np.pi * frequency * amplitude * np.cos(2 * np.pi * frequency * t + phases)


        # --- PD Control ---
        pos_error = qpos_des - d.qpos
        vel_error = qvel_des - d.qvel
        torque = Kp * pos_error + Kd * vel_error

        # Apply torque (must match number of actuators)
        d.ctrl[:] = torque[:m.nu]  # m.nu is number of actuators


        mujoco.mj_step(m, d)

        if step_count % print_interval == 0:
            print("Joint states at t =", d.time)
            for i in range(1):
                name = m.joint(i).name
                pos = d.qpos[i]
                vel = d.qvel[i]
                print(f"torq={torque[0]:.4f}")

        step_count += 1

        with viewer.lock():
            viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTPOINT] = int(d.time % 2)
        viewer.sync()
        time_until_next_step = m.opt.timestep - (time.time() - step_start)
        if time_until_next_step > 0:
            time.sleep(time_until_next_step)