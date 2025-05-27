import mujoco
import mujoco.viewer
import time
import numpy as np
import sys

model_path = "/home/pdebbad/resources/mujoco_menagerie/pal_talos/scene_motor.xml" 
model = mujoco.MjModel.from_xml_path(model_path)
data = mujoco.MjData(model)

model.opt.timestep = 0.001
dt = model.opt.timestep

step_count = 0

#qpos indices for various joints
qpos_indices = {
"arm_left_joints": [11,12,13,14,15,16,17],
"arm_right_joints": [25,26,27,28,29,30,31],
"gripper_left_joint": [18],
"gripper_right_joint": [32],
"leg_left_joints": [39,40,41,42,43,44],
"leg_right_joints": [45,46,47,48,49,50],
"torso_joints": [7,8],
"head_joints": [9,10],
}

qvel_indices = {}
for key, value in qpos_indices.items():
    qvel_indices[key] = value - np.ones_like(value)

#ctrl indices for various joints
ctrl_indices = {
"torso_joints": [0,1],
"head_joints": [2,3],
"arm_left_joints": [4,5,6,7,8,9,10],
"gripper_left_joint": [11],
"arm_right_joints": [12,13,14,15,16,17,18],
"gripper_right_joint": [19],
"leg_left_joints": [20,21,22,23,24,25],
"leg_right_joints": [26,27,28,29,30,31],
}

qpos_indices_list = []
qvel_indices_list = []
for key, value in ctrl_indices.items():
    qpos_indices_list.extend(qpos_indices[key])
    qvel_indices_list.extend(qvel_indices[key])
ref_pos = np.array(model.qpos0[qpos_indices_list])
ref_vel = np.zeros_like(data.qvel[qvel_indices_list])
Kp = 100
Kd = 20

with mujoco.viewer.launch_passive(model, data) as viewer:

    start_time_real = time.time()
    start_time_sim = data.time

    while viewer.is_running():

        time_elapsed = time.time() - start_time_real
        
        while(time_elapsed - (data.time-start_time_sim) >= 0):
            mujoco.mj_step(model, data)

        torques = Kp * (ref_pos - np.array(data.qpos[qpos_indices_list])) + \
                Kd * (ref_vel - np.array(data.qvel[qvel_indices_list]))

        data.ctrl = list(torques)

        if step_count % 10 == 0:
            viewer.sync()
            print((data.time-start_time_sim), ", ", (time.time()-start_time_real))

        prev_time_real = time.time()
        prev_time_sim = data.time

        step_count += 1
