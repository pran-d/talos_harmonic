import mujoco
import mujoco.viewer
import time
import numpy as np
from utils.load_indices import TalosIndices
from controllers.joint_pd import PDController
from controllers.ocp import crocoddylOCP

import sys

model_path = "/home/pdebbad/resources/mujoco_menagerie/pal_talos/scene_motor.xml" 
model = mujoco.MjModel.from_xml_path(model_path)
data = mujoco.MjData(model)

model.opt.timestep = 0.002
dt = model.opt.timestep

step_count = 0

talosIndices = TalosIndices()
jpos_indices, jvel_indices, ctrl_indices = talosIndices.load_mj_indices_list()

jlc_ref_pos = np.array(model.qpos0[jpos_indices])
jlc_ref_vel = np.zeros_like(data.qvel[jvel_indices])

# controller = PDController()
controller = crocoddylOCP()

# Creating the initial state
ocp_qpos0 = model.qpos0[[*range(7),*jpos_indices]]
ocp_qpos0[[3,4,5,6]] = ocp_qpos0[[4,5,6,3]]
ocp_qvel0 = data.qvel[[*range(6),*jvel_indices]]
init_state = controller.createState(ocp_qpos0, ocp_qvel0)

# Creating the action model
action_model = controller.createActionModel()

# Creating the shooting problem
controller.createProblem(action_model, init_state, 0.002, 20)
solved, _, _ = controller.solveProblem()
torques = controller.doControl()

with mujoco.viewer.launch_passive(model, data) as viewer:

    start_time_real = time.time()
    start_time_sim = data.time

    while viewer.is_running():

        time_elapsed = time.time() - start_time_real

        data.ctrl = list(torques)
        # print(solved,torques)

        while(time_elapsed - (data.time-start_time_sim) >= 0):
            mujoco.mj_step(model, data)

        ######## begin: CONTROLLER SPECIFIC CODE ###########

        ###### PD Controller ######
        # torques = controller.doControl(
        # np.array(data.qpos[jpos_indices]),
        # np.array(data.qvel[jvel_indices]),
        # jlc_ref_pos,
        # jlc_ref_vel)

        ###### OCP Crocoddyl Controller ######
        solved, _, _ = controller.solveProblem()
        torques = controller.doControl()
        
        ######## end: CONTROLLER SPECIFIC CODE ###########

        if step_count % 10 == 0:
            viewer.sync()
            print((data.time-start_time_sim), ", ", (time.time()-start_time_real))

        prev_time_real = time.time()
        prev_time_sim = data.time

        step_count += 1
