import numpy as np

class TalosIndices():

    def __init__(self):
        #qpos indices for various joints
        self.qpos_indices = {
        "arm_left_joints": [11,12,13,14,15,16,17],
        "arm_right_joints": [25,26,27,28,29,30,31],
        "gripper_left_joint": [18],
        "gripper_right_joint": [32],
        "leg_left_joints": [39,40,41,42,43,44],
        "leg_right_joints": [45,46,47,48,49,50],
        "torso_joints": [7,8],
        "head_joints": [9,10],
        }

        self.qvel_indices = {}
        for key, value in self.qpos_indices.items():
            self.qvel_indices[key] = value - np.ones_like(value)

        #ctrl indices for various joints
        self.ctrl_indices = {
        "torso_joints": [0,1],
        "head_joints": [2,3],
        "arm_left_joints": [4,5,6,7,8,9,10],
        "gripper_left_joint": [11],
        "arm_right_joints": [12,13,14,15,16,17,18],
        "gripper_right_joint": [19],
        "leg_left_joints": [20,21,22,23,24,25],
        "leg_right_joints": [26,27,28,29,30,31],
        }

        self.ocp_indices = {
            "head_joints": [30,31],
            "torso_joints": [12,13],
            "arm_left_joints": [14,15,16,17,18,19,20],
            "gripper_left_joint": [21],
            "arm_right_joints": [22,23,24,25,26,27,28],
            "gripper_right_joint": [29],
            "leg_left_joints": [0,1,2,3,4,5],
            "leg_right_joints": [6,7,8,9,10,11],
        }

        self.qpos_indices_list = []
        self.qvel_indices_list = []
        self.ctrl_indices_list = []
        self.ocp_indices_list = []
        for key, value in self.ctrl_indices.items():
            self.qpos_indices_list.extend(self.qpos_indices[key])
            self.qvel_indices_list.extend(self.qvel_indices[key])
            self.ctrl_indices_list.extend(value)
            self.ocp_indices_list.extend(self.ocp_indices[key])

    def load_mj_indices_dict(self):
        return (
            self.qpos_indices,
            self.qvel_indices,
            self.ctrl_indices
        )

    def load_mj_indices_list(self):
        return(
            self.qpos_indices_list,
            self.qvel_indices_list,
            self.ctrl_indices_list
        )
    
    def load_ocp_indices_list(self):
        return self.ocp_indices_list