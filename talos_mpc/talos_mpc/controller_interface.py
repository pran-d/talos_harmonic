#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from linear_feedback_controller_msgs.msg import Control
import linear_feedback_controller_msgs_py.lfc_py_types as lfc_py_types
from linear_feedback_controller_msgs_py.numpy_conversions import control_numpy_to_msg
import numpy as np

class ControllerInterface(Node):
    def __init__(self, msg_type=Control, topic='/control', time_interval=0.1):
        super().__init__('controller_publisher')

        # Publisher on /control topic
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.publisher_control_ = self.create_publisher(msg_type, topic, qos)

        # Prepare a fixed Control message
        self.robot_nj = 32
        self.free_flyer_nq = 7
        self.free_flyer_nv = 6
        self.robot_nq = self.robot_nj + self.free_flyer_nq
        self.robot_nv = self.robot_nj + self.free_flyer_nv
        self.joint_names = [
            "leg_left_1_joint",
            "leg_left_2_joint",
            "leg_left_3_joint",
            "leg_left_4_joint",
            "leg_left_5_joint",
            "leg_left_6_joint",
            "leg_right_1_joint",
            "leg_right_2_joint",
            "leg_right_3_joint",
            "leg_right_4_joint",
            "leg_right_5_joint",
            "leg_right_6_joint",
            "torso_1_joint",
            "torso_2_joint",
            "arm_left_1_joint",
            "arm_left_2_joint",
            "arm_left_3_joint",
            "arm_left_4_joint",
            "arm_left_5_joint",
            "arm_left_6_joint",
            "arm_left_7_joint",
            "gripper_left_joint",
            "arm_right_1_joint",
            "arm_right_2_joint",
            "arm_right_3_joint",
            "arm_right_4_joint",
            "arm_right_5_joint",
            "arm_right_6_joint",
            "arm_right_7_joint",
            "gripper_right_joint",
            "head_1_joint",
            "head_2_joint",
        ]

        # Fixed matrices as example
        K_ricatti = np.zeros((self.robot_nj, 2*self.robot_nv))
        tau = np.zeros((self.robot_nj, 1))
        sensor = lfc_py_types.Sensor(
            base_pose=np.array([0,0,1.08, 0,0,0,1]),
            base_twist=np.zeros(6),
            joint_state=lfc_py_types.JointState(
                name=self.joint_names,
                position=np.zeros(self.robot_nj),
                velocity=np.zeros((self.robot_nj, 1)),
                effort=np.zeros((self.robot_nj, 1)),
            ),
            contacts=[],
        )

        self.ctrl_msg_ = lfc_py_types.Control(
            feedback_gain=K_ricatti,
            feedforward=tau,
            initial_state=sensor,
        )

        # Timer to publish message every 0.1s
        self.timer = self.create_timer(time_interval, self.timer_callback)

        self.get_logger().info("controller_publisher started")

    def timer_callback(self):
        msg = control_numpy_to_msg(self.ctrl_msg_)
        self.publisher_control_.publish(msg)