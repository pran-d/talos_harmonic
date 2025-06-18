#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from linear_feedback_controller_msgs.msg import Control
import linear_feedback_controller_msgs_py.lfc_py_types as lfc_py_types
from linear_feedback_controller_msgs_py.numpy_conversions import control_numpy_to_msg
import numpy as np

class PDController(Node):
    def __init__(self):
        super().__init__('fixed_control_publisher')

        # Publisher on /control topic
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.publisher_control_ = self.create_publisher(Control, '/control', qos)

        # Prepare a fixed Control message
        nv = 38  

        # Fixed zero matrices as example
        K_ricatti = np.zeros((nv, 2 * nv))
        tau = np.ones((nv, 1))

        # Create sensor with zeros
        sensor = lfc_py_types.Sensor(
            base_pose=np.zeros(7),
            base_twist=np.zeros(6),
            joint_state=lfc_py_types.JointState(
                name=[
                    "arm_right_1_joint",
                    "arm_right_2_joint",
                    "arm_right_3_joint",
                    "arm_right_4_joint",
                    "arm_right_5_joint",
                    "arm_right_6_joint",
                    "arm_right_7_joint",
                    "arm_left_1_joint",
                    "arm_left_2_joint",
                    "arm_left_3_joint",
                    "arm_left_4_joint",
                    "arm_left_5_joint",
                    "arm_left_6_joint",
                    "arm_left_7_joint",
                    "leg_right_1_joint",
                    "leg_right_2_joint",
                    "leg_right_3_joint",
                    "leg_right_4_joint",
                    "leg_right_5_joint",
                    "leg_right_6_joint",
                    "leg_left_1_joint",
                    "leg_left_2_joint",
                    "leg_left_3_joint",
                    "leg_left_4_joint",
                    "leg_left_5_joint",
                    "leg_left_6_joint",
                    "torso_1_joint",
                    "torso_2_joint",
                    "head_1_joint",
                    "head_2_joint",
                    "gripper_left_joint",
                    "gripper_right_joint",
                ],
                position=np.zeros(nv),
                velocity=np.zeros((nv, 1)),
                effort=np.zeros((nv, 1)),
            ),
            contacts=[],
        )

        self.fixed_ctrl_msg = lfc_py_types.Control(
            feedback_gain=K_ricatti,
            feedforward=tau,
            initial_state=sensor,
        )

        # Timer to publish message every 0.1s
        self.timer = self.create_timer(0.1, self.timer_callback)

        self.get_logger().info("FixedControlPublisher started")

    def timer_callback(self):
        msg = control_numpy_to_msg(self.fixed_ctrl_msg)
        self.publisher_control_.publish(msg)
        self.get_logger().info("Published fixed Control message")


def main(args=None):
    rclpy.init(args=args)
    node = PDController()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
