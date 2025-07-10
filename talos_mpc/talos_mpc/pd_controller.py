#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
import numpy as np

from linear_feedback_controller_msgs.msg import Sensor, Control
import linear_feedback_controller_msgs_py.lfc_py_types as lfc_py_types
from linear_feedback_controller_msgs_py.numpy_conversions import control_numpy_to_msg, sensor_msg_to_numpy
from talos_mpc.talos_controller_interface import TalosControllerInterface

class PDController(TalosControllerInterface):
    def __init__(self):
        super().__init__()

        self.subscription = self.create_subscription(
            Sensor,
            '/sensor',
            self.sensor_state_callback,
            self.qos
        )

        self.current_sensor_state = lfc_py_types.Sensor(
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

        self.current_joint_pos = np.zeros((self.robot_nj, 1))
        self.current_joint_vel = np.zeros((self.robot_nj, 1))

        self.timer = self.create_timer(0.01, self.timer_callback)
    
    def sensor_state_callback(self, msg):
        # Store the current sensor state
        self.get_logger().info("Sensor state received")
        self.current_sensor_state = sensor_msg_to_numpy(msg)
        self.current_joint_pos = np.array(self.current_sensor_state.joint_state.position).reshape((self.robot_nj, 1))
        self.current_joint_vel = np.array(self.current_sensor_state.joint_state.velocity).reshape((self.robot_nj, 1))

    def timer_callback(self):
        self.doControl()
        msg = control_numpy_to_msg(self.ctrl_msg_)
        self.publisher_control_.publish(msg)
            
    def doControl(self):
        # Fixed matrices as example
        K_ricatti = np.zeros((self.robot_nj, 2*self.robot_nv))

        des_joint_pos = np.array([[
            0.25847, 0.173046, -0.0002, -0.525366, 0, 0, 0.1,
            -0.25847, -0.173046, 0.0002, -0.525366, 0, 0, 0.1,
            0, 0.006761,
            0.0, 0.0, -0.411354, 0.859395, -0.448041, -0.001708, 0,
            0.0, 0.0, -0.411354, 0.859395, -0.448041, -0.001708, 0,
            0, 0, 
        ]]).transpose()
        des_joint_vel = np.zeros((self.robot_nj, 1))
        self.tau = np.array(0 * (des_joint_pos - self.current_joint_pos) + 0 * (des_joint_vel -self.current_joint_vel))

        self.ctrl_msg_ = lfc_py_types.Control(
            feedback_gain=K_ricatti,
            feedforward=self.tau,
            initial_state=self.current_sensor_state,
        )
    

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
