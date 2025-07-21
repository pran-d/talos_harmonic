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

        self.setDesiredPos = False
        self.des_joint_pos = np.zeros((self.robot_nj, 1))
        self.des_joint_vel = np.zeros((self.robot_nj, 1))

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

        self.Kp = np.array([
            5000,5000,5000,5000,5000,5000,
            5000,5000,5000,5000,5000,5000,
            10000,10000,
            10000,10000,5000,5000,500,500,100,
            10000,10000,5000,5000,500,500,100,
            300,300
        ]).reshape((self.robot_nj, 1))

        self.Kd = np.array([
            20,20,20,20,20,20,
            20,20,20,20,20,20,
            10,10,
            0.01,0.01,0,0,1,1,1,
            0.01,0.01,0,0,1,1,1,
            0.1,0.1
        ]).reshape((self.robot_nj, 1))

        self.timer = self.create_timer(0.001, self.timer_callback)
    
    def sensor_state_callback(self, msg):
        # Store the current sensor state
        # self.get_logger().info("Sensor state received")
        self.current_sensor_state = sensor_msg_to_numpy(msg)
        self.current_joint_pos = np.array(self.current_sensor_state.joint_state.position).reshape((self.robot_nj, 1))
        self.current_joint_vel = np.array(self.current_sensor_state.joint_state.velocity).reshape((self.robot_nj, 1))
        if not self.setDesiredPos:
            self.des_joint_pos = self.current_joint_pos
            self.setDesiredPos = True

    def timer_callback(self):
        self.get_logger().info(f"Control updated at {self.get_clock().now().nanoseconds}")
        self.doControl()
        msg = control_numpy_to_msg(self.ctrl_msg_)
        self.publisher_control_.publish(msg)
            
    def doControl(self):
        # Fixed matrices as example
        K_ricatti = np.zeros((self.robot_nj, 2*self.robot_nv))

        # des_joint_pos = np.array([[
        #  # leg-left
        #  # leg-right
        #  # torso
        #  # arm-left
        #  # arm-right
        #  # head 
        # ]]).transpose()
        self.tau = np.array(self.Kp * (self.des_joint_pos - self.current_joint_pos) + self.Kd * (self.des_joint_vel - self.current_joint_vel))

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
