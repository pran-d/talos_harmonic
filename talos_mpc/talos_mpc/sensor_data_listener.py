#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from threading import Thread, Lock

from talos_mpc.talos_controller_interface import TalosControllerInterface
from linear_feedback_controller_msgs.msg import Sensor
from linear_feedback_controller_msgs_py.numpy_conversions import sensor_msg_to_numpy
import numpy as np

class SensorDataListener(TalosControllerInterface):
    def __init__(self):
        super().__init__(name='sensor_data_listener')

        # Publisher on /control topic
        self.qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.get_logger().info("sensor_data_listener started")

        self.subscription = self.create_subscription(
            Sensor, 
            '/sensor',
            self.sensor_state_callback,
            self.qos, 
        )
        
        self.current_sensor_state = None

        self.mutex = Lock()
    
    def sensor_state_callback(self, msg):
        # Store the current sensor state
        self.get_logger().info(f"Sensor state received at {self.get_clock().now().seconds_nanoseconds()}")
        
        self.current_sensor_state = sensor_msg_to_numpy(msg)
        self.current_base_pos = np.array(self.current_sensor_state.base_pose).reshape((self.free_flyer_nq, 1))
        self.current_base_vel = np.array(self.current_sensor_state.base_twist).reshape((self.free_flyer_nv, 1))
        self.current_joint_pos = np.array(self.current_sensor_state.joint_state.position).reshape((self.robot_nj, 1))
        self.current_joint_vel = np.array(self.current_sensor_state.joint_state.velocity).reshape((self.robot_nj, 1))
        self.current_joint_eff = np.array(self.current_sensor_state.joint_state.effort).reshape((self.robot_nj, 1))
        
        self.state_q = np.vstack((self.current_base_pos, self.current_joint_pos))
        self.state_v = np.vstack((self.current_base_vel, self.current_joint_vel))
