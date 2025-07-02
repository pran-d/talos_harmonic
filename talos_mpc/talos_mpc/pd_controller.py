#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
import numpy as np

from linear_feedback_controller_msgs.msg import Control
import linear_feedback_controller_msgs_py.lfc_py_types as lfc_py_types
from linear_feedback_controller_msgs_py.numpy_conversions import control_numpy_to_msg
from talos_mpc.controller_interface import ControllerInterface

class PDController(ControllerInterface):
    def __init__(self):
        super().__init__()

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
