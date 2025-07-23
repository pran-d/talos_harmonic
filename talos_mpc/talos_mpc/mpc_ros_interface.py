import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
import numpy as np
from threading import Thread, Lock

import linear_feedback_controller_msgs_py.lfc_py_types as lfc_py_types
from linear_feedback_controller_msgs.msg import Sensor
from linear_feedback_controller_msgs_py.numpy_conversions import control_numpy_to_msg, sensor_msg_to_numpy
from talos_mpc.sensor_data_listener import SensorDataListener
from talos_mpc.talos_controller_interface import TalosControllerInterface
from talos_mpc.mpc_solver import MPCSolver

class MPCRosInterface(TalosControllerInterface):
    """Publish control messages to the LFC based on sensor readings"""

    def __init__(self):
        super().__init__()        

        self.subscription = self.create_subscription(
            Sensor, 
            '/sensor',
            self.sensor_state_callback,
            self.qos, 
        )

        self.ocp = MPCSolver(self.get_logger())

        DT = 1e-2
        N = 100
        self.state_q = np.array([
            0, 0, 1.02, 0, 0, 0, 1,
            0, 0, -0.448041, 0.896082, -0.448041, 0,
            0, 0, -0.448041, 0.896082, -0.448041, 0,
            0, 0,
            0.25847, 0.173046, -0.0002, -0.525366, 0, 0, 0,
            -0.25847, -0.173046, 0.0002, -0.525366, 0, 0, 0,
            0, 0,
        ]).reshape((self.free_flyer_nq+self.robot_nj, 1))
        x0 = self.ocp.createState(self.state_q)
        self.ocp.createInitialProblem(x0, DT, N)

        self.warm_xs = []
        self.warm_us = []

        self.controller_timer = self.create_timer(
            0.01, 
            self.controller_callback, 
        )
    

    def sensor_state_callback(self, msg):
        # Store the current sensor state
        self.sensor_msg_ = sensor_msg_to_numpy(msg)
        # self.ocp.problem.runningDatas[0]
        # self.sensor_msg_.contacts = [
        #     self.wrench_left,
        #     self.wrench_right
        # ]
        

    def controller_callback(self):
        x_measured_ = self.getRobotState()
        self.MPCUpdate(x_measured_)
        msg = control_numpy_to_msg(self.ctrl_msg_)
        self.publisher_control_.publish(msg)


    def getRobotState(self):
        state_q = np.hstack((self.sensor_msg_.base_pose, self.sensor_msg_.joint_state.position.reshape((self.robot_nj,)))).reshape((self.free_flyer_nq+self.robot_nj, 1))
        state_v = np.hstack((self.sensor_msg_.base_twist, self.sensor_msg_.joint_state.velocity.reshape((self.robot_nj,)))).reshape((self.free_flyer_nv+self.robot_nj, 1))
        return np.vstack((state_q, state_v))


    def MPCUpdate(self, x0):
        status_ = self.ocp.updateProblem(x0)
        result_ = self.ocp.solveProblem(x0, 1)

        if status_ and result_:
            tau = self.ocp.getControlSequence()[0]
            KRiccati = self.ocp.getRiccatiGainSequence()[0]
            self.get_logger().info(f"Control state updated after {result_[1]} its at {self.get_clock().now().seconds_nanoseconds()}")
            self.ctrl_msg_.initial_state = self.sensor_msg_
            self.ctrl_msg_.feedforward = tau
            self.ctrl_msg_.feedback_gain = KRiccati

def main(args=None):
    try:
        rclpy.init(args=args)
        mpc_ros_interface = MPCRosInterface()
        rclpy.spin(mpc_ros_interface)

    except KeyboardInterrupt:
        pass
    finally:
        # executor.shutdown()
        mpc_ros_interface.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()