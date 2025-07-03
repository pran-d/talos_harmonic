import rclpy
from rclpy.node import Node
import time
import numpy as np

import linear_feedback_controller_msgs_py.lfc_py_types as lfc_py_types
from linear_feedback_controller_msgs_py.numpy_conversions import control_numpy_to_msg, sensor_msg_to_numpy
from linear_feedback_controller_msgs.msg import Sensor, Control
from talos_mpc.controller_interface import ControllerInterface
from talos_mpc.mpc_solver import MPCSolver

class MPCRosInterface(ControllerInterface):
    def __init__(self):
        super().__init__()
        self.timer = self.create_timer(0.01, self.timer_callback)
        
        self.subscription = self.create_subscription(
            Sensor,
            '/sensor',
            self.sensor_state_callback,
            self.qos
        )

        self.DT = 1e-3
        self.N = 10
        self.ocp = MPCSolver()
        self.diff_model_ = self.ocp.createActionModel()

        self.KRiccati = np.zeros((self.robot_nj, 2*self.robot_nv))
        self.tau = np.zeros((self.robot_nj, 1))

    def sensor_state_callback(self, msg):
        # Store the current sensor state
        self.get_logger().info("Sensor state received")
        self.current_sensor_state = sensor_msg_to_numpy(msg)
        self.current_base_pos = np.array(self.current_sensor_state.base_pose).reshape((self.free_flyer_nq, 1))
        self.current_base_vel = np.array(self.current_sensor_state.base_twist).reshape((self.free_flyer_nv, 1))
        self.current_joint_pos = np.array(self.current_sensor_state.joint_state.position).reshape((self.robot_nj, 1))
        self.current_joint_vel = np.array(self.current_sensor_state.joint_state.velocity).reshape((self.robot_nj, 1))
        
        self.state_q = np.vstack((self.current_base_pos, self.current_joint_pos))
        self.state_v = np.vstack((self.current_base_vel, self.current_joint_vel))

    def timer_callback(self):
        self.doControl()
        msg = control_numpy_to_msg(self.ctrl_msg_)
        self.publisher_control_.publish(msg)

    def doControl(self):
        # Fixed matrices as example
        self.x0 = self.ocp.createState(self.state_q, self.state_v)
        status_ = self.ocp.createProblem(self.diff_model_, self.x0, self.DT, self.N)
        result_ = self.ocp.solveProblem()

        if status_ and result_:
            self.tau = self.ocp.getControl(0)
            self.KRiccati = self.ocp.getRiccatiGains(0)
       
        self.ctrl_msg_ = lfc_py_types.Control(
            feedback_gain=self.KRiccati,
            feedforward=self.tau,
            initial_state=self.x0,
        )

def main(args=None):
    rclpy.init(args=args)
    mpc_ros_interface = MPCRosInterface()
    
    try:
        rclpy.spin(mpc_ros_interface)
    except KeyboardInterrupt:
        pass
    finally:
        mpc_ros_interface.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()