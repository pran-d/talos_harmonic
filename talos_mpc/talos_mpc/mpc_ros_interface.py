import rclpy
from rclpy.node import Node
import time
import numpy as np

import linear_feedback_controller_msgs_py.lfc_py_types as lfc_py_types
from linear_feedback_controller_msgs_py.numpy_conversions import control_numpy_to_msg, sensor_msg_to_numpy
from linear_feedback_controller_msgs.msg import Sensor, Control
from nav_msgs.msg import Odometry
from talos_mpc.controller_interface import ControllerInterface
from talos_mpc.mpc_solver import MPCSolver

class MPCRosInterface(ControllerInterface):
    def __init__(self):
        super().__init__()        
        self.subscription = self.create_subscription(
            Sensor,
            '/sensor',
            self.sensor_state_callback,
            self.qos
        )

        self.ocp = MPCSolver()
        DT = 1e-3
        N = 20
        self.state_q = np.hstack((np.array([0,0,1.08,0,0,0,1]), np.zeros((self.robot_nj,)))).reshape((self.free_flyer_nq+self.robot_nj, 1))
        x0 = self.ocp.createState(self.state_q)
        self.ocp.createInitialProblem(x0, DT, N)

        self.KRiccati = np.zeros((self.robot_nj, 2*self.robot_nv))
        self.tau = np.zeros((self.robot_nj, 1))
        self.current_sensor_state = None

        self.warm_xs = []
        self.warm_us = []

        self.timer = self.create_timer(0.01, self.timer_callback)

    def sensor_state_callback(self, msg):
        # Store the current sensor state
        self.get_logger().info("Sensor state received")
        self.current_sensor_state = sensor_msg_to_numpy(msg)
        self.current_base_pos = np.array(self.current_sensor_state.base_pose).reshape((self.free_flyer_nq, 1))
        self.current_base_vel = np.array(self.current_sensor_state.base_twist).reshape((self.free_flyer_nv, 1))
        self.current_joint_pos = np.array(self.current_sensor_state.joint_state.position).reshape((self.robot_nj, 1))
        self.current_joint_vel = np.array(self.current_sensor_state.joint_state.velocity).reshape((self.robot_nj, 1))
        self.current_joint_eff = np.array(self.current_sensor_state.joint_state.effort).reshape((self.robot_nj, 1))
        
        self.state_q = np.vstack((self.current_base_pos, self.current_joint_pos))
        self.state_v = np.vstack((self.current_base_vel, self.current_joint_vel))

    def timer_callback(self):
        self.doControl()
        msg = control_numpy_to_msg(self.ctrl_msg_)
        self.publisher_control_.publish(msg)

    def doControl(self):
        # Fixed matrices as example
        if self.current_sensor_state is not None:
            init_state = lfc_py_types.Sensor(
                base_pose=self.current_base_pos.squeeze(),
                base_twist=self.current_base_vel.squeeze(),
                joint_state=lfc_py_types.JointState(
                    name=self.joint_names,
                    position=self.current_joint_pos.squeeze(),
                    velocity=self.current_joint_vel.squeeze(),
                    effort=self.current_joint_eff.squeeze(),
                ),
                contacts=[],
            )
            x0 = self.ocp.createState(self.state_q, self.state_v)
            status_ = self.ocp.updateProblem(x0)
            result_ = self.ocp.solveProblem(self.warm_xs, self.warm_us, 100)

            if status_ and result_:
                self.warm_xs = self.ocp.getStateSequence()
                self.warm_us = self.ocp.getControlSequence()
                self.tau = self.warm_us[0]
                self.KRiccati = self.ocp.getRiccatiGainSequence()[0]
                self.get_logger().info(f"Control state updated after {result_[1]} iterations")
        
            self.ctrl_msg_ = lfc_py_types.Control(
                feedback_gain=self.KRiccati,
                feedforward=self.tau,
                initial_state=init_state,
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