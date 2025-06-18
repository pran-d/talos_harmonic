import rclpy
from rclpy.node import Node
import time
import numpy as np

from std_msgs.msg import Header, Float64MultiArray
from geometry_msgs.msg import Pose, Point, Quaternion, Twist, Vector3, Wrench
from sensor_msgs.msg import JointState

from linear_feedback_controller_msgs.msg import Sensor, Control

class ModelPredictiveController(Node):
    def __init__(self):
        super().__init__('mpc_node')

        self.current_joint_state = None

        self.publisher = self.create_publisher(
            Control,
            '/control',
            10
        )

        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        self.timer = self.create_timer(1, self.timer_callback)

    def joint_state_callback(self, msg):
        # Store the current joint state
        self.current_joint_state = msg

    def timer_callback(self):
        # if self.current_joint_state is None:
        #     self.get_logger().warn('No joint state received yet.')
        #     return

        # Create a Control message
        control_msg = Control()
        now = self.get_clock().now().to_msg()
        
        # Fill the header
        control_msg.header = Header()
        control_msg.header.stamp = now
        control_msg.header.frame_id = 'base_link'

        # Fill the joint states
        sensor = Sensor()
        sensor.header.stamp = now
        sensor.joint_state = self.current_joint_state

        # Fill the feedforward control values
        feedforward_data = np.ones(32)

        # Populate the control message
        # control_msg.initial_state = sensor
        control_msg.feedforward.data = data=feedforward_data.tolist()
        
        self.publisher.publish(control_msg)
        self.get_logger().info('Published control message.')
        time.sleep(0.1)


def main(args=None):
    rclpy.init(args=args)
    mpc_node = ModelPredictiveController()
    
    try:
        rclpy.spin(mpc_node)
    except KeyboardInterrupt:
        pass
    finally:
        mpc_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()