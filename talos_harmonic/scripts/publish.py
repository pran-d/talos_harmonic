import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import yaml
import time

class TrajectoryPublisher(Node):
    def __init__(self, yaml_file):
        super().__init__('trajectory_publisher')
        self.publisher_ = self.create_publisher(JointTrajectory, '/jpc/joint_trajectory', 10)
        
        with open(yaml_file, 'r') as f:
            data = yaml.safe_load(f)
            
        self.msg = JointTrajectory()
        self.msg.joint_names = data['joint_names']
        
        for pt in data['points']:
            point = JointTrajectoryPoint()
            point.positions = pt['positions']
            if 'velocities' in pt:
                point.velocities = pt['velocities']
            if 'accelerations' in pt:
                point.accelerations = pt['accelerations']
            if 'effort' in pt:
                point.effort = pt['effort']
            tfs = pt['time_from_start']
            point.time_from_start.sec = tfs['sec']
            point.time_from_start.nanosec = tfs['nanosec']
            self.msg.points.append(point)

    def publish_continuously(self, duration=3, rate=10):
        """Publish message continuously for specified duration"""
        start_time = time.time()
        publish_rate = self.create_rate(rate)
        
        while time.time() - start_time < duration:
            self.publisher_.publish(self.msg)
            self.get_logger().info('Publishing trajectory...')
            publish_rate.sleep()

def main():
    rclpy.init()
    node = TrajectoryPublisher('../messages/initial_position.yaml')
    
    try:
        node.publish_continuously(duration=5, rate=10)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
