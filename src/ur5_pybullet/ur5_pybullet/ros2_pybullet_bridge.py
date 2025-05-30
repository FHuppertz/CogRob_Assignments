#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
import numpy as np
import pybullet as p
import pybullet_data

from pybullet_ur5_robotiq.robot import UR5Robotiq85
from pybullet_ur5_robotiq.env import ClutteredPushGrasp
from utilities import YCBModels, Camera

class ROS2PyBulletBridge(Node):
    def __init__(self):
        super().__init__('ros2_pybullet_bridge')
        
        # Create subscription to end effector pose
        self.subscription = self.create_subscription(
            Pose,
            'ur5/end_effector_pose',  # Topic name - can be changed as needed
            self.pose_callback,
            10)  # QoS profile depth
        
        # Initialize PyBullet simulation
        self.robot = UR5Robotiq85((0, 0.5, 0), (0, 0, 0))
        self.env = ClutteredPushGrasp(self.robot, None, None, vis=True)
        self.env.reset()
        
        self.get_logger().info('ROS2 PyBullet Bridge Node started')

    def pose_callback(self, msg):
        """Handle incoming end effector pose messages"""
        # Extract position and orientation from the message
        position = [msg.position.x, msg.position.y, msg.position.z]
        orientation = [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]
        
        # Combine into a single action array for PyBullet
        action = position + list(p.getEulerFromQuaternion(orientation)) + [0.04]  # 0.04 is default gripper opening
        
        try:
            # Update the robot in PyBullet
            self.env.step(action, control_method='end')
        except Exception as e:
            self.get_logger().error(f'Failed to update robot: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    
    bridge = ROS2PyBulletBridge()
    
    try:
        rclpy.spin(bridge)
    except KeyboardInterrupt:
        pass
    finally:
        bridge.env.close()
        bridge.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 