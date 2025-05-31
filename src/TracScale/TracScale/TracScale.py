import rclpy
from rclpy.node import Node

from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

class TracSale(Node):
    def __init__(self):
        super().__init__('trac_scale_node')

        self.sample_trac_sub = self.create_subscription(Path, '/sample_trac', self.sample_trac_cb, 10)
        self.start_goal_sub = self.create_subscription(Path, '/start_goal', self.start_goal_cb, 10)
        self.trac_pub = self.create_publisher(Path, '/trajectory', 10)

        self.sample_trac = None

    def sample_trac_cb(self, msg):
        # Take sample and make them np.array of [[x,y,z], ...]
        self.sample_trac = np.zeros((len(msg.poses), 3))
        for i in range(len(msg.poses)):
            self.sample_trac[i,:] = [msg.poses[i].pose.position.x, msg.poses[i].pose.position.y, msg.poses[i].pose.position.z]
        
        self.get_logger().info("Sampled Trajectory: " + str(self.sample_trac))

        ntrac = self.normalize_trac(self.sample_trac.copy())
        trac = self.trac_start_goal(self.sample_trac.copy(), np.array([0, 0, 10]), np.array([10, 10, -5]))

        self.get_logger().info(str(np.round(ntrac[-1])))

        ax = plt.figure().add_subplot(projection='3d')
        ax.plot(self.sample_trac[:,0], self.sample_trac[:,1], self.sample_trac[:,2], zdir='z', label='Sample Trac')
        ax.plot(ntrac[:,0], ntrac[:,1], ntrac[:,2], zdir='z', label='Normalized Trac')
        ax.plot(trac[:,0], trac[:,1], trac[:,2], zdir='z', label='Result Trac')
        ax.set_xlim(-10, 10)
        ax.set_ylim(-10, 10)
        ax.set_zlim(-10, 10)
        plt.legend()
        plt.show()

    def start_goal_cb(self, msg):
        if self.sample_trac != None:
            start = msg # start
            goal = msg # goal
            resulting_trac = self.trac_start_goal(self.sample_trac, start, goal)
            
            # MESSAGE TO PUB

            self.trac_pub()

    def normalize_trac(self, sample_trac):
        # Move to origin
        sample_trac -= sample_trac[0]

        if (np.linalg.norm(sample_trac[-1]) == 0):
            return 'Error'
    
        # alpha rotates around x-axis to align vec with z-axis (sign is determind by y-axis)
        if ((sample_trac[-1][1]**2 + sample_trac[-1][2]**2) != 0):
            alpha = np.arccos(sample_trac[-1][2]/((sample_trac[-1][1]**2 + sample_trac[-1][2]**2)**0.5))
        else:
            alpha = 0

        if sample_trac[-1][1] < 0:
            alpha *= -1
        
        # beta rotates around y-axis to align vec with x-axis (sign is determined by z-axis)
        beta = np.arccos(sample_trac[-1][0]/np.linalg.norm(sample_trac[-1]))

        if sample_trac[-1][2] < 0:
            beta *= -1

        # Rotate all trac_vec to align trajectory with x-axis, then scale to be from 0 to 1
        ca = np.cos(alpha)
        sa = np.sin(alpha)

        cb = np.cos(beta)
        sb = np.sin(beta)

        rotX = np.array([
            [1, 0, 0],
            [0, ca, -sa],
            [0, sa, ca]
        ])

        rotY = np.array([
            [cb, 0, sb],
            [0, 1, 0],
            [-sb, 0, cb]
        ])

        #self.get_logger().info(str(alpha*180/np.pi))
        #self.get_logger().info(str(beta*180/np.pi))

        #self.get_logger().info(str((rotX @ sample_trac[-1])))
        return (rotX.T @ rotY @ rotX @ sample_trac.T).T * 1/np.linalg.norm(sample_trac[-1])
        

    def trac_start_goal(self, trac, start, goal):
        
        # Normalize trac
        normal_trac = self.normalize_trac(trac)

        # Get rotation to align with goal
        start_to_goal = goal - start

        if (np.linalg.norm(start_to_goal) == 0):
            return 'Error'

        # beta to rotate normal_trac to align trac to be on same cone as start_to_goal
        beta = -np.arccos(start_to_goal[0]/np.linalg.norm(start_to_goal))
        
        # alpha to rotate normal_trac to align with start_to_goal
        if ((start_to_goal[1]**2 + start_to_goal[2]**2) != 0):
            alpha = -np.arccos(start_to_goal[2]/((start_to_goal[1]**2 + start_to_goal[2]**2)**0.5))
        else:
            alpha = 0

        if start_to_goal[1] < 0:
            alpha *= -1


        ca = np.cos(alpha)
        sa = np.sin(alpha)

        cb = np.cos(beta)
        sb = np.sin(beta)

        rotX = np.array([
            [1, 0, 0],
            [0, ca, -sa],
            [0, sa, ca]
        ])

        rotY = np.array([
            [cb, 0, sb],
            [0, 1, 0],
            [-sb, 0, cb]
        ])

        #self.get_logger().info('Normal trac: ' + str(np.round(normal_trac)))

        #self.get_logger().info(str(beta*180/np.pi))
        #self.get_logger().info(str(alpha*180/np.pi))


        # Rotate the normal_trac and scale to fit start to goal and move it to the start
        return (rotX @ rotY @ rotX.T @ normal_trac.T).T * np.linalg.norm(start_to_goal) + start


def main(args=None):

    rclpy.init(args=args)

    trac_scale_node = TracSale()
    rclpy.spin(trac_scale_node)
    trac_scale_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()