import rclpy
from rclpy.node import Node

from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

import pybullet as p
import pybullet_data
import time

class TracSale(Node):
    def __init__(self):
        super().__init__('trac_scale_node')

        self.sample_trac_sub = self.create_subscription(Path, '/trajectory', self.sample_trac_cb, 10)
        self.start_goal_sub = self.create_subscription(Path, '/start_goal', self.start_goal_cb, 10)
        self.trac_pub = self.create_publisher(Path, '/transformed_trajectory', 10)

        self.sample_trac = None
        self.sample_trac_set = False

    def sample_trac_cb(self, msg):
        # Take sample and make them np.array of [[x,y,z], ...]
        self.sample_trac = np.zeros((len(msg.poses), 3))
        for i in range(len(msg.poses)):
            self.sample_trac[i,:] = [msg.poses[i].pose.position.x, msg.poses[i].pose.position.y, msg.poses[i].pose.position.z]
        
        self.get_logger().info("Sampled Trajectory: " + str(self.sample_trac))
        self.sample_trac_set = True


        # Hard coded start and goal here, due to time constraints
        start_goal = [
            [0.3, 0.2, 0.6],
            [0.3, -0.2, 0.6]
            ]
        
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = 'world'
        
        for pt in start_goal:
            pose = PoseStamped()
            pose.pose.position.x = float(pt[0])
            pose.pose.position.y = float(pt[1])
            pose.pose.position.z = float(pt[2])
            path_msg.poses.append(pose)

        self.start_goal_cb(path_msg)

    def start_goal_cb(self, msg):
        if self.sample_trac_set:
            start = np.array([msg.poses[0].pose.position.x, msg.poses[0].pose.position.y, msg.poses[0].pose.position.z])
            goal = np.array([msg.poses[1].pose.position.x, msg.poses[1].pose.position.y, msg.poses[1].pose.position.z])

            resulting_trac = self.trac_start_goal(self.sample_trac, start, goal)
            
            # MESSAGE TO PUB
            path_msg = Path()
            path_msg.header.stamp = self.get_clock().now().to_msg()
            path_msg.header.frame_id = 'world'
            
            for pt in resulting_trac:
                pose = PoseStamped()
                pose.pose.position.x = float(pt[0])
                pose.pose.position.y = float(pt[1])
                pose.pose.position.z = float(pt[2])
                path_msg.poses.append(pose)
            
            self.get_logger().info("Transformed Trajectory: " + str(resulting_trac))
            self.trac_pub.publish(path_msg)

            #''' 
            # Plot the transformed trajectory
            #self.get_logger().info(str(np.round(ntrac[-1])))

            ax = plt.figure().add_subplot(projection='3d')
            ax.plot(self.sample_trac[:,0], self.sample_trac[:,1], self.sample_trac[:,2], zdir='z', label='Sample Trac')
            ax.plot(resulting_trac[:,0], resulting_trac[:,1], resulting_trac[:,2], zdir='z', label='Result Trac')
            ax.scatter(resulting_trac[0,0], resulting_trac[0,1], resulting_trac[0,2], zdir='z', label='Start Pose', c='y')
            ax.scatter(resulting_trac[-1,0], resulting_trac[-1,1], resulting_trac[-1,2], zdir='z', label='End Pose', c='r')
            ax.set_xlim(-2, 2)
            ax.set_ylim(-2, 2)
            ax.set_zlim(-2, 2)
            plt.legend()
            plt.show()
            #'''

            # Quick Fix due to time constraints
            p.connect(p.GUI)
            p.setAdditionalSearchPath(pybullet_data.getDataPath())

            # Load plane and robot
            p.loadURDF("plane.urdf")
            robot_id = p.loadURDF("kuka_iiwa/model.urdf", useFixedBase=True)

            positions = np.array(resulting_trac)

            # Step simulation
            for target_pos in positions:

                joint_angles = p.calculateInverseKinematics(robot_id, 6, target_pos)

                for i, angle in enumerate(joint_angles):
                    p.setJointMotorControl2(robot_id, i, p.POSITION_CONTROL, targetPosition=angle)

                for _ in range(10):
                    p.stepSimulation()
                    time.sleep(0.005)


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