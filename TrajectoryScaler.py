import matplotlib.pyplot as plt
import numpy as np
import pybullet as p
import pybullet_data
import time


class TrajectoryScaler:
    def __init__(self):
        """Initialize the TrajectoryScaler class."""
        self.sample_trajectory = None
    
    def normalize_trajectory(self, sample_trajectory):
        """
        Normalize a trajectory by moving it to origin and aligning with x-axis.
        
        Args:
            sample_trajectory (np.ndarray): Input trajectory as Nx3 array of points
            
        Returns:
            np.ndarray: Normalized trajectory
        """
        # Create a copy to avoid modifying the input
        sample_trac = sample_trajectory.copy()
        
        # Move to origin
        sample_trac -= sample_trac[0]

        if (np.linalg.norm(sample_trac[-1]) == 0):
            raise ValueError("Invalid trajectory: end point coincides with start point")
    
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

        return (rotX.T @ rotY @ rotX @ sample_trac.T).T * 1/np.linalg.norm(sample_trac[-1])

    def transform_trajectory(self, trajectory, start, goal):
        """
        Transform a trajectory to fit between given start and goal points.
        
        Args:
            trajectory (np.ndarray): Input trajectory as Nx3 array of points
            start (np.ndarray): Start point as 1x3 array
            goal (np.ndarray): Goal point as 1x3 array
            
        Returns:
            np.ndarray: Transformed trajectory
        """
        # Normalize trajectory
        normal_trac = self.normalize_trajectory(trajectory)

        # Get rotation to align with goal
        start_to_goal = goal - start

        if (np.linalg.norm(start_to_goal) == 0):
            raise ValueError("Start and goal points are the same")

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

        # Rotate the normal_trac and scale to fit start to goal and move it to the start
        return (rotX @ rotY @ rotX.T @ normal_trac.T).T * np.linalg.norm(start_to_goal) + start

    def visualize_trajectory(self, sample_trajectory, transformed_trajectory):
        """
        Visualize the original and transformed trajectories in a 3D plot.
        
        Args:
            sample_trajectory (np.ndarray): Original trajectory
            transformed_trajectory (np.ndarray): Transformed trajectory
        """
        ax = plt.figure().add_subplot(projection='3d')
        ax.plot(sample_trajectory[:,0], sample_trajectory[:,1], sample_trajectory[:,2], 
                zdir='z', label='Sample Trajectory')
        ax.plot(transformed_trajectory[:,0], transformed_trajectory[:,1], transformed_trajectory[:,2], 
                zdir='z', label='Transformed Trajectory')
        ax.scatter(transformed_trajectory[0,0], transformed_trajectory[0,1], transformed_trajectory[0,2], 
                  zdir='z', label='Start Pose', c='y')
        ax.scatter(transformed_trajectory[-1,0], transformed_trajectory[-1,1], transformed_trajectory[-1,2], 
                  zdir='z', label='End Pose', c='r')
        ax.set_xlim(-2, 2)
        ax.set_ylim(-2, 2)
        ax.set_zlim(-2, 2)
        plt.legend()
        plt.show()

    def simulate_trajectory(self, trajectory):
        """
        Simulate the trajectory using PyBullet with a KUKA IIWA robot.
        
        Args:
            trajectory (np.ndarray): Trajectory to simulate as Nx3 array of points
        """
        p.connect(p.GUI)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())

        # Load plane and robot
        p.loadURDF("plane.urdf")
        robot_id = p.loadURDF("kuka_iiwa/model.urdf", useFixedBase=True)

        # Step simulation
        for target_pos in trajectory:
            joint_angles = p.calculateInverseKinematics(robot_id, 6, target_pos)

            for i, angle in enumerate(joint_angles):
                p.setJointMotorControl2(robot_id, i, p.POSITION_CONTROL, targetPosition=angle)

            for _ in range(10):
                p.stepSimulation()
                time.sleep(0.005)


if __name__ == "__main__":
    # Create sample trajectory
    sample_trajectory = np.array([
        [0.0, 0.0, 0.0],
        [0.1, 0.1, 0.1],
        [0.2, 0.2, 0.2]
    ])

    # Define start and goal points
    start_point = np.array([0.3, 0.2, 0.6])
    goal_point = np.array([0.3, -0.2, 0.6])

    # Create scaler instance
    scaler = TrajectoryScaler()

    # Transform trajectory
    transformed_trajectory = scaler.transform_trajectory(sample_trajectory, start_point, goal_point)

    # Optionally visualize
    scaler.visualize_trajectory(sample_trajectory, transformed_trajectory)

    # Optionally simulate
    scaler.simulate_trajectory(transformed_trajectory)