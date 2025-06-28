import numpy as np
import pybullet as p
import time
from tqdm import tqdm

from TrajectoryScaler import TrajectoryScaler
from TrajectoryGenerator import TrajectoryGenerator


class TrajectoryTracker:
    def __init__(self):
        self.trajectory = []
        self.last_drawn_index = 0
        self.trajectory_visual_ids = []

        self.tracking = False
        self.playing = False
        self.playing_index = 0

        self.allowed_objects = ["cube"]
        self.learned_trajectory = None

        # Progress bar for playback
        self.playback_bar = None

    def add_point(self, point):
        self.trajectory.append(np.array([time.time(), *point]))

    def increment_trajectory_visualization(self):
        if len(self.trajectory) > 1 and self.last_drawn_index < len(self.trajectory) - 1:
            visual_id = p.addUserDebugLine(
                self.trajectory[self.last_drawn_index][1:4],    # start point
                self.trajectory[-1][1:4],                  # end point
                [0, 0, 1],                       # color: blue
                2,                               # line width
                lifeTime=5.0                    # 0 means forever
            )
            self.trajectory_visual_ids.append(visual_id)
            self.last_drawn_index = len(self.trajectory) - 1

    def update(self, objects, picked_object, picked_object_position):
        self.objects = objects

        if picked_object:
            if picked_object in self.allowed_objects:
                self.tracking = True
                self.playing = False
                
                self.add_point(picked_object_position)
                self.increment_trajectory_visualization()

        else:
            if self.tracking:
                self.learn_trajectory()

                self.playing = True
                self.tracking = False

            if self.playing:
                self.playback_trajectory()

            self.clear()

    def clear(self):
        self.trajectory.clear()
        self.last_drawn_index = 0

    def learn_trajectory(self):
        robot_id = self.objects["robot"]
        n_components = len(self.trajectory)//20

        print(f"Learning trajectory with {len(self.trajectory)} "
              f"points on robot of id {robot_id} with {n_components} components")
        
        if not self.trajectory:
            return
        
        # TODO: Make this dynamic
        start_point = np.array([0.3, 0.2, 0.6])
        goal_point = np.array([0.3, -0.2, 0.6])

        trajectory = np.array(self.trajectory)

        robot_trajectory_generator = TrajectoryGenerator(n_components=n_components)
        learned_trajectory = robot_trajectory_generator.process_demonstration(
            trajectory,
            num_points=len(self.trajectory) * 2,
            )

        scaler = TrajectoryScaler()
        learned_trajectory = scaler.transform_trajectory(
            learned_trajectory[:, 1:4], 
            start_point, 
            goal_point,
            )
        
        # Add time dimension to trajectory
        self.learned_trajectory = np.insert(learned_trajectory, 0, learned_trajectory[:, 0], axis=1)

        # Progress bar for playback
        self.playback_bar = tqdm(
            total=len(self.learned_trajectory) - 1, 
            desc=f"Playing back trajectory on robot of id {robot_id}",
            )

    def playback_trajectory(self):
        if self.learned_trajectory is None:
            return
        
        robot_id = self.objects["robot"]

        joint_angles = p.calculateInverseKinematics(robot_id, 6, self.learned_trajectory[self.playing_index][1:4])

        for i, angle in enumerate(joint_angles):
            p.setJointMotorControl2(robot_id, i, p.POSITION_CONTROL, targetPosition=angle)

        self.playing_index += 1
        self.playback_bar.update(1)

        if self.playing_index >= len(self.learned_trajectory):
            self.playing = False
            self.playing_index = 0

            if self.playback_bar:
                self.playback_bar.close()
                self.playback_bar = None