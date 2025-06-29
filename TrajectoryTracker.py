import hashlib
import numpy as np
import pybullet as p
import time
from tqdm import tqdm
from typing import Optional

from TrajectoryScaler import TrajectoryScaler
from TrajectoryGenerator import TrajectoryGenerator


class TrajectoryTracker:
    def __init__(self):
        # Tracked individual trajectory
        self.tracked_trajectory: list[np.ndarray] = []

        # Store trajectories as a list of (object_name, trajectory) tuples
        self.tracked_actions: list[tuple[str, np.ndarray]] = []
        self.learned_actions: list[tuple[str, np.ndarray]] = []

        # Learning and playing states
        self.learning = True
        self.tracked_object = None

        self.playing = False
        self.playback_action_index = 0
        self.playback_trajectory_index = 0

        self.allowed_objects = ["cube_01", "cube_02"]

        # Visualization
        self.last_drawn_index = 0
        self.trajectory_visual_ids = []

        # Progress bar for playback
        self.playback_bar: Optional[tqdm] = None
        
    def track_trajectory(self, point):
        self.tracked_trajectory.append(np.array([time.time(), *point]))

    def register_trajectory(self, object_name):
        """Register a trajectory for an object while maintaining the overall order of actions."""
        if not self.tracked_trajectory:
            return
            
        trajectory = np.array(self.tracked_trajectory)
        self.tracked_actions.append((object_name, trajectory))
        print(f"Registered trajectory for {object_name} (Action #{len(self.tracked_actions)})")
        
        # Clear the current tracking buffer
        self.tracked_trajectory = []

    def get_action_sequence(self) -> list[tuple[str, np.ndarray]]:
        """Get the sequence of actions in order of demonstration."""
        return self.tracked_actions

    def increment_trajectory_visualization(self):
        if len(self.tracked_trajectory) > 1 and self.last_drawn_index < len(self.tracked_trajectory) - 1:
            visual_id = p.addUserDebugLine(
                self.tracked_trajectory[self.last_drawn_index][1:4],    # start point
                self.tracked_trajectory[-1][1:4],                       # end point
                [0, 0, 1],                                      # color: blue
                2,                                              # line width
                lifeTime=5.0                                    # 0 means forever
            )
            self.trajectory_visual_ids.append(visual_id)
            self.last_drawn_index = len(self.tracked_trajectory) - 1

    def update(self, objects, picked_object, picked_object_position):
        self.objects = objects

        if self.learning and picked_object:
            if picked_object in self.allowed_objects:
                self.tracked_object = picked_object
                self.playing = False
                
                self.track_trajectory(picked_object_position)
                self.increment_trajectory_visualization()

        # When the user stops picking an object, register the trajectory if it's not empty and learn it
        elif self.learning and not picked_object:
            if self.tracked_trajectory:
                self.register_trajectory(self.tracked_object)
                self.learn_trajectory()

                self.clear_tracking()

                self.tracked_object = None

        elif self.playing:
            self.playback_actions()
            
    def clear(self):
        print("Clearing trajectory tracker data...")

        self.tracked_actions.clear()
        self.learned_actions.clear()

        self.clear_tracking()

        self.playing = False
        self.learning = True
        self.tracked_object = None

    def clear_tracking(self):
        self.tracked_trajectory.clear()
        self.last_drawn_index = 0
        self.trajectory_visual_ids.clear()

    def learn_trajectory(self):
        robot_id = self.objects["robot"]
        last_object = self.tracked_actions[-1][0]
        last_trajectory = self.tracked_actions[-1][1]

        n_components = len(last_trajectory)//20

        print(f"Learning trajectory with {len(last_trajectory)} "
              f"points on robot of id {robot_id} with {n_components} components")
        
        if len(last_trajectory) < 2:
            return
        
        start_point = last_trajectory[0][1:4]
        goal_point = last_trajectory[-1][1:4]

        trajectory = np.array(last_trajectory)

        robot_trajectory_generator = TrajectoryGenerator(n_components=n_components)
        learned_trajectory = robot_trajectory_generator.process_demonstration(
            trajectory,
            num_points=len(last_trajectory) * 2,
            )

        scaler = TrajectoryScaler()
        learned_trajectory = scaler.transform_trajectory(
            learned_trajectory[:, 1:4], 
            start_point, 
            goal_point,
            )
        
        # Add time dimension to trajectory
        learned_trajectory = np.insert(learned_trajectory, 0, learned_trajectory[:, 0], axis=1)

        # Add the learned trajectory to the list of learned actions
        self.learned_actions.append((last_object, np.array(learned_trajectory)))

    def playback_actions(self):
        if self.playback_bar is None:
            self.playback_bar = tqdm(
                total=len(self.learned_actions), 
                desc="Playing back trajectory on robot",
                )

        if self.playback_action_index >= len(self.learned_actions):
            self.playing = False

            self.playback_bar.close()
            self.playback_bar = None

            self.playback_action_index = 0

            return
        
        self.playback_trajectory(self.playback_action_index)

    def playback_trajectory(self, action_index):
        learned_trajectory = self.learned_actions[action_index][1]
        
        robot_id = self.objects["robot"]

        joint_angles = p.calculateInverseKinematics(robot_id, 6, learned_trajectory[self.playback_trajectory_index][1:4])

        for i, angle in enumerate(joint_angles):
            p.setJointMotorControl2(robot_id, i, p.POSITION_CONTROL, targetPosition=angle)

        self.playback_trajectory_index += 1
        self.playback_bar.set_postfix(
            trajectory=f"{self.playback_trajectory_index + 1}/{len(learned_trajectory)}",
            )

        if self.playback_trajectory_index >= len(learned_trajectory):
            self.playback_action_index += 1
            self.playback_bar.update(1)

            self.playback_trajectory_index = 0

    def handle_events(self, mouse_events, keyboard_events):
        if keyboard_events:
            for key, value in keyboard_events.items():
                if value & p.KEY_WAS_TRIGGERED:
                    if key == ord('p'):
                        self.playing = True
                        self.learning = False

                    if key == ord('l'):
                        self.learning = True
                        self.playing = False

                    if key == ord('c'):
                        self.clear()