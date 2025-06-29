import numpy as np
import pybullet as p
import pybullet_data
import time

from typing import Optional

from pybullet_utils import getRayFromTo
from TrajectoryTracker import TrajectoryTracker


def setup_objects():
    objects = {
        "plane": p.loadURDF("plane.urdf"),
        "robot": p.loadURDF("kuka_iiwa/model.urdf", useFixedBase=True),
        "cube_01": p.loadURDF("cube.urdf", [0.5, 0.0, 0.1], p.getQuaternionFromEuler([0, 0, 0]),
                            globalScaling=0.1,
                            ),
        "cube_02": p.loadURDF("cube.urdf", [0.0, 0.5, 0.1], p.getQuaternionFromEuler([0, 0, 0]),
                            globalScaling=0.1,
                            ),
    }

    for cube in ["cube_01", "cube_02"]:
        p.changeDynamics(objects[cube], -1, mass=1.0)

    p.changeVisualShape(objects["cube_01"], -1, rgbaColor=[1, 0, 0, 1])  # Set color to red
    p.changeVisualShape(objects["cube_02"], -1, rgbaColor=[0, 1, 0, 1])  # Set color to green

    return objects


def reset_objects(objects):
    """Reset objects to their initial positions."""
    p.resetBasePositionAndOrientation(objects["cube_01"], [0.5, 0.0, 0.1], p.getQuaternionFromEuler([0, 0, 0]))
    p.resetBasePositionAndOrientation(objects["cube_02"], [0.0, 0.5, 0.1], p.getQuaternionFromEuler([0, 0, 0]))


def simulation_loop(
        trajectory_tracker: Optional[TrajectoryTracker] = None,
        ):
    p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81)

    objects = setup_objects()
    print(f"Objects: {objects}")

    # Keep track of picked object and its trajectory
    picked_object = None
    picked_object_position = None

    # Step simulation
    while True:
        # Handle keyboard events
        keyboard_events = p.getKeyboardEvents()
        for key, value in keyboard_events.items():
            if value & p.KEY_WAS_TRIGGERED:
                if key == ord('r'):
                    print("Resetting objects")
                    reset_objects(objects)

        # Handle mouse events for picking
        mouse_events = p.getMouseEvents()
        for e in mouse_events:
            event_type, mx, my, button_index, button_state = e
            
            if event_type == 2:  # Mouse button event
                if button_index == 0:  # Left button, 1 is right, 2 is middle
                    if button_state & p.KEY_WAS_TRIGGERED:  # Button just pressed
                        # Get ray from camera to mouse position
                        ray_from, ray_to, alpha = getRayFromTo(mx, my)
                        
                        # Check what we hit
                        hit = p.rayTest(ray_from, ray_to)[0]
                        hit_uid, hit_link, hit_fraction, hit_pos, hit_normal = hit
                        
                        if hit_uid >= 0:  # If we hit something
                            # Find which object we hit
                            for name, uid in objects.items():
                                if uid == hit_uid:
                                    print(f"Picked up {name}")
                                    picked_object = name

                                    break
                    
                    elif button_state & p.KEY_WAS_RELEASED:  # Button released
                        picked_object = None
                        picked_object_position = None

        if trajectory_tracker:
            trajectory_tracker.handle_events(
                mouse_events=mouse_events,
                keyboard_events=keyboard_events,
            )

        # Record trajectory of picked object
        if picked_object:
            picked_object_position, _ = p.getBasePositionAndOrientation(objects[picked_object])

        if trajectory_tracker:
            trajectory_tracker.update(
                objects=objects,
                picked_object=picked_object,
                picked_object_position=picked_object_position,
                )

        p.stepSimulation()
        time.sleep(0.01)


if __name__ == "__main__":
    trajectory_tracker = TrajectoryTracker()

    simulation_loop(trajectory_tracker)

