import numpy as np
import pybullet as p
import pybullet_data
import time

from typing import Callable

from pybullet_utils import getRayFromTo
from TrajectoryTracker import TrajectoryTracker


def simulation_loop(
        loop_fn: Callable[[], None] = None,
        ):
    p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81)

    objects = {
        "plane": p.loadURDF("plane.urdf"),
        "robot": p.loadURDF("kuka_iiwa/model.urdf", useFixedBase=True),
        "cube": p.loadURDF("cube.urdf", [1, 0, 1], p.getQuaternionFromEuler([0, 0, 0]),
                            globalScaling=0.1,
                            ),
    }
    print(f"Objects: {objects}")

    # Give the cube a mass
    p.changeDynamics(objects["cube"], -1, mass=1.0)

    # Keep track of picked object and its trajectory
    picked_object = None
    picked_object_position = None

    # Step simulation
    while True:
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

        # Record trajectory of picked object
        if picked_object:
            picked_object_position, _ = p.getBasePositionAndOrientation(objects[picked_object])

        if loop_fn:
            loop_fn(
                objects=objects,
                picked_object=picked_object,
                picked_object_position=picked_object_position,
                )

        p.stepSimulation()
        time.sleep(0.01)


if __name__ == "__main__":
    trajectory_tracker = TrajectoryTracker()

    simulation_loop(trajectory_tracker.update)

