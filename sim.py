import pybullet as p
import pybullet_data
import time

from typing import Callable


def simulation_loop(
        loop_fn: Callable[[], None] = None,
        ):
    p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81)

    # Load plane and robot
    plane_id = p.loadURDF("plane.urdf")
    robot_id = p.loadURDF("kuka_iiwa/model.urdf", useFixedBase=True)

    # Add a test cube that can be dragged
    cube_start_pos = [1, 0, 1]
    cube_start_orientation = p.getQuaternionFromEuler([0, 0, 0])
    cube_id = p.loadURDF("cube.urdf", cube_start_pos, cube_start_orientation)
    # Make the cube lighter so it's easier to drag
    p.changeDynamics(cube_id, -1, mass=1.0)

    # Enable mouse picking
    p.configureDebugVisualizer(p.COV_ENABLE_MOUSE_PICKING, 1)

    # Step simulation
    while True:
        # Handle mouse events for picking
        mouse_events = p.getMouseEvents()
        for e in mouse_events:
            if e[0] == 2:  # Mouse button event
                button_index = e[3]  # 0 = left button, 1 = middle, 2 = right
                button_state = e[4]  # KEY_IS_DOWN, KEY_WAS_RELEASED
                
                if button_index == 0:  # Left button
                    if button_state == p.KEY_IS_DOWN:
                        # Enable real-time physics for picked object
                        # Does not need to be done because of stepSimulation below?
                        # p.setRealTimeSimulation(1)
                        pass
                    elif button_state == p.KEY_WAS_RELEASED:
                        # Disable real-time physics when object is released
                        # p.setRealTimeSimulation(0)
                        pass

        if loop_fn is not None:
            loop_fn(
                robot_id=robot_id,
                )

        p.stepSimulation()
        time.sleep(0.005)


if __name__ == "__main__":
    simulation_loop()

