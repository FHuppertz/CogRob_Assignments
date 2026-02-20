# Learning from Demonstration

This repository contains a Python implementation of trajectory learning and transformation for demonstration purposes. The project allows for recording demonstrations, learning from them using Gaussian Mixture Models (GMM), and reproducing the learned behaviors on a robotic arm in a PyBullet simulation environment.

## Project Overview

The project consists of several key components:

- Interactive simulation environment for recording demonstrations
- Trajectory learning using Gaussian Mixture Models (GMM)
- Trajectory scaling and transformation
- Real-time playback of learned behaviors on a KUKA IIWA robot

## Key Components

- `demonstration_sim.py`: Main simulation environment with interactive object manipulation and trajectory recording
- `TrajectoryTracker.py`: Handles recording, learning, and playback of demonstrated trajectories
- `TrajectoryGenerator.py`: Handles the generation of sample trajectories using GMM
- `TrajectoryScaler.py`: Provides functionality to transform trajectories between different start and goal positions
- `TrajectoryGMM.py`: Implements the Gaussian Mixture Model for trajectory learning
- `imitation_sim.py`: Additional simulation script that demonstrates the full pipeline
- `pybullet_utils.py`: Utility functions for the PyBullet simulation

## Usage

### Interactive Simulation

Run the interactive simulation environment:

```bash
python3 demonstration_sim.py
```

The simulation provides the following controls:
- Left mouse button: Click and drag objects to demonstrate trajectories
- 'l' key: Enter learning mode (default mode)
- 'p' key: Play back learned trajectories on the robot
- 'r' key: Reset object positions
- 'c' key: Clear all learned trajectories

### Demonstration Process
1. Press 'l' to ensure you're in learning mode
2. Click and drag the red or green cube to demonstrate a trajectory
3. Release the cube to complete the demonstration
4. The system will automatically learn from your demonstration
5. Press 'p' to watch the robot reproduce your demonstrations
6. Press 'r' to reset objects if needed
7. Press 'c' to clear all demonstrations and start over

The project also includes a Jupyter notebook (`trajectory_learning_from_demonstration.ipynb`) that explains the Gaussian-Mixture-Model approach and its application in detail.

## Learning the Trajectory

The learning process is implemented using Gaussian Mixture Models, which capture the underlying patterns in demonstration trajectories. The implementation allows for:
- Recording real-time demonstrations through object manipulation
- Learning from multiple sequential demonstrations
- Scaling and transforming trajectories to match demonstrated positions
- Real-time playback on a simulated robotic arm
- Visualizing trajectories during demonstration and playback

For a detailed explanation of the GMM approach and its implementation, please refer to the Jupyter notebook in the repository.
