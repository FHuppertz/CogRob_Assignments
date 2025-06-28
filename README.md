# Learning from Demonstration

This repository contains a Python implementation of trajectory learning and transformation for demonstration purposes. The project allows for generating sample trajectories, transforming them to match desired start and goal positions, and visualizing the results.

## Project Overview

The project consists of several key components:

- Trajectory generation using Gaussian Mixture Models (GMM)
- Trajectory scaling and transformation
- Visualization and simulation capabilities

## Key Components

- `TrajectoryGenerator.py`: Handles the generation of sample trajectories using GMM
- `TrajectoryScaler.py`: Provides functionality to transform trajectories between different start and goal positions
- `TrajectoryGMM.py`: Implements the Gaussian Mixture Model for trajectory learning
- `imitation_sim.py`: Main simulation script that demonstrates the full pipeline

## Usage

The project includes a Jupyter notebook (`trajectory_learning_from_demonstration.ipynb`) that explains the Gaussian-Mixture-Model approach and its application in detail.

To run a basic demonstration:

```python
python3 imitation_sim.py
```

This will:
1. Generate a sample trajectory
1. Transform it to match specified start and goal positions
1. Display a visualization of both the original and transformed trajectories
1. Run a simulation of the transformed trajectory

## Learning the Trajectory

The learning process is implemented using Gaussian Mixture Models, which capture the underlying patterns in demonstration trajectories. The implementation allows for:
- Generating sample trajectories
- Scaling and transforming trajectories to new start/goal positions
- Visualizing the results

For a detailed explanation of the GMM approach and its implementation, please refer to the Jupyter notebook in the repository.
