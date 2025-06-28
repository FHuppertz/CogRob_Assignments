import numpy as np
from TrajectoryScaler import TrajectoryScaler
from TrajectoryGenerator import TrajectoryGenerator


# Create sample trajectory
trajectory_generator = TrajectoryGenerator(n_components=5)
sample_trajectory = trajectory_generator.generate_demo_trajectory()

# Define start and goal points
start_point = np.array([0.3, 0.2, 0.6])
goal_point = np.array([0.3, -0.2, 0.6])

# Create scaler instance
scaler = TrajectoryScaler()

# Transform trajectory
transformed_trajectory = scaler.transform_trajectory(
    sample_trajectory[:, 1:4], 
    start_point, 
    goal_point,
    )

# Optionally visualize
scaler.visualize_trajectory(sample_trajectory, transformed_trajectory)

# Optionally simulate
scaler.simulate_trajectory(transformed_trajectory)