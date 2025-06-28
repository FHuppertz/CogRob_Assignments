import numpy as np
from TrajectoryGMM import TrajectoryGMM

class TrajectoryGenerator:
    def __init__(self, n_components=5):
        """Initialize the TrajectoryGenerator class for learning and generating trajectories using GMM.

        This class uses Gaussian Mixture Models (GMM) to learn from demonstration trajectories
        and generate new trajectories that follow similar patterns.

        Args:
            n_components (int, optional): Number of Gaussian components for the GMM model. 
                More components can capture more complex trajectories. Defaults to 5.

        Attributes:
            model (TrajectoryGMM): The GMM model used for trajectory learning and generation.
        """
        self.n_components = n_components
        self.model = None

    def generate_demo_trajectory(self):
        """Generate synthetic demonstration trajectories for testing or example purposes.

        Creates a demonstration trajectory following a sinusoidal pattern in x-y plane
        with linear progression in z-axis. The trajectory is randomly offset from one
        of three predefined positions to add variability.

        Returns:
            np.ndarray: Generated demonstration data with shape (n_points, 4) containing
                time and position data [t, x, y, z]. Time is normalized between 0 and 1,
                and positions follow a sinusoidal pattern with random offsets.
        """
        trajectory = []

        offsets = [(0, 0, 0), (0.2, 0.1, 0.1), (-0.1, -0.2, 0.05)]
        chosen_offset_idx = np.random.randint(0, len(offsets))
        print(f"Chosen offset index: {chosen_offset_idx}")

        chosen_offset = offsets[chosen_offset_idx]
        print(f"Chosen offset: {chosen_offset}")

        # Generate 3 trajectories with different offsets
        t = np.linspace(0, 1, 100)
        x = np.sin(2 * np.pi * t) + chosen_offset[0]
        y = np.cos(2 * np.pi * t) + chosen_offset[1]
        z = t + chosen_offset[2]
            
        traj = np.stack([t, x, y, z], axis=1)
        trajectory.append(traj)

        return np.vstack(trajectory)

    def process_demonstration(self, demonstration_data):
        """Process demonstration data to learn and generate a new trajectory using GMM/GMR.

        Takes demonstration trajectory data, normalizes the time component, trains a GMM model,
        and generates a new trajectory using Gaussian Mixture Regression (GMR).

        Args:
            demonstration_data (np.ndarray): Input demonstration data with shape (n_points, 4)
                containing time-position data [t, x, y, z]. Time should be monotonically
                increasing.

        Returns:
            np.ndarray: Generated trajectory points with shape (n_points, 4) containing
                [t, x, y, z] where t is normalized between 0 and 1, and x, y, z represent
                the predicted positions.

        Note:
            The method normalizes time internally, so input time values can be in any range.
        """
        # Normalize time component
        t_min = demonstration_data[:, 0].min()
        t_max = demonstration_data[:, 0].max()
        normalized_data = demonstration_data.copy()
        normalized_data[:, 0] = (demonstration_data[:, 0] - t_min) / (t_max - t_min + 1e-9)
        
        # Create and train GMM
        self.model = TrajectoryGMM(n_components=self.n_components)
        self.model.train(normalized_data)
        
        # Generate new trajectory
        t_values = np.linspace(0, 1, 100)
        generated_traj = self.model.gmr_predict(t_values)

        # Add time dimension
        generated_traj = np.insert(generated_traj, 0, t_values, axis=1)
        
        return generated_traj

    def process_demonstration_from_points(self, points_list):
        """Process demonstration data provided as a list of timestamped points.

        A convenience method that converts a list of points into a numpy array and
        processes it using the GMM/GMR approach.

        Args:
            points_list (list): List of points where each point is [timestamp, x, y, z].
                The timestamps should be monotonically increasing.

        Returns:
            np.ndarray: Generated trajectory points with shape (n_points, 4) containing
                [t, x, y, z] where t is normalized between 0 and 1.

        Note:
            This is a wrapper around process_demonstration() for easier list-based input.
        """
        demonstration_data = np.array(points_list)
        return self.process_demonstration(demonstration_data)

    def generate_default_trajectory(self):
        """Generate a trajectory using synthetic demonstration data.

        Creates a synthetic demonstration using generate_demo_trajectory() and processes
        it to create a new trajectory. Useful for testing and example purposes.

        Returns:
            np.ndarray: Generated trajectory points with shape (n_points, 4) containing
                [t, x, y, z] where t is normalized between 0 and 1.

        Note:
            This method combines generate_demo_trajectory() and process_demonstration()
            for convenience.
        """
        # Generate synthetic demonstration data
        demo_data = self.generate_demo_trajectory()
        
        # Process the demonstration data
        return self.process_demonstration(demo_data)

    def visualize_trajectory(self, data=None, samples=None):
        """Visualize the original and generated trajectories in 3D space.

        Creates a 3D visualization of the demonstration data and the generated
        trajectory samples using the underlying GMM model's plotting functionality.

        Args:
            data (np.ndarray, optional): Original demonstration data to plot with
                shape (n_points, 4) containing [t, x, y, z].
            samples (np.ndarray, optional): Generated trajectory samples to plot with
                shape (n_points, 4) containing [t, x, y, z].

        Note:
            This method requires a trained model (self.model must not be None).
            If no model exists, a message will be printed suggesting to train
            the model first.
        """
        if self.model is not None:
            self.model.plot_3d(data=data, samples=samples)
        else:
            print("No model available. Train the model first using process_demonstration().")


if __name__ == "__main__":
    # Create generator instance
    generator = TrajectoryGenerator(n_components=5)

    # Method 1: Generate trajectory from synthetic demonstrations
    trajectory = generator.generate_default_trajectory()
    print(f"Generated trajectory with {len(trajectory)} points")
    print(f"Trajectory shape: {trajectory.shape}")
    
    # Visualize the results
    demo_data = generator.generate_demo_trajectory()
    generator.visualize_trajectory(data=demo_data, samples=trajectory)

    # Method 2: Generate trajectory from custom demonstration data
    demo_points = []
    for i in range(100):
        t = i * 0.1
        x = np.sin(t)
        y = np.cos(t)
        z = t * 0.1
        demo_points.append([t, x, y, z])

    custom_trajectory = generator.process_demonstration_from_points(demo_points)
    print(f"Generated custom trajectory with {len(custom_trajectory)} points")
    print(f"Custom trajectory shape: {custom_trajectory.shape}")
    
    # Visualize the custom results
    generator.visualize_trajectory(data=np.array(demo_points), samples=custom_trajectory)

    # Method 3: Generate trajectory from numpy array directly
    demo_data = np.array(demo_points)  # shape: (n_points, 4) for [t, x, y, z]
    direct_trajectory = generator.process_demonstration(demo_data)
    print(f"Generated direct trajectory with {len(direct_trajectory)} points")
    print(f"Direct trajectory shape: {direct_trajectory.shape}")
    
    # Visualize the direct results
    generator.visualize_trajectory(data=demo_data, samples=direct_trajectory)