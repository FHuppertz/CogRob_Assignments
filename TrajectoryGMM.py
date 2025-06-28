import numpy as np
from sklearn.mixture import GaussianMixture
import matplotlib.pyplot as plt
from numpy.linalg import eigh


class TrajectoryGMM:
    def __init__(self, n_components=3):
        """
        Initialize the TrajectoryGMM model.
        Args:
            n_components (int): Number of Gaussian components.
        """
        self.n_components = n_components
        self.gmm = GaussianMixture(n_components=n_components, covariance_type='full')
        
    def train(self, data):
        """
        Train the Gaussian Mixture Model (GMM) using the input data.
        Args:
            data (np.array): Shape (N, 4), each row is [t, x, y, z].
        """
        self.gmm.fit(data)
        
    def sample_trajectory(self, n_samples=100):
        """
        Generate a trajectory by sampling from the trained GMM.
        Args:
            n_samples (int): Number of trajectory points to generate.
        Returns:
            np.array of shape (n_samples, 4)
        """
        samples, _ = self.gmm.sample(n_samples)
        return samples

    def _draw_ellipsoid(self, ax, mean, cov, color='red', alpha=0.2):
        """
        Draw a 3D ellipsoid representing a Gaussian component.
        Only visualizes the spatial dimensions (x, y, z), ignoring time.
        """
        # Extract spatial components (x, y, z) from mean and covariance
        spatial_mean = mean[1:4]  # Skip time dimension
        spatial_cov = cov[1:4, 1:4]  # Get spatial covariance submatrix
        
        vals, vecs = eigh(spatial_cov)
        # Ensure positive eigenvalues for visualization
        vals = np.abs(vals)
        
        u = np.linspace(0, 2 * np.pi, 30)
        v = np.linspace(0, np.pi, 30)
        x = np.outer(np.cos(u), np.sin(v))
        y = np.outer(np.sin(u), np.sin(v))
        z = np.outer(np.ones_like(u), np.cos(v))
        sphere = np.stack((x, y, z), axis=-1)

        for i in range(x.shape[0]):
            for j in range(x.shape[1]):
                point = sphere[i, j]
                point = vecs @ (np.sqrt(vals) * point)
                sphere[i, j] = point + spatial_mean

        ax.plot_surface(sphere[:, :, 0], sphere[:, :, 1], sphere[:, :, 2],
                        rstride=1, cstride=1, color=color, alpha=alpha)

    def plot_3d(self, data=None, samples=None):
        """
        Visualize the GMM components and optionally the training data and sampled trajectory.
        Only plots the spatial dimensions (x, y, z).
        Args:
            data (np.array, optional): Training data to plot
            samples (np.array, optional): Sampled trajectory to plot
        """
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        
        # Plot GMM components
        for i in range(self.gmm.n_components):
            self._draw_ellipsoid(ax, self.gmm.means_[i], self.gmm.covariances_[i])
        
        # Plot data points if provided (only spatial dimensions)
        if data is not None:
            ax.scatter(data[:, 1], data[:, 2], data[:, 3], 
                      label='Data', alpha=0.3)
        
        # Plot sampled trajectory if provided (only spatial dimensions)
        if samples is not None:
            ax.scatter(samples[:, 1], samples[:, 2], samples[:, 3], 
                   label='Sampled Trajectory')
        
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        ax.legend()
        plt.show()

    def gmr_predict(self, t_values, in_idx=0, out_idx=[1, 2, 3]):
        """
        Perform Gaussian Mixture Regression (GMR) to generate a trajectory conditioned on time.
        Args:
            t_values (np.array): Array of time steps to generate the trajectory.
            in_idx (int): Index of the input dimension (e.g., time = 0).
            out_idx (list): Indices of the output dimensions (e.g., [x, y, z] = [1, 2, 3]).
        Returns:
            np.array: Predicted trajectory of shape (len(t_values), len(out_idx))
        """
        means = self.gmm.means_
        covs = self.gmm.covariances_
        weights = self.gmm.weights_
        preds = []

        for t in t_values:
            mus = []
            probs = []
            for k in range(self.n_components):
                mu = means[k]
                cov = covs[k]
                mu_in = mu[in_idx]
                mu_out = mu[out_idx]

                Sigma_ii = cov[in_idx, in_idx]
                Sigma_oi = cov[out_idx, in_idx]

                correction = Sigma_oi * (1.0 / Sigma_ii) * (t - mu_in)
                mu_cond = mu_out + correction
                mus.append(mu_cond)

                p = (1.0 / np.sqrt(2 * np.pi * Sigma_ii)) * np.exp(-0.5 * ((t - mu_in) ** 2) / Sigma_ii)
                probs.append(weights[k] * p)

            probs = np.array(probs)
            probs = probs / np.sum(probs)
            mus = np.array(mus)
            pred = np.sum(probs[:, None] * mus, axis=0)
            preds.append(pred)

        return np.array(preds)
    

if __name__ == "__main__":
    # Generate synthetic data
    t = np.linspace(0, 1, 100)
    x = np.sin(2 * np.pi * t)
    y = np.cos(2 * np.pi * t)
    z = t * 0.1
    data = np.stack([t, x, y, z], axis=1)

    # Create an instance
    model = TrajectoryGMM(n_components=5)

    # Train the model
    model.train(data)

    # Generate samples
    samples = model.sample_trajectory(n_samples=100)

    # Visualize
    model.plot_3d(data=data, samples=samples)

    # Make predictions
    predictions = model.gmr_predict(t)