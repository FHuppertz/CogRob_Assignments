import numpy as np
from sklearn.mixture import GaussianMixture
import matplotlib.pyplot as plt
from numpy.linalg import eigh



def train_gmm(data, n_components=3):
    """
    Train a Gaussian Mixture Model (GMM) using the input data.
    Args:
        data (np.array): Shape (N, 3), each row is a 3D point.
        n_components (int): Number of Gaussian components.
    Returns:
        Trained GMM model.
    """
    gmm = GaussianMixture(n_components=n_components, covariance_type='full')
    gmm.fit(data)
    return gmm

def sample_trajectory(gmm, n_samples=100):
    """
    Generate a trajectory by sampling from a trained GMM.
    Args:
        gmm: Trained GMM model.
        n_samples (int): Number of trajectory points to generate.
    Returns:
        np.array of shape (n_samples, 3)
    """
    samples, _ = gmm.sample(n_samples)
    return samples


def draw_ellipsoid(ax, mean, cov, color='red', alpha=0.2):
    """Draw a 3D ellipsoid representing a Gaussian component."""
    from numpy.linalg import eigh
    vals, vecs = eigh(cov)
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
            sphere[i, j] = point + mean

    ax.plot_surface(sphere[:, :, 0], sphere[:, :, 1], sphere[:, :, 2],
                    rstride=1, cstride=1, color=color, alpha=alpha)

def plot_gmm_3d(gmm, data=None, samples=None):
    """Visualize the GMM components and optionally the training data and sampled trajectory."""
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    for i in range(gmm.n_components):
        draw_ellipsoid(ax, gmm.means_[i], gmm.covariances_[i])
    if data is not None:
        ax.scatter(data[:, 0], data[:, 1], data[:, 2], label='Data', alpha=0.3)
    if samples is not None:
        ax.plot(samples[:, 0], samples[:, 1], samples[:, 2], label='Sampled Trajectory')
    ax.legend()
    plt.show()

def gmr_predict(gmm, t_values, in_idx=0, out_idx=[1, 2, 3]):
    """
    Perform Gaussian Mixture Regression (GMR) to generate a trajectory conditioned on time.
    Args:
        gmm: Trained GMM with input-output dimensions (e.g. [t, x, y, z]).
        t_values (np.array): Array of time steps to generate the trajectory.
        in_idx (int): Index of the input dimension (e.g., time = 0).
        out_idx (list): Indices of the output dimensions (e.g., [x, y, z] = [1, 2, 3]).
    Returns:
        np.array: Predicted trajectory of shape (len(t_values), len(out_idx))
    """
    n_components = gmm.n_components
    means = gmm.means_
    covs = gmm.covariances_
    weights = gmm.weights_
    preds = []

    for t in t_values:
        mus = []
        probs = []
        for k in range(n_components):
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