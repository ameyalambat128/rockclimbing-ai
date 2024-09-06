import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from scipy.spatial.transform import Rotation as R

# Parameters for generating a smoother, organic hold shape
def create_smooth_climbing_hold(hold):
    pos = hold['position']
    rot = hold['rotation']
    width = hold['width']
    depth = hold['depth']
    incut = hold['incut']

    # Create a smoother dome-like surface using spherical coordinates
    u = np.linspace(0, np.pi, 60)  # more points for smoother surface
    v = np.linspace(0, 2 * np.pi, 60)
    radius = (width + depth) / 4

    # Generate the x, y, z points of the smoother dome
    x = radius * np.outer(np.sin(u), np.cos(v)) * (1 - incut / 10)
    y = radius * np.outer(np.sin(u), np.sin(v)) * (1 - incut / 10)
    z = radius * np.outer(np.cos(u), np.ones_like(v))

    # Add some subtle deformations to simulate an organic hold shape
    deformation = 0.1 * np.random.rand(*x.shape)
    z += deformation

    # Apply rotation to the dome shape
    r = R.from_euler('xyz', [rot['x'], rot['y'], rot['z']], degrees=True)
    points = np.vstack([x.flatten(), y.flatten(), z.flatten()]).T
    rotated_points = r.apply(points)

    # Translate the hold to its final position
    translated_points = rotated_points + np.array([pos['x'], pos['y'], pos['z']])

    return translated_points[:, 0], translated_points[:, 1], translated_points[:, 2]

def plot_smoother_climbing_holds(holds):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    for hold in holds:
        x, y, z = create_smooth_climbing_hold(hold)
        ax.plot_trisurf(x, y, z, color='yellow', edgecolor='k', linewidth=0.2)

    # Set axis limits
    ax.set_xlim(0, 10)
    ax.set_ylim(0, 10)
    ax.set_zlim(0, 10)

    # Label the axes
    ax.set_xlabel('X axis')
    ax.set_ylabel('Y axis')
    ax.set_zlabel('Z axis')

    plt.show()

# Sample JSON data
holds = [
    {
        "rotation": {"x": 10, "y": 5, "z": 0},
        "position": {"x": 1.0, "y": 2.0, "z": 3.0},
        "width": 15, "depth": 10, "incut": 2
    },
    {
        "rotation": {"x": 15, "y": 0, "z": 10},
        "position": {"x": 2.5, "y": 3.5, "z": 1.0},
        "width": 12, "depth": 8, "incut": 3
    },
    {
        "rotation": {"x": 0, "y": 20, "z": 15},
        "position": {"x": 3.0, "y": 4.0, "z": 2.5},
        "width": 18, "depth": 12, "incut": 1
    }
]

# Plot the modified smoother climbing holds
plot_smoother_climbing_holds(holds)
