import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import matplotlib.animation as animation

# Parse the output from C++ and store it in a list of tuples (x, y, z, depth, split_dim)
tree_data = [
    (16, 17, 18, 0, 0),
    (7, 8, 9, 1, 1),
    (25, 26, 27, 1, 1),
    (4, 5, 6, 2, 2),
    (13, 14, 15, 2, 2),
    (22, 23, 24, 2, 2),
    (28, 29, 30, 2, 2),
    (1, 2, 3, 3, 0),
    (10, 11, 12, 3, 1),
    (19, 20, 21, 3, 2),
]

tree_data = list(map(list, zip(*tree_data)))  # Transpose the data

# Create a 3D scatter plot
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
sc = ax.scatter([], [], [], s=50, c='red', marker='o')

# Set plot limits
ax.set_xlim(0, 30)
ax.set_ylim(0, 30)
ax.set_zlim(0, 30)

# Set axis labels
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')

# Create planes for visualization
planes = []
for i in range(len(tree_data[0])):
    split_dim = tree_data[4][i]
    if split_dim == 0:
        y_plane, z_plane = np.meshgrid(np.arange(0, 30, 1), np.arange(0, 30, 1))
        x_coord = tree_data[0][i]
        x_plane = x_coord * np.ones_like(y_plane)
        planes.append(ax.plot_surface(x_plane, y_plane, z_plane, color='blue', alpha=0.3, visible=False))
    elif split_dim == 1:
        x_plane, z_plane = np.meshgrid(np.arange(0, 30, 1), np.arange(0, 30, 1))
        y_coord = tree_data[1][i]
        y_plane = y_coord * np.ones_like(x_plane)
        planes.append(ax.plot_surface(x_plane, y_plane, z_plane, color='green', alpha=0.3, visible=False))
    elif split_dim == 2:
        x_plane, y_plane = np.meshgrid(np.arange(0, 30, 1), np.arange(0, 30, 1))
        z_coord = tree_data[2][i]
        z_plane = z_coord * np.ones_like(x_plane)
        planes.append(ax.plot_surface(x_plane, y_plane, z_plane, color='red', alpha=0.3, visible=False))

# Create animation
def update(num):
    # Update scatter plot points
    sc._offsets3d = (tree_data[0][:num], tree_data[1][:num], tree_data[2][:num])

    # Update planes visibility
    for i in range(num):
        planes[i].set_visible(True)

    # Hide all planes after animation is complete
    if num == len(tree_data[0]) - 1:
        for plane in planes:
            plane.set_visible(False)

ani = animation.FuncAnimation(fig, update, frames=len(tree_data[0]), interval=1000, blit=False)

plt.show()
