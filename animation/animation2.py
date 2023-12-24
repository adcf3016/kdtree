import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np

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

# Function to update the animation
def update(num, sc, txt, planes):
    sc._offsets3d = (tree_data[0][:num], tree_data[1][:num], tree_data[2][:num])

    # Check the split_dim and draw the corresponding plane
    split_dim = tree_data[4][num-1]
    if split_dim == 0:
        y_plane, z_plane = np.meshgrid(np.arange(0, 30, 1), np.arange(0, 30, 1))
        x_coord = tree_data[0][num-1]
        x_plane = x_coord * np.ones_like(y_plane)
        planes[0].remove()
        planes[0] = ax.plot_surface(x_plane, y_plane, z_plane, color='blue', alpha=0.3)
    elif split_dim == 1:
        x_plane, z_plane = np.meshgrid(np.arange(0, 30, 1), np.arange(0, 30, 1))
        y_coord = tree_data[1][num-1]
        y_plane = y_coord * np.ones_like(x_plane)
        planes[0].remove()
        planes[0] = ax.plot_surface(x_plane, y_plane, z_plane, color='green', alpha=0.3)
    elif split_dim == 2:
        x_plane, y_plane = np.meshgrid(np.arange(0, 30, 1), np.arange(0, 30, 1))
        z_coord = tree_data[2][num-1]
        z_plane = z_coord * np.ones_like(x_plane)
        planes[0].remove()
        planes[0] = ax.plot_surface(x_plane, y_plane, z_plane, color='red', alpha=0.3)
    else:
        planes[0].remove()

    txt.set_text(f"Depth {tree_data[3][num-1]}, Split Dim: {split_dim}")
    return sc, txt

# Create a 3D scatter plot
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
sc = ax.scatter([], [], [], s=50, c='red', marker='o')
txt = ax.text2D(0.5, 0.95, '', transform=ax.transAxes, fontsize=12, ha='center')

# Set plot limits
ax.set_xlim(0, 30)
ax.set_ylim(0, 30)
ax.set_zlim(0, 30)

# Set axis labels
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')

# Create planes for visualization
y_plane, z_plane = np.meshgrid(np.arange(0, 30, 1), np.arange(0, 30, 1))
x_plane = np.zeros_like(y_plane)
planes = [ax.plot_surface(x_plane, y_plane, z_plane, color='blue', alpha=0.3)]

# Create animation
ani = animation.FuncAnimation(fig, update, frames=len(tree_data[0]), fargs=(sc, txt, planes), interval=1000, blit=False)

# Save animation as a GIF file
ani.save('./animation/gif/kdtree_animation2.gif', writer='pillow', fps=1)

plt.show()
