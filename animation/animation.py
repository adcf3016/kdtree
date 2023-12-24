import matplotlib.pyplot as plt
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
    (10, 11, 12, 3, 0),
    (19, 20, 21, 3, 0),
]

tree_data = list(map(list, zip(*tree_data)))  # Transpose the data

# Function to update the animation
def update(num, sc, txt):
    sc._offsets3d = (tree_data[0][:num], tree_data[1][:num], tree_data[2][:num])
    txt.set_text(f"Depth {tree_data[3][num-1]}, Split Dim: {tree_data[4][num-1]}")
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

# Create animation
ani = animation.FuncAnimation(fig, update, frames=len(tree_data[0]), fargs=(sc, txt), interval=1000, blit=False)

# Save animation as a GIF file
ani.save('./animation/gif/kdtree_animation.gif', writer='pillow', fps=1)

plt.show()
