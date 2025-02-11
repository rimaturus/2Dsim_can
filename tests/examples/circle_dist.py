import numpy as np
import matplotlib.pyplot as plt

# Parameters
r = 2.0         # radius for both circles
d = 1.5         # horizontal distance between the centers (ensures intersection)
n_points = 360  # number of discretization points

# Create 360 angles between 0 and 2pi (excluding the endpoint to avoid duplicate points)
angles = np.linspace(0, 2 * np.pi, n_points, endpoint=False)

# Circle A: centered at (0,0)
A_x = r * np.cos(angles)
A_y = r * np.sin(angles)

# Circle B: centered at (d,0)
B_x = d + r * np.cos(angles)
B_y = r * np.sin(angles)

# For each point on circle B, compute the minimum distance to any point on circle A
min_distances = []
for i in range(n_points):
    # Coordinates of the i-th point on circle B
    bx, by = B_x[i], B_y[i]
    
    # Compute distances from (bx, by) to all points on circle A
    distances = np.sqrt((A_x - bx)**2 + (A_y - by)**2)
    
    # Append the minimum distance
    min_distances.append(distances.min())

min_distances = np.array(min_distances)

# Plot the result: x-axis is the step number (1 to 360), y-axis is the min distance
plt.figure(figsize=(8, 4))
plt.plot(np.arange(1, n_points + 1), min_distances, marker='o', linestyle='-', color='b')
plt.xlabel("Step (1 to 360)")
plt.ylabel("Minimum distance from B to A")
plt.title("Min Distance from Each Point on Circle B to Circle A")
plt.grid(True)
plt.tight_layout()
plt.show()
