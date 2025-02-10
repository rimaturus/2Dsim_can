#!/usr/bin/env python3

import sys
import yaml
import numpy as np
import matplotlib.pyplot as plt
import matplotlib as mpl

def main():
	if len(sys.argv) != 2:
		print("Usage: python manual_track_cleaner.py <input.yaml>")
		sys.exit(1)

	filename = sys.argv[1]
	updated_filename = filename.split('.')[0] + '_updated.yaml'

	# 1. Load the YAML file
	with open(filename, 'r') as f:
		data = yaml.safe_load(f)

	cones = data.get('cones', [])
	if not cones:
		print("No 'cones' list found in the given YAML file.")
		sys.exit(0)

	# Extract x, y, color from the cones
	xs = [cone['x'] for cone in cones]
	ys = [cone['y'] for cone in cones]
	cone_colors = [cone['color'] for cone in cones]

	# Optional map from color name (as in YAML) to a valid matplotlib color
	color_map = {
		'yellow': 'orange',
		'blue': 'blue',
		'orange': 'orange',
		'red': 'red',
		'green': 'green'
	}

	# Convert each color to an RGBA tuple so we can modify them if needed
	# If a color isn't in the map, fall back to 'black'
	rgba_colors = [mpl.colors.to_rgba(color_map.get(c.lower(), 'black'))
				   for c in cone_colors]

	# Create a figure and scatter plot
	fig, ax = plt.subplots()

	# Plot the cones; supply the RGBA array directly
	scatter = ax.scatter(xs, ys, c=rgba_colors, picker=True)

	# We'll keep a copy of the original colors so we can "unselect" (restore)
	original_rgba = np.array(rgba_colors, dtype=float)

	# This array will be what we actually modify
	current_rgba = np.array(rgba_colors, dtype=float)

	# Keep track of selected (clicked) points by their index
	indices_to_delete = set()

	def on_pick(event):
		"""Toggle the color of the clicked point between original and red."""
		# Typically event.ind is a list of picked points; we assume 1 here
		ind = event.ind[0]

		if ind in indices_to_delete:
			# Already marked for deletion, so "unmark" and restore its color
			indices_to_delete.remove(ind)
			current_rgba[ind] = original_rgba[ind]
		else:
			# Mark for deletion by coloring the point red
			indices_to_delete.add(ind)
			current_rgba[ind] = (1.0, 0.0, 0.0, 1.0)  # RGBA for red

		# Update the scatter object's facecolors
		scatter.set_facecolors(current_rgba)

		# Refresh the figure
		fig.canvas.draw_idle()

	# Connect the pick event to our callback
	fig.canvas.mpl_connect('pick_event', on_pick)

	ax.set_title("Click points to mark/unmark for deletion. Close to confirm.")
	plt.xlabel("X")
	plt.ylabel("Y")
	plt.show()

	# Once the window is closed, remove selected (red) points from the data
	indices_to_delete = sorted(indices_to_delete, reverse=True)
	for i in indices_to_delete:
		del cones[i]

	# Finally, rewrite the YAML file without the deleted points
	with open(updated_filename, 'w') as f:
		yaml.dump(data, f, sort_keys=False)

	print(f"Deleted {len(indices_to_delete)} point(s). File '{updated_filename}' updated.")

if __name__ == "__main__":
	main()
