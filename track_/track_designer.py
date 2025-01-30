import numpy as np
import matplotlib.pyplot as plt
import yaml
from scipy.interpolate import splprep, splev
from scipy.interpolate import interp1d
import sys

class FreehandDrawing:
    def __init__(self, ax):
        self.ax = ax
        self.line, = ax.plot([], [], 'k-', linewidth=2)  # Line object
        self.xs = []
        self.ys = []
        self.cid_press = self.line.figure.canvas.mpl_connect('button_press_event', self.on_press)
        self.cid_release = self.line.figure.canvas.mpl_connect('button_release_event', self.on_release)
        self.cid_move = None
        self.is_drawing = False

    def on_press(self, event):
        if event.inaxes != self.ax:
            return
        if event.button != 1:  # Left mouse button
            return
        self.is_drawing = True
        self.xs = [event.xdata]
        self.ys = [event.ydata]
        self.line.set_data(self.xs, self.ys)
        self.cid_move = self.line.figure.canvas.mpl_connect('motion_notify_event', self.on_move)

    def on_move(self, event):
        if not self.is_drawing:
            return
        if event.inaxes != self.ax:
            return
        self.xs.append(event.xdata)
        self.ys.append(event.ydata)
        self.line.set_data(self.xs, self.ys)
        self.line.figure.canvas.draw()

    def on_release(self, event):
        if event.button != 1:  # Left mouse button
            return
        self.is_drawing = False
        if self.cid_move is not None:
            self.line.figure.canvas.mpl_disconnect(self.cid_move)
            self.cid_move = None

    def get_path(self):
        return list(zip(self.xs, self.ys))

def save_to_yaml(left_cones, right_cones, filename='track_cones.yaml'):
    """
    Save cone positions to a YAML file with a unified structure.
    Each cone has a color, x, and y coordinate.
    """
    cones = []
    # Interleave left and right cones
    for left, right in zip(left_cones, right_cones):
        cones.append({'color': 'yellow', 'x': float(left[0]), 'y': float(left[1])})
        cones.append({'color': 'blue', 'x': float(right[0]), 'y': float(right[1])})
    
    # Handle cases where left_cones and right_cones have different lengths
    if len(left_cones) > len(right_cones):
        for left in left_cones[len(right_cones):]:
            cones.append({'color': 'yellow', 'x': float(left[0]), 'y': float(left[1])})
    elif len(right_cones) > len(left_cones):
        for right in right_cones[len(left_cones):]:
            cones.append({'color': 'blue', 'x': float(right[0]), 'y': float(right[1])})
    
    data = {'cones': cones}
    with open(filename, 'w') as file:
        yaml.dump(data, file)
    print(f"Cones positions saved to {filename}")

def calculate_total_length(path_points):
    diffs = np.diff(path_points, axis=0)
    segment_lengths = np.hypot(diffs[:, 0], diffs[:, 1])
    return np.sum(segment_lengths)

def scale_path(path_points, desired_length=5000.0):
    total_length = calculate_total_length(path_points)
    if total_length == 0:
        print("Total path length is zero. Cannot scale.")
        return path_points
    scale_factor = desired_length / total_length
    scaled_path = path_points * scale_factor
    print(f"Path scaled by a factor of {scale_factor:.3f} to achieve a total length of {desired_length} pixels.")
    return scaled_path

def remove_duplicate_consecutive_points(path_points):
    # Using numpy to remove consecutive duplicate points
    if len(path_points) == 0:
        return path_points
    path_array = np.array(path_points)
    # Find indices where the point is different from the previous
    diffs = np.diff(path_array, axis=0)
    non_duplicate_indices = np.any(diffs != 0, axis=1)
    # Always include the first point
    cleaned_path = np.vstack([path_array[0], path_array[1:][non_duplicate_indices]])
    return cleaned_path

def handle_non_finite(path_points):
    # Remove any points that have NaN or Inf
    path_array = np.array(path_points)
    finite_mask = np.all(np.isfinite(path_array), axis=1)
    cleaned_path = path_array[finite_mask]
    return cleaned_path

def close_path_if_needed(path_points, closure_threshold=10.0):
    """
    Automatically closes the path by appending the start point to the end if the path is not closed.
    """
    start_point = path_points[0]
    end_point = path_points[-1]
    distance = np.linalg.norm(start_point - end_point)
    if distance > closure_threshold:
        print(f"Path is not closed (distance between start and end: {distance:.2f} > {closure_threshold}). Closing the path.")
        closed_path = np.vstack([path_points, start_point])
        return closed_path, True
    else:
        print(f"Path is closed (distance between start and end: {distance:.2f} <= {closure_threshold}).")
        return path_points, False

def calculate_cones(sampled_points, tangents_unit, distance):
    """
    Calculate left and right cones based on sampled points and unit tangent vectors.
    """
    normals_unit = np.vstack((-tangents_unit[:,1], tangents_unit[:,0])).T
    left_cones = sampled_points + normals_unit * distance
    right_cones = sampled_points - normals_unit * distance
    return left_cones, right_cones

def map_to_rectangle(points, target_width=800, target_height=600, padding=50):
    """
    Maps a set of points to fit within a target rectangle while preserving aspect ratio.

    Parameters:
    - points (np.ndarray): Array of shape (N, 2) containing x and y coordinates.
    - target_width (float): Width of the target rectangle.
    - target_height (float): Height of the target rectangle.
    - padding (float): Optional padding to be added around the track.

    Returns:
    - mapped_points (np.ndarray): Transformed points fitting within the target rectangle.
    """
    min_x, min_y = np.min(points, axis=0)
    max_x, max_y = np.max(points, axis=0)
    
    # Compute scaling factors
    scale_x = (target_width - 2 * padding) / (max_x - min_x) if max_x - min_x != 0 else 1.0
    scale_y = (target_height - 2 * padding) / (max_y - min_y) if max_y - min_y != 0 else 1.0
    scale = min(scale_x, scale_y)  # Preserve aspect ratio
    
    # Scale points
    scaled_points = (points - np.array([min_x, min_y])) * scale
    
    # Compute translation to center the track within the target rectangle
    current_width = (max_x - min_x) * scale
    current_height = (max_y - min_y) * scale
    translate_x = padding + (target_width - 2 * padding - current_width) / 2
    translate_y = padding + (target_height - 2 * padding - current_height) / 2
    
    # Translate points
    mapped_points = scaled_points + np.array([translate_x, translate_y])
    
    return mapped_points

def resample_cones(cones, desired_spacing=50.0):
    """
    Resamples cone positions to ensure regular spacing.

    Parameters:
    - cones (np.ndarray): Array of shape (N, 2) containing x and y coordinates of cones.
    - desired_spacing (float): Desired spacing between consecutive cones.

    Returns:
    - resampled_cones (np.ndarray): Array of shape (M, 2) with resampled cone positions.
    """
    if len(cones) < 2:
        return cones  # Not enough cones to resample

    # Calculate cumulative distances
    diffs = np.diff(cones, axis=0)
    distances = np.hypot(diffs[:, 0], diffs[:, 1])
    cumulative_distance = np.concatenate(([0], np.cumsum(distances)))
    
    # Define new distances
    new_distances = np.arange(0, cumulative_distance[-1], desired_spacing)
    
    # Interpolate new points
    interp_func_x = interp1d(cumulative_distance, cones[:, 0], kind='linear')
    interp_func_y = interp1d(cumulative_distance, cones[:, 1], kind='linear')
    
    new_x = interp_func_x(new_distances)
    new_y = interp_func_y(new_distances)
    
    resampled_cones = np.vstack((new_x, new_y)).T
    return resampled_cones

def main():
    # Step 1: Draw freehand path
    fig, ax = plt.subplots()
    ax.set_title('Draw Path Freehand\nPress and hold left mouse button to draw')
    ax.set_xlim(0, 1000)
    ax.set_ylim(0, 1000)
    drawer = FreehandDrawing(ax)
    plt.show()
    path_points = drawer.get_path()

    if len(path_points) < 2:
        print("Not enough points drawn. Please draw a path with multiple points.")
        sys.exit(1)

    # Convert path_points to numpy array
    path_points = np.array(path_points)

    # Step 2: Remove duplicate consecutive points
    path_points = remove_duplicate_consecutive_points(path_points)
    print(f"Path points after removing duplicates: {len(path_points)}")

    if len(path_points) < 4:
        print("Not enough unique points after removing duplicates. Please draw a more detailed path.")
        sys.exit(1)

    # Step 3: Handle non-finite values
    path_points = handle_non_finite(path_points)
    print(f"Path points after removing non-finite values: {len(path_points)}")

    if len(path_points) < 4:
        print("Not enough valid points after removing non-finite values. Please redraw the path.")
        sys.exit(1)

    # Step 4: Automatically close the path if needed
    closure_threshold = 10.0  # pixels
    path_points, was_closed = close_path_if_needed(path_points, closure_threshold=closure_threshold)

    # Step 5: Scale the path to have at least 5000 pixels in total length
    desired_total_length = 5000.0  # in pixels
    total_length = calculate_total_length(path_points)
    print(f"Original total path length: {total_length:.2f} pixels.")

    if total_length < desired_total_length:
        path_points = scale_path(path_points, desired_length=desired_total_length)
    else:
        print("No scaling needed. Path length meets the requirement.")

    # Step 6: Fit spline to the path
    try:
        # Ensure the path is closed for periodic spline
        if was_closed:
            per = True
        else:
            per = False

        # Fit spline with periodic parameter if path is closed
        tck, u = splprep([path_points[:, 0], path_points[:, 1]], s=10.0, per=per)

    except Exception as e:
        print(f"Error during spline fitting: {e}")
        sys.exit(1)

    # Generate a high-resolution set of points along the spline
    num_fine_points = 5000  # Increased for smoother sampling
    u_fine = np.linspace(0, 1, num=num_fine_points)
    x_fine, y_fine = splev(u_fine, tck)

    # Compute cumulative arc length
    dx = np.diff(x_fine)
    dy = np.diff(y_fine)
    ds = np.hypot(dx, dy)
    s = np.concatenate(([0], np.cumsum(ds)))  # Cumulative distance
    total_length_fine = s[-1]

    # Interpolation function from arc length to parameter u
    try:
        u_of_s = interp1d(s, u_fine)
    except Exception as e:
        print(f"Error during interpolation: {e}")
        sys.exit(1)

    # Decide on desired spacing between cones along the path
    distance_between_cones_along_path = 50.0  # pixels

    # Generate sample points at regular intervals along the arc length
    s_sampled = np.arange(0, total_length_fine, distance_between_cones_along_path)
    # Handle the case where s_sampled may exceed s[-1] due to floating point
    s_sampled = s_sampled[s_sampled < s[-1]]
    u_sampled = u_of_s(s_sampled)
    x_sampled, y_sampled = splev(u_sampled, tck)
    sampled_points = np.vstack((x_sampled, y_sampled)).T

    # Compute derivatives at sampled points (tangent vectors)
    dxdu, dydu = splev(u_sampled, tck, der=1)
    tangents = np.vstack((dxdu, dydu)).T
    tangent_norms = np.hypot(dxdu, dydu)
    # Avoid division by zero
    tangent_norms[tangent_norms == 0] = 1.0
    tangents_unit = tangents / tangent_norms[:, np.newaxis]

    # Calculate cone positions on both sides
    distance_to_cones = 100.0  # pixels
    left_cones, right_cones = calculate_cones(sampled_points, tangents_unit, distance_to_cones)

    # Step 7: Resample cones to ensure regular spacing
    desired_cone_spacing = distance_between_cones_along_path  # pixels
    left_cones = resample_cones(left_cones, desired_spacing=desired_cone_spacing)
    right_cones = resample_cones(right_cones, desired_spacing=desired_cone_spacing)
    print(f"Left cones resampled to {len(left_cones)} cones.")
    print(f"Right cones resampled to {len(right_cones)} cones.")

    # Step 8: Map all points to the target rectangle
    # Combine path and cones for transformation
    all_path_points = np.vstack([x_fine, y_fine]).T
    all_cones = np.vstack([left_cones, right_cones])

    # Map the path
    mapped_path = map_to_rectangle(all_path_points, target_width=800, target_height=600, padding=50)

    # Map the cones
    mapped_cones = map_to_rectangle(all_cones, target_width=800, target_height=600, padding=50)

    # Split back the mapped cones into left and right
    mapped_left_cones = mapped_cones[:len(left_cones)]
    mapped_right_cones = mapped_cones[len(left_cones):]

    # Step 9: Plot the result
    fig, ax = plt.subplots(figsize=(10, 8))
    
    # Optionally, draw the target rectangle for reference
    rectangle = plt.Rectangle((0, 0), 800, 600, linewidth=2, edgecolor='green', facecolor='none', linestyle='--', label='Target Rectangle')
    ax.add_patch(rectangle)
    
    ax.plot(mapped_path[:, 0], mapped_path[:, 1], 'k-', linewidth=2, label='Smoothed Path')  # Draw smoothed path
    ax.plot(mapped_left_cones[:, 0], mapped_left_cones[:, 1], 'yo', markersize=4, label='Left Cones (Yellow)')  # Draw left cones
    ax.plot(mapped_right_cones[:, 0], mapped_right_cones[:, 1], 'bo', markersize=4, label='Right Cones (Blue)')  # Draw right cones
    ax.legend()
    ax.set_title('Mapped Closed Path with Regularly Displaced Cones within Rectangle (0,0) to (800,600)')
    ax.set_xlim(0, 800)
    ax.set_ylim(0, 600)
    ax.set_aspect('equal', 'box')
    plt.show()

    # Step 10: Save to YAML
    # Combine left and right cones
    save_to_yaml(mapped_left_cones, mapped_right_cones)

def smooth_cone_positions(cones, window_size=5):
    """
    Smooth cone positions using a simple moving average.
    """
    smoothed = []
    half_window = window_size // 2
    for i in range(len(cones)):
        window = cones[max(i - half_window, 0) : min(i + half_window + 1, len(cones))]
        smoothed_point = np.mean(window, axis=0)
        smoothed.append(smoothed_point)
    return np.array(smoothed)

if __name__ == "__main__":
    main()
