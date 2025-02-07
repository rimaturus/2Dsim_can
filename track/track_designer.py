#!/usr/bin/env python3
import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import splprep, splev
import yaml

def polygon_area(x, y):
    return 0.5 * np.sum(x[:-1] * y[1:] - x[1:] * y[:-1])

class FreehandDrawing:
    def __init__(self):
        self.xs = []
        self.ys = []
        self.drawing = False

        self.fig, self.ax = plt.subplots()
        self.ax.set_title("Draw the centerline (click & drag). Close window when done.")
        self.line, = self.ax.plot([], [], '-k', lw=2)

        self.cid_press = self.fig.canvas.mpl_connect('button_press_event', self.on_press)
        self.cid_release = self.fig.canvas.mpl_connect('button_release_event', self.on_release)
        self.cid_motion = self.fig.canvas.mpl_connect('motion_notify_event', self.on_motion)

    def on_press(self, event):
        if event.inaxes != self.ax:
            return
        self.drawing = True
        self.xs.append(event.xdata)
        self.ys.append(event.ydata)
        self.line.set_data(self.xs, self.ys)
        self.fig.canvas.draw()

    def on_motion(self, event):
        if not self.drawing:
            return
        if event.inaxes != self.ax:
            return
        self.xs.append(event.xdata)
        self.ys.append(event.ydata)
        self.line.set_data(self.xs, self.ys)
        self.fig.canvas.draw()

    def on_release(self, event):
        self.drawing = False

    def get_points(self):
        return np.array(self.xs), np.array(self.ys)

def main():
    print("A drawing window will open. Click and drag to draw the track's centerline.")
    print("When finished, close the window.")
    drawing = FreehandDrawing()
    plt.show()  

    x, y = drawing.get_points()
    if len(x) < 2:
        print("Not enough points were drawn. Exiting.")
        return

    try:
        track_width  = float(input("Enter track width (in meters): "))
        area_width   = float(input("Enter track area width (in meters): "))
        area_height  = float(input("Enter track area height (in meters): "))
        cone_spacing = float(input("Enter spacing between cones of the same color (in meters): "))
    except ValueError:
        print("Invalid input. Exiting.")
        return

    min_x, max_x = np.min(x), np.max(x)
    min_y, max_y = np.min(y), np.max(y)
    if (max_x - min_x) == 0 or (max_y - min_y) == 0:
        print("Degenerate drawing. Exiting.")
        return

    # Uniform scale to fit inside [0, area_width] x [0, area_height]
    scale_factor = min(area_width/(max_x-min_x), area_height/(max_y-min_y))
    x_scaled = (x - min_x)*scale_factor
    y_scaled = (y - min_y)*scale_factor

    # If not closed, close it.
    if np.hypot(x_scaled[0]-x_scaled[-1], y_scaled[0]-y_scaled[-1]) > 0.1*cone_spacing:
        x_scaled = np.append(x_scaled, x_scaled[0])
        y_scaled = np.append(y_scaled, y_scaled[0])

    area_poly = polygon_area(x_scaled, y_scaled)
    if area_poly >= 0:
        print("Detected CCW drawing: inside on LEFT, outside on RIGHT.")
    else:
        print("Detected CW drawing: inside on RIGHT, outside on LEFT.")

    points = [x_scaled, y_scaled]
    tck, u = splprep(points, s=0, per=True)

    num_dense = 1000
    u_dense = np.linspace(0, 1, num_dense)
    x_dense, y_dense = splev(u_dense, tck)
    dx = np.diff(x_dense)
    dy = np.diff(y_dense)
    ds = np.sqrt(dx**2 + dy**2)
    arc_length = np.concatenate(([0], np.cumsum(ds)))
    total_length = arc_length[-1]

    num_samples = int(np.floor(total_length / cone_spacing))
    if num_samples < 1:
        print("Cone spacing bigger than track length. Exiting.")
        return

    sample_lengths = np.linspace(0, total_length, num_samples, endpoint=False)
    u_samples = np.interp(sample_lengths, arc_length, u_dense)

    x_samples, y_samples = splev(u_samples, tck)
    dx_samples, dy_samples = splev(u_samples, tck, der=1)
    tangents = np.vstack((dx_samples, dy_samples)).T
    norms = np.linalg.norm(tangents, axis=1, keepdims=True)
    tangents_normalized = tangents / norms

    # Perp vectors
    left_normals = np.column_stack((-tangents_normalized[:, 1], tangents_normalized[:, 0]))
    right_normals = -left_normals

    if area_poly >= 0:
        inside_offset = left_normals*(track_width/2)
        outside_offset = right_normals*(track_width/2)
    else:
        inside_offset = right_normals*(track_width/2)
        outside_offset = left_normals*(track_width/2)

    blue_cones = np.column_stack((x_samples + inside_offset[:,0],
                                  y_samples + inside_offset[:,1]))
    yellow_cones = np.column_stack((x_samples + outside_offset[:,0],
                                    y_samples + outside_offset[:,1]))

    # -------------------------------------------------
    # AVOID TOO-CLOSE CONES (post-processing step)
    # -------------------------------------------------
    min_dist = 2.0  # for example, 2 meters
    final_cone_list = []
    
    def distance2D(a, b):
        return np.sqrt((a[0] - b[0])**2 + (a[1] - b[1])**2)
    
    # We will store them in the same alternating order
    # but skip any new cone that is within min_dist of an already stored cone.
    raw_cones = []
    for i in range(num_samples):
        raw_cones.append(('yellow', float(yellow_cones[i, 0]), float(yellow_cones[i, 1])))
        raw_cones.append(('blue',   float(blue_cones[i, 0]),   float(blue_cones[i, 1])))

    for (color, xC, yC) in raw_cones:
        too_close = False
        for c in final_cone_list:
            if distance2D((xC, yC), (c['x'], c['y'])) < min_dist:
                too_close = True
                break
        if not too_close:
            final_cone_list.append({'color': color, 'x': xC, 'y': yC})

    # If we wanted strictly alternating color, we could do a more complicated pass,
    # but this simple approach will just skip any new cone that is too close to a previously
    # accepted cone (regardless of color). Adjust to your preference.

    data = {'cones': final_cone_list}
    yaml_filename = 'track.yaml'
    with open(yaml_filename, 'w') as f:
        yaml.dump(data, f, default_flow_style=False)

    print(f"Final cones saved to '{yaml_filename}' with near-duplicates removed.")

    # Plot the result for visualization
    plt.figure()
    plt.plot(x_scaled, y_scaled, 'k--', label='Centerline')
    # Extract final cones by color:
    fx_yellow = [c['x'] for c in final_cone_list if c['color']=='yellow']
    fy_yellow = [c['y'] for c in final_cone_list if c['color']=='yellow']
    fx_blue   = [c['x'] for c in final_cone_list if c['color']=='blue']
    fy_blue   = [c['y'] for c in final_cone_list if c['color']=='blue']

    plt.plot(fx_yellow, fy_yellow, 'yo', label='Yellow (outside) final')
    plt.plot(fx_blue,   fy_blue,   'bo', label='Blue (inside) final')
    plt.legend()
    plt.title("Track with final cone placement")
    plt.axis('equal')
    plt.show()

if __name__ == '__main__':
    main()
