#!/usr/bin/env python3

import cv2
import numpy as np
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
import yaml

from shapely.geometry import LineString
from shapely.geometry import Point


def polygon_area(xs, ys):
    """
    Returns signed area of the polygon formed by (xs, ys).
    Positive if counterclockwise, negative if clockwise.
    """
    return 0.5 * np.sum(xs[:-1]*ys[1:] - xs[1:]*ys[:-1])

def sample_linestring_by_distance(line: LineString, spacing: float):
    """
    Given a Shapely LineString, sample points at regular arc-length
    intervals of 'spacing'. Returns lists of x_coords, y_coords.
    """
    length = line.length
    # We'll stop before 'length' so it doesn't duplicate endpoints if closed
    distances = np.arange(0, length, spacing)
    xs = []
    ys = []
    for d in distances:
        pt = line.interpolate(d)
        xs.append(pt.x)
        ys.append(pt.y)
    return xs, ys

def largest_line_part(geom):
    """
    If 'geom' is a MultiLineString, return the sub-part with greatest length.
    If it's a single LineString, return it directly.
    """
    if geom.is_empty:
        return None
    if geom.geom_type == 'LineString':
        return geom
    if geom.geom_type == 'MultiLineString':
        # pick the one with the greatest length
        return max(geom, key=lambda g: g.length)
    return None

def main():
    #-----------------------------------------------------------------------
    # 1) LOAD IMAGE + FIND LARGEST CONTOUR
    #-----------------------------------------------------------------------
    image_path = "/home/edo/Music/f1_Catalunya_circuit.png"
    img = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)
    if img is None:
        print(f"Could not load image '{image_path}'. Exiting.")
        return

    # Threshold (invert so black line becomes white shape)
    _, thresh = cv2.threshold(img, 50, 255, cv2.THRESH_BINARY_INV)

    contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    if not contours:
        print("No contour found in image. Exiting.")
        return

    largest_contour = max(contours, key=cv2.contourArea)
    largest_contour = np.squeeze(largest_contour).astype(float)  # shape (N,2)
    if largest_contour.ndim != 2 or largest_contour.shape[1] != 2:
        print("Contour shape unexpected. Exiting.")
        return

    xs, ys = largest_contour[:,0], largest_contour[:,1]

    #-----------------------------------------------------------------------
    # 2) PARAMETERS
    #-----------------------------------------------------------------------
    track_width  = 3.0    # meters
    cone_spacing = 1.0    # meters
    area_width   = 100.0
    area_height  = 100.0

    #-----------------------------------------------------------------------
    # 3) SCALE INTO 300×300 BOX
    #-----------------------------------------------------------------------
    min_x, max_x = np.min(xs), np.max(xs)
    min_y, max_y = np.min(ys), np.max(ys)
    dx = max_x - min_x
    dy = max_y - min_y
    if dx < 1e-6 or dy < 1e-6:
        print("Degenerate contour. Exiting.")
        return

    scale_factor = min(area_width / dx, area_height / dy)
    xs_scaled = (xs - min_x)*scale_factor
    ys_scaled = (ys - min_y)*scale_factor

    # Ensure it's closed (append first point at the end if needed)
    if np.hypot(xs_scaled[0] - xs_scaled[-1],
                ys_scaled[0] - ys_scaled[-1]) > 1e-3:
        xs_scaled = np.append(xs_scaled, xs_scaled[0])
        ys_scaled = np.append(ys_scaled, ys_scaled[0])

    # Detect if it's CCW or CW
    signed_area = polygon_area(xs_scaled, ys_scaled)
    if signed_area > 0:
        print("Track is counterclockwise; inside cones on left.")
        inside_is_left = True
    else:
        print("Track is clockwise; inside cones on right.")
        inside_is_left = False

    #-----------------------------------------------------------------------
    # 4) BUILD SHAPELY LINESTRING FOR THE "CENTER"
    #-----------------------------------------------------------------------
    center_coords = list(zip(xs_scaled, ys_scaled))
    center_line = LineString(center_coords)

    #-----------------------------------------------------------------------
    # 5) OFFSET EACH SIDE USING `parallel_offset`
    #    We want half the track_width offset on each side.
    #-----------------------------------------------------------------------
    offset_dist = track_width / 2.0

    if inside_is_left:
        # inside is left, outside is right
        inside_geom  = center_line.parallel_offset(offset_dist, 'left',
                                                   join_style=1, # round corners
                                                   resolution=256)
        outside_geom = center_line.parallel_offset(offset_dist, 'right',
                                                   join_style=1,
                                                   resolution=256)
    else:
        # inside is right, outside is left
        inside_geom  = center_line.parallel_offset(offset_dist, 'right',
                                                   join_style=1,
                                                   resolution=256)
        outside_geom = center_line.parallel_offset(offset_dist, 'left',
                                                   join_style=1,
                                                   resolution=256)

    inside_line  = largest_line_part(inside_geom)
    outside_line = largest_line_part(outside_geom)
    if not inside_line or not outside_line:
        print("Offset failed or produced empty geometry.")
        return

    #-----------------------------------------------------------------------
    # 6) SAMPLE EACH OFFSET LINE BY ARC LENGTH
    #-----------------------------------------------------------------------
    blue_x,   blue_y   = sample_linestring_by_distance(inside_line,  cone_spacing)
    yellow_x, yellow_y = sample_linestring_by_distance(outside_line, cone_spacing)

    #-----------------------------------------------------------------------
    # 7) SHIFT COORDINATES TO ENSURE ALL ARE >= 0
    #    We find the minimum x and y across all cones. If negative,
    #    we shift everything up so that min becomes 0.
    #-----------------------------------------------------------------------
    all_x = np.concatenate([blue_x, yellow_x])
    all_y = np.concatenate([blue_y, yellow_y])

    min_all_x = np.min(all_x)
    min_all_y = np.min(all_y)

    # Calculate how much to shift (only if negative)
    shift_x = 0.0
    shift_y = 0.0
    if min_all_x < 0:
        shift_x = -min_all_x
    if min_all_y < 0:
        shift_y = -min_all_y

    # Apply shift
    blue_x   = [x + shift_x for x in blue_x]
    blue_y   = [y + shift_y for y in blue_y]
    yellow_x = [x + shift_x for x in yellow_x]
    yellow_y = [y + shift_y for y in yellow_y]

    #-----------------------------------------------------------------------
    # 8) SAVE YAML
    #    We'll simply alternate: yellow -> blue -> yellow -> blue
    #-----------------------------------------------------------------------
    cone_entries = []
    N = min(len(blue_x), len(yellow_x))
    for i in range(N):
        cone_entries.append({
            'color': 'yellow',
            'x': float(yellow_x[i]),
            'y': float(yellow_y[i]),
        })
        cone_entries.append({
            'color': 'blue',
            'x': float(blue_x[i]),
            'y': float(blue_y[i]),
        })
    data = {'cones': cone_entries}

    outfile = 'Catalunya.yaml'
    with open(outfile, 'w') as f:
        yaml.dump(data, f, sort_keys=False)

    print(f"Saved {len(cone_entries)} cones to {outfile}")
    print("All coordinates are guaranteed >= 0.")

    #-----------------------------------------------------------------------
    # 9) OPTIONAL: PLOT
    #-----------------------------------------------------------------------
    plt.figure()
    plt.scatter(blue_x,   blue_y,   c='b', s=10, label='Blue (inside, shifted)')
    plt.scatter(yellow_x, yellow_y, c='y', s=10, label='Yellow (outside, shifted)')
    plt.axis('equal')
    plt.legend()
    plt.title('Shapely offset + shift to positive quadrant')
    plt.show()

if __name__ == "__main__":
    main()
