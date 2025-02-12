import cv2
import numpy as np
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
from math import atan2, degrees, pi
from scipy.interpolate import splprep, splev
import yaml

##############################################################################
# 1)  Thresholding & morphological close
##############################################################################
def load_track_contours(filename):
    """
    Reads the image, thresholds "gray" track in HSV,
    does morphological close, then finds all contours
    (outer boundary + potential inner hole).

    Returns (image, [contour_1, contour_2, ...]) sorted by area (desc).
    """
    image = cv2.imread(filename)
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    # Threshold "gray"
    lower_gray = np.array([0, 0, 120], dtype=np.uint8)
    upper_gray = np.array([180, 40, 255], dtype=np.uint8)
    mask = cv2.inRange(hsv, lower_gray, upper_gray)

    # Morphological close to fill small gaps
    kernel = np.ones((25, 25), np.uint8)
    closed = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    # Find contours with RETR_CCOMP (or RETR_TREE) to capture both outer and inner
    contours, hierarchy = cv2.findContours(closed, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        raise ValueError("No contour found - check your threshold or image path!")
    
    # Sort by area descending
    contours = sorted(contours, key=cv2.contourArea, reverse=True)
    return image, contours

##############################################################################
# 2)  Simplify a contour with approxPolyDP to remove small bumps/noise
##############################################################################
def simplify_contour(contour, epsilon=5.0):
    # Convert from (N,1,2) -> (N,2)
    pts = contour.squeeze()
    if pts.ndim < 2:
        return contour  # fail-safe if contour is too small

    # Convert back to (N,1,2) for approxPolyDP
    c_reshape = pts.reshape(-1, 1, 2)
    approx = cv2.approxPolyDP(c_reshape, epsilon, True)
    return approx

##############################################################################
# 3)  Angle between consecutive edges (to detect "straight" merges)
##############################################################################
def angle_between_vectors(x1, y1, x2, y2):
    dot = x1 * x2 + y1 * y2
    mag1 = np.hypot(x1, y1)
    mag2 = np.hypot(x2, y2)
    if mag1 < 1e-9 or mag2 < 1e-9:
        return 0
    cos_ = dot / (mag1 * mag2)
    cos_ = max(-1.0, min(1.0, cos_))  # clip numerical noise
    theta = np.degrees(np.arccos(cos_))
    return theta

##############################################################################
# 4)  Split a simplified polygon into segments by "straight" corners
##############################################################################
def segment_straights(approx, straight_angle_thresh=5.0):
    """
    approx: shape (M,1,2) from approxPolyDP.
    We measure the angle between consecutive edges and if it's near 180°,
    merge them into one "straight" segment. Otherwise, start a new segment.
    """
    pts = approx.squeeze()  # shape (M,2)
    if pts.ndim < 2 or len(pts) < 2:
        return [pts.tolist()]  # trivial

    # Ensure closed polygon: if first and last differ, append the first point.
    if not np.all(pts[0] == pts[-1]):
        pts = np.vstack([pts, pts[0]])
    
    segments = []
    current_segment = [tuple(pts[0])]

    for i in range(1, len(pts) - 1):
        # vector from pts[i-1] -> pts[i]
        x1 = pts[i][0] - pts[i - 1][0]
        y1 = pts[i][1] - pts[i - 1][1]
        # vector from pts[i] -> pts[i+1]
        x2 = pts[i + 1][0] - pts[i][0]
        y2 = pts[i + 1][1] - pts[i][1]

        ang = angle_between_vectors(x1, y1, x2, y2)
        delta_from_180 = abs(180 - ang)

        current_segment.append(tuple(pts[i]))
        # If the turn is significant, end the current segment.
        if delta_from_180 > straight_angle_thresh:
            segments.append(current_segment)
            current_segment = [tuple(pts[i])]

    # Append the final segment.
    current_segment.append(tuple(pts[-1]))
    segments.append(current_segment)
    
    return segments

##############################################################################
# 5)  Fit a line or a spline
##############################################################################
def fit_line_segment(pts, n_samples=20):
    """
    Fit a line (degree=1) to the points.
    """
    pts = np.array(pts)
    x = pts[:, 0]
    y = pts[:, 1]
    t = np.arange(len(pts))
    
    px = np.polyfit(t, x, 1)
    py = np.polyfit(t, y, 1)

    t_new = np.linspace(0, len(pts) - 1, n_samples)
    x_new = np.polyval(px, t_new)
    y_new = np.polyval(py, t_new)
    return x_new, y_new

def fit_spline_segment(pts, n_samples=30, smoothing=0):
    """
    Fit a small B-spline for curvy segments.
    """
    pts = np.array(pts)
    x = pts[:, 0]
    y = pts[:, 1]
    t = np.arange(len(pts), dtype=float)
    if len(pts) < 3:
        # Not enough points for a spline => fallback to line
        return fit_line_segment(pts, n_samples)

    # Normalize parameter for spline
    t_norm = (t - t.min()) / (t.max() - t.min() + 1e-9)
    tck, _ = splprep([x, y], u=t_norm, s=smoothing, per=False)
    t_spl = np.linspace(0, 1, n_samples)
    x_spl, y_spl = splev(t_spl, tck)
    return x_spl, y_spl

def is_segment_straight(pts, line_rms_thresh=5.0):
    """
    Quick check: fit a line and measure RMS error.
    If RMS error is less than line_rms_thresh, treat as straight.
    """
    pts = np.array(pts)
    if len(pts) < 3:
        return True  # trivial short segment => treat as straight

    x = pts[:, 0]
    y = pts[:, 1]
    t = np.arange(len(pts))

    px = np.polyfit(t, x, 1)
    py = np.polyfit(t, y, 1)
    x_fit = np.polyval(px, t)
    y_fit = np.polyval(py, t)
    
    err = np.sqrt((x - x_fit)**2 + (y - y_fit)**2)
    rms = np.mean(err)
    return (rms < line_rms_thresh)

##############################################################################
# 6)  End-to-end function for "polynomial on straights, spline on curves"
##############################################################################
def process_contour(contour, 
                    epsilon=5.0, 
                    straight_angle_thresh=5.0, 
                    line_rms_thresh=5.0,
                    line_samples=15, 
                    spline_samples=30,
                    spline_smoothing=0):
    """
    1) Simplify with approxPolyDP(epsilon).
    2) Split polygon into segments by angle.
    3) For each segment, if it is "straight" (by RMS error), use a line fit;
       otherwise, fit a spline.
    4) Return combined x and y arrays representing the boundary.
    """
    # A) Simplify the contour
    approx = simplify_contour(contour, epsilon=epsilon)
    # B) Segment the polygon by angle
    rough_segments = segment_straights(approx, straight_angle_thresh=straight_angle_thresh)

    x_final = []
    y_final = []

    # C) Fit each segment (line or spline)
    for seg in rough_segments:
        if is_segment_straight(seg, line_rms_thresh=line_rms_thresh):
            xs, ys = fit_line_segment(seg, n_samples=line_samples)
        else:
            xs, ys = fit_spline_segment(seg, n_samples=spline_samples, smoothing=spline_smoothing)
        x_final.append(xs)
        y_final.append(ys)

    # D) Combine segments into one curve
    x_all = np.hstack(x_final)
    y_all = np.hstack(y_final)
    return x_all, y_all

##############################################################################
# 7)  Utility function to sample equispaced points along a curve
##############################################################################
def equispaced_points(x, y, spacing):
    """
    Given arrays x and y defining a curve, return new x and y arrays where the
    points are equispaced in terms of the curve's arc length.

    Parameters:
      x, y   : arrays of coordinates (in pixels)
      spacing: desired distance (in pixels) between consecutive points

    Returns:
      new_x, new_y: arrays of equispaced coordinates
    """
    x = np.array(x)
    y = np.array(y)
    # Compute distances between consecutive points
    dx = np.diff(x)
    dy = np.diff(y)
    dists = np.sqrt(dx**2 + dy**2)
    cum_dist = np.concatenate(([0], np.cumsum(dists)))
    total_length = cum_dist[-1]
    
    # Determine the number of points based on spacing
    n_points = max(int(total_length // spacing) + 1, 2)
    # Create new equally spaced distances along the curve
    new_d = np.linspace(0, total_length, n_points)
    new_x = np.interp(new_d, cum_dist, x)
    new_y = np.interp(new_d, cum_dist, y)
    return new_x, new_y

##############################################################################
# 8)  Main script: Process boundaries, compute cone positions, scale and output YAML
##############################################################################
def main():
    # A) Load the image and contours from the input file.
    image, contours = load_track_contours("Screenshots_1.png")
    # Largest contour is taken as the outer boundary.
    outer_contour = contours[0]
    # Second largest (if available) is the inner boundary.
    inner_contour = contours[1] if len(contours) > 1 else None

    # B) Process the outer contour.
    out_x, out_y = process_contour(
        outer_contour,
        epsilon=5.0,            # contour simplification parameter
        straight_angle_thresh=5.0,
        line_rms_thresh=5.0,
        line_samples=20,
        spline_samples=30,
        spline_smoothing=0
    )

    # C) Process the inner contour, if it exists.
    in_x, in_y = None, None
    if inner_contour is not None:
        in_x, in_y = process_contour(
            inner_contour,
            epsilon=5.0,
            straight_angle_thresh=5.0,
            line_rms_thresh=5.0,
            line_samples=20,
            spline_samples=30,
            spline_smoothing=0
        )

    # D) Compute equispaced points along each boundary.
    # Set cone_spacing = 100 px (which corresponds to 1 m when 1px = 0.01 m)
    cone_spacing = 100
    eq_out_x, eq_out_y = equispaced_points(out_x, out_y, cone_spacing)
    if in_x is not None:
        eq_in_x, eq_in_y = equispaced_points(in_x, in_y, cone_spacing)

    # E) Interlace the outer (yellow) and inner (blue) cone positions.
    cones = []
    if in_x is not None:
        n = max(len(eq_out_x), len(eq_in_x))
        for i in range(n):
            if i < len(eq_out_x):
                cones.append({"color": "yellow", "x": eq_out_x[i], "y": eq_out_y[i]})
            if i < len(eq_in_x):
                cones.append({"color": "blue", "x": eq_in_x[i], "y": eq_in_y[i]})
    else:
        for x, y in zip(eq_out_x, eq_out_y):
            cones.append({"color": "yellow", "x": x, "y": y})

    # F) Scale the cone positions so they fit within a window of 1800x1200 pixels.
    #    Then convert pixel coordinates to meters using: 1 px = 0.01 m.
    if cones:
        xs = [cone["x"] for cone in cones]
        ys = [cone["y"] for cone in cones]
        min_x, max_x = min(xs), max(xs)
        min_y, max_y = min(ys), max(ys)
        width = max_x - min_x
        height = max_y - min_y
        # Use a uniform scale factor so that the scaled bounding box fits in 1800x1200.
        scale_factor = min(1800/width, 1200/height) if width > 0 and height > 0 else 1.0

        # Convert the numpy scalars to plain Python floats.
        new_cones = []
        for cone in cones:
            scaled_x = (cone["x"] - min_x) * scale_factor
            scaled_y = (cone["y"] - min_y) * scale_factor
            new_cone = {
                "color": cone["color"],
                "x": float(scaled_x * 0.01),  # convert pixels to meters
                "y": float(scaled_y * 0.01),  # convert pixels to meters
            }
            new_cones.append(new_cone)
        cones = new_cones

    # G) Write the cone positions to a YAML file.
    cones_yaml = {"cones": cones}
    with open("cones.yaml", "w") as f:
        yaml.dump(cones_yaml, f, sort_keys=False)

    # H) (Optional) Plot the original image with the cone positions (unsclaed, for visualization).
    fig, ax = plt.subplots(figsize=(10, 6))
    ax.imshow(cv2.cvtColor(image, cv2.COLOR_BGR2RGB))
    from matplotlib.patches import RegularPolygon
    # For visualization, we use a cone radius in pixel-space.
    vis_cone_radius = 5  # 5 px (approximately corresponding to 5 cm at 0.01 m/px)

    # Draw outer cones (using the original equispaced points, not the scaled ones)
    for (x, y) in zip(eq_out_x, eq_out_y):
        cone = RegularPolygon((x, y), numVertices=3, radius=vis_cone_radius,
                                orientation=pi/2, color='yellow', ec='black')
        ax.add_patch(cone)

    # Draw inner cones if available
    if in_x is not None:
        for (x, y) in zip(eq_in_x, eq_in_y):
            cone = RegularPolygon((x, y), numVertices=3, radius=vis_cone_radius,
                                    orientation=pi/2, color='blue', ec='black')
            ax.add_patch(cone)
    
    # Create custom legend entries
    import matplotlib.patches as mpatches
    outer_patch = mpatches.Patch(color='yellow', label='Outer cones')
    inner_patch = mpatches.Patch(color='blue', label='Inner cones')
    ax.legend(handles=[outer_patch, inner_patch])

    ax.axis('off')
    ax.set_title("Track Delimiting Cones (Visualization)")
    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    main()
