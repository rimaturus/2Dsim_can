import matplotlib.pyplot as plt
import networkx as nx
import time
import os
import numpy as np
from scipy.interpolate import splprep, splev
from sklearn.cluster import KMeans

def read_graph(filename):
    """
    Reads the graph data from a file and returns poses, landmarks, and edges.
    """
    poses = {}
    landmarks = {}
    edges = []

    if not os.path.exists(filename):
        return poses, landmarks, edges

    with open(filename, 'r') as file:
        lines = file.readlines()
        mode = None

        for line in lines:
            line = line.strip()
            if line == "Poses:":
                mode = "poses"
                continue
            elif line == "Landmarks:":
                mode = "landmarks"
                continue
            elif line == "Measurements:":
                mode = "measurements"
                continue

            if mode == "poses":
                parts = line.split()
                if len(parts) >= 4:
                    idx = int(parts[0])
                    x = float(parts[1])
                    y = float(parts[2])
                    theta = float(parts[3])
                    poses[idx] = (x, y)
            elif mode == "landmarks":
                parts = line.split()
                if len(parts) >= 3:
                    idx = int(parts[0])
                    x = float(parts[1])
                    y = float(parts[2])
                    landmarks[idx] = (x, y)
            elif mode == "measurements":
                parts = line.split()
                if len(parts) >= 4:
                    pose_id = int(parts[0])
                    landmark_id = int(parts[1])
                    # Additional measurement data (range, bearing) can be read here if needed
                    edges.append((pose_id, f"L{landmark_id}"))

    return poses, landmarks, edges

def separate_landmarks(landmarks, poses):
    """
    Separates landmarks into inner and outer groups using K-Means clustering.
    
    Parameters:
    - landmarks: dict of landmark_id to (x, y)
    - poses: dict of pose_id to (x, y)
    
    Returns:
    - inner_landmarks: list of (x, y)
    - outer_landmarks: list of (x, y)
    """
    if len(landmarks) < 2:
        return [], []

    # Extract landmark coordinates
    landmark_coords = np.array(list(landmarks.values()))

    # Optional: Incorporate robot's pose to influence clustering
    # For simplicity, we'll proceed with K-Means on landmark positions only

    # Perform K-Means clustering with 2 clusters (inner and outer)
    kmeans = KMeans(n_clusters=2, random_state=0)
    labels = kmeans.fit_predict(landmark_coords)

    # Determine which cluster is inner and which is outer based on distance from robot's path
    # Compute centroid of each cluster
    centroids = kmeans.cluster_centers_

    # Compute average distance of each centroid from all poses
    centroid_distances = []
    pose_coords = np.array(list(poses.values()))
    for centroid in centroids:
        distances = np.linalg.norm(pose_coords - centroid, axis=1)
        centroid_distances.append(np.mean(distances))
    
    # The cluster with the smaller centroid distance is inner
    inner_cluster = 0 if centroid_distances[0] < centroid_distances[1] else 1

    # Separate landmarks
    inner_landmarks = landmark_coords[labels == inner_cluster].tolist()
    outer_landmarks = landmark_coords[labels != inner_cluster].tolist()

    return inner_landmarks, outer_landmarks

def plot_track_limits(inner_landmarks, outer_landmarks):
    """
    Fits splines to inner and outer landmarks and returns their coordinates.
    
    Parameters:
    - inner_landmarks: list of (x, y)
    - outer_landmarks: list of (x, y)
    
    Returns:
    - (x_inner, y_inner): coordinates for inner spline
    - (x_outer, y_outer): coordinates for outer spline
    """
    def fit_spline(landmarks):
        if len(landmarks) < 3:
            return None, None  # Not enough points to fit a spline
        landmarks = sorted(landmarks, key=lambda point: (point[0], point[1]))  # Sort to improve spline fitting
        x, y = zip(*landmarks)
        try:
            # Fit a spline to the landmark data
            tck, u = splprep([x, y], s=0, per=True)  # per=True for closed loop if applicable
            # Generate points along the spline
            u_fine = np.linspace(0, 1, 1000)
            x_fine, y_fine = splev(u_fine, tck)
            return x_fine, y_fine
        except Exception as e:
            print(f"Error fitting spline: {e}")
            return None, None

    x_inner, y_inner = fit_spline(inner_landmarks)
    x_outer, y_outer = fit_spline(outer_landmarks)

    return (x_inner, y_inner), (x_outer, y_outer)

def visualize_live(filename):
    """
    Visualizes the GraphSLAM data in real-time, including inner and outer track limits.
    
    Parameters:
    - filename: path to the graph data file
    """
    plt.ion()  # Turn on interactive mode
    fig, ax = plt.subplots(figsize=(12, 8))

    while True:
        poses, landmarks, edges = read_graph(filename)

        if not poses and not landmarks:
            time.sleep(0.5)
            continue

        G = nx.Graph()

        # Add pose nodes
        for idx, (x, y) in poses.items():
            G.add_node(idx, pos=(x, y), node_type='pose')

        # Add landmark nodes
        for idx, (x, y) in landmarks.items():
            G.add_node(f"L{idx}", pos=(x, y), node_type='landmark')

        # Add edges
        G.add_edges_from(edges)

        pos = nx.get_node_attributes(G, 'pos')
        node_types = nx.get_node_attributes(G, 'node_type')

        ax.clear()

        # Draw pose nodes
        pose_nodes = [n for n in G.nodes if node_types.get(n) == 'pose']
        nx.draw_networkx_nodes(G, pos, nodelist=pose_nodes, node_color='skyblue', node_size=100, ax=ax, label='Poses')

        # Draw landmark nodes
        landmark_nodes = [n for n in G.nodes if node_types.get(n) == 'landmark']
        nx.draw_networkx_nodes(G, pos, nodelist=landmark_nodes, node_color='orange', node_shape='s', node_size=100, ax=ax, label='Landmarks')

        # Draw edges
        nx.draw_networkx_edges(G, pos, edgelist=edges, edge_color='gray', ax=ax)

        # Draw labels
        labels = {n: n for n in G.nodes}
        nx.draw_networkx_labels(G, pos, labels, font_size=8, ax=ax)

        # Separate landmarks into inner and outer
        inner_landmarks, outer_landmarks = separate_landmarks(landmarks, poses)

        # Plot track limits by fitting splines to inner and outer landmarks
        (x_inner, y_inner), (x_outer, y_outer) = plot_track_limits(inner_landmarks, outer_landmarks)

        if x_inner is not None and y_inner is not None:
            ax.plot(x_inner, y_inner, 'r-', linewidth=2, label='Inner Track Limit')

        if x_outer is not None and y_outer is not None:
            ax.plot(x_outer, y_outer, 'b-', linewidth=2, label='Outer Track Limit')

        # Adjust plot settings
        ax.set_aspect('equal')
        ax.set_title('Real-Time GraphSLAM Visualization with Track Limits')
        ax.legend(loc='upper right')
        ax.grid(True)

        plt.draw()
        plt.pause(0.1)

        # Save the current map as an image
        plt.savefig('graph_slam_map.png')

        time.sleep(0.5)  # Wait before the next update

def separate_landmarks(landmarks, poses):
    """
    Separates landmarks into inner and outer groups using K-Means clustering.
    
    Parameters:
    - landmarks: dict of landmark_id to (x, y)
    - poses: dict of pose_id to (x, y)
    
    Returns:
    - inner_landmarks: list of (x, y)
    - outer_landmarks: list of (x, y)
    """
    if len(landmarks) < 2:
        return [], []

    # Extract landmark coordinates
    landmark_coords = np.array(list(landmarks.values()))

    # Perform K-Means clustering with 2 clusters (inner and outer)
    kmeans = KMeans(n_clusters=2, random_state=0)
    labels = kmeans.fit_predict(landmark_coords)

    # Determine which cluster is inner and which is outer based on distance from robot's poses
    # Compute centroid of each cluster
    centroids = kmeans.cluster_centers_

    # Compute average distance of each centroid from all poses
    centroid_distances = []
    pose_coords = np.array(list(poses.values()))
    for centroid in centroids:
        distances = np.linalg.norm(pose_coords - centroid, axis=1)
        centroid_distances.append(np.mean(distances))
    
    # The cluster with the smaller centroid distance is inner
    inner_cluster = 0 if centroid_distances[0] < centroid_distances[1] else 1

    # Separate landmarks
    inner_landmarks = landmark_coords[labels == inner_cluster].tolist()
    outer_landmarks = landmark_coords[labels != inner_cluster].tolist()

    return inner_landmarks, outer_landmarks

def plot_track_limits(inner_landmarks, outer_landmarks):
    """
    Fits splines to inner and outer landmarks and returns their coordinates.
    
    Parameters:
    - inner_landmarks: list of (x, y)
    - outer_landmarks: list of (x, y)
    
    Returns:
    - (x_inner, y_inner): coordinates for inner spline
    - (x_outer, y_outer): coordinates for outer spline
    """
    def fit_spline(landmarks):
        if len(landmarks) < 3:
            return None, None  # Not enough points to fit a spline
        landmarks = sorted(landmarks, key=lambda point: (point[0], point[1]))  # Sort to improve spline fitting
        x, y = zip(*landmarks)
        try:
            # Fit a spline to the landmark data
            tck, u = splprep([x, y], s=0, per=True)  # per=True for closed loop if applicable
            # Generate points along the spline
            u_fine = np.linspace(0, 1, 1000)
            x_fine, y_fine = splev(u_fine, tck)
            return x_fine, y_fine
        except Exception as e:
            print(f"Error fitting spline: {e}")
            return None, None

    x_inner, y_inner = fit_spline(inner_landmarks)
    x_outer, y_outer = fit_spline(outer_landmarks)

    return (x_inner, y_inner), (x_outer, y_outer)

if __name__ == "__main__":
    visualize_live("graph_live_output.txt")
