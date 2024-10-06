import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import time
import threading
import os

# Path to the graph_live_output.txt file
GRAPH_FILE = 'graph_live_output.txt'

# Define colors for different types of landmarks
LANDMARK_COLORS = {
    'blue': 'blue',
    'yellow': 'yellow',
    'unknown': 'gray'
}

# Define markers for different types of landmarks
LANDMARK_MARKERS = {
    'blue': 'o',      # Circle for blue cones
    'yellow': 's',    # Square for yellow cones
    'unknown': 'x'     # Cross for unknown
}

def parse_graph_file(filepath):
    """
    Parses the graph_live_output.txt file and extracts landmarks.

    Parameters:
        filepath (str): Path to the graph_live_output.txt file.

    Returns:
        List of landmarks, where each landmark is a dictionary with keys:
            'id', 'x', 'y', 'type'
    """
    landmarks = []
    try:
        with open(filepath, 'r') as file:
            lines = file.readlines()
        
        # Find the index where Landmarks section starts
        try:
            landmarks_start = lines.index("Landmarks:\n") + 1
        except ValueError:
            # Landmarks section not found
            return landmarks
        
        # Iterate through the landmarks section
        for line in lines[landmarks_start:]:
            line = line.strip()
            if line == "" or line.endswith(":"):
                # Reached the end of Landmarks section
                break
            parts = line.split()
            if len(parts) < 4:
                # Malformed line
                continue
            landmark = {
                'id': int(parts[0]),
                'x': float(parts[1]),
                'y': float(parts[2]),
                'type': parts[3].lower()  # Ensure type is lowercase
            }
            landmarks.append(landmark)
    except FileNotFoundError:
        print(f"Graph file '{filepath}' not found.")
    except Exception as e:
        print(f"Error parsing graph file: {e}")
    
    return landmarks

def update_plot(ax, scatter_plots, landmarks):
    """
    Updates the plot with the current landmarks.

    Parameters:
        ax (matplotlib.axes.Axes): The axes to plot on.
        scatter_plots (dict): Dictionary to hold scatter plot objects for each type.
        landmarks (list): List of landmarks to plot.
    """
    # Clear previous scatter plots
    for key in scatter_plots:
        scatter_plots[key].remove()
    scatter_plots.clear()
    
    # Separate landmarks by type
    landmarks_by_type = {}
    for lm in landmarks:
        lm_type = lm['type']
        if lm_type not in landmarks_by_type:
            landmarks_by_type[lm_type] = {'x': [], 'y': []}
        landmarks_by_type[lm_type]['x'].append(lm['x'])
        landmarks_by_type[lm_type]['y'].append(lm['y'])
    
    # Plot landmarks for each type
    for lm_type, coords in landmarks_by_type.items():
        color = LANDMARK_COLORS.get(lm_type, 'black')
        marker = LANDMARK_MARKERS.get(lm_type, 'x')
        scatter = ax.scatter(coords['x'], coords['y'], c=color, marker=marker, label=lm_type.capitalize())
        scatter_plots[lm_type] = scatter
    
    # Update the legend to avoid duplicates
    handles, labels = ax.get_legend_handles_labels()
    unique = dict(zip(labels, handles))
    ax.legend(unique.values(), unique.keys())
    
    # Redraw the plot
    plt.draw()
    plt.pause(0.001)

def plot_landmarks():
    """
    Initializes the plot and updates it in real-time based on the graph file.
    """
    plt.ion()  # Turn on interactive mode
    fig, ax = plt.subplots(figsize=(10, 10))
    ax.set_title("Real-Time Landmarks Visualization")
    ax.set_xlabel("X Position")
    ax.set_ylabel("Y Position")
    ax.grid(True)
    ax.set_aspect('equal', 'box')
    
    # Initialize scatter plots dictionary
    scatter_plots = {}
    
    # Initial plot
    landmarks = parse_graph_file(GRAPH_FILE)
    update_plot(ax, scatter_plots, landmarks)
    
    while True:
        if os.path.exists(GRAPH_FILE):
            landmarks = parse_graph_file(GRAPH_FILE)
            update_plot(ax, scatter_plots, landmarks)
        else:
            print(f"Graph file '{GRAPH_FILE}' does not exist. Waiting...")
        time.sleep(1)  # Update every 1 second

def main():
    """
    Main function to start the landmark plotting.
    """
    try:
        plot_landmarks()
    except KeyboardInterrupt:
        print("\nExiting landmark plotting.")
    except Exception as e:
        print(f"An error occurred: {e}")
    finally:
        plt.ioff()
        plt.show()

if __name__ == "__main__":
    main()
