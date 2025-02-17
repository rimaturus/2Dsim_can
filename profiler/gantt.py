import csv
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
import numpy as np
import statistics
import threading

def read_intervals(csv_file):
    # Read all rows and compute a baseline (first time)
    with open(csv_file, 'r') as f:
        reader = list(csv.reader(f))
        if not reader:
            return []
        # Use the third field from the first row as baseline
        baseline = float(reader[3][2]) # the first 3 rows are headers
    
    start_times = {}
    intervals = []
    
    # Process rows with normalized times (in microseconds)
    for row in reader:
        # Skip every line that doesn't start with "["
        if not row[0].startswith('['):
            continue

        task, event, t = row
        t = float(t) - baseline  # normalized time in microseconds
        if event == "START":
            start_times[task] = t
        elif event == "END":
            start_t = start_times.pop(task, None)
            if start_t is not None:
                intervals.append((task, start_t, t))
    return intervals

def compute_stats(intervals):
    # Group intervals per task.
    stats_by_task = {}
    # To compute periodicity, store all start times (normalized)
    starts_by_task = {}
    
    for task, start, end in intervals:
        stats_by_task.setdefault(task, []).append(end - start)
        starts_by_task.setdefault(task, []).append(start)
    
    # Create subplots for distribution
    num_tasks = len(stats_by_task)
    fig, axs = plt.subplots(num_tasks, 1, figsize=(10, 4*num_tasks))
    if num_tasks == 1:
        axs = [axs]
    
    for idx, (task, runtimes) in enumerate(stats_by_task.items()):
        count = len(runtimes)
        mean_rt = sum(runtimes) / count
        dev_std = statistics.stdev(runtimes) if count > 1 else 0.0
        percentile_99 = np.percentile(runtimes, 99)
        
        # Sort the start times so we can compute periodicity
        starts = sorted(starts_by_task[task])
        if len(starts) > 1:
            periodicities = [j - i for i, j in zip(starts[:-1], starts[1:])]
            mean_periodicity = sum(periodicities) / len(periodicities)
            # Count deadline misses (assuming deadline = periodicity)
            deadline_misses = sum(1 for rt in runtimes if rt > mean_periodicity)
        else:
            mean_periodicity = 0.0
            deadline_misses = 0
        
        print(f"Task: {task}")
        print(f"  Count: {count}")
        print(f"  Mean Runtime: {mean_rt:.4f} µs")
        print(f"  99th Percentile Runtime: {percentile_99:.4f} µs")
        print(f"  Dev_std: {dev_std:.4f}")
        print(f"  Mean Periodicity: {mean_periodicity:.4f} µs")
        print(f"  Deadline Misses: {deadline_misses}\n")
        
        # Plot runtime distribution
        axs[idx].hist(runtimes, bins=30, density=True)
        axs[idx].axvline(mean_rt, color='r', linestyle='dashed', label='Mean')
        axs[idx].axvline(percentile_99, color='g', linestyle='dashed', label='99th percentile')
        if mean_periodicity > 0:
            axs[idx].axvline(mean_periodicity, color='y', linestyle='dashed', label='Period')
        axs[idx].set_title(f'{task} Runtime Distribution')
        axs[idx].set_xlabel('Runtime (µs)')
        axs[idx].set_ylabel('Density')
        axs[idx].legend()
    
    plt.tight_layout()
    plt.show()

def plot_gantt(intervals):
    import concurrent.futures
    
    try:
        import cupy as cp  # GPU acceleration
    except ImportError:
        print("Warning: cupy not found, using numpy instead")
        cp = np
    
    # Sort tasks by name or customized order
    tasks = sorted(set(task for task, _, _ in intervals))
    task_y = {task: i for i, task in enumerate(tasks)}
    
    fig, ax = plt.subplots()
    
    # Convert intervals data to GPU arrays
    starts = cp.array([start for _, start, _ in intervals])
    durations = cp.array([(end - start) for _, start, end in intervals])
    
    # Create thread pool
    lock = threading.Lock()
    def plot_interval(item):
        task, start, end = item
        with lock:
            ax.broken_barh([(float(start), float(end - start))],
                          (task_y[task]*10, 9),
                          facecolors=('tab:blue'))
    
    # Plot intervals in parallel
    with concurrent.futures.ThreadPoolExecutor() as executor:
        executor.map(plot_interval, intervals)
    
    # Compute time bounds on GPU
    min_time = float(cp.min(starts))
    max_time = float(cp.max(starts + durations))
    
    ax.set_yticks([task_y[t]*10 + 4.5 for t in tasks])
    ax.set_yticklabels(tasks)
    ax.set_xlabel("Time (µs)")
    ax.set_ylabel("Tasks")
    ax.set_title("Real-Time Task Execution Timeline")
    
    ax.set_xticks(cp.arange(min_time, max_time + 1000, 100000).get())
    ax.grid(True)
    
    plt.show()

if __name__ == "__main__":
    intervals = read_intervals("runtime.csv")
    if not intervals:
        print("No intervals found.")
    else:
        compute_stats(intervals)
        plot_gantt(intervals)
