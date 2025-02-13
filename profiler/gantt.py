import csv
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
import numpy as np
import statistics

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
    
    for task in stats_by_task:
        runtimes = stats_by_task[task]
        count = len(runtimes)
        mean_rt = sum(runtimes) / count
        dev_std = statistics.stdev(runtimes) if count > 1 else 0.0
        percentile_99 = np.percentile(runtimes, 99)
        
        # Sort the start times so we can compute periodicity
        starts = sorted(starts_by_task[task])
        if len(starts) > 1:
            periodicities = [j - i for i, j in zip(starts[:-1], starts[1:])]
            mean_periodicity = sum(periodicities) / len(periodicities)
        else:
            mean_periodicity = 0.0
        
        print(f"Task: {task}")
        print(f"  Count: {count}")
        print(f"  Mean Runtime: {mean_rt:.4f} µs")
        print(f"  99th Percentile Runtime: {percentile_99:.4f} µs")
        print(f"  Dev_std: {dev_std:.4f}")
        print(f"  Mean Periodicity: {mean_periodicity:.4f} µs\n")

def plot_gantt(intervals):
    # Sort tasks by name or customized order
    tasks = sorted(set(task for task, _, _ in intervals))
    task_y = {task: i for i, task in enumerate(tasks)}
    
    fig, ax = plt.subplots()
    for task, start, end in intervals:
        ax.broken_barh([(start, end - start)],
                       (task_y[task]*10, 9),
                       facecolors=('tab:blue'))
    
    ax.set_yticks([task_y[t]*10 + 4.5 for t in tasks])
    ax.set_yticklabels(tasks)
    ax.set_xlabel("Time (µs)")
    ax.set_ylabel("Tasks")
    ax.set_title("Real-Time Task Execution Timeline")
    
    # Set grid with 1000 µs == 1 ms spacing.
    # Find the overall time bounds.
    min_time = min(start for _, start, _ in intervals)
    max_time = max(end for _, _, end in intervals)
    ax.set_xticks(np.arange(min_time, max_time + 1000, 100000))
    ax.grid(True)
    
    plt.show()

if __name__ == "__main__":
    intervals = read_intervals("runtime.csv")
    if not intervals:
        print("No intervals found.")
    else:
        compute_stats(intervals)
        plot_gantt(intervals)
