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
        # Convert times to milliseconds.
        runtime_ms = (end - start) / 1000.0
        start_ms = start / 1000.0
        stats_by_task.setdefault(task, []).append(runtime_ms)
        starts_by_task.setdefault(task, []).append(start_ms)
    
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
        print(f"  Mean Runtime: {mean_rt:.4f} ms")
        print(f"  99th Percentile Runtime: {percentile_99:.4f} ms")
        print(f"  Dev_std: {dev_std:.4f}")
        print(f"  Mean Periodicity: {mean_periodicity:.4f} ms\n")

def plot_gantt(intervals):
    # Convert times to milliseconds for plotting.
    ms_intervals = [(task, start/1000.0, end/1000.0) for task, start, end in intervals]
    
    # Sort tasks by name or customized order
    tasks = sorted(set(task for task, _, _ in ms_intervals))
    task_y = {task: i for i, task in enumerate(tasks)}
    
    fig, ax = plt.subplots()
    for task, start, end in ms_intervals:
        ax.broken_barh([(start, end - start)],
                       (task_y[task]*10, 9),
                       facecolors=('tab:blue'))
    
    ax.set_yticks([task_y[t]*10 + 4.5 for t in tasks])
    ax.set_yticklabels(tasks)
    ax.set_xlabel("Time (ms)")
    ax.set_ylabel("Tasks")
    ax.set_title("Real-Time Task Execution Timeline")
    
    # Set grid with 100 ms spacing.
    min_time = min(start for _, start, _ in ms_intervals)
    max_time = max(end for _, _, end in ms_intervals)
    ax.set_xticks(np.arange(min_time, max_time + 1, 100))
    ax.grid(True)
    
    plt.show()

if __name__ == "__main__":
    intervals = read_intervals("runtime.csv")
    if not intervals:
        print("No intervals found.")
    else:
        compute_stats(intervals)
        plot_gantt(intervals)