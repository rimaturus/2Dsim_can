#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <pthread.h>
#include <plplot/plplot.h>  // For plotting
#include <csv.h>            // libcsv for CSV parsing
#include <unistd.h>  // for sleep()

#define MAX_TASKS 100
#define MAX_INTERVALS 100000
#define MAX_LINE 100000

typedef struct {
    char task[64];
    double start;
    double end;
} Interval;

typedef struct {
    char task[64];
    double start_time;
} StartTime;

typedef struct {
    Interval* intervals;
    int count;
    char* title_suffix;
} PlotArgs;

// Store intervals and start times
Interval intervals[MAX_INTERVALS];
StartTime start_times[MAX_TASKS];
int interval_count = 0;
int start_times_count = 0;

double baseline = 0.0;

// Function to compare strings for qsort
int compare_strings(const void* a, const void* b) {
    return strcmp(*(const char**)a, *(const char**)b);
}

void compute_stats(Interval* intervals, int count) {
    // Group intervals by task
    char* unique_tasks[MAX_TASKS];
    int task_count = 0;
    
    // Collect unique tasks
    for (int i = 0; i < count; i++) {
        int found = 0;
        for (int j = 0; j < task_count; j++) {
            if (strcmp(intervals[i].task, unique_tasks[j]) == 0) {
                found = 1;
                break;
            }
        }
        if (!found) {
            unique_tasks[task_count] = strdup(intervals[i].task);
            task_count++;
        }
    }
    
    // Sort tasks alphabetically
    qsort(unique_tasks, task_count, sizeof(char*), compare_strings);
    
    // Compute statistics for each task
    for (int i = 0; i < task_count; i++) {
        double runtimes[MAX_INTERVALS];
        double starts[MAX_INTERVALS];
        int runtime_count = 0;
        
        // Collect runtimes and start times for this task
        for (int j = 0; j < count; j++) {
            if (strcmp(intervals[j].task, unique_tasks[i]) == 0) {
                runtimes[runtime_count] = intervals[j].end - intervals[j].start;
                starts[runtime_count] = intervals[j].start;
                runtime_count++;
            }
        }
        
        // Calculate statistics
        double mean_rt = 0.0;
        double mean_periodicity = 0.0;
        
        for (int j = 0; j < runtime_count; j++) {
            mean_rt += runtimes[j];
            if (j > 0) {
                mean_periodicity += starts[j] - starts[j-1];
            }
        }
        
        mean_rt /= runtime_count;
        mean_periodicity = (runtime_count > 1) ? mean_periodicity / (runtime_count - 1) : 0.0;
        
        printf("Task: %s\n", unique_tasks[i]);
        printf("  Count: %d\n", runtime_count);
        printf("  Mean Runtime: %.4f µs\n", mean_rt);
        printf("  Mean Periodicity: %.4f µs\n\n", mean_periodicity);
        
        free(unique_tasks[i]);
    }
}

void* plot_thread(void* args) {
    PlotArgs* plot_args = (PlotArgs*)args;
    Interval* intervals = plot_args->intervals;
    int count = plot_args->count;

    fprintf(stderr, "Starting plot_thread with %d intervals\n", count);

    // Initialize PLplot with X11 driver
    plsdev("xwin");
    plinit();
    fprintf(stderr, "plinit complete\n");

    // Set background color to white and foreground to black
    plscol0(0, 255, 255, 255);  // background
    plscol0(15, 0, 0, 0);       // foreground text
    plscol0(1, 0, 0, 255);      // blue for bars
    
    // Create plot
    char* unique_tasks[MAX_TASKS];
    int task_count = 0;
    
    // Collect unique tasks
    for (int i = 0; i < count; i++) {
        int found = 0;
        for (int j = 0; j < task_count; j++) {
            if (strcmp(intervals[i].task, unique_tasks[j]) == 0) {
                found = 1;
                break;
            }
        }
        if (!found) {
            unique_tasks[task_count] = strdup(intervals[i].task);
            task_count++;
        }
    }
    
    // Sort tasks
    qsort(unique_tasks, task_count, sizeof(char*), compare_strings);
    
    // Set up plot
    pladv(0);
    plvsta();
    
    // Find time range
    double min_time = intervals[0].start;
    double max_time = intervals[0].end;
    for (int i = 1; i < count; i++) {
        if (intervals[i].start < min_time) min_time = intervals[i].start;
        if (intervals[i].end > max_time) max_time = intervals[i].end;
    }
    
    plwind(min_time, max_time, 0, task_count * 10);
    
    // Draw intervals
    for (int i = 0; i < count; i++) {
        int task_idx = 0;
        for (int j = 0; j < task_count; j++) {
            if (strcmp(intervals[i].task, unique_tasks[j]) == 0) {
                task_idx = j;
                break;
            }
        }
        
        PLFLT x[4] = {intervals[i].start, intervals[i].end, intervals[i].end, intervals[i].start};
        PLFLT y[4] = {task_idx * 10, task_idx * 10, task_idx * 10 + 9, task_idx * 10 + 9};
        plcol0(1);  // Blue color
        plfill(4, x, y);
    }
    
    // Add labels and title
    plcol0(15);  // White color for text
    pllab("Time (µs)", "Tasks", plot_args->title_suffix);
    
    // Clean up
    for (int i = 0; i < task_count; i++) {
        free(unique_tasks[i]);
    }
    
    // Force display update and wait
    plflush();
    sleep(10);  // Give the window time to display
    
    // Don't call plend() immediately - wait for user input
    printf("Press Enter to close the plot...\n");
    getchar();
    
    plend();
    return NULL;
}

// Add this function before main()
int read_csv_file(const char* filename, Interval* intervals, int* interval_count) {
    fprintf(stderr, "Attempting to open file: %s\n", filename);
    fflush(stderr);

    FILE* fp = fopen(filename, "r");
    if (!fp) {
        fprintf(stderr, "Error: Cannot open file %s\n", filename);
        fflush(stderr);
        return 0;
    }

    char line[MAX_LINE];
    int row = 0;
    *interval_count = 0;
    start_times_count = 0;
    double local_baseline = 0.0;

    // Skip header rows
    while (row < 3 && fgets(line, MAX_LINE, fp)) {
        row++;
    }
    fprintf(stderr, "Skipped %d header rows\n", row);
    fflush(stderr);

    if (row < 3) {
        fprintf(stderr, "Error: File %s has insufficient headers\n", filename);
        fflush(stderr);
        fclose(fp);
        return 0;
    }

    // Get baseline from third row
    char* token = strtok(line, ",");
    if (!token) goto error;
    token = strtok(NULL, ",");
    if (!token) goto error;
    token = strtok(NULL, ",");
    if (!token) goto error;
    local_baseline = atof(token);
    fprintf(stderr, "Local baseline set to %.4f\n", local_baseline);
    fflush(stderr);

    while (fgets(line, MAX_LINE, fp)) {
        if (line[0] != '[') continue;

        char task[64] = {0};
        char event[10] = {0};
        char* token;

        token = strtok(line, ",");
        if (!token) continue;
        strncpy(task, token, sizeof(task) - 1);

        token = strtok(NULL, ",");
        if (!token) continue;
        strncpy(event, token, sizeof(event) - 1);

        token = strtok(NULL, ",");
        if (!token) continue;
        double time = atof(token) - local_baseline;

        if (strcmp(event, "START") == 0) {
            if (start_times_count >= MAX_TASKS) {
                fprintf(stderr, "Warning: Too many START events\n");
                fflush(stderr);
                continue;
            }
            strncpy(start_times[start_times_count].task, task, sizeof(start_times[0].task) - 1);
            start_times[start_times_count].start_time = time;
            start_times_count++;
        } else if (strcmp(event, "END") == 0) {
            for (int i = 0; i < start_times_count; i++) {
                if (strcmp(start_times[i].task, task) == 0) {
                    if (*interval_count >= MAX_INTERVALS) {
                        fprintf(stderr, "Warning: Too many intervals\n");
                        fflush(stderr);
                        goto done;
                    }
                    strncpy(intervals[*interval_count].task, task, sizeof(intervals[0].task) - 1);
                    intervals[*interval_count].start = start_times[i].start_time;
                    intervals[*interval_count].end = time;
                    (*interval_count)++;

                    for (int j = i; j < start_times_count - 1; j++) {
                        start_times[j] = start_times[j + 1];
                    }
                    start_times_count--;
                    break;
                }
            }
        }
    }

done:
    fprintf(stderr, "Finished reading file. Total intervals: %d\n", *interval_count);
    fflush(stderr);
    fclose(fp);
    return 1;

error:
    fclose(fp);
    return 0;
}

int main() {
    Interval* intervals1 = malloc(MAX_INTERVALS * sizeof(Interval));
    Interval* intervals2 = malloc(MAX_INTERVALS * sizeof(Interval));
    if (!intervals1 || !intervals2) {
        fprintf(stderr, "Memory allocation failed\n");
        return 1;
    }
    int count1 = 0, count2 = 0;
    
    if (read_csv_file("/home/edo/unipi/1anno/istr/2Dsim_can/profiler/runtime.csv", intervals1, &count1)) {
        fprintf(stderr, "runtime.csv: count1 = %d\n", count1);
        if (count1 > 0) {
            compute_stats(intervals1, count1);
            PlotArgs args1 = {intervals1, count1, "Normal"};
            plot_thread(&args1);
        }
    } else {
        fprintf(stderr, "Failed to read runtime.csv\n");
    }
    
    if (read_csv_file("/home/edo/unipi/1anno/istr/2Dsim_can/profiler/runtime_autonomous.csv", intervals2, &count2)) {
        fprintf(stderr, "runtime_autonomous.csv: count2 = %d\n", count2);
        if (count2 > 0) {
            compute_stats(intervals2, count2);
            PlotArgs args2 = {intervals2, count2, "Autonomous"};
            plot_thread(&args2);
        }
    } else {
        fprintf(stderr, "Failed to read runtime_autonomous.csv\n");
    }
    
    free(intervals1);
    free(intervals2);
    
    return 0;
}