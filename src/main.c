#include <stdio.h>
#include <stdlib.h>
#include <pthread.h>
#include <signal.h>
#include "lidar.h"
#include "imu.h"
#include "perception.h"
#include "graph_slam.h"
#include "trajectory_planner.h"
#include "control.h"
#include "visualization.h"

pthread_t lidar, imu, perception, graph_slam, trajectory_planner, control, visualization;

void terminate_program(int signum) {
    printf("Terminating program...\n");
    pthread_cancel(lidar);
    pthread_cancel(imu);
    pthread_cancel(perception);
    pthread_cancel(graph_slam);
    pthread_cancel(trajectory_planner);
    pthread_cancel(control);
    pthread_cancel(visualization);
    exit(0);
}

int main() {
    signal(SIGINT, terminate_program);  // Capture CTRL+C to clean up threads

    pthread_create(&lidar, NULL, lidar_thread, NULL);
    pthread_create(&imu, NULL, imu_thread, NULL);
    pthread_create(&perception, NULL, perception_thread, NULL);
    pthread_create(&graph_slam, NULL, graph_slam_thread, NULL);
    pthread_create(&trajectory_planner, NULL, trajectory_planner_thread, NULL);
    pthread_create(&control, NULL, control_thread, NULL);
    pthread_create(&visualization, NULL, visualization_thread, NULL);

    pthread_join(lidar, NULL);
    pthread_join(imu, NULL);
    pthread_join(perception, NULL);
    pthread_join(graph_slam, NULL);
    pthread_join(trajectory_planner, NULL);
    pthread_join(control, NULL);
    pthread_join(visualization, NULL);

    return 0;
}
