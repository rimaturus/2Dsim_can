#include <pthread.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include "can_utils.h"
#include "trajectory_planner.h"

#define GRAPH_SLAM_CAN_ID 0x400
#define TRAJECTORY_PLANNER_CAN_ID 0x500

void* trajectory_planner_thread(void* args) {
    int can_sock = setup_can_socket(VCAN_INTERFACE);
    struct can_frame frame;
    int reference_trajectory[2] = {0, 0};  // {x, y}

    while (1) {
        read_can_message(can_sock, &frame);

        if (frame.can_id == GRAPH_SLAM_CAN_ID) {
            int estimated_pose[3];
            memcpy(estimated_pose, frame.data, sizeof(estimated_pose));
            printf("Trajectory Planner: Received car pose {x: %d, y: %d}\n", estimated_pose[0], estimated_pose[1]);

            // Generate reference trajectory (dummy values)
            reference_trajectory[0] = estimated_pose[0] + 5;
            reference_trajectory[1] = estimated_pose[1] + 5;

            send_can_message(can_sock, TRAJECTORY_PLANNER_CAN_ID, reference_trajectory, sizeof(reference_trajectory));
        }
        usleep(200000);  // 200 ms delay
    }
}
