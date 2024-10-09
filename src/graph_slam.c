#include <pthread.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "can_utils.h"
#include "graph_slam.h"

#define PERCEPTION_CAN_ID 0x200
#define IMU_CAN_ID 0x300
#define GRAPH_SLAM_CAN_ID 0x400

void* graph_slam_thread(void* args) {
    int can_sock = setup_can_socket(VCAN_INTERFACE);
    struct can_frame frame;
    int estimated_pose[3] = {0, 0, 0};  // {x_hat, y_hat, yaw_hat}

    while (1) {
        read_can_message(can_sock, &frame);

        if (frame.can_id == PERCEPTION_CAN_ID) {
            // Process detected cones (dummy processing)
            int cones[2];
            memcpy(cones, frame.data, sizeof(cones));
            printf("Graph SLAM: Received cone data {x: %d, y: %d}\n", cones[0], cones[1]);
        }

        if (frame.can_id == IMU_CAN_ID) {
            // Process IMU data
            int imu_data[2];
            memcpy(imu_data, frame.data, sizeof(imu_data));
            printf("Graph SLAM: Received IMU data {ax: %d, ay: %d}\n", imu_data[0], imu_data[1]);
        }

        // Estimate car pose (dummy values)
        estimated_pose[0] += 1;  // x position
        estimated_pose[1] += 2;  // y position
        estimated_pose[2] += 3;  // yaw

        send_can_message(can_sock, GRAPH_SLAM_CAN_ID, estimated_pose, sizeof(estimated_pose));
        usleep(300000);  // 300 ms delay
    }
    return NULL;  // Ensure function ends with a return statement
}
