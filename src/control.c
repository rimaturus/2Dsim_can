#include <pthread.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include "can_utils.h"
#include "control.h"
#include <string.h>

#define TRAJECTORY_PLANNER_CAN_ID 0x500
#define CONTROL_CAN_ID 0x600

void* control_thread(void* args) {
    int can_sock = setup_can_socket(VCAN_INTERFACE);
    struct can_frame frame;
    int control_signal[2] = {0, 0};  // {steer, pedal}

    while (1) {
        read_can_message(can_sock, &frame);

        if (frame.can_id == TRAJECTORY_PLANNER_CAN_ID) {
            int trajectory[2];
            memcpy(trajectory, frame.data, sizeof(trajectory));
            printf("Control: Received trajectory {x: %d, y: %d}\n", trajectory[0], trajectory[1]);

            // Generate control signals (dummy values)
            control_signal[0] = trajectory[0] / 2;  // steer
            control_signal[1] = trajectory[1] / 2;  // pedal

            send_can_message(can_sock, CONTROL_CAN_ID, control_signal, sizeof(control_signal));
        }
        usleep(100000);  // 100 ms delay
    }
}
