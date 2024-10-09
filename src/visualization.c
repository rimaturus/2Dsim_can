#include <pthread.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include "can_utils.h"
#include "visualization.h"

#define CONTROL_CAN_ID 0x600

void* visualization_thread(void* args) {
    int can_sock = setup_can_socket(VCAN_INTERFACE);
    struct can_frame frame;

    while (1) {
        read_can_message(can_sock, &frame);

        if (frame.can_id == CONTROL_CAN_ID) {
            int control_signal[2];
            memcpy(control_signal, frame.data, sizeof(control_signal));
            printf("Visualization: Received control signals {steer: %d, pedal: %d}\n", control_signal[0], control_signal[1]);
        }
        usleep(500000);  // 500 ms delay
    }
}
