#include <pthread.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include "can_utils.h"
#include "perception.h"

#define LIDAR_CAN_ID 0x100
#define PERCEPTION_CAN_ID 0x200

void* perception_thread(void* args) {
    int can_sock = setup_can_socket(VCAN_INTERFACE);
    struct can_frame frame;

    while (1) {
        // Listen for incoming LiDAR data
        read_can_message(can_sock, &frame);

        if (frame.can_id == LIDAR_CAN_ID) {
            int scan_angle;
            memcpy(&scan_angle, frame.data, sizeof(int));

            // Process and detect cones (dummy data)
            int detected_cones[2] = {scan_angle, scan_angle + 5};
            printf("Perception: Detected cones at angle %d\n", scan_angle);

            // Send cone information via CAN
            send_can_message(can_sock, PERCEPTION_CAN_ID, detected_cones, sizeof(detected_cones));
        }
        usleep(100000);  // 100 ms delay
    }
}
