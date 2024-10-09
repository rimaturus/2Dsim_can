#include <pthread.h>
#include <unistd.h>
#include <stdio.h>
#include "can_utils.h"
#include "lidar.h"

#define LIDAR_CAN_ID 0x100

void* lidar_thread(void* args) {
    int can_sock = setup_can_socket(VCAN_INTERFACE);
    int scan_angle = 0;

    while (1) {
        // Simulate point cloud data and send via CAN
        scan_angle = (scan_angle + 10) % 360;
        send_can_message(can_sock, LIDAR_CAN_ID, &scan_angle, sizeof(scan_angle));

        usleep(500000);  // 500 ms
    }
}
