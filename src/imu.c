#include <pthread.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include "can_utils.h"
#include "imu.h"

#define IMU_CAN_ID 0x300

void* imu_thread(void* args) {
    int can_sock = setup_can_socket(VCAN_INTERFACE);
    int imu_data[2] = {0, 0};  // {ax, ay} acceleration values

    while (1) {
        // Generate dummy IMU data
        imu_data[0] = rand() % 10;  // Simulate acceleration in x
        imu_data[1] = rand() % 10;  // Simulate acceleration in y
        send_can_message(can_sock, IMU_CAN_ID, imu_data, sizeof(imu_data));

        usleep(200000);  // 200 ms delay
    }
}
