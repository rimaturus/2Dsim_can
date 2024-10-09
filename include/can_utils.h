#ifndef CAN_UTILS_H
#define CAN_UTILS_H

#include <linux/can.h>

#define VCAN_INTERFACE "vcan0"

// Function prototypes
int setup_can_socket(const char *ifname);
void send_can_message(int s, int can_id, const void *data, int len);
int read_can_message(int s, struct can_frame *frame);

#endif // CAN_UTILS_H
