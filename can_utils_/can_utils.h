#ifndef CAN_UTILS_H
#define CAN_UTILS_H

#include <stdint.h>

// Existing functions
void cansend(int can_id, float data);
void canreceive();
void cansend_start(int can_id, float data);
void canreceive_start();

// Modified function
float can_get_value(uint32_t can_id);

// New function
int can_add_filter(uint32_t can_id);

#endif // CAN_UTILS_H
