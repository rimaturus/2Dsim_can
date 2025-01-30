#include "can_utils.h"
#include <unistd.h>
#include <stdio.h>

int main() {
    // Start receiving CAN messages in a separate thread
    canreceive_start();

    // Add CAN IDs to filter
    can_add_filter(0x123);
    can_add_filter(0x456);

    // Send some CAN messages
    cansend_start(0x123, 42.0f);
    cansend_start(0x456, 24.0f);

    // Retrieve the values
    float value1 = can_get_value(0x123);
    printf("Value for CAN ID 0x123: %f\n", value1);

    float value2 = can_get_value(0x456);
    printf("Value for CAN ID 0x456: %f\n", value2);

    // Continue to receive and print new values
    while (1) {
        float new_value = can_get_value(0x123);
        printf("New value for CAN ID 0x123: %f\n", new_value);
    }

    return 0;
}
