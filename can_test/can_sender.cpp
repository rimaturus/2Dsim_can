#include <iostream>
#include <cstring>      // For std::memcpy
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

int main() {
    // Define the two float values to send
    float range = 3.73873f;       // Example range
    float bearing = 18.23f;       // Example bearing

    // Create a CAN frame
    struct can_frame frame;
    frame.can_id = 0x4AA;        // CAN ID
    frame.can_dlc = 8;           // Data length code (8 bytes)

    // Copy the floats into the frame data (little-endian)
    std::memcpy(frame.data, &range, sizeof(float));
    std::memcpy(frame.data + 4, &bearing, sizeof(float));

    // Open a socket
    int s = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (s < 0) {
        perror("Socket");
        return 1;
    }

    // Set up the CAN interface
    struct ifreq ifr;
    std::strcpy(ifr.ifr_name, "vcan0");
    ioctl(s, SIOCGIFINDEX, &ifr);

    // Bind the socket to the CAN interface
    struct sockaddr_can addr;
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (bind(s, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        perror("Bind");
        return 1;
    }

    // Send the frame
    if (write(s, &frame, sizeof(struct can_frame)) != sizeof(struct can_frame)) {
        perror("Write");
        return 1;
    }

    std::cout << "Sent CAN frame for Cone with Range: " << range << " m and Bearing: " << bearing << " degrees" << std::endl;

    // Close the socket
    close(s);

    return 0;
}
