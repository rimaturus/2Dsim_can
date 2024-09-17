#include <iostream>
#include <iomanip>      // For std::hex and std::setw
#include <cstring>      // For std::memcpy
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>
#include <climits>


// Function to swap endianness
template<typename T>
T swapEndian(T u) {
    static_assert (CHAR_BIT == 8, "CHAR_BIT != 8");

    union {
        T u;
        uint8_t u8[sizeof(T)];
    } source, dest;

    source.u = u;

    for (size_t k = 0; k < sizeof(T); k++)
        dest.u8[k] = source.u8[sizeof(T) - k - 1];

    return dest.u;
}

int main() {
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

    struct can_frame frame;

    while (true) {
        // Read a CAN frame
        int nbytes = read(s, &frame, sizeof(struct can_frame));
        if (nbytes < 0) {
            perror("Read");
            return 1;
        }

        // Print raw CAN data
        std::cout << "Received CAN data: ";
        for (int i = 0; i < frame.can_dlc; ++i) {
            std::cout << std::hex << std::setw(2) << std::setfill('0') << (int)frame.data[i] << " ";
        }
        std::cout << std::dec << std::endl;

        // Decode little-endian
        int id = frame.can_id;
        float range, bearing;
        std::memcpy(&range, frame.data, sizeof(float));
        std::memcpy(&bearing, frame.data + 4, sizeof(float));
        std::cout << "ID: " << id << " Decoded (Little-endian) - Range: " << range << " meters, Bearing: " << bearing << " degrees" << std::endl;

        // Decode big-endian
        float range_be = swapEndian(range);
        float bearing_be = swapEndian(bearing);
        std::cout << "ID: " << id << " Decoded (Big-endian) - Range: " << range_be << " meters, Bearing: " << bearing_be << " degrees" << std::endl;
    }

    // Close the socket
    close(s);

    return 0;
}
