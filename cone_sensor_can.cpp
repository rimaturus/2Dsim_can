// cone_sensor_can.cpp
// g++ cone_sensor_can.cpp -o cone_sensor_can -lyaml-cpp

#include <iostream>
#include <vector>
#include <string>
#include <cmath>
#include <yaml-cpp/yaml.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <cstring>
#include <fcntl.h>
#include <errno.h>
#include <random>

// Conversion factor: 50 pixels correspond to 1 meter
const float PIXELS_PER_METER = 50.0f; // Pixels per meter

// Structure for Cone
struct Cone {
    float x;
    float y;
    std::string color;
};

// Function to load cones from YAML file
std::vector<Cone> loadCones(const std::string& filename) {
    YAML::Node conesNode = YAML::LoadFile(filename)["cones"];
    std::vector<Cone> cones;

    for (const auto& node : conesNode) {
        Cone cone;
        cone.x = node["x"].as<float>();
        cone.y = node["y"].as<float>();
        cone.color = node["color"].as<std::string>();
        cones.push_back(cone);
    }
    return cones;
}

// Function to set up CAN socket
int setupCANSocket(const std::string& interface_name) {
    int can_socket;
    struct sockaddr_can addr;
    struct ifreq ifr;

    // Create a socket
    if ((can_socket = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
        perror("Error while opening socket");
        return -1;
    }

    strcpy(ifr.ifr_name, interface_name.c_str());
    ioctl(can_socket, SIOCGIFINDEX, &ifr);

    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    // Bind the socket to the CAN interface
    if (bind(can_socket, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        perror("Error in socket bind");
        return -2;
    }

    // Set socket to non-blocking mode
    int flags = fcntl(can_socket, F_GETFL, 0);
    fcntl(can_socket, F_SETFL, flags | O_NONBLOCK);

    return can_socket;
}

int main() {
    // Load cones from YAML file
    std::vector<Cone> cones = loadCones("cones.yaml");

    // Set up CAN socket
    int can_socket = setupCANSocket("vcan0");
    if (can_socket < 0) {
        return -1;
    }

    std::cout << "Cone sensor script is running. Listening for car data over CAN bus..." << std::endl;

    float car_x = 0.0f;
    float car_y = 0.0f;
    float car_angle = 0.0f;

    bool has_car_x = false;
    bool has_car_y = false;
    bool has_car_angle = false;

    struct can_frame frame;

    // Random number generators for Gaussian noise
    std::default_random_engine generator(std::random_device{}());

    // Standard deviations for noise (in meters and degrees)
    const double rangeNoiseStdDev = 0.1;       // Adjust as needed (meters)
    const double bearingNoiseStdDev = 1.0;     // Adjust as needed (degrees)

    // Distributions for noise
    std::normal_distribution<double> rangeNoiseDist(0.0, rangeNoiseStdDev);
    std::normal_distribution<double> bearingNoiseDist(0.0, bearingNoiseStdDev);

    while (true) {
        int nbytes = read(can_socket, &frame, sizeof(struct can_frame));

        if (nbytes < 0) {
            if (errno != EAGAIN && errno != EWOULDBLOCK) {
                perror("CAN read error");
                break;
            }
            // No data available, sleep briefly
            usleep(1000);
            continue;
        }

        if (frame.can_dlc != sizeof(float)) {
            std::cerr << "Received CAN frame with unexpected data length" << std::endl;
            continue;
        }

        float value;
        memcpy(&value, frame.data, sizeof(float));

        switch (frame.can_id) {
            case 0x200:
                car_x = value;
                has_car_x = true;
                break;
            case 0x201:
                car_y = value;
                has_car_y = true;
                break;
            case 0x202:
                car_angle = value;
                has_car_angle = true;
                break;
            default:
                // Unknown CAN ID
                break;
        }

        // If all data received, compute range and bearing
        if (has_car_x && has_car_y && has_car_angle) {
            // Convert car position from pixels to meters
            float car_x_m = car_x / PIXELS_PER_METER;
            float car_y_m = car_y / PIXELS_PER_METER;

            std::cout << "Received car data: X=" << car_x_m << " m, Y=" << car_y_m
                      << " m, Angle=" << car_angle * 180.0 / M_PI << " degrees" << std::endl;

            // Compute range and bearing to each cone
            for (const auto& cone : cones) {
                // Convert cone position from pixels to meters
                float cone_x_m = cone.x / PIXELS_PER_METER;
                float cone_y_m = cone.y / PIXELS_PER_METER;

                float dx = cone_x_m - car_x_m;
                float dy = cone_y_m - car_y_m;

                // Rotate into car's coordinate frame (negative angle)
                float cos_angle = cos(-car_angle);
                float sin_angle = sin(-car_angle);
                float x_rel = dx * cos_angle - dy * sin_angle;
                float y_rel = dx * sin_angle + dy * cos_angle;

                // Compute range and bearing
                float range = sqrt(x_rel * x_rel + y_rel * y_rel);
                float bearing = atan2(y_rel, x_rel);  // Bearing in radians

                // Convert bearing to degrees
                float bearing_deg = bearing * 180.0 / M_PI;

                // Add Gaussian noise to range and bearing
                double noisyRange = range + rangeNoiseDist(generator);
                double noisyBearingDeg = bearing_deg + bearingNoiseDist(generator);

                // Ensure range is not negative
                if (noisyRange < 0.0) {
                    noisyRange = 0.0;
                }

                // Output the result
                std::cout << "Cone at (" << cone_x_m << " m, " << cone_y_m << " m): Range = "
                          << noisyRange << " m, Bearing = " << noisyBearingDeg << " degrees" << std::endl;
            }

            std::cout << "-----\n" << std::endl;

            // Reset flags to wait for new data
            has_car_x = false;
            has_car_y = false;
            has_car_angle = false;
        }
    }

    // Close CAN socket
    close(can_socket);

    return 0;
}
