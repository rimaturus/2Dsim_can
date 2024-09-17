#ifndef PERCEPTION_H
#define PERCEPTION_H

#include <iostream>
#include <vector>
#include <string>
#include <cmath>
#include <yaml-cpp/yaml.h>
#include <linux/can.h>
#include <mutex>
#include <condition_variable>
#include <queue>

// Constants
extern float PIXELS_PER_METER; // Pixels per meter

// CAN IDs
extern int CAR_X_CAN_ID;
extern int CAR_Y_CAN_ID;
extern int CAR_ANGLE_CAN_ID;

// Noise parameters
extern double RANGE_NOISE_STD_DEV;
extern double BEARING_NOISE_STD_DEV;

// Structure for Cone
struct Cone {
    float x_pixels; // Position in pixels
    float y_pixels;
    std::string color;
};

// Structure for CAN data to send
struct CANData {
    int id;
    float range;
    float bearing;
};


// Cone-related functions
std::vector<Cone> loadCones(const std::string& filename);
void loadConfig(const std::string& filename);

// CAN socket setup functions
int setupCANSocket(const std::string& interface_name);

// Thread functions
void sendCANData(int send_can_socket, std::queue<CANData>& dataQueue, std::mutex& queueMutex, std::condition_variable& cv);

// Helper functions
void processCANFrame(const struct can_frame& frame, float& car_x, float& car_y, float& car_angle, bool& has_car_x, bool& has_car_y, bool& has_car_angle);
void computeAndSendConeData(const std::vector<Cone>& cones, float car_x_m, float car_y_m, float car_angle, std::queue<CANData>& dataQueue, std::mutex& queueMutex, std::condition_variable& cv);

#endif // PERCEPTION_H
