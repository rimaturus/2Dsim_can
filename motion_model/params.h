#ifndef PARAMS_H
#define PARAMS_H

#include <stdbool.h>

// Structure to hold vehicle physical parameters
typedef struct {
    double wheelbase;
    double mass;
    double inertia;
    // Add other physical parameters as needed
} PhysicalModel;

// Structure to hold actuation parameters
typedef struct {
    double max_torque_throttle;
    double max_torque_brake;
    double max_steering;
    double steering_resolution;
    double throttle_resolution;
    double brake_resolution;
} Actuation;

// Structure to hold IMU parameters
typedef struct {
    double noise_std_ax;
    double noise_std_ay;
    double noise_std_yaw;
} IMUParams;

// Structure to represent the vehicle's pose
typedef struct {
    double x;
    double y;
    double yaw;
} Pose;

// Structure to represent accelerations
typedef struct {
    double ax;
    double ay;
    double yaw_rate;
} Acceleration;

// Structure to represent the Vehicle Model
typedef struct {
    PhysicalModel physical;
    Actuation actuation;
    IMUParams imu_params;
    Pose pose;
    // Internal state for random number generation
    unsigned int seed;
} VehicleModel;

// Function to trim whitespace from a string (in-place)
void trim(char *str);

// Function to load configuration from a YAML file
bool load_config(const char *filename,
                PhysicalModel *physical,
                Actuation *actuation,
                IMUParams *imu_params);

// Function to initialize the Vehicle Model
bool init_vehicle_model(const char *config_file, VehicleModel *vehicle);

// Function to compute accelerations and update pose
bool IMU(VehicleModel *vehicle, double steering_rad, double wheel_torque, Acceleration *accel, Pose *new_pose);

#endif // PARAMS_H
