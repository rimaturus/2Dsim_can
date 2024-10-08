#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <time.h>
#include "params.h"

// Maximum line length for reading the config file
#define MAX_LINE_LENGTH 256

// Function to trim whitespace from a string (in-place)
void trim(char *str) {
    // Trim leading whitespace
    char *start = str;
    while(*start && (*start == ' ' || *start == '\t' || *start == '\n' || *start == '\r')) {
        start++;
    }
    
    // Trim trailing whitespace
    char *end = start + strlen(start) - 1;
    while(end > start && (*end == ' ' || *end == '\t' || *end == '\n' || *end == '\r')) {
        *end = '\0';
        end--;
    }
    
    // Shift the trimmed string to the beginning
    if(start != str) {
        memmove(str, start, strlen(start) + 1);
    }
}

// Function to load configuration from a YAML file
bool load_config(const char *filename,
                PhysicalModel *physical,
                Actuation *actuation,
                IMUParams *imu_params) {
    FILE *file = fopen(filename, "r");
    if (!file) {
        fprintf(stderr, "Failed to open config file: %s\n", filename);
        return false;
    }

    char line[MAX_LINE_LENGTH];
    char current_section[MAX_LINE_LENGTH] = "";

    while (fgets(line, sizeof(line), file)) {
        trim(line);
        if (strlen(line) == 0 || line[0] == '#') {
            continue; // Skip empty lines and comments
        }

        if (line[0] == '[' && line[strlen(line)-1] == ']') {
            // Section header
            strncpy(current_section, line + 1, strlen(line) - 2);
            current_section[strlen(line) - 2] = '\0';
            continue;
        }

        // Key-value pair
        char *delimiter = strstr(line, ":");
        if (!delimiter) {
            continue; // Invalid line
        }

        *delimiter = '\0';
        char *key = line;
        char *value_str = delimiter + 1;

        trim(key);
        trim(value_str);
        double value = atof(value_str);

        if (strcmp(current_section, "PhysicalModel") == 0) {
            if (strcmp(key, "wheelbase") == 0) {
                physical->wheelbase = value;
            } else if (strcmp(key, "mass") == 0) {
                physical->mass = value;
            } else if (strcmp(key, "inertia") == 0) {
                physical->inertia = value;
            }
            // Add more physical parameters as needed
        }
        else if (strcmp(current_section, "Actuation") == 0) {
            if (strcmp(key, "max_torque_throttle") == 0) {
                actuation->max_torque_throttle = value;
            } else if (strcmp(key, "max_torque_brake") == 0) {
                actuation->max_torque_brake = value;
            } else if (strcmp(key, "max_steering") == 0) {
                actuation->max_steering = value;
            } else if (strcmp(key, "steering_resolution") == 0) {
                actuation->steering_resolution = value;
            } else if (strcmp(key, "throttle_resolution") == 0) {
                actuation->throttle_resolution = value;
            } else if (strcmp(key, "brake_resolution") == 0) {
                actuation->brake_resolution = value;
            }
        }
        else if (strcmp(current_section, "IMU") == 0) {
            if (strcmp(key, "noise_std_ax") == 0) {
                imu_params->noise_std_ax = value;
            } else if (strcmp(key, "noise_std_ay") == 0) {
                imu_params->noise_std_ay = value;
            } else if (strcmp(key, "noise_std_yaw") == 0) {
                imu_params->noise_std_yaw = value;
            }
        }
    }

    fclose(file);
    return true;
}

// Function to initialize the Vehicle Model
bool init_vehicle_model(const char *config_file, VehicleModel *vehicle) {
    if (!load_config(config_file, &(vehicle->physical), &(vehicle->actuation), &(vehicle->imu_params))) {
        return false;
    }

    // Initialize pose to zero
    vehicle->pose.x = 0.0;
    vehicle->pose.y = 0.0;
    vehicle->pose.yaw = 0.0;

    // Initialize random seed
    vehicle->seed = (unsigned int)time(NULL);
    srand(vehicle->seed);

    return true;
}

// Function to generate Gaussian noise using Box-Muller transform
double generate_gaussian_noise(double std_dev, unsigned int *seed) {
    double u1 = rand_r(seed) / (double)RAND_MAX;
    double u2 = rand_r(seed) / (double)RAND_MAX;
    double z0 = sqrt(-2.0 * log(u1)) * cos(2.0 * M_PI * u2);
    return z0 * std_dev;
}

// Function to apply steering limits and resolution
double apply_steering_limits(double steering, const Actuation *actuation) {
    if (steering > actuation->max_steering) {
        steering = actuation->max_steering;
    }
    else if (steering < -actuation->max_steering) {
        steering = -actuation->max_steering;
    }

    // Apply resolution
    steering = round(steering / actuation->steering_resolution) * actuation->steering_resolution;
    return steering;
}

// Function to apply torque limits and resolution
double apply_torque_limits(double torque, const Actuation *actuation) {
    if (torque > actuation->max_torque_throttle) {
        torque = actuation->max_torque_throttle;
    }
    else if (torque < -actuation->max_torque_brake) {
        torque = -actuation->max_torque_brake;
    }

    // Apply resolution
    if (torque >= 0.0) {
        torque = round(torque / actuation->throttle_resolution) * actuation->throttle_resolution;
    }
    else {
        torque = round(torque / actuation->brake_resolution) * actuation->brake_resolution;
    }

    return torque;
}

// Simple Pacejka tire model approximation
double Pacejka(double alpha, const PhysicalModel *physical) {
    // Parameters for Pacejka model (simplified)
    const double B = 10.0;
    const double C = 1.9;
    const double D = (physical->mass * 9.81 * 0.5); // Assuming front tire load

    return D * sin(C * atan(B * alpha));
}

// Function to compute accelerations and update pose
bool IMU(VehicleModel *vehicle, double steering_rad, double wheel_torque, Acceleration *accel, Pose *new_pose) {
    // Apply actuation limits and resolutions
    steering_rad = apply_steering_limits(steering_rad, &(vehicle->actuation));
    wheel_torque = apply_torque_limits(wheel_torque, &(vehicle->actuation));

    // Compute forces using a simplified Pacejka tire model
    double F_yf = Pacejka(steering_rad, &(vehicle->physical));
    double F_yr = Pacejka(0.0, &(vehicle->physical)); // Assuming rear steering is zero

    // Compute accelerations
    double ax = (wheel_torque / vehicle->physical.mass) - (F_yf * sin(vehicle->pose.yaw)) / vehicle->physical.mass;
    double ay = (F_yf * cos(vehicle->pose.yaw) + F_yr * cos(vehicle->pose.yaw)) / vehicle->physical.mass;
    double yaw_rate = (vehicle->physical.wheelbase * (F_yf - F_yr)) / vehicle->physical.inertia;

    // Add Gaussian noise
    double noise_ax = generate_gaussian_noise(vehicle->imu_params.noise_std_ax, &(vehicle->seed));
    double noise_ay = generate_gaussian_noise(vehicle->imu_params.noise_std_ay, &(vehicle->seed));
    double noise_yaw = generate_gaussian_noise(vehicle->imu_params.noise_std_yaw, &(vehicle->seed));

    accel->ax = ax + noise_ax;
    accel->ay = ay + noise_ay;
    accel->yaw_rate = yaw_rate + noise_yaw;

    // Update pose with a fixed time step (dt = 0.01 seconds)
    double dt = 0.01;
    new_pose->x = vehicle->pose.x + accel->ax * cos(vehicle->pose.yaw) * dt - accel->ay * sin(vehicle->pose.yaw) * dt;
    new_pose->y = vehicle->pose.y + accel->ax * sin(vehicle->pose.yaw) * dt + accel->ay * cos(vehicle->pose.yaw) * dt;
    new_pose->yaw = vehicle->pose.yaw + accel->yaw_rate * dt;

    // Update the vehicle's pose
    vehicle->pose = *new_pose;

    return true;
}
