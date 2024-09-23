// vehicle_state.h

#ifndef VEHICLE_STATE_H
#define VEHICLE_STATE_H

#include <Eigen/Dense>

// Struct to hold IMU measurements
struct IMUMeasurements {
    struct {
        double x; // Accelerometer X-axis
        double y; // Accelerometer Y-axis
    } accel;
    double gyro_z; // Gyroscope Z-axis
};

// Struct to hold vehicle forces
struct VehicleForces {
    Eigen::Vector2d force; // Force in X and Y directions
    double moment;         // Yaw moment
};

// Struct to hold vehicle state
struct VehicleState {
    Eigen::Vector2d position; // Position in X and Y
    double psi;               // Orientation (yaw angle)
    Eigen::Vector2d velocity; // Velocity in X and Y
    double r;                 // Yaw rate
};

// Struct to hold motion covariance matrix
struct MotionCovariance {
    Eigen::Matrix3d P; // Covariance matrix
};

#endif // VEHICLE_STATE_H
