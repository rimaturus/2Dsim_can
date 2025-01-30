// imu_sensor.h

#ifndef IMU_SENSOR_H
#define IMU_SENSOR_H

#include <random>
#include <thread>
#include <mutex> // Added
#include <sys/types.h>
#include <sys/socket.h>
#include <unistd.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <cstring>
#include <cerrno>
#include <sys/ioctl.h>
#include <fcntl.h>

#include "vehicle_state.h"
#include <fcntl.h>

class IMUSensor {
public:
    IMUSensor(double mass,
              double accel_noise_std, double gyro_noise_std,
              double accel_bias_std, double gyro_bias_std,
              double accel_drift_rate, double gyro_drift_rate);

    // Starts the IMU sensor thread
    void start(double dt);

    // Stops the IMU sensor thread
    void stop();

    // Waits for the IMU thread to finish
    void join();

    // Provide a method to send VehicleForces and VehicleState to the IMU
    void updateForcesAndState(const VehicleForces& forces, const VehicleState& state);

private:
    void run(double dt);

    // IMU sensor parameters and state variables
    double mass_;

    double accel_noise_std_;
    double gyro_noise_std_;
    double accel_bias_std_;
    double gyro_bias_std_;
    double accel_drift_rate_;
    double gyro_drift_rate_;

    double accel_bias_;
    double gyro_bias_;

    std::random_device rd_;
    std::mt19937 gen_;
    std::normal_distribution<> accel_noise_dist_;
    std::normal_distribution<> gyro_noise_dist_;
    std::normal_distribution<> accel_bias_dist_;
    std::normal_distribution<> gyro_bias_dist_;

    std::thread imu_thread_;
    bool running_;

    // Variables to store the latest forces and state
    VehicleForces latest_forces_;
    VehicleState latest_state_;
    std::mutex data_mutex_;

    // CAN socket for sending IMU data
    int can_socket_;
    bool initCANSocket(const std::string& ifname);
};

#endif // IMU_SENSOR_H
