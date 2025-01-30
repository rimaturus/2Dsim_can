// imu_sensor.cpp

#include "imu_sensor.h"
#include <chrono>
#include <iostream> 

#define IMU_ACCEL_X_CAN_ID 0x500
#define IMU_ACCEL_Y_CAN_ID 0x501
#define IMU_GYRO_Z_CAN_ID  0x502

IMUSensor::IMUSensor(double mass,
                     double accel_noise_std, double gyro_noise_std,
                     double accel_bias_std, double gyro_bias_std,
                     double accel_drift_rate, double gyro_drift_rate)
    : mass_(mass),
      accel_noise_std_(accel_noise_std),
      gyro_noise_std_(gyro_noise_std),
      accel_bias_std_(accel_bias_std),
      gyro_bias_std_(gyro_bias_std),
      accel_drift_rate_(accel_drift_rate),
      gyro_drift_rate_(gyro_drift_rate),
      gen_(rd_()),
      accel_noise_dist_(0.0, accel_noise_std_),
      gyro_noise_dist_(0.0, gyro_noise_std_),
      accel_bias_dist_(0.0, accel_bias_std_),
      gyro_bias_dist_(0.0, gyro_bias_std_),
      running_(false),
      can_socket_(-1) {
    // Initialize biases to zero
    accel_bias_ = 0.0;
    gyro_bias_ = 0.0;
}

bool IMUSensor::initCANSocket(const std::string& ifname) {
    struct sockaddr_can addr;
    struct ifreq ifr;

    // Create CAN RAW socket
    if ((can_socket_ = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
        perror("IMU Sensor: Error while opening CAN socket");
        return false;
    }

    strcpy(ifr.ifr_name, ifname.c_str());
    ioctl(can_socket_, SIOCGIFINDEX, &ifr);

    memset(&addr, 0, sizeof(addr));
    addr.can_family  = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    // Bind the socket to the CAN interface
    if (bind(can_socket_, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        perror("IMU Sensor: Error in socket bind");
        return false;
    }

    return true;
}

void IMUSensor::start(double dt) {
    if (!initCANSocket("vcan0")) {
        std::cerr << "IMU Sensor: Failed to initialize CAN socket. IMU will not send data over CAN.\n";
    }
    running_ = true;
    imu_thread_ = std::thread(&IMUSensor::run, this, dt);
}

void IMUSensor::stop() {
    running_ = false;
}

void IMUSensor::join() {
    if (imu_thread_.joinable()) {
        imu_thread_.join();
    }
    if (can_socket_ >= 0) {
        close(can_socket_);
    }
}

void IMUSensor::updateForcesAndState(const VehicleForces& forces, const VehicleState& state) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    latest_forces_ = forces;
    latest_state_ = state;
}

void IMUSensor::run(double dt) {
    while (running_) {
        VehicleForces forces;
        VehicleState state;

        {
            std::lock_guard<std::mutex> lock(data_mutex_);
            forces = latest_forces_;
            state = latest_state_;
        }

        // Update biases with drift
        accel_bias_ += accel_drift_rate_ * dt + accel_bias_dist_(gen_);
        gyro_bias_ += gyro_drift_rate_ * dt + gyro_bias_dist_(gen_);

        // Compute true accelerations in the vehicle frame
        double a_x_true = forces.force[0] / mass_;
        double a_y_true = forces.force[1] / mass_;

        // Add noise and bias to accelerations
        double accel_x = a_x_true + accel_bias_ + accel_noise_dist_(gen_);
        double accel_y = a_y_true + accel_bias_ + accel_noise_dist_(gen_);

        // Add noise and bias to gyroscope measurement
        double gyro_z = state.r + gyro_bias_ + gyro_noise_dist_(gen_);

        // Send IMU measurements over CAN bus
        if (can_socket_ >= 0) {
            struct can_frame frame;

            // Send accelerometer X
            frame.can_id = IMU_ACCEL_X_CAN_ID;
            frame.can_dlc = sizeof(float);
            float accel_x_f = static_cast<float>(accel_x);
            memcpy(frame.data, &accel_x_f, sizeof(float));
            write(can_socket_, &frame, sizeof(struct can_frame));

            // Send accelerometer Y
            frame.can_id = IMU_ACCEL_Y_CAN_ID;
            float accel_y_f = static_cast<float>(accel_y);
            memcpy(frame.data, &accel_y_f, sizeof(float));
            write(can_socket_, &frame, sizeof(struct can_frame));

            // Send gyroscope Z
            frame.can_id = IMU_GYRO_Z_CAN_ID;
            float gyro_z_f = static_cast<float>(gyro_z);
            memcpy(frame.data, &gyro_z_f, sizeof(float));
            write(can_socket_, &frame, sizeof(struct can_frame));
        }

        // Sleep for the time step duration
        std::this_thread::sleep_for(std::chrono::duration<double>(dt));
    }
}
