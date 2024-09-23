// vehicle_simulation.cpp

#include <iostream>    
#include <vector>
#include <cmath>
#include <Eigen/Dense>
#include <random>
#include <tuple>
#include <fstream>
#include <iomanip>
#include <thread>
#include <mutex>       
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
#include "imu_sensor.h" 
#include "thread_safe_queue.h"

// Constants for CAN IDs
const uint32_t STEERING_CAN_ID = 0x300;
const uint32_t THROTTLE_CAN_ID = 0x301;
const uint32_t IMU_ACCEL_X_CAN_ID = 0x500;
const uint32_t IMU_ACCEL_Y_CAN_ID = 0x501;
const uint32_t IMU_GYRO_Z_CAN_ID  = 0x502;


// Class to handle data logging
class Logger {
public:
    Logger(const std::string& filename) {
        log_file_.open(filename);
        if (log_file_.is_open()) {
            // Write CSV header
            log_file_ << "Time, Steering, Throttle, Fx, Fy, Mz, x, y, psi, "
                      << "P00, P01, P02, P11, P12, P22, "
                      << "IMU_a_x, IMU_a_y, IMU_gyro_z\n";
            log_file_ << std::fixed << std::setprecision(6);
        } else {
            std::cerr << "Failed to open log file: " << filename << "\n";
        }
    }

    ~Logger() {
        if (log_file_.is_open()) {
            log_file_.close();
        }
    }

    void log(double time, double steering_angle_deg, double throttle,
             const VehicleForces& forces, const VehicleState& state,
             const MotionCovariance& cov, const IMUMeasurements& imu) {
        if (log_file_.is_open()) {
            log_file_ << time << ","
                      << steering_angle_deg << ","
                      << throttle << ","
                      << forces.force[0] << ","
                      << forces.force[1] << ","
                      << forces.moment << ","
                      << state.position[0] << ","
                      << state.position[1] << ","
                      << state.psi << ","
                      << cov.P(0,0) << ","
                      << cov.P(0,1) << ","
                      << cov.P(0,2) << ","
                      << cov.P(1,1) << ","
                      << cov.P(1,2) << ","
                      << cov.P(2,2) << ","
                      << imu.accel.x << ","
                      << imu.accel.y << ","
                      << imu.gyro_z << "\n";
        }
    }

private:
    std::ofstream log_file_;
};

// Class to model the vehicle dynamics
class VehicleModel {
public:
    VehicleModel(double mass, double Iz, double lf, double lr, double track_width, double wheel_radius, double mu)
        : mass_(mass), Iz_(Iz), lf_(lf), lr_(lr), track_width_(track_width),
          wheel_radius_(wheel_radius), mu_(mu) {
        // Initialize state
        state_.position = {0.0, 0.0};
        state_.psi = 0.0;
        state_.velocity = {0.0, 0.0};
        state_.r = 0.0;

        // Initialize covariance matrix (assuming zero initial uncertainty)
        P_ = Eigen::Matrix3d::Zero();

        // Define process noise covariance matrix Q
        // These values can be tuned based on expected uncertainties
        // Q is diagonal; adjust values as necessary
        Q_ << 0.01, 0.0, 0.0,
               0.0, 0.01, 0.0,
               0.0, 0.0, 0.005;
    }

    // Update function returns VehicleForces and MotionCovariance
    std::pair<VehicleForces, MotionCovariance> update(double delta, double throttle, const IMUMeasurements& imu_measurements, double dt) {
        // delta: Steering angle (rad)
        // throttle: Throttle input (arbitrary units, to be converted to torque)
        // imu_measurements: IMU accelerations (m/s^2) and gyro_z (rad/s)
        // dt: Time step (s)

        // Constants
        const double g = 9.81; // Gravity (m/s^2)

        // ----- Load Transfer Calculations -----
        // Total weight
        double weight = mass_ * g;

        // Static load distribution
        double load_front = weight * lr_ / (lf_ + lr_);
        double load_rear = weight * lf_ / (lf_ + lr_);

        // Longitudinal load transfer due to acceleration
        double load_transfer_long = mass_ * imu_measurements.accel.x * h_ / (lf_ + lr_);
        load_front -= load_transfer_long * lr_ / (lf_ + lr_);
        load_rear += load_transfer_long * lf_ / (lf_ + lr_);

        // Lateral load transfer due to yaw rate and lateral acceleration
        double lateral_acc = imu_measurements.accel.y; // Using IMU measurement for lateral acceleration
        double load_transfer_lat = mass_ * lateral_acc * h_ / track_width_;
        load_front += load_transfer_lat;
        load_rear -= load_transfer_lat;

        // Distribute loads to individual tires (assuming symmetric)
        double Fz_fl = load_front / 2.0;
        double Fz_fr = load_front / 2.0;
        double Fz_rl = load_rear / 2.0;
        double Fz_rr = load_rear / 2.0;

        // ----- Slip Angle Calculations -----
        // Avoid division by zero by setting a minimum velocity
        double v_x = std::max(state_.velocity[0], 0.1);
        double v_y = state_.velocity[1];

        double alpha_f = delta - (v_y + lf_ * state_.r) / v_x;
        double alpha_r = -(v_y - lr_ * state_.r) / v_x;

        // ----- Tire Forces using Pacejka Model -----
        double Fyf = pacejkaModel(alpha_f, Fz_fl + Fz_fr);
        double Fyr = pacejkaModel(alpha_r, Fz_rl + Fz_rr);

        // Convert throttle input to rear torque
        // Assuming throttle ranges from 0 to 100 units, scale to torque
        double T_rear = throttle / 100.0 * max_torque_;

        // Longitudinal force from rear torque (assuming no slip and full torque transfer)
        double Fxr = T_rear / wheel_radius_;

        // Total forces
        VehicleForces forces;
        forces.force[0] = Fxr - Fyf * std::sin(delta);
        forces.force[1] = Fyf * std::cos(delta) + Fyr;
        forces.moment = lf_ * Fyf * std::cos(delta) - lr_ * Fyr;

        // ----- Vehicle Dynamics Update -----
        // Accelerations
        double a_x = forces.force[0] / mass_;
        double a_y = forces.force[1] / mass_;
        double yaw_acc = forces.moment / Iz_;

        // Update velocities
        state_.velocity[0] += a_x * dt;
        state_.velocity[1] += a_y * dt;
        state_.r += yaw_acc * dt;

        // Update position and orientation
        state_.position[0] += (state_.velocity[0] * std::cos(state_.psi) - state_.velocity[1] * std::sin(state_.psi)) * dt;
        state_.position[1] += (state_.velocity[0] * std::sin(state_.psi) + state_.velocity[1] * std::cos(state_.psi)) * dt;
        state_.psi += state_.r * dt;

        // Normalize psi to [-pi, pi]
        state_.psi = std::atan2(std::sin(state_.psi), std::cos(state_.psi));

        // ----- Covariance Matrix Update -----
        // State vector: [x, y, psi]
        Eigen::Matrix3d F_matrix = Eigen::Matrix3d::Identity();
        F_matrix(0, 2) = -state_.velocity[0] * std::sin(state_.psi) * dt - state_.velocity[1] * std::cos(state_.psi) * dt;
        F_matrix(1, 2) = state_.velocity[0] * std::cos(state_.psi) * dt - state_.velocity[1] * std::sin(state_.psi) * dt;

        // Predict covariance
        P_ = F_matrix * P_ * F_matrix.transpose() + Q_;

        // Prepare covariance output
        MotionCovariance motion_cov;
        motion_cov.P = P_;

        return std::make_pair(forces, motion_cov);
    }

    VehicleState getState() const {
        return state_;
    }

private:
    // Simplified Pacejka Magic Formula (one parameter per force)
    double pacejkaModel(double slip_angle, double load) {
        // Parameters can be tuned based on tire characteristics
        // Here we use a simplified version with fixed parameters
        const double B = 10.0; // Stiffness factor
        const double C = 1.9;  // Shape factor
        const double D = mu_ * load; // Peak factor
        const double E = 0.97; // Curvature factor

        return D * std::sin(C * std::atan(B * slip_angle - E * (B * slip_angle - std::atan(B * slip_angle))));
    }

    VehicleState state_;

    // Vehicle parameters
    double mass_;        // Mass (kg)
    double Iz_;          // Yaw moment of inertia (kg·m²)
    double lf_;          // Distance from CG to front axle (m)
    double lr_;          // Distance from CG to rear axle (m)
    double track_width_; // Track width (m)
    double h_ = 0.5;     // CG height (m)
    double wheel_radius_; // Wheel radius (m)
    double mu_;          // Friction coefficient
    double max_torque_ = 1000.0; // Maximum rear torque (Nm)

    // Covariance matrices
    Eigen::Matrix3d P_; // State covariance matrix
    Eigen::Matrix3d Q_; // Process noise covariance matrix
};

// Function to initialize CAN socket
int initCANSocket(const std::string& ifname) {
    int s;
    struct sockaddr_can addr;
    struct ifreq ifr;

    // Create CAN RAW socket
    if ((s = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
        perror("Error while opening CAN socket");
        return -1;
    }

    strcpy(ifr.ifr_name, ifname.c_str());
    ioctl(s, SIOCGIFINDEX, &ifr);

    memset(&addr, 0, sizeof(addr));
    addr.can_family  = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    // Bind the socket to the CAN interface
    if (bind(s, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        perror("Error in socket bind");
        close(s);
        return -1;
    }

    return s;
}

// Function to read IMU measurements from CAN bus
bool readIMUMeasurements(int can_socket, IMUMeasurements& imu_measurements) {
    struct can_frame frame;
    int nbytes;

    // Use select to wait for data with a timeout
    fd_set read_fds;
    struct timeval timeout;
    timeout.tv_sec = 0;
    timeout.tv_usec = 1000; // 1 ms timeout

    FD_ZERO(&read_fds);
    FD_SET(can_socket, &read_fds);

    int ret = select(can_socket + 1, &read_fds, NULL, NULL, &timeout);
    if (ret > 0 && FD_ISSET(can_socket, &read_fds)) {
        nbytes = read(can_socket, &frame, sizeof(struct can_frame));
        if (nbytes < 0) {
            if (errno != EAGAIN && errno != EWOULDBLOCK) {
                perror("CAN read error");
            }
            return false;
        }

        if (nbytes < sizeof(struct can_frame)) {
            fprintf(stderr, "Incomplete CAN frame\n");
            return false;
        }

        // Unpack float value (little-endian)
        float value;
        memcpy(&value, frame.data, sizeof(float));

        if (frame.can_id == IMU_ACCEL_X_CAN_ID) {
            imu_measurements.accel.x = value;
            return true;
        } else if (frame.can_id == IMU_ACCEL_Y_CAN_ID) {
            imu_measurements.accel.y = value;
            return true;
        } else if (frame.can_id == IMU_GYRO_Z_CAN_ID) {
            imu_measurements.gyro_z = value;
            return true;
        }
    }

    return false;
}

// Function to read steering and throttle from CAN bus
bool readControlCommands(int can_socket, double& steering_angle_deg, double& throttle) {
    struct can_frame frame;
    int nbytes = read(can_socket, &frame, sizeof(struct can_frame));
    if (nbytes < 0) {
        if (errno == EAGAIN || errno == EWOULDBLOCK) {
            // No data available; this is not an error in non-blocking mode
            return false;
        } else {
            perror("CAN read error");
            return false;
        }
    }

    if (nbytes < sizeof(struct can_frame)) {
        fprintf(stderr, "Incomplete CAN frame\n");
        return false;
    }

    if (frame.can_id == STEERING_CAN_ID) {
        // Unpack float value (little-endian)
        float steering_value;
        memcpy(&steering_value, frame.data, sizeof(float));
        steering_angle_deg = steering_value;
        return true;
    } else if (frame.can_id == THROTTLE_CAN_ID) {
        // Unpack float value (little-endian)
        float throttle_value;
        memcpy(&throttle_value, frame.data, sizeof(float));
        throttle = throttle_value;
        return true;
    }
    return false;
}


int main() {
    // ----- Simulation Parameters -----
    double dt = 0.01;             // Time step (s)
    double simulation_time = 1000.0; // Total simulation time (s)

    // Control inputs initialized to zero
    double steering_angle_deg = 0.0; // degrees
    double throttle = 0.0;           // arbitrary units

    // ----- Vehicle Parameters -----
    double mass = 1500.0;          // Mass (kg)
    double Iz = 3000.0;            // Yaw moment of inertia (kg·m²)
    double lf = 1.2;               // Distance from CG to front axle (m)
    double lr = 1.6;               // Distance from CG to rear axle (m)
    double track_width = 1.5;      // Track width (m)
    double wheel_radius = 0.3;     // Wheel radius (m)
    double mu = 1.0;               // Friction coefficient

    // ----- IMU Noise and Bias Parameters -----
    double accel_noise_std = 0.05;   // Accelerometer noise standard deviation (m/s^2)
    double gyro_noise_std = 0.005;    // Gyroscope noise standard deviation (rad/s)
    double accel_bias_std = 0.01;     // Accelerometer bias standard deviation (m/s^2)
    double gyro_bias_std = 0.001;     // Gyroscope bias standard deviation (rad/s)
    double accel_drift_rate = 0.001;  // Accelerometer bias drift rate (m/s^3)
    double gyro_drift_rate = 0.0001;  // Gyroscope bias drift rate (rad/s^2)

    // ----- Initialize Vehicle Model -----
    VehicleModel vehicle(mass, Iz, lf, lr, track_width, wheel_radius, mu);

    // ----- Initialize IMU Sensor -----
    IMUSensor imu_sensor(mass,
                         accel_noise_std, gyro_noise_std,
                         accel_bias_std, gyro_bias_std,
                         accel_drift_rate, gyro_drift_rate);

    // ----- Start IMU Sensor Thread -----
    imu_sensor.start(dt);

    // ----- Initialize CAN Socket -----
    int can_socket = initCANSocket("vcan0");
    if (can_socket < 0) {
        std::cerr << "Failed to initialize CAN socket. Exiting.\n";
        return -1;
    }

    // Set CAN socket to non-blocking mode
    int flags = fcntl(can_socket, F_GETFL, 0);
    fcntl(can_socket, F_SETFL, flags | O_NONBLOCK);

    // ----- Initialize Logger -----
    Logger logger("simulation_log.csv");

    // IMU measurements initialization
    IMUMeasurements imu_measurements;
    imu_measurements.accel.x = 0.0;
    imu_measurements.accel.y = 0.0;
    imu_measurements.gyro_z = 0.0;

    bool imu_accel_x_received = false;
    bool imu_accel_y_received = false;
    bool imu_gyro_z_received = false;

    // ----- Simulation Loop -----
    double elapsed_time = 0.0;
    auto start_time = std::chrono::steady_clock::now();

    VehicleForces forces;
    MotionCovariance motion_cov;
    VehicleState state;

    while (elapsed_time <= simulation_time) {
        // Read control commands from CAN bus
        double new_steering_angle_deg = steering_angle_deg;
        double new_throttle = throttle;

        bool command_received = false;
        while (readControlCommands(can_socket, new_steering_angle_deg, new_throttle)) {
            // Update control inputs
            steering_angle_deg = new_steering_angle_deg;
            throttle = new_throttle;
            command_received = true;
        }

        // Read IMU measurements from CAN bus
        // Reset flags at the beginning of each iteration
        imu_accel_x_received = false;
        imu_accel_y_received = false;
        imu_gyro_z_received = false;

        // Attempt to read multiple messages per time step
        for (int i = 0; i < 10; ++i) {
            if (readIMUMeasurements(can_socket, imu_measurements)) {
                // Continue reading until no more data
                continue;
            } else {
                // No more data available
                break;
            }
        }

        // Ensure we have all IMU data before updating the vehicle state
        if (imu_accel_x_received && imu_accel_y_received && imu_gyro_z_received) {
            // Convert steering angle from degrees to radians
            double steering_angle_rad = steering_angle_deg * M_PI / 180.0;

            // Update vehicle state using IMU measurements
            std::tie(forces, motion_cov) = vehicle.update(steering_angle_rad, throttle, imu_measurements, dt);

            // Get current vehicle state
            state = vehicle.getState();

            // Update IMU sensor with latest forces and state
            imu_sensor.updateForcesAndState(forces, state);

            // Log data
            logger.log(elapsed_time, steering_angle_deg, throttle, forces, state, motion_cov, imu_measurements);
        }

        // Optional: Print to console at specific intervals
        if (static_cast<int>(elapsed_time * 100) % 1000 == 0) { // Every 10 seconds
            std::cout << "Time: " << elapsed_time << " s\n";
            std::cout << "Steering Angle: " << steering_angle_deg << " degrees\n";
            std::cout << "Throttle: " << throttle << " units\n";
            std::cout << "Forces: (F_x: " << forces.force[0] << " N, F_y: " << forces.force[1] << " N)\n";
            std::cout << "Moment: M_z: " << forces.moment << " Nm\n";
            std::cout << "Position: (x: " << state.position[0] << " m, y: " << state.position[1] << " m)\n";
            std::cout << "Orientation: psi: " << state.psi << " rad\n";
            std::cout << "IMU Measurements: (a_x: " << imu_measurements.accel.x
                      << " m/s^2, a_y: " << imu_measurements.accel.y
                      << " m/s^2, gyro_z: " << imu_measurements.gyro_z << " rad/s)\n\n";
        }

        // Sleep until the next time step
        std::this_thread::sleep_until(start_time + std::chrono::duration<double>(elapsed_time + dt));
        elapsed_time += dt;
    }

    // ----- Stop IMU Sensor Thread -----
    imu_sensor.stop();
    imu_sensor.join();

    // Close CAN socket
    close(can_socket);

    std::cout << "Simulation completed. Data logged to 'simulation_log.csv'.\n";

    return 0;
}
