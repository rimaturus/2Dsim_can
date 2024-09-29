// graph_slam.cpp

/**
 * @file graph_slam.cpp
 * @brief Implementation of an enhanced GraphSLAM system with SocketCAN integration.
 *
 * This file contains the implementation of a GraphSLAM system that processes
 * pose and landmark data from a CAN bus interface, constructs a graph-based
 * representation of the environment, and performs optimization to compute
 * the posterior probability of the robot's poses and landmark positions.
 */

#include <iostream>
#include <fstream>
#include <vector>
#include <unordered_map>
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <cmath>
#include <chrono>
#include <thread>
#include <cstring>
#include <mutex>
#include <string>

// SocketCAN headers
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <unistd.h>

// YAML-CPP header
#include <yaml-cpp/yaml.h>

using namespace std;
using namespace Eigen;

/**
 * @struct Config
 * @brief Holds configuration parameters loaded from a YAML file.
 */
struct Config {
    double pixels_per_meter;

    /**
     * @struct Car
     * @brief Car parameters.
     */
    struct Car {
        double wheelbase;      ///< Distance between front and rear axles.
        double max_throttle;   ///< Maximum throttle value.
        double max_speed;      ///< Maximum speed.
    } car;

    /**
     * @struct Perception
     * @brief Perception noise parameters.
     */
    struct Perception {
        double range_noise_std_dev;      ///< Standard deviation of range noise.
        double bearing_noise_std_dev;    ///< Standard deviation of bearing noise.
        double detection_range;          ///< Maximum detection range.
    } perception;

    /**
     * @struct PIDController
     * @brief PID controller parameters.
     */
    struct PIDController {
        double Kp;          ///< Proportional gain.
        double Ki;          ///< Integral gain.
        double Kd;          ///< Derivative gain.
        double setpoint;    ///< Desired setpoint.
    } pid_controller;

    /**
     * @struct Control
     * @brief Control parameters.
     */
    struct Control {
        double target_speed;    ///< Target speed.
        double throttle_Kp;     ///< Proportional gain for throttle control.
    } control;

    /**
     * @struct CANIDs
     * @brief CAN IDs for communication.
     */
    struct CANIDs {
        uint32_t STEERING_CAN_ID;       ///< CAN ID for steering commands.
        uint32_t THROTTLE_CAN_ID;       ///< CAN ID for throttle commands.
        uint32_t CAR_X_CAN_ID;          ///< CAN ID for car's X position.
        uint32_t CAR_Y_CAN_ID;          ///< CAN ID for car's Y position.
        uint32_t CAR_ANGLE_CAN_ID;      ///< CAN ID for car's angle.
        uint32_t CONE_CAN_ID_START;     ///< Starting CAN ID for cones.
    } can_ids;

    /**
     * @struct SensorCovariance
     * @brief Sensor covariance parameters.
     */
    struct SensorCovariance {
        double sigma_r;    ///< Standard deviation for range measurements.
        double sigma_b;    ///< Standard deviation for bearing measurements.
    } sensor_covariance;
};

/**
 * @brief Loads configuration parameters from a YAML file.
 * @param filepath Path to the YAML configuration file.
 * @param config Reference to a Config struct to be filled.
 * @return True if loading is successful, false otherwise.
 */
bool load_config(const std::string& filepath, Config& config) {
    try {
        YAML::Node config_yaml = YAML::LoadFile(filepath);

        // Load basic parameters
        config.pixels_per_meter = config_yaml["PIXELS_PER_METER"].as<double>();

        // Load car parameters
        config.car.wheelbase = config_yaml["car"]["wheelbase"].as<double>();
        config.car.max_throttle = config_yaml["car"]["max_throttle"].as<double>();
        config.car.max_speed = config_yaml["car"]["max_speed"].as<double>();

        // Load perception parameters
        config.perception.range_noise_std_dev = config_yaml["perception"]["range_noise_std_dev"].as<double>();
        config.perception.bearing_noise_std_dev = config_yaml["perception"]["bearing_noise_std_dev"].as<double>();
        config.perception.detection_range = config_yaml["perception"]["detection_range"].as<double>();

        // Load PID controller parameters
        config.pid_controller.Kp = config_yaml["pid_controller"]["Kp"].as<double>();
        config.pid_controller.Ki = config_yaml["pid_controller"]["Ki"].as<double>();
        config.pid_controller.Kd = config_yaml["pid_controller"]["Kd"].as<double>();
        config.pid_controller.setpoint = config_yaml["pid_controller"]["setpoint"].as<double>();

        // Load control parameters
        config.control.target_speed = config_yaml["control"]["target_speed"].as<double>();
        config.control.throttle_Kp = config_yaml["control"]["throttle_Kp"].as<double>();

        // Load CAN IDs
        config.can_ids.STEERING_CAN_ID = std::stoul(config_yaml["CAN_IDS"]["STEERING_CAN_ID"].as<std::string>(), nullptr, 16);
        config.can_ids.THROTTLE_CAN_ID = std::stoul(config_yaml["CAN_IDS"]["THROTTLE_CAN_ID"].as<std::string>(), nullptr, 16);
        config.can_ids.CAR_X_CAN_ID = std::stoul(config_yaml["CAN_IDS"]["CAR_X_CAN_ID"].as<std::string>(), nullptr, 16);
        config.can_ids.CAR_Y_CAN_ID = std::stoul(config_yaml["CAN_IDS"]["CAR_Y_CAN_ID"].as<std::string>(), nullptr, 16);
        config.can_ids.CAR_ANGLE_CAN_ID = std::stoul(config_yaml["CAN_IDS"]["CAR_ANGLE_CAN_ID"].as<std::string>(), nullptr, 16);
        config.can_ids.CONE_CAN_ID_START = std::stoul(config_yaml["CAN_IDS"]["CONE_CAN_ID_START"].as<std::string>(), nullptr, 16);

        // Load sensor covariance parameters
        config.sensor_covariance.sigma_r = config_yaml["LIDAR_COVARIANCE"]["SIGMA_RANGE"].as<double>();
        config.sensor_covariance.sigma_b = config_yaml["LIDAR_COVARIANCE"]["SIGMA_BEARING"].as<double>();

        return true;
    }
    catch (const YAML::Exception& e) {
        std::cerr << "Error parsing config file: " << e.what() << std::endl;
        return false;
    }
}

/**
 * @struct Pose
 * @brief Represents the robot's pose in 2D space.
 */
struct Pose {
    double x;      ///< X-coordinate
    double y;      ///< Y-coordinate
    double theta;  ///< Orientation (heading) in radians

    /**
     * @brief Constructor for Pose.
     * @param x X-coordinate
     * @param y Y-coordinate
     * @param theta Orientation (heading) in radians
     */
    Pose(double x = 0.0, double y = 0.0, double theta = 0.0)
        : x(x), y(y), theta(theta) {}
};

/**
 * @struct Landmark
 * @brief Represents a landmark (e.g., cone) in the environment.
 */
struct Landmark {
    int id;           ///< Unique identifier for the landmark
    double x;         ///< X-coordinate
    double y;         ///< Y-coordinate
    std::string type; ///< Type of landmark (e.g., "blue", "yellow")

    /**
     * @brief Default constructor for Landmark.
     */
    Landmark() : id(0), x(0.0), y(0.0), type("unknown") {}

    /**
     * @brief Constructor for Landmark.
     * @param id Unique identifier
     * @param x X-coordinate
     * @param y Y-coordinate
     * @param type Type of landmark
     */
    Landmark(int id, double x = 0.0, double y = 0.0, const std::string& type = "unknown")
        : id(id), x(x), y(y), type(type) {}
};

/**
 * @struct Measurement
 * @brief Represents a measurement linking a pose to a landmark.
 */
struct Measurement {
    int pose_id;      ///< ID of the pose
    int landmark_id;  ///< ID of the landmark
    double range;     ///< Measured range to the landmark
    double bearing;   ///< Measured bearing to the landmark

    /**
     * @brief Constructor for Measurement.
     * @param pid Pose ID
     * @param lid Landmark ID
     * @param range Measured range
     * @param bearing Measured bearing
     */
    Measurement(int pid, int lid, double range, double bearing)
        : pose_id(pid), landmark_id(lid), range(range), bearing(bearing) {}
};

/**
 * @class GraphSLAM
 * @brief Implements an enhanced GraphSLAM algorithm.
 *
 * The GraphSLAM class maintains a graph-based representation of the robot's poses
 * and landmarks in the environment. It provides methods to add poses, landmarks,
 * and measurements, and performs optimization to compute the posterior probability
 * of the robot's trajectory and landmark positions.
 */
class GraphSLAM {
public:
    std::vector<Pose> poses;                          ///< List of robot poses
    std::unordered_map<int, Landmark> landmarks;      ///< Map of landmarks
    std::vector<Measurement> measurements;            ///< List of measurements

    /**
     * @brief Adds a new pose to the SLAM graph.
     * @param x X-coordinate
     * @param y Y-coordinate
     * @param theta Orientation in radians
     */
    void addPose(double x, double y, double theta);

    /**
     * @brief Adds or updates a landmark in the SLAM graph.
     * @param id Landmark ID
     * @param x X-coordinate
     * @param y Y-coordinate
     * @param type Type of landmark (e.g., "blue", "yellow")
     */
    void addLandmark(int id, double x, double y, const std::string& type);

    /**
     * @brief Adds a measurement linking a pose to a landmark.
     * @param pose_id ID of the pose
     * @param landmark_id ID of the landmark
     * @param range Measured range
     * @param bearing Measured bearing
     */
    void addMeasurement(int pose_id, int landmark_id, double range, double bearing);

    /**
     * @brief Performs optimization on the SLAM graph to compute posterior probability.
     */
    void optimize();

private:
    std::mutex mtx;  ///< Mutex for thread safety

    /**
     * @brief Maps each variable to a unique index for matrix construction.
     * @param index_map Reference to an unordered_map to store variable-index mappings.
     * @return Total number of variables.
     */
    int mapIndices(std::unordered_map<std::string, int>& index_map);

    /**
     * @brief Computes the posterior probability of the current SLAM graph.
     * @param A Information matrix.
     * @param b Information vector.
     * @return Posterior probability.
     */
    double computePosterior(const MatrixXd& A, const VectorXd& b_vec);

    /**
     * @brief Saves the current SLAM graph to a file.
     * @param filename Name of the file.
     */
    void saveGraph(const std::string& filename);
};

void GraphSLAM::addPose(double x, double y, double theta) {
    std::lock_guard<std::mutex> lock(mtx);

    // Create a new pose object
    Pose new_pose(x, y, theta);

    // Optional: Check for duplicate poses
    if (!poses.empty()) {
        const Pose& last_pose = poses.back();
        if (std::abs(new_pose.x - last_pose.x) < 1e-6 &&
            std::abs(new_pose.y - last_pose.y) < 1e-6 &&
            std::abs(new_pose.theta - last_pose.theta) < 1e-6) {
            std::cout << "Duplicate pose detected; skipping addition." << std::endl;
            return;
        }
    }

    // Add the new pose to the list
    poses.push_back(new_pose);
    std::cout << "Added Pose: x=" << x << ", y=" << y << ", theta=" << theta << std::endl;

    // Save the updated graph
    saveGraph("graph_live_output.txt");
}

void GraphSLAM::addLandmark(int id, double x, double y, const std::string& type) {
    std::lock_guard<std::mutex> lock(mtx);

    // Check if the landmark already exists
    auto it = landmarks.find(id);
    if (it != landmarks.end()) {
        // Update the existing landmark's position and type
        it->second.x = x;
        it->second.y = y;
        it->second.type = type;
        std::cout << "Updated " << type << " Landmark ID " << id << " to new position." << std::endl;
    }
    else {
        // Add a new landmark
        landmarks[id] = Landmark(id, x, y, type);
        std::cout << "Added new " << type << " Landmark ID " << id << "." << std::endl;
    }

    // Save the updated graph
    saveGraph("graph_live_output.txt");
}

void GraphSLAM::addMeasurement(int pose_id, int landmark_id, double range, double bearing) {
    std::lock_guard<std::mutex> lock(mtx);

    // Validate pose_id and landmark_id
    if (pose_id < 0 || pose_id >= poses.size()) {
        std::cerr << "Invalid pose_id: " << pose_id << std::endl;
        return;
    }
    if (landmarks.find(landmark_id) == landmarks.end()) {
        std::cerr << "Invalid landmark_id: " << landmark_id << std::endl;
        return;
    }

    // Create a new measurement
    Measurement measurement(pose_id, landmark_id, range, bearing);

    // Add the measurement to the list
    measurements.push_back(measurement);
    std::cout << "Added Measurement: Pose ID=" << pose_id << ", Landmark ID=" << landmark_id << std::endl;

    // Save the updated graph
    saveGraph("graph_live_output.txt");
}

int GraphSLAM::mapIndices(std::unordered_map<std::string, int>& index_map) {
    int index = 0;

    // Map poses
    for (size_t i = 0; i < poses.size(); ++i) {
        index_map["pose_" + to_string(i) + "_x"] = index++;
        index_map["pose_" + to_string(i) + "_y"] = index++;
        index_map["pose_" + to_string(i) + "_theta"] = index++;
    }

    // Map landmarks
    for (const auto& [id, landmark] : landmarks) {
        index_map["landmark_" + to_string(id) + "_x"] = index++;
        index_map["landmark_" + to_string(id) + "_y"] = index++;
    }

    return index;
}

double GraphSLAM::computePosterior(const MatrixXd& A, const VectorXd& b_vec) {
    // Compute the posterior probability up to a normalization constant
    // P(poses, landmarks | measurements) ~ exp(-0.5 * (A * x - b)^T * (A * x - b))
    // Since x has been optimized to minimize this, we can compute:
    // Exponent = 0.5 * b^T * A^-1 * b
    // Posterior ~ exp(-Exponent)

    // Compute b^T * A^-1 * b
    VectorXd Ax = A.inverse() * b_vec;
    double exponent = 0.5 * b_vec.dot(Ax);
    double posterior = std::exp(-exponent);

    return posterior;
}

void GraphSLAM::optimize() {
    std::lock_guard<std::mutex> lock(mtx);

    // Check if there are enough data points
    if (poses.size() < 2 || measurements.empty()) {
        std::cout << "Not enough data to optimize." << std::endl;
        return;
    }

    std::cout << "Starting optimization..." << std::endl;

    // Map variables to indices
    std::unordered_map<std::string, int> index_map;
    int num_vars = mapIndices(index_map);

    // Initialize information matrix A and vector b
    MatrixXd A = MatrixXd::Zero(num_vars, num_vars);
    VectorXd b_vec = VectorXd::Zero(num_vars);

    // Information matrix for measurements (inverse of sensor covariance)
    Matrix2d Omega;
    double sigma_r = 1.0; // Example value, should be set based on config
    double sigma_b = 1.0; // Example value, should be set based on config
    Omega << 1.0 / (sigma_r * sigma_r), 0,
             0, 1.0 / (sigma_b * sigma_b);

    // Loop over all measurements to build A and b
    for (const auto& m : measurements) {
        int pose_idx = m.pose_id;
        int landmark_id = m.landmark_id;

        Pose& pose = poses[pose_idx];
        Landmark& landmark = landmarks[landmark_id];

        // Compute expected measurement based on current estimates
        double dx = landmark.x - pose.x;
        double dy = landmark.y - pose.y;
        double expected_range = std::sqrt(dx * dx + dy * dy);
        double expected_bearing = std::atan2(dy, dx) - pose.theta;
        // Normalize bearing to [-pi, pi]
        expected_bearing = std::atan2(std::sin(expected_bearing), std::cos(expected_bearing));

        // Measurement error
        double range_error = m.range - expected_range;
        double bearing_error = m.bearing - expected_bearing;
        bearing_error = std::atan2(std::sin(bearing_error), std::cos(bearing_error));

        // Jacobians
        double q = dx * dx + dy * dy;
        double sqrt_q = std::sqrt(q);
        double dr_dx = -dx / sqrt_q;
        double dr_dy = -dy / sqrt_q;
        double db_dx = dy / q;
        double db_dy = -dx / q;
        double db_dtheta = -1.0;

        // Partial derivatives w.r.t pose
        VectorXd J_pose(3);
        J_pose << dr_dx, dr_dy, db_dtheta;

        // Partial derivatives w.r.t landmark
        VectorXd J_landmark(2);
        J_landmark << dr_dx, dr_dy;

        // Assemble the full Jacobian
        VectorXd J = VectorXd::Zero(num_vars);
        // Pose indices
        std::string pose_x = "pose_" + to_string(pose_idx) + "_x";
        std::string pose_y = "pose_" + to_string(pose_idx) + "_y";
        std::string pose_theta = "pose_" + to_string(pose_idx) + "_theta";

        int idx_pose_x = index_map[pose_x];
        int idx_pose_y = index_map[pose_y];
        int idx_pose_theta = index_map[pose_theta];

        // Landmark indices
        std::string landmark_x = "landmark_" + to_string(landmark_id) + "_x";
        std::string landmark_y = "landmark_" + to_string(landmark_id) + "_y";

        int idx_landmark_x = index_map[landmark_x];
        int idx_landmark_y = index_map[landmark_y];

        // Fill Jacobian
        J(idx_pose_x) = J_pose(0);
        J(idx_pose_y) = J_pose(1);
        J(idx_pose_theta) = J_pose(2);
        J(idx_landmark_x) = J_landmark(0);
        J(idx_landmark_y) = J_landmark(1);

        // Update A and b
        MatrixXd J_transpose_Omega = J.transpose() * Omega;
        A += J_transpose_Omega * J;
        b_vec += J_transpose_Omega * Vector2d(range_error, bearing_error);
    }

    // Solve for delta_x: A * delta_x = b
    VectorXd delta_x = VectorXd::Zero(num_vars);
    // Using Eigen's LLT decomposition for symmetric positive definite matrices
    Eigen::LLT<MatrixXd> llt(A);
    if (llt.info() == Eigen::NumericalIssue) {
        std::cerr << "LLT failed: Matrix might not be positive definite." << std::endl;
        return;
    }
    delta_x = llt.solve(b_vec);

    // Update poses and landmarks
    for (size_t i = 0; i < poses.size(); ++i) {
        std::string pose_x = "pose_" + to_string(i) + "_x";
        std::string pose_y = "pose_" + to_string(i) + "_y";
        std::string pose_theta = "pose_" + to_string(i) + "_theta";

        poses[i].x += delta_x[index_map[pose_x]];
        poses[i].y += delta_x[index_map[pose_y]];
        poses[i].theta += delta_x[index_map[pose_theta]];
    }

    for (auto& [id, landmark] : landmarks) {
        std::string landmark_x = "landmark_" + to_string(id) + "_x";
        std::string landmark_y = "landmark_" + to_string(id) + "_y";

        landmark.x += delta_x[index_map[landmark_x]];
        landmark.y += delta_x[index_map[landmark_y]];
    }

    // Compute posterior probability
    double posterior = computePosterior(A, b_vec);
    std::cout << "Posterior Probability: " << posterior << std::endl;

    // Save the optimized graph
    saveGraph("graph_live_output.txt");
}

double GraphSLAM::computePosterior(const MatrixXd& A, const VectorXd& b_vec) {
    // Compute the posterior probability up to a normalization constant
    // P(poses, landmarks | measurements) ~ exp(-0.5 * (A * x - b)^T * (A * x - b))
    // Since x has been optimized to minimize this, we can compute:
    // Exponent = 0.5 * b^T * A^-1 * b
    // Posterior ~ exp(-Exponent)

    // Compute b^T * A^-1 * b
    VectorXd Ax = A.inverse() * b_vec;
    double exponent = 0.5 * b_vec.dot(Ax);
    double posterior = std::exp(-exponent);

    return posterior;
}

int GraphSLAM::mapIndices(std::unordered_map<std::string, int>& index_map) {
    int index = 0;

    // Map poses
    for (size_t i = 0; i < poses.size(); ++i) {
        index_map["pose_" + to_string(i) + "_x"] = index++;
        index_map["pose_" + to_string(i) + "_y"] = index++;
        index_map["pose_" + to_string(i) + "_theta"] = index++;
    }

    // Map landmarks
    for (const auto& [id, landmark] : landmarks) {
        index_map["landmark_" + to_string(id) + "_x"] = index++;
        index_map["landmark_" + to_string(id) + "_y"] = index++;
    }

    return index;
}

void GraphSLAM::saveGraph(const std::string& filename) {
    // Assume mtx is already locked

    std::ofstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Error: Could not open file for writing!" << std::endl;
        return;
    }

    // Write poses
    file << "Poses:\n";
    for (size_t i = 0; i < poses.size(); ++i) {
        file << i << " " << poses[i].x << " " << poses[i].y << " " << poses[i].theta << "\n";
    }

    // Write landmarks
    file << "Landmarks:\n";
    for (const auto& [id, landmark] : landmarks) {
        file << id << " " << landmark.x << " " << landmark.y << " " << landmark.type << "\n";
    }

    // Write measurements
    file << "Measurements:\n";
    for (const auto& m : measurements) {
        file << m.pose_id << " " << m.landmark_id << " " << m.range << " " << m.bearing << "\n";
    }

    file.close();
}

/**
 * @brief Initializes a CAN socket for communication.
 * @param interface_name Name of the CAN interface (e.g., "vcan0").
 * @return File descriptor of the CAN socket, or -1 on error.
 */
int init_can_socket(const std::string& interface_name) {
    // Create a socket
    int s = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (s < 0) {
        perror("Error while opening socket");
        return -1;
    }

    // Specify the CAN interface to use
    struct ifreq ifr;
    strcpy(ifr.ifr_name, interface_name.c_str());
    if (ioctl(s, SIOCGIFINDEX, &ifr) < 0) {
        perror("Error getting interface index");
        return -1;
    }

    // Bind the socket to the CAN interface
    struct sockaddr_can addr = {};
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (bind(s, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        perror("Error in socket bind");
        return -1;
    }

    return s;
}

/**
 * @brief Processes a received CAN frame and updates the SLAM graph.
 * @param frame The received CAN frame.
 * @param slam Reference to the GraphSLAM object.
 * @param config Configuration parameters.
 */
void process_can_frame(const struct can_frame& frame, GraphSLAM& slam, const Config& config) {
    uint32_t can_id = frame.can_id & CAN_EFF_MASK;  // Extract CAN ID
    float value;

    // Ensure the data length is correct
    if (frame.can_dlc != 4 && frame.can_dlc != 8) {
        fprintf(stderr, "Unexpected CAN frame length: %d\n", frame.can_dlc);
        return;
    }

    // Process pose data
    static double x = 0.0, y = 0.0, theta = 0.0;

    if (can_id == config.can_ids.CAR_X_CAN_ID) {
        memcpy(&value, frame.data, sizeof(float));
        x = value;
    }
    else if (can_id == config.can_ids.CAR_Y_CAN_ID) {
        memcpy(&value, frame.data, sizeof(float));
        y = value;
    }
    else if (can_id == config.can_ids.CAR_ANGLE_CAN_ID) {
        memcpy(&value, frame.data, sizeof(float));
        theta = value;
        // After receiving theta, we can add a new pose
        slam.addPose(x, y, theta);
        std::cout << "Added Pose: x=" << x << ", y=" << y << ", theta=" << theta << "\n";
    }
    else {
        // Process landmark data (cones)
        bool is_blue_cone = false;
        bool is_yellow_cone = false;
        int cone_id = -1;
        std::string cone_type;

        if (can_id >= config.can_ids.CONE_CAN_ID_START &&
            can_id < config.can_ids.CONE_CAN_ID_START + 0x80) { // Assuming 0x80 blue cones
            // Blue cones
            is_blue_cone = true;
            cone_id = can_id - config.can_ids.CONE_CAN_ID_START;
            cone_type = "blue";
        }
        else if (can_id >= config.can_ids.CONE_CAN_ID_START + 0x80 &&
                 can_id < config.can_ids.CONE_CAN_ID_START + 0x100) { // Assuming 0x80 yellow cones
            // Yellow cones
            is_yellow_cone = true;
            cone_id = can_id - (config.can_ids.CONE_CAN_ID_START + 0x80);
            cone_type = "yellow";
        }

        if (is_blue_cone || is_yellow_cone) {
            float range, bearing;
            if (frame.can_dlc == 8) {
                memcpy(&range, frame.data, sizeof(float));
                memcpy(&bearing, frame.data + 4, sizeof(float));

                // Convert bearing from degrees to radians if necessary
                double bearing_rad = bearing * M_PI / 180.0;

                // Convert range and bearing to landmark position
                double landmark_x = x + range * cos(theta + bearing_rad);
                double landmark_y = y + range * sin(theta + bearing_rad);

                // Add the landmark with type
                slam.addLandmark(cone_id, landmark_x, landmark_y, cone_type);
                std::cout << "Added " << cone_type << " Landmark: id=" << cone_id
                          << ", x=" << landmark_x << ", y=" << landmark_y << "\n";

                // Add the measurement
                slam.addMeasurement(slam.poses.size() - 1, cone_id, range, bearing_rad);
            }
            else {
                fprintf(stderr, "Invalid data length for landmark\n");
            }
        }
        else {
            // Unknown CAN ID
            fprintf(stderr, "Unknown CAN ID: 0x%X\n", can_id);
        }
    }
}

/**
 * @brief Reads CAN messages and processes them.
 * @param socket_fd File descriptor of the CAN socket.
 * @param slam Reference to the GraphSLAM object.
 * @param config Configuration parameters.
 */
void read_can_messages(int socket_fd, GraphSLAM& slam, const Config& config) {
    struct can_frame frame;
    while (true) {
        int nbytes = read(socket_fd, &frame, sizeof(struct can_frame));
        if (nbytes < 0) {
            perror("CAN read error");
            break;
        }
        else if (nbytes < sizeof(struct can_frame)) {
            fprintf(stderr, "Incomplete CAN frame\n");
            continue;
        }

        // Process the frame
        process_can_frame(frame, slam, config);
    }
}

/**
 * @brief Main function of the GraphSLAM application.
 * @param argc Argument count.
 * @param argv Argument vector.
 * @return Exit status.
 */
int main(int argc, char* argv[]) {
    // Check for config file path argument
    if (argc < 2) {
        std::cerr << "Usage: " << argv[0] << " <path_to_config.yaml>" << std::endl;
        return -1;
    }

    std::string config_path = argv[1];
    Config config;

    // Load configuration
    if (!load_config(config_path, config)) {
        return -1;
    }

    GraphSLAM slam;

    // Initialize CAN socket
    int can_socket = init_can_socket("vcan0");
    if (can_socket < 0) {
        return -1;
    }

    // Start a thread to read CAN messages
    std::thread can_thread(read_can_messages, can_socket, std::ref(slam), std::ref(config));

    // Main thread can perform other tasks
    while (true) {
        // Perform optimization periodically
        slam.optimize();
        std::this_thread::sleep_for(std::chrono::seconds(5));
    }

    // Join the CAN thread (this line may not be reached in this example)
    can_thread.join();

    // Close the CAN socket
    close(can_socket);

    return 0;
}
