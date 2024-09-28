// graph_slam.cpp

// Include necessary headers
#include <iostream>
#include <fstream>
#include <vector>
#include <unordered_map>
#include <Eigen/Dense>
#include <cmath>
#include <chrono>
#include <thread>
#include <cstring>
#include <mutex>

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

// Configuration structure
struct Config {
    double pixels_per_meter;

    struct Car {
        double wheelbase;
        double max_throttle;
        double max_speed;
    } car;

    struct Perception {
        double range_noise_std_dev;
        double bearing_noise_std_dev;
        double detection_range;
    } perception;

    struct PIDController {
        double Kp;
        double Ki;
        double Kd;
        double setpoint;
    } pid_controller;

    struct Control {
        double target_speed;
        double throttle_Kp;
    } control;

    struct CANIDs {
        uint32_t STEERING_CAN_ID;
        uint32_t THROTTLE_CAN_ID;
        uint32_t CAR_X_CAN_ID;
        uint32_t CAR_Y_CAN_ID;
        uint32_t CAR_ANGLE_CAN_ID;
        uint32_t CONE_CAN_ID_START;
    } can_ids;
};

// Function to load configuration
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

        return true;
    }
    catch (const YAML::Exception& e) {
        std::cerr << "Error parsing config file: " << e.what() << std::endl;
        return false;
    }
}

// Data structures for Pose, Landmark, and Measurement
struct Pose {
    double x, y, theta;
    Pose(double x = 0.0, double y = 0.0, double theta = 0.0)
        : x(x), y(y), theta(theta) {}
};

struct Landmark {
    int id;
    double x, y;

    // Default constructor
    Landmark() : id(0), x(0.0), y(0.0) {}

    // Parameterized constructor
    Landmark(int id, double x = 0.0, double y = 0.0)
        : id(id), x(x), y(y) {}
};

struct Measurement {
    int pose_id;
    int landmark_id;
    double range, bearing;
    Measurement(int pid, int lid, double range, double bearing)
        : pose_id(pid), landmark_id(lid), range(range), bearing(bearing) {}
};

// GraphSLAM class
class GraphSLAM {
public:
    std::vector<Pose> poses;
    std::unordered_map<int, Landmark> landmarks;
    std::vector<Measurement> measurements;

    // Mutex for thread safety
    std::mutex mtx;

    void addPose(double x, double y, double theta) {
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

    void addLandmark(int id, double x, double y) {
        std::lock_guard<std::mutex> lock(mtx);

        // Check if the landmark already exists
        auto it = landmarks.find(id);
        if (it != landmarks.end()) {
            // Update the existing landmark's position
            it->second.x = x;
            it->second.y = y;
            std::cout << "Updated Landmark ID " << id << " to new position." << std::endl;
        } else {
            // Add a new landmark
            landmarks[id] = Landmark(id, x, y);
            std::cout << "Added new Landmark ID " << id << "." << std::endl;
        }

        // Save the updated graph
        saveGraph("graph_live_output.txt");
    }

    void addMeasurement(int pose_id, int landmark_id, double range, double bearing) {
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

    void optimize() {
        std::lock_guard<std::mutex> lock(mtx);

        // Check if there are enough data points
        if (poses.size() < 2 || measurements.empty()) {
            std::cout << "Not enough data to optimize." << std::endl;
            return;
        }

        std::cout << "Starting optimization..." << std::endl;

        // Parameters for optimization
        const int max_iterations = 10;
        const double convergence_threshold = 1e-4;
        double prev_total_error = std::numeric_limits<double>::max();

        // Optimization loop
        for (int iter = 0; iter < max_iterations; ++iter) {
            double total_error = 0.0;

            // Loop over all measurements
            for (const auto& m : measurements) {
                int pose_idx = m.pose_id;
                int landmark_id = m.landmark_id;

                Pose& pose = poses[pose_idx];
                Landmark& landmark = landmarks[landmark_id];

                // Compute expected measurement
                double dx = landmark.x - pose.x;
                double dy = landmark.y - pose.y;
                double predicted_range = std::sqrt(dx * dx + dy * dy);
                double predicted_bearing = std::atan2(dy, dx) - pose.theta;

                // Normalize bearing to [-pi, pi]
                predicted_bearing = std::atan2(std::sin(predicted_bearing), std::cos(predicted_bearing));

                // Compute errors
                double range_error = m.range - predicted_range;
                double bearing_error = m.bearing - predicted_bearing;
                bearing_error = std::atan2(std::sin(bearing_error), std::cos(bearing_error));

                // Accumulate total error
                total_error += range_error * range_error + bearing_error * bearing_error;

                // Compute Jacobians (simplified)
                double q = dx * dx + dy * dy;
                double sqrt_q = std::sqrt(q);
                double dr_dx = -dx / sqrt_q;
                double dr_dy = -dy / sqrt_q;
                double db_dx = dy / q;
                double db_dy = -dx / q;

                // Update estimates with a simple gradient descent
                double alpha = 0.1;  // Learning rate

                // Update pose
                pose.x -= alpha * (-range_error * dr_dx - bearing_error * db_dx);
                pose.y -= alpha * (-range_error * dr_dy - bearing_error * db_dy);
                pose.theta -= alpha * (-bearing_error);

                // Update landmark
                landmark.x -= alpha * (range_error * dr_dx + bearing_error * db_dx);
                landmark.y -= alpha * (range_error * dr_dy + bearing_error * db_dy);
            }

            std::cout << "Iteration " << iter + 1 << ": Total error = " << total_error << std::endl;

            // Check for convergence
            if (std::abs(prev_total_error - total_error) < convergence_threshold) {
                std::cout << "Converged after " << iter + 1 << " iterations." << std::endl;
                break;
            }

            prev_total_error = total_error;
        }

        // Save the optimized graph
        saveGraph("graph_live_output.txt");
    }

private:
    void saveGraph(const std::string& filename) {
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
            file << id << " " << landmark.x << " " << landmark.y << "\n";
        }

        // Write measurements
        file << "Measurements:\n";
        for (const auto& m : measurements) {
            file << m.pose_id << " " << m.landmark_id << " " << m.range << " " << m.bearing << "\n";
        }

        file.close();
    }
};

// Initialize CAN socket
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

// Process CAN frames
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

    if (can_id == config.can_ids.CAR_X_CAN_ID){ // CAR_X_CAN_ID
        memcpy(&value, frame.data, sizeof(float));
        x = value;
    }
    else if (can_id == config.can_ids.CAR_Y_CAN_ID){ // CAR_Y_CAN_ID
        memcpy(&value, frame.data, sizeof(float));
        y = value;
    }
    else if (can_id == config.can_ids.CAR_ANGLE_CAN_ID){ // CAR_ANGLE_CAN_ID
        memcpy(&value, frame.data, sizeof(float));
        theta = value;
        // After receiving theta, we can add a new pose
        slam.addPose(x, y, theta);
        std::cout << "Added Pose: x=" << x << ", y=" << y << ", theta=" << theta << "\n";
    }
    else{
        // Process landmark data (cones)
        if (can_id >= config.can_ids.CONE_CAN_ID_START && can_id < (config.can_ids.CONE_CAN_ID_START + 0x100)) {
            int cone_id = can_id - config.can_ids.CONE_CAN_ID_START;
            float range, bearing;
            if (frame.can_dlc == 8) {
                memcpy(&range, frame.data, sizeof(float));
                memcpy(&bearing, frame.data + 4, sizeof(float));

                // Convert bearing from degrees to radians if necessary
                double bearing_rad = bearing * M_PI / 180.0;

                // Convert range and bearing to landmark position
                double landmark_x = x + range * cos(theta + bearing_rad);
                double landmark_y = y + range * sin(theta + bearing_rad);

                // Add the landmark
                slam.addLandmark(cone_id, landmark_x, landmark_y);
                std::cout << "Added Landmark: id=" << cone_id << ", x=" << landmark_x << ", y=" << landmark_y << "\n";

                // Add the measurement
                slam.addMeasurement(slam.poses.size() - 1, cone_id, range, bearing_rad);
            } else {
                fprintf(stderr, "Invalid data length for landmark\n");
            }
        } else {
            // Unknown CAN ID
            fprintf(stderr, "Unknown CAN ID: 0x%X\n", can_id);
        }
    }
}

// Read CAN messages
void read_can_messages(int socket_fd, GraphSLAM& slam, const Config& config) {
    struct can_frame frame;
    while (true) {
        int nbytes = read(socket_fd, &frame, sizeof(struct can_frame));
        if (nbytes < 0) {
            perror("CAN read error");
            break;
        } else if (nbytes < sizeof(struct can_frame)) {
            fprintf(stderr, "Incomplete CAN frame\n");
            continue;
        }

        // Process the frame
        process_can_frame(frame, slam, config);
    }
}

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
