#include <iostream>
#include <cmath>
#include <thread>
#include <mutex>
#include <atomic>
#include <chrono>
#include <unordered_map>
#include <cstring>
#include <csignal>
#include <vector>
#include <algorithm>
#include <condition_variable>

#include <yaml-cpp/yaml.h>

// CAN headers
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <unistd.h>

// Constants (Default values, may be changed by config.yaml)
float PIXELS_PER_METER = 10.0f;
float MAX_CONE_DETECTION_RANGE = 5.0f;

// CAN IDs
int STEERING_CAN_ID = 0x300;    // Steering angle in degrees
int THROTTLE_CAN_ID = 0x301;    // Throttle in arbitrary units
int CONE_CAN_ID_START = 0x400;  // Starting CAN ID for cones

// PID Controller parameters
// Steering 
float Kp = 1.0f;
float Ki = 0.0f;
float Kd = 0.1f;
float setpoint = 0.0f; // Desired deviation distance from centerline

// Throttle
float target_speed = 15.0f; // meters per second
float throttle_Kp = 1.0f;



// PID Controller Class
class PIDController {
public:
    PIDController(float Kp, float Ki, float Kd, float setpoint)
        : Kp_(Kp), Ki_(Ki), Kd_(Kd), setpoint_(setpoint),
          integral_(0.0f), previous_error_(0.0f), previous_time_(std::chrono::steady_clock::now()) {}

    float compute(float measurement) {
        std::lock_guard<std::mutex> lock(mutex_);
        auto current_time = std::chrono::steady_clock::now();
        std::chrono::duration<float> elapsed = current_time - previous_time_;
        float delta_time = elapsed.count();

        if (delta_time <= 1e-6f) {
            delta_time = 1e-6f; // Prevent division by zero
        }

        float error = setpoint_ - measurement;
        integral_ += error * delta_time;
        float derivative = (error - previous_error_) / delta_time;

        float output = Kp_ * error + Ki_ * integral_ + Kd_ * derivative;

        // Save for next iteration
        previous_error_ = error;
        previous_time_ = current_time;

        return output;
    }

    void reset() {
        std::lock_guard<std::mutex> lock(mutex_);
        integral_ = 0.0f;
        previous_error_ = 0.0f;
        previous_time_ = std::chrono::steady_clock::now();
    }

private:
    float Kp_, Ki_, Kd_;
    float setpoint_;
    float integral_;
    float previous_error_;
    std::chrono::steady_clock::time_point previous_time_;
    std::mutex mutex_;
};

// Function to initialize CAN socket
int initialize_can_socket(const std::string& ifname) {
    struct sockaddr_can addr;
    struct ifreq ifr;
    int sock = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (sock < 0) {
        perror("Error while opening socket");
        exit(EXIT_FAILURE);
    }

    strncpy(ifr.ifr_name, ifname.c_str(), IFNAMSIZ - 1);
    ifr.ifr_name[IFNAMSIZ - 1] = '\0'; // Ensure null-termination
    if (ioctl(sock, SIOCGIFINDEX, &ifr) < 0) {
        perror("Error getting interface index");
        close(sock);
        exit(EXIT_FAILURE);
    }

    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (bind(sock, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        perror("Error in socket bind");
        close(sock);
        exit(EXIT_FAILURE);
    }

    return sock;
}

// Function to send float data over CAN
void send_can_float(int sock, int can_id, float value) {
    struct can_frame frame;
    frame.can_id = can_id;
    frame.can_dlc = sizeof(float);
    std::memset(frame.data, 0, sizeof(frame.data)); // Clear data
    std::memcpy(frame.data, &value, sizeof(float));  // Little-endian

    int bytes_sent = write(sock, &frame, sizeof(struct can_frame));
    if (bytes_sent != sizeof(struct can_frame)) {
        perror("Error sending CAN frame");
    }
}

// Function to receive float data from CAN
bool receive_can_float(int sock, struct can_frame& frame) {
    int nbytes = read(sock, &frame, sizeof(struct can_frame));
    if (nbytes < 0) {
        // No data received or error
        return false;
    } else if (nbytes < sizeof(struct can_frame)) {
        std::cerr << "[ERROR] Incomplete CAN frame received." << std::endl;
        return false;
    }
    return true;
}

// Global flag for graceful termination
std::atomic<bool> running(true);

// Shared data structures and synchronization primitives
std::unordered_map<int, std::pair<float, float>> cones_data; // Cone index -> (range, bearing)
std::mutex data_mutex;
std::condition_variable data_cond;

std::atomic<float> planned_deviation(0.0f); // Shared variable for planned deviation
std::mutex control_mutex; // Mutex for control data
std::condition_variable control_cond;

// Signal handler for graceful shutdown
void signal_handler(int signum) {
    std::cout << "\n[INFO] Interrupt signal (" << signum << ") received. Terminating..." << std::endl;
    running = false;
    // Notify all condition variables to unblock threads
    data_cond.notify_all();
    control_cond.notify_all();
}

// Function to load configuration from config.yaml
void loadConfig(const std::string& filename) {
    try {
        YAML::Node config = YAML::LoadFile(filename);

        // Load PIXELS_PER_METER
        if (config["PIXELS_PER_METER"]) {
            PIXELS_PER_METER = config["PIXELS_PER_METER"].as<float>();
        }

        // Load CAN IDs
        if (config["CAN_IDS"]) {
            if (config["CAN_IDS"]["STEERING_CAN_ID"]) {
                STEERING_CAN_ID = config["CAN_IDS"]["STEERING_CAN_ID"].as<int>();
            }
            if (config["CAN_IDS"]["THROTTLE_CAN_ID"]) {
                THROTTLE_CAN_ID = config["CAN_IDS"]["THROTTLE_CAN_ID"].as<int>();
            }
            if (config["CAN_IDS"]["CONE_CAN_ID_START"]) {
                CONE_CAN_ID_START = config["CAN_IDS"]["CONE_CAN_ID_START"].as<int>();
            }
        }

        // Load PID Controller parameters
        if (config["pid_controller"]) {
            Kp = config["pid_controller"]["Kp"].as<float>();
            Ki = config["pid_controller"]["Ki"].as<float>();
            Kd = config["pid_controller"]["Kd"].as<float>();
            setpoint = config["pid_controller"]["setpoint"].as<float>();
        }

        // Load Control parameters
        if (config["control"]) {
            target_speed = config["control"]["target_speed"].as<float>();
            throttle_Kp = config["control"]["throttle_Kp"].as<float>();
        }

        // Load perception parameters
        if (config["perception"]) {
            if (config["perception"]["detection_range"]) {
                MAX_CONE_DETECTION_RANGE = config["perception"]["detection_range"].as<float>();
            }
        }

        // Optionally print out loaded values for debugging
        std::cout << "Configuration loaded from " << filename << std::endl;
        std::cout << "PIXELS_PER_METER: " << PIXELS_PER_METER << std::endl;
        std::cout << "CAN IDs:" << std::endl;
        std::cout << "  STEERING_CAN_ID: 0x" << std::hex << STEERING_CAN_ID << std::dec << std::endl;
        std::cout << "  THROTTLE_CAN_ID: 0x" << std::hex << THROTTLE_CAN_ID << std::dec << std::endl;
        std::cout << "  CONE_CAN_ID_START: 0x" << std::hex << CONE_CAN_ID_START << std::dec << std::endl;
        std::cout << "PID Controller parameters:" << std::endl;
        std::cout << "  Kp: " << Kp << ", Ki: " << Ki << ", Kd: " << Kd << ", Setpoint: " << setpoint << std::endl;
        std::cout << "Control parameters:" << std::endl;
        std::cout << "  Target speed: " << target_speed << " m/s" << std::endl;
        std::cout << "  Throttle Kp: " << throttle_Kp << std::endl;
        std::cout << "Perception parameters:" << std::endl;
        std::cout << "  Max cone detection range: " << MAX_CONE_DETECTION_RANGE << " m" << std::endl;
    } catch (const std::exception& e) {
        std::cerr << "[ERROR] Failed to load " << filename << ": " << e.what() << std::endl;
        std::cerr << "Using default configuration values." << std::endl;
    }
}

int main() {
    // Register signal handler for CTRL+C
    signal(SIGINT, signal_handler);

    // Load configuration
    loadConfig("config.yaml");

    // Load cones.yaml to map cone indices to colors
    std::unordered_map<int, std::string> cone_colors;
    try {
        YAML::Node config = YAML::LoadFile("cones.yaml");
        const YAML::Node& cones = config["cones"];
        for (size_t i = 0; i < cones.size(); ++i) {
            if (cones[i]["color"]) {
                cone_colors[i] = cones[i]["color"].as<std::string>();
            } else {
                cone_colors[i] = "unknown";
                std::cerr << "[WARN] Cone " << i << " does not have a 'color' field. Defaulting to 'unknown'." << std::endl;
            }
        }
    } catch (const std::exception& e) {
        std::cerr << "[ERROR] Failed to load cones.yaml: " << e.what() << std::endl;
        return EXIT_FAILURE;
    }

    // Initialize CAN socket (single socket for both sending and receiving)
    int can_sock = initialize_can_socket("vcan0");

    // For simplicity, using a single socket
    int recv_sock = can_sock;
    int send_sock = can_sock;

    // Initialize PID controller with loaded gains
    PIDController pid(Kp, Ki, Kd, setpoint);

    // Thread to receive CAN messages
    std::thread can_receiver([&]() {
        struct can_frame frame;
        while (running) {
            if (receive_can_float(recv_sock, frame)) {
                if (frame.can_id >= CONE_CAN_ID_START) {
                    int cone_index = frame.can_id - CONE_CAN_ID_START;
                    if (frame.can_dlc >= 8) { // 4 bytes for range, 4 bytes for bearing
                        float range, bearing;
                        std::memcpy(&range, frame.data, sizeof(float));
                        std::memcpy(&bearing, frame.data + 4, sizeof(float));

                        // Debugging: Print raw data
                        std::cout << "[DEBUG] Received Cone " << cone_index
                                  << " | Range: " << range << " m | Bearing: " << bearing << " degrees" << std::endl;

                        // Validate range and bearing
                        if (range < 0.0f || range > MAX_CONE_DETECTION_RANGE) {
                            std::cerr << "[WARN] Invalid range for Cone " << cone_index
                                      << ": " << range << " m" << std::endl;
                            continue;
                        }
                        if (bearing < -180.0f || bearing > 180.0f) {
                            std::cerr << "[WARN] Invalid bearing for Cone " << cone_index
                                      << ": " << bearing << " degrees" << std::endl;
                            continue;
                        }

                        // Store valid data
                        {
                            std::lock_guard<std::mutex> lock(data_mutex);
                            cones_data[cone_index] = std::make_pair(range, bearing);
                        }
                        data_cond.notify_one();
                    } else {
                        std::cerr << "[WARN] Received CAN frame for Cone " << (frame.can_id - CONE_CAN_ID_START)
                                  << " with insufficient data length: " << frame.can_dlc << std::endl;
                    }
                }
            } else {
                // No data received; sleep briefly to prevent busy waiting
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
        }
        std::cout << "[INFO] CAN Receiver thread terminated." << std::endl;
    });

    // Thread for trajectory planning
    std::thread trajectory_planner([&]() {
        while (running) {
            std::unique_lock<std::mutex> lock(data_mutex);
            data_cond.wait(lock, [] { return !cones_data.empty() || !running; });

            if (!running) break; // Exit if program is stopping

            // Make a local copy of cones_data to process
            auto local_cones = cones_data;
            cones_data.clear(); // Clear after copying
            lock.unlock(); // Unlock early

            // Separate cones into left (blue) and right (yellow) based on colors
            std::vector<std::pair<float, float>> left_cones;
            std::vector<std::pair<float, float>> right_cones;

            for (const auto& [idx, data] : local_cones) {
                std::string color = cone_colors[idx];
                if (color == "blue") {
                    left_cones.push_back(data);
                } else if (color == "yellow") {
                    right_cones.push_back(data);
                } else {
                    std::cerr << "[WARN] Unknown color for Cone " << idx << ": " << color << std::endl;
                }
            }

            if (!left_cones.empty() || !right_cones.empty()) { // Allow processing even if one side is empty
                // Calculate weighted centerline deviation
                float total_weight = 0.0f;
                float weighted_centerline_bearing = 0.0f;

                for (const auto& [range, bearing] : left_cones) {
                    float weight = 1.0f / (range + 0.1f); // Weight inversely proportional to range
                    weighted_centerline_bearing += weight * (bearing + 90.0f); // Adjust bearing
                    total_weight += weight;
                }

                for (const auto& [range, bearing] : right_cones) {
                    float weight = 1.0f / (range + 0.1f);
                    weighted_centerline_bearing += weight * (bearing - 90.0f);
                    total_weight += weight;
                }

                if (total_weight > 0.0f) {
                    // Normalize weighted average
                    weighted_centerline_bearing /= total_weight;

                    // Debugging: Print centerline deviation
                    std::cout << "[DEBUG] Weighted Centerline Bearing: " << weighted_centerline_bearing << " degrees" << std::endl;

                    // Store deviation for control thread
                    {
                        std::lock_guard<std::mutex> control_lock(control_mutex);
                        planned_deviation = weighted_centerline_bearing;
                    }
                    control_cond.notify_one();
                } else {
                    std::cerr << "[WARN] Total weight is zero. Cannot compute weighted centerline bearing." << std::endl;
                }
            }
        }
        std::cout << "[INFO] Trajectory Planner thread terminated." << std::endl;
    });

    // Thread for control
    std::thread controller([&]() {
        while (running) {
            std::unique_lock<std::mutex> lock(control_mutex);
            control_cond.wait(lock, [&] { return planned_deviation != 0.0f || !running; });

            if (!running) break; // Exit if program is stopping

            // Retrieve planned deviation
            float deviation = planned_deviation.exchange(0.0f); // Reset after reading

            if (deviation == 0.0f) {
                // No new deviation to process
                continue;
            }

            // Compute steering command using PID
            float steering = pid.compute(deviation);

            // Clamp steering to +/-60 degrees
            steering = std::max(std::min(steering, 60.0f), -60.0f);

            // Compute throttle command (simple proportional control to maintain speed)
            float current_speed = 0.0f; // Placeholder; replace with actual speed sensor data

            float throttle = throttle_Kp * (target_speed - current_speed);

            // Clamp throttle to [0, 100]
            throttle = std::max(std::min(throttle, 100.0f), 0.0f);

            // Send steering and throttle commands over CAN
            send_can_float(send_sock, STEERING_CAN_ID, steering);
            send_can_float(send_sock, THROTTLE_CAN_ID, throttle);

            // Print debug information
            std::cout << "Deviation: " << deviation << " | Steering: " << steering
                      << " deg | Throttle: " << throttle << " units" << std::endl;

            // Control loop rate
            std::this_thread::sleep_for(std::chrono::milliseconds(100)); // Control rate
        }
        std::cout << "[INFO] Controller thread terminated." << std::endl;
    });

    // Join threads and clean up
    can_receiver.join();
    trajectory_planner.join();
    controller.join();

    close(can_sock); // Close the single CAN socket

    std::cout << "[INFO] PID Controller terminated gracefully." << std::endl;

    return 0;
}
