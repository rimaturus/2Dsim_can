// visualization.cpp

#include <allegro5/allegro.h>
#include <allegro5/allegro_primitives.h>
#include <allegro5/allegro_font.h>
#include <allegro5/allegro_ttf.h>
#include <yaml-cpp/yaml.h>
#include <vector>
#include <string>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <fcntl.h>
#include <errno.h>
#include <thread>
#include <mutex>
#include <atomic>
#include <chrono>
#include <iostream> // Added for std::cerr and std::cout

// CAN headers
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <unistd.h>

const float PIXELS_PER_METER = 10.0f; // Pixels per meter

// CAN IDs
const int STEERING_CAN_ID = 0x300; // Steering angle in degrees
const int THROTTLE_CAN_ID = 0x301; // Throttle in units
const int CAR_X_CAN_ID = 0x200;    // X_car
const int CAR_Y_CAN_ID = 0x201;    // Y_car
const int CAR_ANGLE_CAN_ID = 0x202;// Yaw_car


struct Cone {
    float x;
    float y;
    std::string color;
};

struct Car {
    float x;              // m
    float y;              // m
    float angle;          // radians
    float steeringAngle;  // degrees
    float throttle;       // units
};

// Load cones from YAML file
std::vector<Cone> loadCones(const std::string& filename) {
    YAML::Node conesNode = YAML::LoadFile(filename)["cones"];
    std::vector<Cone> cones;

    for (const auto& node : conesNode) {
        Cone cone;
        cone.x = node["x"].as<float>();
        cone.y = node["y"].as<float>();
        cone.color = node["color"].as<std::string>();
        cones.push_back(cone);
    }
    return cones;
}

// Color mapping function
ALLEGRO_COLOR getColor(const std::string& colorName) {
    if (colorName == "yellow") return al_map_rgb(255, 255, 0);
    if (colorName == "blue") return al_map_rgb(0, 0, 255);

    return al_map_rgb(255, 255, 255); // Default color
}

// Draw cones
void drawCones(const std::vector<Cone>& cones) {
    for (const auto& cone : cones) {
        ALLEGRO_COLOR color = getColor(cone.color);
        al_draw_filled_circle(cone.x, cone.y, 5, color);
    }
}

// Update camera position (Removed for fixed camera)
/*
void updateCamera(const Car& car, float displayWidth, float displayHeight) {
    ALLEGRO_TRANSFORM transform;
    al_identity_transform(&transform);

    // Center the car on the screen
    al_translate_transform(&transform, displayWidth / 2 - car.x, displayHeight / 2 - car.y);

    al_use_transform(&transform);
}
*/

// CAN-sender (float)
void sendFloatCAN(int send_can_socket, int can_id, float value) {
    struct can_frame frame;
    frame.can_id = can_id;
    frame.can_dlc = sizeof(float);
    memcpy(frame.data, &value, sizeof(float));

    int nbytes = write(send_can_socket, &frame, sizeof(struct can_frame));
    if (nbytes != sizeof(struct can_frame)) {
        perror("CAN write error");
    }
}

// CAN-receiver (float)
void receiveCANMessagesThread(int can_socket, std::atomic<bool>& done, Car& car, std::mutex& carMutex) {
    while (!done) {
        struct can_frame frame;
        int nbytes = read(can_socket, &frame, sizeof(struct can_frame));

        if (nbytes < 0) {
            if (errno != EAGAIN && errno != EWOULDBLOCK) {
                perror("CAN read error");
            }
            // Sleep briefly to avoid busy waiting
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            continue;
        }

        if (nbytes < sizeof(struct can_frame)) {
            std::cerr << "Incomplete CAN frame received." << std::endl;
            continue;
        }

        // Process the received CAN frame
        if (frame.can_id == STEERING_CAN_ID) {
            if (frame.can_dlc == sizeof(float)) {
                float steeringDeg;
                memcpy(&steeringDeg, frame.data, sizeof(float));
                {
                    std::lock_guard<std::mutex> lock(carMutex);
                    car.steeringAngle = steeringDeg;
                }
                // DEBUG
                // printf("Received Steering Angle: %.2f degrees\n", steeringDeg);
            }
        }
        else if (frame.can_id == THROTTLE_CAN_ID) {
            if (frame.can_dlc == sizeof(float)) {
                float throttle;
                memcpy(&throttle, frame.data, sizeof(float));
                {
                    std::lock_guard<std::mutex> lock(carMutex);
                    car.throttle = throttle;
                }
                // DEBUG
                // printf("Received Throttle: %.2f units\n", throttle);
            }
        }
    }
}

// Function to send car data over CAN (thread function)
void sendCarDataThread(int send_can_socket, Car& car, std::mutex& carMutex, std::atomic<bool>& done) {
    while (!done) {
        {
            std::lock_guard<std::mutex> lock(carMutex);

            sendFloatCAN(send_can_socket, CAR_X_CAN_ID, car.x);
            sendFloatCAN(send_can_socket, CAR_Y_CAN_ID, car.y);
            sendFloatCAN(send_can_socket, CAR_ANGLE_CAN_ID, car.angle);
        }
        // Sleep to control the sending rate (60 Hz)
        std::this_thread::sleep_for(std::chrono::milliseconds(16));
    }
}

float calculateDistance(float x1, float y1, float x2, float y2) {
    float dx = x2 - x1;
    float dy = y2 - y1;
    return sqrt(dx * dx + dy * dy);
}

int main() {
    // Initialize Allegro
    if (!al_init()) {
        fprintf(stderr, "Failed to initialize Allegro.\n");
        return -1;
    }

    if (!al_init_primitives_addon()) {
        fprintf(stderr, "Failed to initialize primitives addon.\n");
        return -1;
    }

    if (!al_init_font_addon()) {
        fprintf(stderr, "Failed to initialize font addon.\n");
        return -1;
    }

    if (!al_init_ttf_addon()) {
        fprintf(stderr, "Failed to initialize TTF addon.\n");
        return -1;
    }

    // Create display
    ALLEGRO_DISPLAY* display = al_create_display(800, 600);
    if (!display) {
        fprintf(stderr, "Failed to create display.\n");
        return -1;
    }

    // Load font
    ALLEGRO_FONT* font = al_create_builtin_font();
    if (!font) {
        fprintf(stderr, "Failed to create built-in font.\n");
        return -1;
    }

    // Event queue and timer
    ALLEGRO_EVENT_QUEUE* event_queue = al_create_event_queue();
    ALLEGRO_TIMER* timer = al_create_timer(1.0 / 60); // 60 FPS

    al_register_event_source(event_queue, al_get_timer_event_source(timer));
    al_register_event_source(event_queue, al_get_display_event_source(display));

    // CAN socket setup for receiving (steering and throttle)
    int can_socket;
    struct sockaddr_can addr;
    struct ifreq ifr;

    if ((can_socket = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
        perror("Error while opening receive socket");
        return -1;
    }

    strcpy(ifr.ifr_name, "vcan0");
    if (ioctl(can_socket, SIOCGIFINDEX, &ifr) < 0) {
        perror("Error getting interface index");
        close(can_socket);
        return -1;
    }

    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (bind(can_socket, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        perror("Error in receive socket bind");
        close(can_socket);
        return -2;
    }

    // Set CAN socket to non-blocking
    int flags = fcntl(can_socket, F_GETFL, 0);
    if (fcntl(can_socket, F_SETFL, flags | O_NONBLOCK) < 0) {
        perror("Error setting non-blocking mode");
        close(can_socket);
        return -3;
    }

    // Sending CAN socket setup
    int send_can_socket;
    struct sockaddr_can send_addr;
    struct ifreq send_ifr;

    if ((send_can_socket = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
        perror("Error while opening send socket");
        close(can_socket);
        return -1;
    }

    strcpy(send_ifr.ifr_name, "vcan0");
    if (ioctl(send_can_socket, SIOCGIFINDEX, &send_ifr) < 0) {
        perror("Error getting interface index for send socket");
        close(can_socket);
        close(send_can_socket);
        return -1;
    }

    send_addr.can_family = AF_CAN;
    send_addr.can_ifindex = send_ifr.ifr_ifindex;

    if (bind(send_can_socket, (struct sockaddr *)&send_addr, sizeof(send_addr)) < 0) {
        perror("Error in send socket bind");
        close(can_socket);
        close(send_can_socket);
        return -2;
    }

    // Set send CAN socket to non-blocking (optional)
    int send_flags = fcntl(send_can_socket, F_GETFL, 0);
    if (fcntl(send_can_socket, F_SETFL, send_flags | O_NONBLOCK) < 0) {
        perror("Error setting non-blocking mode for send socket");
        close(can_socket);
        close(send_can_socket);
        return -3;
    }

    // Load cones from YAML file
    std::vector<Cone> cones = loadCones("cones.yaml");

    // Initialize car
    Car car = {400.0f, 300.0f, 0.0f, 0.0f, 0.0f}; // Starting position (pixels), angle (radians), steeringAngle (degrees), throttle

    // Mutex for protecting car data
    std::mutex carMutex;

    // Atomic flag for thread control
    std::atomic<bool> done(false);

    // Start the receive CAN messages thread
    std::thread canReceiveThread(receiveCANMessagesThread, can_socket, std::ref(done), std::ref(car), std::ref(carMutex));

    // Start the send car data thread
    std::thread canSendThread(sendCarDataThread, send_can_socket, std::ref(car), std::ref(carMutex), std::ref(done));

    // Start the timer
    al_start_timer(timer);

    bool should_exit = false;

    while (!should_exit) {
        ALLEGRO_EVENT ev;
        al_wait_for_event(event_queue, &ev);

        if (ev.type == ALLEGRO_EVENT_TIMER) {
            float deltaTime = 1.0f / 60.0f; // 60 FPS

            // Update car's position based on steering angle and throttle
            {
                std::lock_guard<std::mutex> lock(carMutex);
                // Convert steering angle from degrees to radians for calculation
                float steeringRad = car.steeringAngle * M_PI / 180.0f;

                // Simple car kinematics
                float wheelBase = 50.0f; // in pixels (1 meter)
                float angularVelocity = (car.throttle * tan(steeringRad)) / wheelBase; // radians per second

                // Update orientation
                car.angle += angularVelocity * deltaTime;

                // Keep angle within -PI to PI
                if (car.angle > ALLEGRO_PI)
                    car.angle -= 2 * ALLEGRO_PI;
                else if (car.angle < -ALLEGRO_PI)
                    car.angle += 2 * ALLEGRO_PI;

                // Update position
                car.x += car.throttle * cos(car.angle) * deltaTime;
                car.y += car.throttle * sin(car.angle) * deltaTime;
            }

            // Render
            al_clear_to_color(al_map_rgb(0, 0, 0));
            drawCones(cones);

            // Draw a red dot at the car's position (measurement point)
            {
                std::lock_guard<std::mutex> lock(carMutex);
                al_draw_filled_circle(car.x, car.y, 5, al_map_rgb(255, 0, 0)); // Red dot
            }

            // Draw lines from the car to each cone within 5 meters
            {
                std::lock_guard<std::mutex> lock(carMutex);
                for (const auto& cone : cones) {
                    // Calculate distance in meters
                    float dx = (cone.x - car.x) / PIXELS_PER_METER;
                    float dy = (cone.y - car.y) / PIXELS_PER_METER;
                    float distance = sqrt(dx * dx + dy * dy);

                    if (distance <= 5.0f) { // Within 5 meters
                        al_draw_line(car.x, car.y, cone.x, cone.y, al_map_rgb(0, 255, 0), 1); // Green line
                    }
                }
            }

            // Save the current transformation
            ALLEGRO_TRANSFORM prevTransform;
            al_copy_transform(&prevTransform, al_get_current_transform());

            // Build the car transformation
            ALLEGRO_TRANSFORM carTransform;
            {
                std::lock_guard<std::mutex> lock(carMutex);
                al_identity_transform(&carTransform);
                al_rotate_transform(&carTransform, car.angle);
                al_translate_transform(&carTransform, car.x, car.y);
            }
            al_use_transform(&carTransform);

            // Draw the car centered at the origin in local coordinates
            al_draw_filled_rectangle(-10, -5, 10, 5, al_map_rgb(255, 255, 0));

            // Restore the previous transformation
            al_use_transform(&prevTransform);

            // Reset to identity transform for UI elements
            ALLEGRO_TRANSFORM identity;
            al_identity_transform(&identity);
            al_use_transform(&identity);

            // Prepare the coordinate string (in meters)
            char coordText[100];
            {
                std::lock_guard<std::mutex> lock(carMutex);
                float car_x_m = car.x / PIXELS_PER_METER;
                float car_y_m = car.y / PIXELS_PER_METER;
                sprintf(coordText, "X: %.2f m, Y: %.2f m", car_x_m, car_y_m);
            }

            // Draw the text at position (10, 10)
            al_draw_text(font, al_map_rgb(255, 255, 255), 10, 10, ALLEGRO_ALIGN_LEFT, coordText);

            // Flip the display
            al_flip_display();
        }
        else if (ev.type == ALLEGRO_EVENT_DISPLAY_CLOSE) {
            should_exit = true;
            done = true;
        }
    }

    // Wait for threads to finish
    canReceiveThread.join();
    canSendThread.join();

    // Close CAN sockets
    close(can_socket);
    close(send_can_socket);

    // Destroy the font
    al_destroy_font(font);

    // Shutdown Allegro font add-ons
    al_shutdown_ttf_addon();
    al_shutdown_font_addon();

    // Shutdown Allegro
    al_destroy_timer(timer);
    al_destroy_event_queue(event_queue);
    al_destroy_display(display);

    return 0;
}
