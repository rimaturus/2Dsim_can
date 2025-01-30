#include "visualization.h"

#include <allegro5/allegro_primitives.h>
#include <allegro5/allegro_ttf.h>
#include <fcntl.h>
#include <errno.h>
#include <thread>
#include <chrono>
#include <cstring>
#include <iostream>
#include <cmath>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <unistd.h>

// Constants (Default values, may be changed by config.yaml)
float PIXELS_PER_METER = 10.0f;
double DETECTION_RANGE = 5.0;

// CAN IDs
int STEERING_CAN_ID = 0x300; // [degrees]
int THROTTLE_CAN_ID = 0x301; // [units: 0-100]
int CAR_X_CAN_ID = 0x200;    // [meters]
int CAR_Y_CAN_ID = 0x201;    // [meters]
int CAR_ANGLE_CAN_ID = 0x202;// [radians]

// Load configuration from YAML file
void loadConfig(const std::string& filename, Car& car) {
    try {
        YAML::Node config = YAML::LoadFile(filename);

        if (config["PIXELS_PER_METER"]) {
            PIXELS_PER_METER = config["PIXELS_PER_METER"].as<float>();
        }

        if (config["perception"]) {
            DETECTION_RANGE = config["perception"]["detection_range"].as<double>();
        }

        if (config["car"]) {
            car.params.wheelbase = config["car"]["wheelbase"].as<float>();
            car.params.max_throttle = config["car"]["max_throttle"].as<float>();
            car.params.max_speed = config["car"]["max_speed"].as<float>();
        }

        if (config["CAN_IDS"]) {
            STEERING_CAN_ID = config["CAN_IDS"]["STEERING_CAN_ID"].as<int>();
            THROTTLE_CAN_ID = config["CAN_IDS"]["THROTTLE_CAN_ID"].as<int>();
            CAR_X_CAN_ID = config["CAN_IDS"]["CAR_X_CAN_ID"].as<int>();
            CAR_Y_CAN_ID = config["CAN_IDS"]["CAR_Y_CAN_ID"].as<int>();
            CAR_ANGLE_CAN_ID = config["CAN_IDS"]["CAR_ANGLE_CAN_ID"].as<int>();
        }

    } catch (const std::exception& e) {
        std::cerr << "Error loading configuration from " << filename << ": " << e.what() << std::endl;
        std::cerr << "Using default configuration values." << std::endl;
    }
}

// Load cones from YAML file
std::vector<Cone> loadCones(const std::string& filename) {
    std::vector<Cone> cones;
    try {
        YAML::Node conesNode = YAML::LoadFile(filename)["cones"];
        if (!conesNode) {
            std::cerr << "Error: 'cones' key not found in " << filename << std::endl;
            return cones;
        }
        for (const auto& node : conesNode) {
            Cone cone;
            cone.x_pixels = node["x"].as<float>(); // Positions in pixels
            cone.y_pixels = node["y"].as<float>();
            cone.color = node["color"].as<std::string>();
            cones.push_back(cone);
        }
        std::cout << "Loaded " << cones.size() << " cones from " << filename << std::endl;
    } catch (const std::exception& e) {
        std::cerr << "Error loading cones from " << filename << ": " << e.what() << std::endl;
    }
    return cones;
}

// Set colors of cones
ALLEGRO_COLOR getColor(const std::string& colorName) {
    if (colorName == "yellow") return al_map_rgb(255, 255, 0);
    if (colorName == "blue") return al_map_rgb(0, 0, 255);

    return al_map_rgb(255, 255, 255); // Default color
}

// Draw cones
void drawCones(const std::vector<Cone>& cones) {
    for (const auto& cone : cones) {
        ALLEGRO_COLOR color = getColor(cone.color);
        al_draw_filled_circle(cone.x_pixels, cone.y_pixels, 5, color);
    }
}

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
        // Sleep to control the sending rate (100 Hz)
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}

float calculateDistance(float x1_pixels, float y1_pixels, float x2_pixels, float y2_pixels) {
    float dx = x2_pixels - x1_pixels;
    float dy = y2_pixels - y1_pixels;
    return sqrt(dx * dx + dy * dy);
}

// Initialize Allegro and related components
bool initializeAllegro(ALLEGRO_DISPLAY*& display, ALLEGRO_FONT*& font, ALLEGRO_EVENT_QUEUE*& event_queue, ALLEGRO_TIMER*& timer) {
    if (!al_init()) {
        fprintf(stderr, "Failed to initialize Allegro.\n");
        return false;
    }

    if (!al_init_primitives_addon()) {
        fprintf(stderr, "Failed to initialize primitives addon.\n");
        return false;
    } else {
        std::cout << "Allegro primitives addon initialized successfully." << std::endl;
    }

    if (!al_init_font_addon()) {
        fprintf(stderr, "Failed to initialize font addon.\n");
        return false;
    }

    if (!al_init_ttf_addon()) {
        fprintf(stderr, "Failed to initialize TTF addon.\n");
        return false;
    }

    // Create display
    display = al_create_display(800, 600);
    if (!display) {
        fprintf(stderr, "Failed to create display.\n");
        return false;
    }

    // Load font
    font = al_create_builtin_font();
    if (!font) {
        fprintf(stderr, "Failed to create built-in font.\n");
        al_destroy_display(display);
        return false;
    }

    // Event queue and timer
    event_queue = al_create_event_queue();
    timer = al_create_timer(1.0 / 60); // 60 FPS

    al_register_event_source(event_queue, al_get_timer_event_source(timer));
    al_register_event_source(event_queue, al_get_display_event_source(display));

    return true;
}

// Cleanup Allegro components
void cleanupAllegro(ALLEGRO_DISPLAY* display, ALLEGRO_FONT* font, ALLEGRO_TIMER* timer, ALLEGRO_EVENT_QUEUE* event_queue) {
    if (font) {
        al_destroy_font(font);
    }
    al_shutdown_ttf_addon();
    al_shutdown_font_addon();

    if (timer) {
        al_destroy_timer(timer);
    }
    if (event_queue) {
        al_destroy_event_queue(event_queue);
    }
    if (display) {
        al_destroy_display(display);
    }
}

// Setup CAN socket for receiving
int setupCANSocket(const char* interface_name) {
    int can_socket;
    struct sockaddr_can addr;
    struct ifreq ifr;

    if ((can_socket = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
        perror("Error while opening receive socket");
        return -1;
    }

    strcpy(ifr.ifr_name, interface_name);
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

    return can_socket;
}

// Setup CAN socket for sending
int setupSendCANSocket(const char* interface_name) {
    int send_can_socket;
    struct sockaddr_can send_addr;
    struct ifreq send_ifr;

    if ((send_can_socket = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
        perror("Error while opening send socket");
        return -1;
    }

    strcpy(send_ifr.ifr_name, interface_name);
    if (ioctl(send_can_socket, SIOCGIFINDEX, &send_ifr) < 0) {
        perror("Error getting interface index for send socket");
        close(send_can_socket);
        return -1;
    }

    send_addr.can_family = AF_CAN;
    send_addr.can_ifindex = send_ifr.ifr_ifindex;

    if (bind(send_can_socket, (struct sockaddr *)&send_addr, sizeof(send_addr)) < 0) {
        perror("Error in send socket bind");
        close(send_can_socket);
        return -2;
    }

    // Set send CAN socket to non-blocking (optional)
    int send_flags = fcntl(send_can_socket, F_GETFL, 0);
    if (fcntl(send_can_socket, F_SETFL, send_flags | O_NONBLOCK) < 0) {
        perror("Error setting non-blocking mode for send socket");
        close(send_can_socket);
        return -3;
    }

    return send_can_socket;
}

// Initialize car
void initializeCar(Car& car) {
    car.x = 40.0f; // meters
    car.y = 30.0f; // meters
    car.angle = 0.0f;
    car.steeringAngle = 0.0f;
    car.throttle = 0.0f;
    car.speed = 0.0f;
    // car.params will be loaded from config.yaml
}

// Update car's position
void updateCarPosition(Car& car, float deltaTime, std::mutex& carMutex) {
    // Call the desired vehicle model function
    updateCarPositionSingleTrackModel(car, deltaTime, carMutex);
}

// Update car's position using Single Track Model
void updateCarPositionSingleTrackModel(Car& car, float deltaTime, std::mutex& carMutex) {
    // Car parameters
    const float L = car.params.wheelbase;       // Wheelbase in meters
    const float MAX_THROTTLE = car.params.max_throttle; // Max throttle units
    const float MAX_SPEED = car.params.max_speed;      // Max speed in m/s

    std::lock_guard<std::mutex> lock(carMutex);

    // Convert steering angle from degrees to radians
    float delta = car.steeringAngle * M_PI / 180.0f;

    // Compute speed based on throttle
    float v = (car.throttle / MAX_THROTTLE) * MAX_SPEED;

    // Compute slip angle beta
    float beta = atan2(tan(delta), 2.0f); // Assuming front-wheel steering

    // Update position
    car.x += v * cos(car.angle + beta) * deltaTime;
    car.y += v * sin(car.angle + beta) * deltaTime;

    // Update orientation
    car.angle += (v / L) * sin(delta) * deltaTime;

    // Keep angle within -PI to PI
    if (car.angle > M_PI)
        car.angle -= 2 * M_PI;
    else if (car.angle < -M_PI)
        car.angle += 2 * M_PI;

    // Update car speed
    car.speed = v;
}

// Render the scene
void renderScene(const std::vector<Cone>& cones, Car& car, ALLEGRO_FONT* font, std::mutex& carMutex) {
    float car_x_pixels, car_y_pixels, car_angle;

    {
        std::lock_guard<std::mutex> lock(carMutex);
        car_x_pixels = car.x * PIXELS_PER_METER;
        car_y_pixels = car.y * PIXELS_PER_METER;
        car_angle = car.angle;
    }

    // Apply camera transformation to center the view on the car
    ALLEGRO_TRANSFORM camera_transform;
    al_identity_transform(&camera_transform);
    al_translate_transform(&camera_transform, -car_x_pixels + 400, -car_y_pixels + 300); // Adjust to center car in window
    al_use_transform(&camera_transform);

    // Clear display
    al_clear_to_color(al_map_rgb(0, 0, 0));

    // Draw cones
    drawCones(cones);

    // Draw lines to nearby cones
    {
        std::lock_guard<std::mutex> lock(carMutex);
        for (const auto& cone : cones) {
            // Calculate distance in pixels
            float dx = cone.x_pixels - car_x_pixels;
            float dy = cone.y_pixels - car_y_pixels;
            float distance_pixels = sqrt(dx * dx + dy * dy);

            // Convert 5 meters to pixels for comparison
            float detection_radius_pixels = DETECTION_RANGE * PIXELS_PER_METER;

            if (distance_pixels <= detection_radius_pixels) {
                al_draw_line(car_x_pixels, car_y_pixels, cone.x_pixels, cone.y_pixels, al_map_rgb(0, 255, 0), 1);
            }
        }
    }

    // Draw car
    ALLEGRO_TRANSFORM carTransform;
    al_identity_transform(&carTransform);
    al_rotate_transform(&carTransform, car_angle);
    al_translate_transform(&carTransform, car_x_pixels, car_y_pixels);
    al_compose_transform(&carTransform, &camera_transform);
    al_use_transform(&carTransform);

    // Draw car shape
    al_draw_filled_rectangle(-10, -5, 10, 5, al_map_rgb(255, 255, 0));

    // Restore transformation to camera
    al_use_transform(&camera_transform);

    // Reset to identity for UI
    ALLEGRO_TRANSFORM identity;
    al_identity_transform(&identity);
    al_use_transform(&identity);

    // Prepare the coordinate string (in meters)
    char coordText[100];
    {
        std::lock_guard<std::mutex> lock(carMutex);
        sprintf(coordText, "X: %.2f m, Y: %.2f m", car.x, car.y);
    }

    // Draw the text at position (10, 10)
    al_draw_text(font, al_map_rgb(255, 255, 255), 10, 10, ALLEGRO_ALIGN_LEFT, coordText);

    // Flip the display
    al_flip_display();
}

int main() {
    // Initialize Allegro
    ALLEGRO_DISPLAY* display = nullptr;
    ALLEGRO_FONT* font = nullptr;
    ALLEGRO_EVENT_QUEUE* event_queue = nullptr;
    ALLEGRO_TIMER* timer = nullptr;

    if (!initializeAllegro(display, font, event_queue, timer)) {
        return -1;
    }

    // Initialize car
    Car car;
    initializeCar(car);

    // Load configuration
    loadConfig("config.yaml", car);

    // Setup CAN sockets
    int can_socket = setupCANSocket("vcan0");
    if (can_socket < 0) {
        cleanupAllegro(display, font, timer, event_queue);
        return -1;
    }

    int send_can_socket = setupSendCANSocket("vcan0");
    if (send_can_socket < 0) {
        close(can_socket);
        cleanupAllegro(display, font, timer, event_queue);
        return -1;
    }

    // Load cones from YAML file
    std::vector<Cone> cones = loadCones("/media/edo/Volume/UNIPI/1anno/ISTR/2Dsim_can/track_/cones.yaml");

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
            updateCarPosition(car, deltaTime, carMutex);

            // Render the scene
            renderScene(cones, car, font, carMutex);
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

    // Cleanup Allegro
    cleanupAllegro(display, font, timer, event_queue);

    return 0;
}
