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

// CAN headers
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <unistd.h>

// Structures
struct Cone {
    float x;
    float y;
    std::string color;
};

struct Car {
    float x;
    float y;
    float angle;         // Orientation of the car in radians
    float steeringAngle; // Current steering angle in radians
    float speed;
    float wheelBase;     // Distance between front and rear axles
};

// Global variables
bool keys[4] = {false, false, false, false}; // A, D, W, S

// Function to load cones from YAML file
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
    if (colorName == "red") return al_map_rgb(255, 0, 0);
    if (colorName == "blue") return al_map_rgb(0, 0, 255);
    // Add more colors as needed
    return al_map_rgb(255, 255, 255); // Default color
}

// Function to draw cones
void drawCones(const std::vector<Cone>& cones) {
    for (const auto& cone : cones) {
        ALLEGRO_COLOR color = getColor(cone.color);
        al_draw_filled_circle(cone.x, cone.y, 5, color);
    }
}

// Update steering angle function
void updateSteeringAngle(Car& car, float deltaTime, bool keys[4]) {
    const float maxSteeringAngle = ALLEGRO_PI / 6; // 30 degrees
    const float steeringSpeed = ALLEGRO_PI;        // Radians per second

    if (keys[0]) { // A key - turn left
        car.steeringAngle += steeringSpeed * deltaTime;
        if (car.steeringAngle > maxSteeringAngle)
            car.steeringAngle = maxSteeringAngle;
    } else if (keys[1]) { // D key - turn right
        car.steeringAngle -= steeringSpeed * deltaTime;
        if (car.steeringAngle < -maxSteeringAngle)
            car.steeringAngle = -maxSteeringAngle;
    } else {
        // Gradually return steering angle to zero
        if (car.steeringAngle > 0) {
            car.steeringAngle -= steeringSpeed * deltaTime;
            if (car.steeringAngle < 0)
                car.steeringAngle = 0;
        } else if (car.steeringAngle < 0) {
            car.steeringAngle += steeringSpeed * deltaTime;
            if (car.steeringAngle > 0)
                car.steeringAngle = 0;
        }
    }
}

// Function to update car movement
void updateCar(Car& car, float deltaTime, bool keys[4]) {
    const float acceleration = 200.0f; // Units per second squared
    const float friction = 0.99f;      // Friction coefficient

    // Update speed
    if (keys[2]) // W key - accelerate
        car.speed += acceleration * deltaTime;
    if (keys[3]) // S key - decelerate
        car.speed -= acceleration * deltaTime;

    // Apply friction
    car.speed *= friction;

    // Limit speed
    const float maxSpeed = 500.0f;
    if (car.speed > maxSpeed) car.speed = maxSpeed;
    if (car.speed < -maxSpeed) car.speed = -maxSpeed;

    // Update steering angle
    updateSteeringAngle(car, deltaTime, keys);

    // Calculate angular velocity
    float angularVelocity = (car.speed * tan(car.steeringAngle)) / car.wheelBase;

    // Update orientation
    car.angle += angularVelocity * deltaTime;

    // Keep angle within -PI to PI
    if (car.angle > ALLEGRO_PI)
        car.angle -= 2 * ALLEGRO_PI;
    else if (car.angle < -ALLEGRO_PI)
        car.angle += 2 * ALLEGRO_PI;

    // Update position
    car.x += car.speed * cos(car.angle) * deltaTime;
    car.y += car.speed * sin(car.angle) * deltaTime;
}

// Function to update camera
void updateCamera(const Car& car, float displayWidth, float displayHeight) {
    ALLEGRO_TRANSFORM transform;
    al_identity_transform(&transform);

    // Center the car on the screen
    al_translate_transform(&transform, displayWidth / 2 - car.x, displayHeight / 2 - car.y);

    al_use_transform(&transform);
}

// Function to receive CAN messages
void receiveCANMessages(int can_socket, bool keys[4]) {
    struct can_frame frame;
    int nbytes = read(can_socket, &frame, sizeof(struct can_frame));

    if (nbytes < 0) {
        if (errno != EAGAIN && errno != EWOULDBLOCK) {
            perror("CAN read error");
        }
        return;
    }

    // Process the received CAN frame
    if (frame.can_id == 0x100) {
        // Key Pressed
        if (frame.data[0] < 4) {
            keys[frame.data[0]] = true;
        }
    } else if (frame.can_id == 0x101) {
        // Key Released
        if (frame.data[0] < 4) {
            keys[frame.data[0]] = false;
        }
    }
}

int main() {
    // Allegro initialization
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

    // CAN socket setup
    int can_socket;
    struct sockaddr_can addr;
    struct ifreq ifr;

    if ((can_socket = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
        perror("Error while opening socket");
        return -1;
    }

    strcpy(ifr.ifr_name, "vcan0");
    ioctl(can_socket, SIOCGIFINDEX, &ifr);

    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (bind(can_socket, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        perror("Error in socket bind");
        return -2;
    }

    // Set CAN socket to non-blocking
    int flags = fcntl(can_socket, F_GETFL, 0);
    fcntl(can_socket, F_SETFL, flags | O_NONBLOCK);

    // Load cones from YAML file
    std::vector<Cone> cones = loadCones("cones.yaml");

    // Initialize car
    Car car = {400.0f, 300.0f, 0.0f, 0.0f, 0.0f, 50.0f}; // Starting position and parameters

    // Start the timer
    al_start_timer(timer);

    bool done = false;

    while (!done) {
        ALLEGRO_EVENT ev;
        al_wait_for_event(event_queue, &ev);

        if (ev.type == ALLEGRO_EVENT_TIMER) {
            float deltaTime = 1.0 / 60;

            // Receive CAN messages
            receiveCANMessages(can_socket, keys);

            // Update game state
            updateCar(car, deltaTime, keys);

            // Update camera before drawing
            updateCamera(car, 800, 600);

            // Render
            al_clear_to_color(al_map_rgb(0, 0, 0));
            drawCones(cones);

            // Save the current transformation
            ALLEGRO_TRANSFORM prevTransform;
            al_copy_transform(&prevTransform, al_get_current_transform());

            // Build the car transformation
            ALLEGRO_TRANSFORM carTransform;
            al_identity_transform(&carTransform);
            al_rotate_transform(&carTransform, car.angle);
            al_translate_transform(&carTransform, car.x, car.y);
            al_use_transform(&carTransform);

            // Draw the car centered at the origin in local coordinates
            al_draw_filled_rectangle(-10, -5, 10, 5, al_map_rgb(255, 255, 0));

            // Restore the previous transformation
            al_use_transform(&prevTransform);

            // Reset to identity transform for UI elements
            ALLEGRO_TRANSFORM identity;
            al_identity_transform(&identity);
            al_use_transform(&identity);

            // Prepare the coordinate string
            char coordText[100];
            sprintf(coordText, "X: %.2f, Y: %.2f", car.x, car.y);

            // Draw the text at position (10, 10)
            al_draw_text(font, al_map_rgb(255, 255, 255), 10, 10, ALLEGRO_ALIGN_LEFT, coordText);

            // Flip the display
            al_flip_display();
        } else if (ev.type == ALLEGRO_EVENT_DISPLAY_CLOSE) {
            done = true;
        }
    }

    // Close CAN socket
    close(can_socket);

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
