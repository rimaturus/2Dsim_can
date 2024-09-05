#include <allegro5/allegro.h>
#include <allegro5/allegro_primitives.h>
#include <yaml-cpp/yaml.h>
#include <iostream>
#include <vector>
#include <cmath>  // For trigonometry

// Structure to store cone data
struct Cone {
    float x, y;
    ALLEGRO_COLOR color;
};

// Structure for car
struct Car {
    float x, y;         // Car position
    float angle;        // Car orientation (angle in radians)
    float speed;        // Car speed
    float steeringAngle; // Steering angle in radians
    float wheelBase;    // Distance between front and rear wheels
    bool up, down, left, right; // Control flags
};

// Constants for car movement
const float MAX_STEERING_ANGLE = ALLEGRO_PI / 6.0;  // Max steering angle (30 degrees)
const float MAX_SPEED = 4.0;
const float ACCELERATION = 0.2;
const float DECELERATION = 0.1;
const float FRICTION = 0.05;
const float VECTOR_LENGTH = 50.0f; // Length of the direction vector

// Function to convert YAML color to Allegro color
ALLEGRO_COLOR getColorFromString(const std::string& color) {
    if (color == "blue") return al_map_rgb(0, 0, 255);
    if (color == "yellow") return al_map_rgb(255, 255, 0);
    return al_map_rgb(255, 255, 255);  // Default to white
}

// Function to load cones from YAML file
std::vector<Cone> loadConesFromYAML(const std::string& filename) {
    std::vector<Cone> cones;
    YAML::Node config = YAML::LoadFile(filename);

    for (auto coneNode : config["cones"]) {
        Cone cone;
        cone.x = coneNode["x"].as<float>();
        cone.y = coneNode["y"].as<float>();
        cone.color = getColorFromString(coneNode["color"].as<std::string>());
        cones.push_back(cone);
    }

    return cones;
}

// Function to update the car position using Ackermann steering model
void updateCar(Car& car) {
    // Handle car controls for each car separately
    if (car.up) car.speed = std::min(car.speed + ACCELERATION, MAX_SPEED);  // Up
    if (car.down) car.speed = std::max(car.speed - DECELERATION, -MAX_SPEED); // Down
    if (car.left) car.steeringAngle = std::min(car.steeringAngle + 0.05f, MAX_STEERING_ANGLE); // Left
    if (car.right) car.steeringAngle = std::max(car.steeringAngle - 0.05f, -MAX_STEERING_ANGLE); // Right

    if (!car.left && !car.right) car.steeringAngle = 0;  // Reset steering when no left or right

    // If the steering angle is zero, the car moves straight
    if (fabs(car.steeringAngle) > 1e-5) {
        float turningRadius = car.wheelBase / tan(car.steeringAngle);
        car.x += car.speed * cos(car.angle);
        car.y += car.speed * sin(car.angle);
        car.angle += car.speed / turningRadius;
    } else {
        // Move in a straight line
        car.x += car.speed * cos(car.angle);
        car.y += car.speed * sin(car.angle);
    }

    // Apply friction (gradual deceleration)
    if (car.speed > 0) car.speed -= FRICTION;
    if (car.speed < 0) car.speed += FRICTION;

    if (fabs(car.speed) < 1e-3) car.speed = 0;  // Stop the car if it's too slow
}

int main() {
    // Allegro initialization
    if (!al_init()) {
        std::cerr << "Failed to initialize Allegro!\n";
        return -1;
    }

    // Initialize Allegro primitives for shapes and keyboard input
    al_init_primitives_addon();
    al_install_keyboard();

    // Create Allegro display window
    ALLEGRO_DISPLAY* display = al_create_display(800, 600);
    if (!display) {
        std::cerr << "Failed to create display!\n";
        return -1;
    }

    // Load cones from YAML file
    std::string yamlFile = "track.yaml";
    std::vector<Cone> cones = loadConesFromYAML(yamlFile);

    // Initialize a vector of cars
    std::vector<Car> cars = {
        {400, 300, 0, 0, 0, 50, false, false, false, false},  // First car
        // {500, 400, 0, 0, 0, 50, false, false, false, false}   // Second car
    };

    // Main loop
    bool running = true;
    ALLEGRO_EVENT_QUEUE* event_queue = al_create_event_queue();
    ALLEGRO_TIMER* timer = al_create_timer(1.0 / 60.0);
    ALLEGRO_EVENT event;

    al_register_event_source(event_queue, al_get_display_event_source(display));
    al_register_event_source(event_queue, al_get_timer_event_source(timer));
    al_register_event_source(event_queue, al_get_keyboard_event_source());
    al_start_timer(timer);

    while (running) {
        al_wait_for_event(event_queue, &event);

        // Handle events
        if (event.type == ALLEGRO_EVENT_DISPLAY_CLOSE) {
            running = false;
        } else if (event.type == ALLEGRO_EVENT_TIMER) {
            // Update each car
            for (auto& car : cars) {
                updateCar(car);
            }

            // Clear screen with black
            al_clear_to_color(al_map_rgb(0, 0, 0));

            // Draw the track and cones statically (no transformation)
            al_draw_filled_rectangle(50, 100, 750, 500, al_map_rgb(100, 100, 100));  // Gray track
            for (const Cone& cone : cones) {
                al_draw_filled_circle(cone.x, cone.y, 10, cone.color);  // Cone as a circle
            }

            // Draw each car and its direction vector, with transformations applied only to each car
            for (const auto& car : cars) {
                // Apply transformation only for the current car
                ALLEGRO_TRANSFORM car_transform;
                al_identity_transform(&car_transform);
                al_translate_transform(&car_transform, -25, -12.5); // Center the car's rotation
                al_rotate_transform(&car_transform, car.angle); // Rotate the car based on its angle
                al_translate_transform(&car_transform, car.x, car.y); // Translate the car to its position
                al_use_transform(&car_transform);

                // Draw the car as a rectangle
                al_draw_filled_rectangle(0, 0, 50, 25, al_map_rgb(0, 255, 0)); // Draw the car

                // Draw the direction vector attached to the car (in local coordinates)
                al_draw_line(25, 12.5, 25 + VECTOR_LENGTH, 12.5, al_map_rgb(255, 0, 0), 2.0); // Red line for the direction

                // Reset the transformation to ensure subsequent cars and the rest of the scene are unaffected
                al_identity_transform(&car_transform);
                al_use_transform(&car_transform);
            }

            // Flip the display (show the updated frame)
            al_flip_display();
        } else if (event.type == ALLEGRO_EVENT_KEY_DOWN) {
            // Handle control for each car separately based on unique keybindings
            switch (event.keyboard.keycode) {
                // Controls for Car 1
                case ALLEGRO_KEY_W:
                    cars[0].up = true;
                    break;
                case ALLEGRO_KEY_S:
                    cars[0].down = true;
                    break;
                case ALLEGRO_KEY_A:
                    cars[0].left = true;
                    break;
                case ALLEGRO_KEY_D:
                    cars[0].right = true;
                    break;

                // Controls for Car 2
                case ALLEGRO_KEY_UP:
                    cars[1].up = true;
                    break;
                case ALLEGRO_KEY_DOWN:
                    cars[1].down = true;
                    break;
                case ALLEGRO_KEY_LEFT:
                    cars[1].left = true;
                    break;
                case ALLEGRO_KEY_RIGHT:
                    cars[1].right = true;
                    break;
            }
        } else if (event.type == ALLEGRO_EVENT_KEY_UP) {
            // Stop control for each car when key is released
            switch (event.keyboard.keycode) {
                // Controls for Car 1
                case ALLEGRO_KEY_W:
                    cars[0].up = false;
                    break;
                case ALLEGRO_KEY_S:
                    cars[0].down = false;
                    break;
                case ALLEGRO_KEY_A:
                    cars[0].left = false;
                    break;
                case ALLEGRO_KEY_D:
                    cars[0].right = false;
                    break;

                // Controls for Car 2
                case ALLEGRO_KEY_UP:
                    cars[1].up = false;
                    break;
                case ALLEGRO_KEY_DOWN:
                    cars[1].down = false;
                    break;
                case ALLEGRO_KEY_LEFT:
                    cars[1].left = false;
                    break;
                case ALLEGRO_KEY_RIGHT:
                    cars[1].right = false;
                    break;
            }
        }
    }

    // Cleanup
    al_destroy_display(display);
    al_destroy_timer(timer);
    al_destroy_event_queue(event_queue);
    al_shutdown_primitives_addon();

    return 0;
}
