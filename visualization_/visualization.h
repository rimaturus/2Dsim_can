
#ifndef VISUALIZATION_H
#define VISUALIZATION_H

#include <vector>
#include <string>
#include <atomic>
#include <mutex>
#include <yaml-cpp/yaml.h>
#include <allegro5/allegro.h>
#include <allegro5/allegro_font.h>       // Added this include
#include <allegro5/allegro_primitives.h> // Added this include
#include <allegro5/allegro_ttf.h>        // Added this include
#include <linux/can.h>
#include <linux/can/raw.h>
#include <cmath>

// Constants
extern  float PIXELS_PER_METER;

// CAN IDs
extern  int STEERING_CAN_ID;
extern  int THROTTLE_CAN_ID;
extern  int CAR_X_CAN_ID;
extern  int CAR_Y_CAN_ID;
extern  int CAR_ANGLE_CAN_ID;

// Structures
struct Cone {
    float x_pixels; // positions in pixels
    float y_pixels;
    std::string color;
};

// Car parameters
struct CarParameters {
    float wheelbase;      // Wheelbase in meters
    float max_throttle;   // Maximum throttle units
    float max_speed;      // Maximum speed in m/s
};

struct Car {
    float x;              // meters
    float y;              // meters
    float angle;          // radians
    float steeringAngle;  // degrees
    float throttle;       // units
    float speed;          // m/s
    CarParameters params;    
};

// Function prototypes
void loadConfig(const std::string& filename, Car& car);

bool initializeAllegro(ALLEGRO_DISPLAY*& display, ALLEGRO_FONT*& font, ALLEGRO_EVENT_QUEUE*& event_queue, ALLEGRO_TIMER*& timer);
ALLEGRO_COLOR getColor(const std::string& colorName);
void cleanupAllegro(ALLEGRO_DISPLAY* display, ALLEGRO_FONT* font, ALLEGRO_TIMER* timer, ALLEGRO_EVENT_QUEUE* event_queue);

std::vector<Cone> loadCones(const std::string& filename);
void drawCones(const std::vector<Cone>& cones);

int setupCANSocket(const char* interface_name);
int setupSendCANSocket(const char* interface_name);
void sendCarDataThread(int send_can_socket, Car& car, std::mutex& carMutex, std::atomic<bool>& done);
void sendFloatCAN(int send_can_socket, int can_id, float value);
void receiveCANMessagesThread(int can_socket, std::atomic<bool>& done, Car& car, std::mutex& carMutex);

void initializeCar(Car& car);
float calculateDistance(float x1_pixels, float y1_pixels, float x2_pixels, float y2_pixels);
void updateCarPosition(Car& car, float deltaTime, std::mutex& carMutex);
void updateCarPositionSingleTrackModel(Car& car, float deltaTime, std::mutex& carMutex);

void renderScene(const std::vector<Cone>& cones, Car& car, ALLEGRO_FONT* font, std::mutex& carMutex);

#endif // VISUALIZATION_H
