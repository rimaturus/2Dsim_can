#ifndef VISUALIZATION_H
#define VISUALIZATION_H

#include <allegro5/allegro.h>
#include <allegro5/allegro_font.h>

// Structure definitions
typedef struct {
    float x_pixels;
    float y_pixels;
    char color[16];
} Cone;

typedef struct {
    float wheelbase;
    float max_throttle;
    float max_speed;
} CarParams;

typedef struct {
    float       x;
    float       y;
    float       angle;
    float       steeringAngle;
    float       throttle;
    float       speed;
    CarParams   params;
} Car;

// Function prototypes
void loadConfig(const char *filename, Car *car);
Cone *loadCones(const char *filename, int *num_cones);

int initializeAllegro(ALLEGRO_DISPLAY **display, ALLEGRO_FONT **font, ALLEGRO_EVENT_QUEUE **event_queue, ALLEGRO_TIMER **timer);
ALLEGRO_COLOR getColor(const char *colorName);
void drawCones(const Cone *cones, int num_cones);
void renderScene(const Cone *cones, int num_cones, Car *car, ALLEGRO_FONT *font, pthread_mutex_t *carMutex);
float calculateDistance(float x1_pixels, float y1_pixels, float x2_pixels, float y2_pixels);
void cleanupAllegro(ALLEGRO_DISPLAY *display, ALLEGRO_FONT *font, ALLEGRO_TIMER *timer, ALLEGRO_EVENT_QUEUE *event_queue);

int setupCANSocket(const char *interface_name);
int setupSendCANSocket(const char *interface_name);
void sendFloatCAN(int send_can_socket, int can_id, float value);
void *receiveCANMessagesThread(void *arg);
void *sendCarDataThread(void *arg);

void initializeCar(Car *car);
void updateCarPosition(Car *car, float deltaTime, pthread_mutex_t *carMutex);
void updateCarPositionSingleTrackModel(Car *car, float deltaTime, pthread_mutex_t *carMutex);

int visualization_main(const char *config_file, const char *track_file);

#endif
