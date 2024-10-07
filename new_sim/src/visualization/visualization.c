#include "visualization.h"
#include "configStruct.h"

#include <allegro5/allegro_primitives.h>
#include <allegro5/allegro_ttf.h>
#include <allegro5/keyboard.h>
#include <allegro5/events.h>

#include <fcntl.h>
#include <errno.h>
#include <pthread.h>
#include <unistd.h>
#include <math.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <yaml.h>
#include <linux/can.h>
#include <linux/can/raw.h>

//---------------------------------------------------
//
// Default value, that may be changed by config.yaml
//
//---------------------------------------------------

// Global variables
CarState car_state;

// Lock for thread-safe updates
volatile int done = 0;
pthread_mutex_t carMutex = PTHREAD_MUTEX_INITIALIZER;

// Define ThreadArgs structure globally
struct ThreadArgs {
    int can_socket;
    int send_can_socket;
    Config *config;
};

// ---------------------------------------------------------------------
//                  LOAD CONFIG and TRACK FILES (libyaml)
// ---------------------------------------------------------------------

Cone *loadCones(const char *filename, int *num_cones) {
    FILE *file;
    yaml_parser_t parser;
    yaml_document_t document;
    yaml_node_t *root_node;
    yaml_node_t *cones_node = NULL;
    yaml_node_item_t *item;
    Cone *cones = NULL;
    int count = 0;
    int i = 0;

    file = fopen(filename, "r");
    if (!file) {
        fprintf(stderr, "Failed to open cones file: %s\n", filename);
        return NULL;
    }

    if (!yaml_parser_initialize(&parser)) {
        fprintf(stderr, "Failed to initialize YAML parser\n");
        fclose(file);
        return NULL;
    }

    yaml_parser_set_input_file(&parser, file);
    if (!yaml_parser_load(&parser, &document)) {
        fprintf(stderr, "Failed to parse YAML file\n");
        yaml_parser_delete(&parser);
        fclose(file);
        return NULL;
    }

    root_node = yaml_document_get_root_node(&document);
    if (!root_node || root_node->type != YAML_MAPPING_NODE) {
        fprintf(stderr, "Invalid YAML format in cones file\n");
        yaml_document_delete(&document);
        yaml_parser_delete(&parser);
        fclose(file);
        return NULL;
    }

    // Get the 'cones' node
    for (yaml_node_pair_t *pair = root_node->data.mapping.pairs.start;
         pair < root_node->data.mapping.pairs.top; pair++) {
        yaml_node_t *key_node   = yaml_document_get_node(&document, pair->key);
        yaml_node_t *value_node = yaml_document_get_node(&document, pair->value);

        if (strcmp((char *)key_node->data.scalar.value, "cones") == 0) {
            cones_node = value_node;
            break;
        }
    }

    if (!cones_node || cones_node->type != YAML_SEQUENCE_NODE) {
        fprintf(stderr, "No 'cones' sequence found in YAML file\n");
        yaml_document_delete(&document);
        yaml_parser_delete(&parser);
        fclose(file);
        return NULL;
    }

    // Count the number of cones
    count = cones_node->data.sequence.items.top - cones_node->data.sequence.items.start;
    *num_cones = count;

    // Allocate memory for cones
    cones = (Cone *)malloc(sizeof(Cone) * count);
    if (!cones) {
        fprintf(stderr, "Failed to allocate memory for cones\n");
        yaml_document_delete(&document);
        yaml_parser_delete(&parser);
        fclose(file);
        return NULL;
    }

    // Parse each cone
    i = 0;
    for (item = cones_node->data.sequence.items.start;
         item < cones_node->data.sequence.items.top; item++) {
        yaml_node_t *cone_node = yaml_document_get_node(&document, *item);
        if (cone_node->type != YAML_MAPPING_NODE) {
            continue;
        }

        // Initialize cone
        cones[i].x_pixels = 0.0f;
        cones[i].y_pixels = 0.0f;
        strcpy(cones[i].color, "white");

        // Parse cone properties
        for (yaml_node_pair_t *cone_pair = cone_node->data.mapping.pairs.start;
             cone_pair < cone_node->data.mapping.pairs.top; cone_pair++) {
            yaml_node_t *key_node   = yaml_document_get_node(&document, cone_pair->key);
            yaml_node_t *value_node = yaml_document_get_node(&document, cone_pair->value);

            if (strcmp((char *)key_node->data.scalar.value, "x") == 0) {
                cones[i].x_pixels = atof((char *)value_node->data.scalar.value);
            } else if (strcmp((char *)key_node->data.scalar.value, "y") == 0) {
                cones[i].y_pixels = atof((char *)value_node->data.scalar.value);
            } else if (strcmp((char *)key_node->data.scalar.value, "color") == 0) {
                strncpy(cones[i].color, (char *)value_node->data.scalar.value, sizeof(cones[i].color) - 1);
                cones[i].color[sizeof(cones[i].color) - 1] = '\0';
            }
        }
        i++;
    }

    yaml_document_delete(&document);
    yaml_parser_delete(&parser);
    fclose(file);

    return cones;
}

// ---------------------------------------------------------------------
//                  SETUP ALLEGRO v5
// ---------------------------------------------------------------------

int initializeAllegro(ALLEGRO_DISPLAY **display, ALLEGRO_FONT **font, ALLEGRO_EVENT_QUEUE **event_queue, ALLEGRO_TIMER **timer) {
    if (!al_init()) {
        fprintf(stderr, "Failed to initialize Allegro.\n");
        return 0;
    }

    if (!al_init_primitives_addon()) {
        fprintf(stderr, "Failed to initialize primitives addon.\n");
        return 0;
    } else {
        printf("Allegro primitives addon initialized successfully.\n");
    }

    if (!al_init_font_addon()) {
        fprintf(stderr, "Failed to initialize font addon.\n");
        return 0;
    }

    if (!al_init_ttf_addon()) {
        fprintf(stderr, "Failed to initialize TTF addon.\n");
        return 0;
    }

    if (!al_install_keyboard()) {
        fprintf(stderr, "Failed to install keyboard.\n");
        return 0;
    }

    // Create display
    *display = al_create_display(800, 600);
    if (!*display) {
        fprintf(stderr, "Failed to create display.\n");
        return 0;
    }

    // Load font
    *font = al_create_builtin_font();
    if (!*font) {
        fprintf(stderr, "Failed to create built-in font.\n");
        al_destroy_display(*display);
        return 0;
    }

    // Event queue and timer
    *event_queue = al_create_event_queue();
    *timer       = al_create_timer(1.0 / 60);  // 60 FPS

    al_register_event_source(*event_queue, al_get_timer_event_source(*timer));
    al_register_event_source(*event_queue, al_get_display_event_source(*display));
    al_register_event_source(*event_queue, al_get_keyboard_event_source());

    return 1;
}

ALLEGRO_COLOR getColor(const char *colorName) {
    if (strcmp(colorName, "yellow") == 0) {
        return al_map_rgb(255, 255, 0);
    }
    if (strcmp(colorName, "blue") == 0) {
        return al_map_rgb(0, 0, 255);
    }
    return al_map_rgb(255, 255, 255);  // Default color
}

void drawCones(const Cone *cones, int num_cones, Config *config) {
    int i;
    float ppm = config->visualization_params.pixel_per_meter;
    for (i = 0; i < num_cones; i++) {
        ALLEGRO_COLOR color = getColor(cones[i].color);
        float x_pixels = cones[i].x_pixels * ppm;
        float y_pixels = cones[i].y_pixels * ppm;
        al_draw_filled_circle(x_pixels, y_pixels, 5, color);
    }
}

void renderScene(const Cone *cones, int num_cones, CarState *car_state, Config* config, ALLEGRO_FONT *font, pthread_mutex_t *carMutex) {
    float            car_x_pixels;
    float            car_y_pixels;
    float            car_angle;
    ALLEGRO_TRANSFORM    camera_transform;
    ALLEGRO_TRANSFORM    carTransform;
    ALLEGRO_TRANSFORM    identity;
    char            coordText[100];
    int            i;

    float PIXELS_PER_METER = config->visualization_params.pixel_per_meter;
    float DETECTION_RANGE = config->perception_params.detection_range;

    pthread_mutex_lock(carMutex);
    car_x_pixels = car_state->x * PIXELS_PER_METER;
    car_y_pixels = car_state->y * PIXELS_PER_METER;
    car_angle    = car_state->angle;
    pthread_mutex_unlock(carMutex);

    // Apply camera transformation to center the view on the car
    al_identity_transform(&camera_transform);
    al_translate_transform(&camera_transform, -car_x_pixels + 400, -car_y_pixels + 300);  // Adjust to center car in window
    al_use_transform(&camera_transform);

    // Clear display
    al_clear_to_color(al_map_rgb(0, 0, 0));

    // Draw cones
    drawCones(cones, num_cones, config);

    // Draw lines to nearby cones
    pthread_mutex_lock(carMutex);
    for (i = 0; i < num_cones; i++) {
        // Calculate distance in pixels
        float dx                    = (cones[i].x_pixels - car_state->x) * PIXELS_PER_METER;
        float dy                    = (cones[i].y_pixels - car_state->y) * PIXELS_PER_METER;
        float distance_pixels       = sqrtf(dx * dx + dy * dy);
        float detection_radius_pixels = DETECTION_RANGE * PIXELS_PER_METER;

        if (distance_pixels <= detection_radius_pixels) {
            al_draw_line(car_x_pixels, car_y_pixels, cones[i].x_pixels * PIXELS_PER_METER, cones[i].y_pixels * PIXELS_PER_METER, al_map_rgb(0, 255, 0), 1);
        }
    }
    pthread_mutex_unlock(carMutex);

    // Draw car
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
    al_identity_transform(&identity);
    al_use_transform(&identity);

    // Prepare the coordinate string (in meters)
    pthread_mutex_lock(carMutex);
    snprintf(coordText, sizeof(coordText), "X: %.2f m, Y: %.2f m", car_state->x, car_state->y);
    pthread_mutex_unlock(carMutex);

    // Draw the text at position (10, 10)
    al_draw_text(font, al_map_rgb(255, 255, 255), 10, 10, ALLEGRO_ALIGN_LEFT, coordText);

    // Flip the display
    al_flip_display();
}

float calculateDistance(float x1_pixels, float y1_pixels, float x2_pixels, float y2_pixels) {
    float dx = x2_pixels - x1_pixels;
    float dy = y2_pixels - y1_pixels;
    return sqrtf(dx * dx + dy * dy);
}

void cleanupAllegro(ALLEGRO_DISPLAY *display, ALLEGRO_FONT *font, ALLEGRO_TIMER *timer, ALLEGRO_EVENT_QUEUE *event_queue) {
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

// ---------------------------------------------------------------------
//                  CAN SOCKET SETUP AND HOW TO USE
// ---------------------------------------------------------------------

int setupCANSocket(const char *interface_name) {
    int can_socket;
    struct sockaddr_can addr;
    struct ifreq ifr;
    int flags;

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

    addr.can_family  = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (bind(can_socket, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        perror("Error in receive socket bind");
        close(can_socket);
        return -2;
    }

    // Set CAN socket to non-blocking
    flags = fcntl(can_socket, F_GETFL, 0);
    if (fcntl(can_socket, F_SETFL, flags | O_NONBLOCK) < 0) {
        perror("Error setting non-blocking mode");
        close(can_socket);
        return -3;
    }

    return can_socket;
}

int setupSendCANSocket(const char *interface_name) {
    int send_can_socket;
    struct sockaddr_can send_addr;
    struct ifreq send_ifr;
    int send_flags;

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

    send_addr.can_family  = AF_CAN;
    send_addr.can_ifindex = send_ifr.ifr_ifindex;

    if (bind(send_can_socket, (struct sockaddr *)&send_addr, sizeof(send_addr)) < 0) {
        perror("Error in send socket bind");
        close(send_can_socket);
        return -2;
    }

    // Set send CAN socket to non-blocking (optional)
    send_flags = fcntl(send_can_socket, F_GETFL, 0);
    if (fcntl(send_can_socket, F_SETFL, send_flags | O_NONBLOCK) < 0) {
        perror("Error setting non-blocking mode for send socket");
        close(send_can_socket);
        return -3;
    }

    return send_can_socket;
}

void sendFloatCAN(int send_can_socket, int can_id, float value) {
    struct can_frame frame;
    int nbytes;

    frame.can_id  = can_id;
    frame.can_dlc = sizeof(float);
    memcpy(frame.data, &value, sizeof(float));

    nbytes = write(send_can_socket, &frame, sizeof(struct can_frame));
    if (nbytes != sizeof(struct can_frame)) {
        perror("CAN write error");
    }
}

void *receiveCANMessagesThread(void *arg) {
    struct ThreadArgs *args = (struct ThreadArgs *)arg;
    int can_socket = args->can_socket;
    Config *config = args->config;

    struct can_frame frame;
    int nbytes;
    float steeringDeg;
    float throttle;

    while (!done) {
        nbytes = read(can_socket, &frame, sizeof(struct can_frame));
        if (nbytes >= 0) {
            if (nbytes >= sizeof(struct can_frame)) {
                // Process the received CAN frame
                if (frame.can_id == config->ids.steer_id) {
                    if (frame.can_dlc == sizeof(float)) {
                        memcpy(&steeringDeg, frame.data, sizeof(float));
                        pthread_mutex_lock(&carMutex);
                        car_state.steeringAngle = steeringDeg;
                        pthread_mutex_unlock(&carMutex);
                    }
                } else if (frame.can_id == config->ids.throttle_id) {
                    if (frame.can_dlc == sizeof(float)) {
                        memcpy(&throttle, frame.data, sizeof(float));
                        pthread_mutex_lock(&carMutex);
                        car_state.throttle = throttle;
                        pthread_mutex_unlock(&carMutex);
                    }
                }
            } else {
                fprintf(stderr, "Incomplete CAN frame received.\n");
            }
        } else {
            if (nbytes < 0 && (errno != EAGAIN && errno != EWOULDBLOCK)) {
                perror("CAN read error");
            }
            // Sleep briefly to avoid busy waiting
            usleep(1000);
        }
    }
    return NULL;
}

void *sendCarDataThread(void *arg) {
    struct ThreadArgs *args = (struct ThreadArgs *)arg;
    int send_can_socket = args->send_can_socket;
    Config *config = args->config;

    while (!done) {
        pthread_mutex_lock(&carMutex);
        // Send car state over CAN
        sendFloatCAN(send_can_socket, config->ids.carX_id, car_state.x);
        sendFloatCAN(send_can_socket, config->ids.carY_id, car_state.y);
        sendFloatCAN(send_can_socket, config->ids.carAngle_id, car_state.angle);
        pthread_mutex_unlock(&carMutex);
        // Sleep to control the sending rate (100 Hz)
        usleep(10000);
    }
    return NULL;
}

// ---------------------------------------------------------------------
//                  VEHICLE DESCRIPTION & MOTION MODEL
// ---------------------------------------------------------------------

void initializeCar(CarState *car_state) {
    car_state->x             = 40.0f;    // initial x-position [meters]
    car_state->y             = 30.0f;    // initial y-position [meters]
    car_state->angle         = 0.0f;     // initial orientation [rad]
    car_state->steeringAngle = 0.0f;
    car_state->throttle      = 0.0f;
    car_state->speed         = 0.0f;
    // car->params will be loaded from config.yaml
}

void updateCarPosition(CarState *car_state, Config *config, float deltaTime, pthread_mutex_t *carMutex, const Cone *cones, int num_cones) {
    // Call the desired vehicle model function
    updateCarPositionSingleTrackModel(car_state, config, deltaTime, carMutex, cones, num_cones);
}

void updateCarPositionSingleTrackModel(CarState *car_state, Config *config, float deltaTime, pthread_mutex_t *carMutex, const Cone *cones, int num_cones) {
    // Car parameters
    const float L              = config->car_params.wheelbase;          // Wheelbase in meters
    const float MAX_THROTTLE   = config->actuation_params.max_throttle; // Max throttle units
    const float MAX_SPEED      = config->actuation_params.max_speed;    // Max speed in m/s
    const float CAR_RADIUS     = 1.0f;   // Car collision radius (meters)
    const float CONE_RADIUS    = 0.5f;   // Cone collision radius (meters)
    float delta;
    float v;
    float beta;

    pthread_mutex_lock(carMutex);

    // Convert steering angle from degrees to radians
    delta = car_state->steeringAngle * M_PI / 180.0f;

    // Compute speed based on throttle
    v = (car_state->throttle / MAX_THROTTLE) * MAX_SPEED;

    // Compute slip angle beta
    beta = atan2f(tanf(delta), 2.0f);  // Assuming front-wheel steering

    // THEORETICAL POSE: we need to check if there isn't collision between car and cones
    float proposed_x = car_state->x + v * cosf(car_state->angle + beta) * deltaTime;
    float proposed_y = car_state->y + v * sinf(car_state->angle + beta) * deltaTime;

    // CHECK COLLISION
    int collision = 0;

    for (int i = 0; i < num_cones; i++) {
        float dx = proposed_x - cones[i].x_pixels;
        float dy = proposed_y - cones[i].y_pixels;
        float distance = sqrtf(dx * dx + dy * dy);

        if (distance < (CAR_RADIUS + CONE_RADIUS)) {
            collision = 1;
            break;
        }
    }

    if (!collision) {
        // Update position
        car_state->x = proposed_x;
        car_state->y = proposed_y;
    } else {
        // Collision detected; prevent movement
        v = 0.0f;
    }

    car_state->angle += (v / L) * sinf(delta) * deltaTime;

    // Keep angle within -PI to PI
    if (car_state->angle > M_PI) {
        car_state->angle -= 2 * M_PI;
    } else if (car_state->angle < -M_PI) {
        car_state->angle += 2 * M_PI;
    }

    car_state->speed = v;

    pthread_mutex_unlock(carMutex);
}

// ---------------------------------------------------------------------
//                              MAIN
// ---------------------------------------------------------------------

int visualization_main(Config *config, const char *track_file) {
    // Initialize Allegro
    ALLEGRO_DISPLAY     *display     = NULL;
    ALLEGRO_FONT        *font        = NULL;
    ALLEGRO_EVENT_QUEUE *event_queue = NULL;
    ALLEGRO_TIMER       *timer       = NULL;

    int can_socket;
    int send_can_socket;

    Cone *cones = NULL;
    int num_cones = 0;

    pthread_t canReceiveThreadId;
    pthread_t canSendThreadId;
    int should_exit = 0;

    ALLEGRO_EVENT ev;
    float deltaTime;

    if (!initializeAllegro(&display, &font, &event_queue, &timer)) {
        return -1;
    }

    initializeCar(&car_state);

    can_socket = setupCANSocket("vcan0");
    if (can_socket < 0) {
        fprintf(stderr, "VCAN interface not configured\n");
        fprintf(stderr, "Please run 'bash setup_vcan.sh' before this\n");
        cleanupAllegro(display, font, timer, event_queue);
        return -1;
    }

    send_can_socket = setupSendCANSocket("vcan0");
    if (send_can_socket < 0) {
        close(can_socket);
        cleanupAllegro(display, font, timer, event_queue);
        return -1;
    }

    // Load cones from YAML file
    cones = loadCones(track_file, &num_cones);
    if (!cones) {
        fprintf(stderr, "Failed to load cones.\n");
        close(can_socket);
        close(send_can_socket);
        cleanupAllegro(display, font, timer, event_queue);
        return -1;
    }

    // Allocate argument structures dynamically
    struct ThreadArgs *receiveArgs = malloc(sizeof(struct ThreadArgs));
    if (!receiveArgs) {
        fprintf(stderr, "Failed to allocate memory for receiveArgs\n");
        close(can_socket);
        close(send_can_socket);
        cleanupAllegro(display, font, timer, event_queue);
        return -1;
    }
    receiveArgs->can_socket = can_socket;
    receiveArgs->send_can_socket = 0;
    receiveArgs->config = config;

    struct ThreadArgs *sendArgs = malloc(sizeof(struct ThreadArgs));
    if (!sendArgs) {
        fprintf(stderr, "Failed to allocate memory for sendArgs\n");
        free(receiveArgs);
        close(can_socket);
        close(send_can_socket);
        cleanupAllegro(display, font, timer, event_queue);
        return -1;
    }
    sendArgs->can_socket = 0;
    sendArgs->send_can_socket = send_can_socket;
    sendArgs->config = config;

    // Start the receive CAN messages thread
    if (pthread_create(&canReceiveThreadId, NULL, receiveCANMessagesThread, receiveArgs) != 0) {
        perror("Failed to create receive CAN messages thread");
        free(receiveArgs);
        free(sendArgs);
        close(can_socket);
        close(send_can_socket);
        cleanupAllegro(display, font, timer, event_queue);
        return -1;
    }

    // Start the send car data thread
    if (pthread_create(&canSendThreadId, NULL, sendCarDataThread, sendArgs) != 0) {
        perror("Failed to create send car data thread");
        done = 1;  // Signal the receive thread to exit
        pthread_join(canReceiveThreadId, NULL);
        free(receiveArgs);
        free(sendArgs);
        close(can_socket);
        close(send_can_socket);
        cleanupAllegro(display, font, timer, event_queue);
        return -1;
    }

    // Start the timer
    al_start_timer(timer);

    while (!should_exit) {
        al_wait_for_event(event_queue, &ev);

        if (ev.type == ALLEGRO_EVENT_TIMER) {
            deltaTime = 1.0f / 60.0f;  // 60 FPS

            // Update car's position based on steering angle and throttle
            updateCarPosition(&car_state, config, deltaTime, &carMutex, cones, num_cones);

            // Render the scene
            renderScene(cones, num_cones, &car_state, config, font, &carMutex);
        } else if (ev.type == ALLEGRO_EVENT_DISPLAY_CLOSE) {
            should_exit = 1;
            done        = 1;
        } else if (ev.type == ALLEGRO_EVENT_KEY_DOWN) {
            if (ev.keyboard.keycode == ALLEGRO_KEY_ESCAPE) {
                should_exit = 1;
                done = 1;
            }
        }
    }

    // Wait for threads to finish
    pthread_join(canReceiveThreadId, NULL);
    pthread_join(canSendThreadId, NULL);

    // Free dynamically allocated argument structures
    free(receiveArgs);
    free(sendArgs);

    close(can_socket);
    close(send_can_socket);

    cleanupAllegro(display, font, timer, event_queue);

    // Cleanup of memory reserved for cones
    if (cones) {
        free(cones);
    }

    return 0;
}
