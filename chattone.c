#include <stdio.h>
#include <stdlib.h>
#include <allegro.h>
#include <math.h>
#include <yaml.h>
#include <string.h>
#include "ptask/ptask.h"    // ptask.h must be in your include path

/* --------------------------*/
/*        CONSTANTS         */
/* --------------------------*/

#define PX_PER_METER      100.0    // 100 pixels = 1 meter (1 px = 1 cm)
#define DEG2RAD           0.0174533

// LiDAR configuration
const int   angle_step = 1;           // in degrees
const float MAX_RANGE  = 10.0;         // maximum range in meters
const float DIST_RES   = 0.01;         // distance resolution in meters (1 cm)

// Derived constant for lidar
const int N_ANGLES  = 360 / angle_step;  

// For the perception view, we create an off-screen bitmap with a fixed size.
// (The perception view is rendered using its own coordinate system and then drawn unscaled.)
const int PERCEPTION_VIEW_SIZE = 300;  // in pixels

// Task periods (in milliseconds)
#define PERCEPTION_PERIOD  100
#define CONTROL_PERIOD     50
#define DISPLAY_PERIOD     17   // ~60Hz (1000/17 ≈ 58.8 Hz)

/* --------------------------*/
/*        STRUCTURES        */
/* --------------------------*/

// A cone drawn on the track
typedef struct {
    float x;
    float y;
    int color;
} cone;

// A single LiDAR detection result
typedef struct {
    float distance;   // in meters
    int color;        // detected color (or -1 if none)
} detection;

/* --------------------------*/
/*     GLOBAL VARIABLES     */
/* --------------------------*/

// Global bitmaps
BITMAP *background = NULL;
BITMAP *track      = NULL;
BITMAP *car_bitmap = NULL;
BITMAP *perception_bitmap = NULL; // off-screen buffer for sensor view
BITMAP *display_buffer    = NULL; // off-screen buffer for final compositing

// Global vehicle state (updated by the control task)
float car_x = 4.5;      // in meters
float car_y = 3.0;      // in meters
int   car_angle = 27;   // in degrees

// Mutex for synchronizing access to shared bitmaps and state
pthread_mutex_t draw_mutex = PTHREAD_MUTEX_INITIALIZER;

/* --------------------------*/
/*   FUNCTION DECLARATIONS  */
/* --------------------------*/

// Periodic task functions (using ptask.h notation)
void *perception_task(void *arg);
void *control_task(void *arg);
void *display_task(void *arg);

// Simulation helper functions (adapted from your original code)
void init_cones(cone *cones, int max_cones);
void load_cones_positions(const char *filename, cone *cones, int max_cones);
void lidar(float car_center_x, float car_center_y, detection *measures);
void keyboard_control(float *car_x, float *car_y, int *car_angle);
void vehicle_model(float *car_x, float *car_y, int *car_angle, float speed, float steering);
float angle_rotation_sprite(float angle);

/* --------------------------*/
/*   FUNCTION DEFINITIONS   */
/* --------------------------*/

// Convert a conventional angle (in degrees) to the fixed‐point angle used by Allegro.
float angle_rotation_sprite(float angle)
{
    return 64.0f - 128.0f * angle / 180.0f;
}

/* 
   MAIN: Initializes Allegro, loads resources, creates off-screen buffers,
   and then creates three periodic tasks:
     - Perception task (computes a rotating 30-ray view)
     - Control task (reads keyboard input and updates vehicle state)
     - Display task (composites all layers and updates the screen at ~60Hz)
*/
int main(void)
{
    printf("Starting simulation...\n");

    /* Initialize Allegro */
    allegro_init();
    install_keyboard();
    install_mouse();

    int XMAX = (int)(10 * PX_PER_METER);
    int YMAX = (int)(10 * PX_PER_METER);
    
    set_color_depth(32);
    set_gfx_mode(GFX_AUTODETECT_WINDOWED, XMAX, YMAX, 0, 0);
    
    /* Create the off-screen display buffer */
    display_buffer = create_bitmap(XMAX, YMAX);
    clear_to_color(display_buffer, makecol(255,255,255));

    /* Create background and track bitmaps */
    background = create_bitmap(XMAX, YMAX);
    clear_bitmap(background);
    clear_to_color(background, makecol(78,91,49));  // grass green

    track = create_bitmap(XMAX, YMAX);
    clear_bitmap(track);
    clear_to_color(track, makecol(128,126,120));     // asphalt gray

    /* Load cones and draw them on the track */
    const int max_cones = 1000;
    cone cones[max_cones];
    init_cones(cones, max_cones);
    load_cones_positions("track/cones.yaml", cones, max_cones);
    printf("Cones loaded\n");
    for (int i = 0; i < max_cones; i++) {
        if (cones[i].color != -1) {  // draw valid cones only
            circlefill(track, (int)cones[i].x, (int)cones[i].y,
                       (int)(0.05 * PX_PER_METER), cones[i].color);
        }
    }

    /* Load the car sprite */
    car_bitmap = load_bitmap("bitmaps/f1_car_pink.bmp", NULL);
    if (!car_bitmap) {
        printf("Error loading car bitmap\n");
        exit(1);
    }

    /* Create the perception bitmap.
       (This bitmap will be rendered unscaled; its size is fixed.)
    */
    perception_bitmap = create_bitmap(PERCEPTION_VIEW_SIZE, PERCEPTION_VIEW_SIZE);
    clear_to_color(perception_bitmap, makecol(0,0,0));  // clear with black

    /* Initial display composition */
    pthread_mutex_lock(&draw_mutex);
      clear_to_color(display_buffer, makecol(255,255,255));
      draw_sprite(display_buffer, background, 0, 0);
      draw_sprite(display_buffer, track, 0, 0);
      rotate_scaled_sprite(display_buffer, car_bitmap,
                           (int)(car_x * PX_PER_METER),
                           (int)(car_y * PX_PER_METER),
                           ftofix(angle_rotation_sprite(car_angle)),
                           ftofix(0.5));
      /* Initially, the perception view will later be drawn centered on the car */
      blit(display_buffer, screen, 0, 0, 0, 0, XMAX, YMAX);
    pthread_mutex_unlock(&draw_mutex);

    /* Initialize the periodic task system (using SCHED_OTHER) */
    ptask_init(SCHED_OTHER);

    /* Create periodic tasks:
         Task 1: Perception task
         Task 2: Control task
         Task 3: Display task (approx. 60Hz)
    */
    if (task_create(1, perception_task, PERCEPTION_PERIOD, PERCEPTION_PERIOD, 15, ACT) != 0) {
        fprintf(stderr, "Failed to create Perception Task\n");
        exit(EXIT_FAILURE);
    }
    if (task_create(2, control_task, CONTROL_PERIOD, CONTROL_PERIOD, 20, ACT) != 0) {
        fprintf(stderr, "Failed to create Control Task\n");
        exit(EXIT_FAILURE);
    }
    if (task_create(3, display_task, DISPLAY_PERIOD, DISPLAY_PERIOD, 25, ACT) != 0) {
        fprintf(stderr, "Failed to create Display Task\n");
        exit(EXIT_FAILURE);
    }

    /* Wait for tasks to terminate (they will exit when ESC is pressed) */
    wait_for_task_end(1);
    wait_for_task_end(2);
    wait_for_task_end(3);

    printf("Exiting simulation...\n");
    clear_keybuf();
    readkey();
    allegro_exit();
    return 0;
}

/* --------------------------*/
/*   PERIODIC TASK FUNCTIONS*/
/* --------------------------*/

/*
   Perception Task:
   - Computes LiDAR measurements (all 360°) using the current vehicle position.
   - Renders a rotating sliding window of 30 rays into the off-screen perception_bitmap.
   - In this view the car is centered.
*/
void *perception_task(void *arg)
{
    int task_id = get_task_index(arg);
    wait_for_activation(task_id);

    detection measures[N_ANGLES];
    static int start_angle = 0;  // rotating start angle for 30-ray window

    while (!key[KEY_ESC])
    {
        /* For the sensor simulation, we assume the sensor is at the car's center.
           (You could adjust this if the sensor is mounted off-center.)
        */
        float center_car_x = car_x;
        float center_car_y = car_y;

        /* Obtain LiDAR measurements over all angles */
        lidar(center_car_x, center_car_y, measures);

        /* Render the perception view:
           Clear the off-screen perception bitmap and draw 30 consecutive rays,
           using a sliding window that rotates over successive periods.
           In this view, we center the coordinate system on the car.
        */
        pthread_mutex_lock(&draw_mutex);
            clear_to_color(perception_bitmap, makecol(255,0,255));  // black background
            int cx = PERCEPTION_VIEW_SIZE / 2;
            int cy = PERCEPTION_VIEW_SIZE / 2;
            float ignore_distance = 0.5; // in meters

            for (int i = 0; i < 30; i++) {
                int lidar_angle = (start_angle + i) % 360;
                float cos_angle = cos(lidar_angle * DEG2RAD);
                float sin_angle = sin(lidar_angle * DEG2RAD);

                int x0 = cx;// + (int)(ignore_distance * cos_angle * PX_PER_METER);  // using 100 px/m for perception view
                int y0 = cy;// + (int)(ignore_distance * sin_angle * PX_PER_METER);
                int x_det = cx + (int)(measures[lidar_angle].distance * cos_angle * 100);
                int y_det = cy + (int)(measures[lidar_angle].distance * sin_angle * 100);

                int color_line = (measures[lidar_angle].color == -1) ? makecol(255,0,0) : measures[lidar_angle].color;
                line(perception_bitmap, x0, y0, x_det, y_det, color_line);
            }
            /* Mark the center (the sensor/car location) in the perception view */
            circlefill(perception_bitmap, cx, cy, 3, makecol(0,255,0));
        pthread_mutex_unlock(&draw_mutex);

        /* Update the start angle for the sliding window in the next period */
        start_angle = (start_angle + 1) % 360;

        wait_for_period(task_id);
    }
    return NULL;
}

/*
   Control Task:
   - Reads keyboard input and updates the vehicle state via keyboard_control()
     (which in turn calls vehicle_model()).
*/
void *control_task(void *arg)
{
    int task_id = get_task_index(arg);
    wait_for_activation(task_id);

    while (!key[KEY_ESC])
    {
        keyboard_control(&car_x, &car_y, &car_angle);
        wait_for_period(task_id);
    }
    return NULL;
}

/*
   Display Task (approx. 60Hz):
   - Composites the final image using the off-screen display_buffer.
   - Draws background, track, the rotated car sprite, and then overlays the
     perception view centered on the car (unscaled).
   - Finally, blits the composed frame to the screen.
*/
void *display_task(void *arg)
{
    int task_id = get_task_index(arg);
    wait_for_activation(task_id);

    int XMAX = display_buffer->w;
    int YMAX = display_buffer->h;

    while (!key[KEY_ESC])
    {
        pthread_mutex_lock(&draw_mutex);
            clear_to_color(display_buffer, makecol(255,255,255));
            /* Composite layers in order: background, track, car */
            draw_sprite(display_buffer, background, 0, 0);
            draw_sprite(display_buffer, track, 0, 0);
            rotate_scaled_sprite(display_buffer, car_bitmap,
                                 (int)(car_x * PX_PER_METER),
                                 (int)(car_y * PX_PER_METER),
                                 ftofix(angle_rotation_sprite(car_angle)),
                                 ftofix(0.5));
            /* Now overlay the perception view centered on the car.
               Compute the top-left coordinate such that the perception bitmap is centered. */
            {
                int car_center_x = (int)(car_x * PX_PER_METER);
                int car_center_y = (int)(car_y * PX_PER_METER);
                int px = car_center_x - PERCEPTION_VIEW_SIZE / 2;
                int py = car_center_y - PERCEPTION_VIEW_SIZE / 2;
                draw_sprite(display_buffer, perception_bitmap, px, py);
            }
            /* Blit the composed frame to the screen */
            blit(display_buffer, screen, 0, 0, 0, 0, XMAX, YMAX);
        pthread_mutex_unlock(&draw_mutex);

        wait_for_period(task_id);
    }
    return NULL;
}

/* --------------------------*/
/*   SIMULATION FUNCTIONS   */
/* --------------------------*/

/*
   LiDAR simulation:
   For each angle from 0° to 359°, "ray-cast" until either the maximum range is reached
   or a cone is detected (by checking pixel colors on the screen).
*/
void lidar(float car_center_x, float car_center_y, detection *measures)
{
    for (int lidar_angle = 0; lidar_angle < 360; lidar_angle += angle_step)
    {
        float distance_measured = MAX_RANGE;
        int color_detected = -1;
        int stop = 0;
        for (float d = 0; d < MAX_RANGE && !stop; d += DIST_RES)
        {
            float x = car_center_x + d * cos(lidar_angle * DEG2RAD);
            float y = car_center_y + d * sin(lidar_angle * DEG2RAD);
            int x_px = (int)(x * PX_PER_METER);
            int y_px = (int)(y * PX_PER_METER);

            int pixel_color = getpixel(screen, x_px, y_px);
            if (pixel_color == makecol(254,221,0)) {  // yellow cone
                distance_measured = d;
                color_detected = makecol(254,221,0);
                stop = 1;
            }
            else if (pixel_color == makecol(46,103,248)) {  // blue cone
                distance_measured = d;
                color_detected = makecol(46,103,248);
                stop = 1;
            }
        }
        measures[lidar_angle].distance = distance_measured;
        measures[lidar_angle].color = color_detected;
    }
}

/*
   Reads keyboard input and updates the vehicle speed/steering,
   then calls vehicle_model() to update the position.
*/
void keyboard_control(float *car_x, float *car_y, int *car_angle)
{
    const float accel_step = 0.01;      // speed increment per key press
    const float steering_step = 0.05;   // steering increment (radians) per key press
    const float max_steering = 45 * DEG2RAD;
    static float speed = 0.0;
    static float steering = 0.0;

    if (key[KEY_UP])
        speed += accel_step;
    if (key[KEY_DOWN])
        speed -= accel_step;

    if (key[KEY_LEFT]) {
        steering += steering_step;
        if (steering > max_steering)
            steering = max_steering;
    }
    if (key[KEY_RIGHT]) {
        steering -= steering_step;
        if (steering < -max_steering)
            steering = -max_steering;
    }

    vehicle_model(car_x, car_y, car_angle, speed, steering);
}

/*
   Simple vehicle model using a bicycle (Ackermann) approximation.
*/
void vehicle_model(float *car_x, float *car_y, int *car_angle, float speed, float steering)
{
    const float dt = 0.1;
    const float wheelbase = 1.5;
    float theta = (*car_angle) * DEG2RAD;

    *car_x += speed * cos(-theta) * dt;
    *car_y += speed * sin(-theta) * dt;
    theta += (speed / wheelbase) * tan(steering) * dt;

    *car_angle = (int)(theta / DEG2RAD);
}

/* --------------------------*/
/*  AUXILIARY FUNCTIONS     */
/* --------------------------*/

/* Dummy implementation: Initialize all cones to “unused” */

void init_cones(cone* cones, int max_cones)
{
	for (int i = 0; i < max_cones; i++)
	{
		cones[i].x = 0;
		cones[i].y = 0;
		cones[i].color = -1;
	}
}

#define tmp_scale  1.5 
// [TODO] Need to be substituted with corrected px_per_meter
// Load cone positions (the YAML file must contain x,y in meters considering that
// the vehicle is 2 x 1 meters and the cones are 0.1 x 0.1 meters)

void load_cones_positions(const char *filename, cone *cones, int max_cones) 
{   
    printf("Loading cones from file: %s\n", filename);
    FILE *file = fopen(filename, "r");
    if (!file) {
        fprintf(stderr, "Error: Unable to open file %s\n", filename);
        return;
    }

    yaml_parser_t parser;
    yaml_event_t event;

    // Initialize YAML parser
    if (!yaml_parser_initialize(&parser)) {
        fprintf(stderr, "Error: Failed to initialize YAML parser.\n");
        fclose(file);
        return;
    }
    yaml_parser_set_input_file(&parser, file);

    int i = 0; // Current cone index
    int state = 0; // Parser state
    char current_key[256] = {0}; // Buffer to store the current key

    /*
        Parser States:
        0 - Start (expecting 'cones' key)
        1 - Found 'cones' key
        2 - Inside 'cones' sequence
        3 - Inside a cone mapping
    */

    while (1) {
        if (!yaml_parser_parse(&parser, &event)) {
            fprintf(stderr, "Error: Failed to parse YAML file.\n");
            break;
        }

        if (event.type == YAML_STREAM_END_EVENT) {
            yaml_event_delete(&event);
            break;
        }

        switch (event.type) {
            case YAML_SCALAR_EVENT:
                if (state == 0) {
                    // Expecting the 'cones' key
                    if (strcmp((char *)event.data.scalar.value, "cones") == 0) {
                        state = 1;
                    }
                }
                else if (state == 3) {
                    // Inside a cone mapping, expecting key or value
                    if (strlen(current_key) == 0) {
                        // Expecting a key
                        strncpy(current_key, (char *)event.data.scalar.value, sizeof(current_key) - 1);
                        current_key[sizeof(current_key) - 1] = '\0'; // Ensure null-termination
                    }
                    else {
                        // Expecting a value corresponding to the current key
                        if (strcmp(current_key, "x") == 0) {
                            cones[i].x = ( atof((char *)event.data.scalar.value) ) * tmp_scale;
                        } 
                        else if (strcmp(current_key, "y") == 0) { 
                            cones[i].y = ( atof((char *)event.data.scalar.value) ) * tmp_scale;
                        } 
                        else if (strcmp(current_key, "color") == 0) {
                            if (strcmp((char *)event.data.scalar.value, "yellow") == 0) {
                                cones[i].color = makecol(254, 221, 0); // Yellow color
                            } 
                            else if (strcmp((char *)event.data.scalar.value, "blue") == 0) {
                                cones[i].color = makecol(46, 103, 248); // Blue color
                            }
                            // Add more colors as needed
                        }
                        // Reset current_key for the next key-value pair
                        current_key[0] = '\0';
                    }
                }
                break;

            case YAML_SEQUENCE_START_EVENT:
                if (state == 1) {
                    // Found the start of the 'cones' sequence
                    state = 2;
                }
                break;

            case YAML_MAPPING_START_EVENT:
                if (state == 2) {
                    // Starting a new cone mapping
                    if (i >= max_cones) {
                        fprintf(stderr, "Warning: Maximum number of cones (%d) reached. Additional cones will be ignored.\n", max_cones);
                        yaml_event_delete(&event);
                        goto cleanup;
                    }
                    // Initialize the current cone
                    cones[i].x = 0.0f;
                    cones[i].y = 0.0f;
                    cones[i].color = 0; // Reset color to indicate it's being set
                    state = 3;
                }
                break;

            case YAML_MAPPING_END_EVENT:
                if (state == 3) {
                    // Finished reading a cone's data
                    state = 2; // Return to 'cones' sequence
                    i++; // Move to the next cone
                }
                break;

            default:
                break;
        }

        yaml_event_delete(&event);
    }

cleanup:
    yaml_parser_delete(&parser);
    fclose(file);
    
    printf("Total cones loaded: %d\n", i);
}
