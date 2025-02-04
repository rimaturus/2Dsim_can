#include <stdio.h>
#include <stdlib.h>
#include <allegro.h>
#include <math.h>

// --------------------------------
// YAML
// --------------------------------
#include <yaml.h>
#include <string.h>
// --------------------------------

// --------------------------------
// PTHREADs
// --------------------------------
#include <pthread.h>
#include <sched.h>
#include <time.h>
#include "ptask/ptask.h" // custom functions for periodic task

// TASK PERIODS (in milliseconds)
#define PERCEPTION_PERIOD 100
#define CONTROL_PERIOD 50
#define DISPLAY_PERIOD 17   // ~60Hz (1000/17 ≈ 58.8 Hz)

// DEADLINES (in milliseconds)
#define PERCEPTION_DL 100
#define CONTROL_DL 50
#define DISPLAY_DL 17

// Priorities (lower number means higher priority)
#define PERCEPTION_PRIORITY 15
#define CONTROL_PRIORITY 20
#define DISPLAY_PRIORITY 25
// --------------------------------


#define PERCEPTION
#define CONTROL
// --------------------------------
// CONSTANTS
// --------------------------------
#define px_per_meter 1e2 // 100 pixels = 1 meters (1 px = 1 cm)
#define deg2rad 0.0174533 // degrees to radians

// TRACK CONES
typedef struct {
	float x;
	float y;
	int color;
} cone;

// --------------------------------
// LiDAR CONFIGURATION
// --------------------------------
typedef struct {
    int x_px;
    int y_px;
    float distance;
    int color;
} detection;

const int   angle_step = 1;     // deg
const float MAXrange = 10;      //[m]
const float distance_resolution = 0.01; // [m] = 1 cm

const int   N_angles  = (int)(360 / angle_step);
const int   distance_steps = (int)(MAXrange / distance_resolution);
// --------------------------------

// --------------------------------
// AUXILIARY FUNCTIONS
// --------------------------------
float angle_rotation_sprite(float angle);
void    init_cones( cone *cones, int max_cones );
void    load_cones_positions( const char *filename, cone *cones, int max_cones );
void    lidar( float car_x, float car_y, detection *measures );
void    keyboard_control( float *car_x, float *car_y, int *car_angle );
void    vehicle_model( float *car_x, float *car_y, int *car_angle, float speed, float steering );

int main()
{
	printf("Starting sim...\n");

	// Initializing Allegro
	allegro_init(); // initialize graphics data structures
	install_keyboard(); // initialize the keyboard
	install_mouse(); // initialize the mouse
	
	// Initialize the graphics mode
	set_color_depth(32); // set the color depth to 8 bits for each of the RGB channels and 8 bits for the alpha channel (faster than 24 bit since it is aligned to 32 bits)
	const int XMAX = ( 10 * px_per_meter );
    const int YMAX = ( 10 * px_per_meter ); // screen resolution


	// Colors
	const int grass_green = makecol(78,91,49); // army_green
	const int asphalt_gray = makecol(128,126,120); // asphalt
	const int white = makecol(255, 255, 255); // white
	
	set_gfx_mode(GFX_AUTODETECT_WINDOWED, XMAX, YMAX, 0, 0); // enters the graphics mode (windowed) with resolution 1920x1080
	clear_to_color(screen, white); // clear the screen making all pixels to white

	// Initialize the scenes
	BITMAP *background;
	BITMAP *track;
    BITMAP *car;
    BITMAP *perception;

	// background
	background = create_bitmap(XMAX, YMAX);
	clear_bitmap(background);
	clear_to_color(background, grass_green);
    draw_sprite(screen, background, 0, 0);

	// track
	track = create_bitmap(XMAX, YMAX);
	clear_bitmap(track);
	clear_to_color(track, asphalt_gray);
	
    const int max_cones = 1000;
	cone 	cones[max_cones];
    const char    filename[100] = "track/cones.yaml";
			init_cones( &cones, max_cones );
			load_cones_positions( &filename, &cones, max_cones );
            printf("Cones loaded\n");

            // plot cones
            for (int i = 0; i < 1000; i++)
            {
                if (cones[i].color != -1) // plot only track cones
                {
                    circlefill(
                        track, 
                        cones[i].x, 
                        cones[i].y, 
                        0.05 * px_per_meter, // radius = 5 cm
                        cones[i].color
                    );
                }
            }

    draw_sprite(screen, track, 0, 0);

    // Vehicle
    float car_x = 4.5;
    float car_y = 3;
    int car_angle = 27; // deg

    car = load_bitmap("bitmaps/f1_car_pink.bmp", NULL); // load a bitmap image
    if (car == NULL) {
        printf("Error loading sprite\n");
        exit(1);
    }  

    // // make trasnparent background for the car
    // [TODO] Need to fix the mask to have more clean borders (may try HSV)

    // int black = makecol(0, 0, 0);
    // int pink = makecol(255, 0, 255);
    // BITMAP *car_mask = create_bitmap(car->w, car->h);

    // for (int x = 0; x < car->w; x++)
    //     for (int y = 0; y < car->h; y++)
    //     {
    //         int c = getpixel(car, x, y);
    //         if (c == white) c = black;
    //         putpixel(car_mask, x, y, c);
    //     }
    
    // save_bitmap("bitmaps/f1_car_pink.bmp", car_mask, NULL);


    //blit(car, screen, 0, 0, carpos_x, carpos_y, car->w, car->h); // draw the sprite on the screen at (300, 300)
    //draw_sprite(screen, car, carpos_x, carpos_y); // draw the sprite on the screen at (300, 300)
    rotate_scaled_sprite(
        screen, car, 
        (int)(car_x * px_per_meter), 
        (int)(car_y * px_per_meter), 
        ftofix(angle_rotation_sprite(car_angle)),  // deg to fixed point
        ftofix(0.5)); // draw the sprite on the screen at (300, 300)
    // choose this one to have the transparent background


    // Sensors run
    detection     measures[N_angles]; // results



    // Simulation loop
    do {
        printf("Running sim...\n");
        // Update the car position
        float center_car_x = car_x + (float)(car->w / 4) / px_per_meter;
        float center_car_y = car_y + (float)(car->h / 4) / px_per_meter;

        

#ifdef PERCEPTION
        // Perception
        lidar(center_car_x, center_car_y, &measures);

        // Plot the detected cones
        for (int lidar_angle = 0; lidar_angle < 360; lidar_angle += angle_step)
        {   
            printf("Lidar angle: %d \t %f\n", lidar_angle, measures[lidar_angle].distance);

            // plot lines from car to detected point
            float cos_angle = cos(lidar_angle * deg2rad);
            float sin_angle = sin(lidar_angle * deg2rad);

            float ignore_distance = 0.5; // ignore the first 0.5 meters
            float x0 = center_car_x + cos_angle * ignore_distance;
            float y0 = center_car_y + sin_angle * ignore_distance;

            float x_detection = center_car_x + (measures[lidar_angle].distance * cos_angle);
            float y_detection = center_car_y + (measures[lidar_angle].distance * sin_angle);
            
            // create the perception bitmap as large as the max view range
            perception = create_bitmap(MAXrange*px_per_meter, MAXrange*px_per_meter);
            clear_bitmap(perception);
            clear_to_color(perception, makecol(255, 0, 255));
            
            if (measures[lidar_angle].color == -1) // no cone detected
            {
                line(
                    perception, 
                    (int)(x0 * px_per_meter), 
                    (int)(y0 * px_per_meter), 
                    (int)(x_detection * px_per_meter), 
                    (int)(y_detection * px_per_meter), 
                    makecol(255, 0, 0)
                );
            }
            else
            {
                line(
                    perception, 
                    (int)(x0 * px_per_meter), 
                    (int)(y0 * px_per_meter), 
                    (int)(x_detection * px_per_meter), 
                    (int)(y_detection * px_per_meter), 
                    measures[lidar_angle].color);
            }

            masked_blit(
                perception, screen, 
                0, 0, // source: center of the perception bitmap
                0, 0,  // destination: center of the vehicle
                perception->w, perception->h
            );

            // clear_keybuf();
            // readkey(); // wait for a key press to close the window

            if ((lidar_angle % 30) == 0){   // plot only 17 angles for better visualization at each iteration
                printf("%d\n", (lidar_angle % 10) );
                clear_bitmap(screen);

                draw_sprite(screen, track, 0, 0);
                rotate_scaled_sprite(
                    screen, car, 
                    (int)(car_x * px_per_meter), 
                    (int)(car_y * px_per_meter), 
                    ftofix(angle_rotation_sprite(car_angle)),  // deg to fixed point
                    ftofix(0.5)
                );
            }

            
        }
#endif /* PERCEPTION */

#ifdef CONTROL
        // Control
        keyboard_control(&car_x, &car_y, &car_angle);

        clear_keybuf();
	    readkey();

        draw_sprite(screen, track, 0, 0);
        rotate_scaled_sprite(
            screen, car, 
            (int)(car_x * px_per_meter), 
            (int)(car_y * px_per_meter), 
            ftofix(angle_rotation_sprite(car_angle)),  // deg to fixed point
            ftofix(0.5)
        );

#endif /* CONTROL */



    } while (!key[KEY_ESC]); // exit the loop when the ESC key is pressed








	printf("Press any key to exit...\n");
	clear_keybuf();
	readkey(); // wait for a key press to close the window
	allegro_exit(); // deallocate graphics data structures

	return 0;
}

// Sensors
// LiDAR
void    lidar(float car_x, float car_y, detection *measures)
{
    int stop_distance; 

    // Check for each angle in the range [0, 360] with a step of angle_step
    for (int lidar_angle = 0; lidar_angle < 360; lidar_angle += angle_step)
    {
        int   current_distance = 0; // initialize distance at 0

        // Initialize the measure with the maximum range and no color
        measures[lidar_angle].distance = MAXrange;
        measures[lidar_angle].color = -1; // cone not detected

        stop_distance = 0;

        // Check each pixel in the range [0, MAXrange] with a step of distance_resolution
        for (float distance = 0; distance < MAXrange; distance += distance_resolution)
        {
            // Calculate the x and y coordinates of the pixel at the current distance and angle
            float x = car_x + ( distance * cos((float)(lidar_angle) * deg2rad) );
            float y = car_y + ( distance * sin((float)(lidar_angle) * deg2rad) );

            int x_px = x * px_per_meter;
            int y_px = y * px_per_meter;

            //putpixel(screen, x_px, y_px, makecol(255, 0, 0));

            if (stop_distance != 1)
            {
                if (getpixel(screen, x_px, y_px) == makecol(254, 221, 0)) // yellow
                {
                    printf("Cone detected\n");
                    measures[lidar_angle].distance = distance; // convert to meters
                    measures[lidar_angle].color = makecol(254, 221, 0); // yellow
                    stop_distance = 1;  // cone detected, stop the loop
                }
                else if (getpixel(screen, x_px, y_px) == makecol(46, 103, 248)) // blue
                {   
                    printf("Cone detected\n");
                    measures[lidar_angle].distance = distance; // convert to meters
                    measures[lidar_angle].color = makecol(46, 103, 248); // blue
                    stop_distance = 1;  // cone detected, stop the loop
                }
            }            
        }
    }
}

// Control
void vehicle_model(float *car_x, float *car_y, int *car_angle, float speed, float steering)
{
// Simulation parameters
const float     dt = 0.1;             // time step
const float     wheelbase = 1.5;      // distance between front and rear axles [m]

float theta;
    
    // Update vehicle position using a simple bicycle model (Ackermann steering)
    theta = (*car_angle) * deg2rad;              // convert current heading to radians
    *car_x += speed * cos(-theta) * dt;                   // update x position
    *car_y += speed * sin(-theta) * dt;                   // update y position
    theta += (speed / wheelbase) * tan(steering) * dt;   // update heading

    // Store updated heading in degrees
    *car_angle = (int)(theta / deg2rad);

}

void keyboard_control(float *car_x, float *car_y, int *car_angle)
{
const float     accel_step = 0.01;    // speed increment per key press
const float     steering_step = 0.05; // steering increment in radians per key press
const float     max_steering = 45 * deg2rad;     // maximum steering angle in radians

// Persistent state across calls
static float    speed = 0.0;         // current speed in m per step
static float    steering = 0.0;      // current steering angle in radians

    // Adjust speed
    if (key[KEY_UP])
    {
        speed += accel_step;
    }
    if (key[KEY_DOWN])
    {
        speed -= accel_step;
        //if (speed < 0) speed = 0; // prevent going backwards
    }

    // Adjust steering angle
    if (key[KEY_LEFT])
    {
        steering += steering_step;
        if (steering > max_steering)
            steering = max_steering;
    }
    if (key[KEY_RIGHT])
    {
        steering -= steering_step;
        if (steering < -max_steering)
            steering = -max_steering;
    }

    // Motion model of the vehicle
    vehicle_model(car_x, car_y, car_angle, speed, steering);
}



// Auxiliary functions

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