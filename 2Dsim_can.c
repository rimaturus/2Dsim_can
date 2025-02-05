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
#define PERCEPTION_PERIOD 1
#define CONTROL_PERIOD 50
#define DISPLAY_PERIOD 34   // ~30Hz (1000/34 ≈ 29.41 Hz)

// DEADLINES (in milliseconds)
#define PERCEPTION_DEADLINE PERCEPTION_PERIOD
#define CONTROL_DEADLINE 50
#define DISPLAY_DEADLINE 17

// Priorities (lower number means higher priority)
#define PERCEPTION_PRIORITY 15
#define CONTROL_PRIORITY 20
#define DISPLAY_PRIORITY 25

// TASKs (using ptask.h notation)
void *perception_task(void *arg);   // LiDAR simulation
void *control_task(void *arg);      // Keyboard control
void *display_task(void *arg);      // View simulation

// Mutex for synchronizing access to shared bitmaps and state
pthread_mutex_t draw_mutex = PTHREAD_MUTEX_INITIALIZER;

// --------------------------------


// --------------------------------
// CONSTANTS
// --------------------------------
#define px_per_meter 1e2 // 100 pixels = 1 meters (1 px = 1 cm)
#define deg2rad 0.0174533 // degrees to radians

// Screen resolution (in pixels) == Window size
const int XMAX = ( 10 * px_per_meter ); // == 2000 px
const int YMAX = ( 10 * px_per_meter ); // == 1000 px

// CAR initial position
float 	car_x		= 4.5;      // in meters
float 	car_y		= 3.0;      // in meters
int		car_angle	= 90; 		// in degrees

// Global bitmaps
BITMAP	*background		= NULL;
BITMAP	*track      	= NULL;
BITMAP	*car			= NULL;

BITMAP	*perception		= NULL; // off-screen buffer for sensor view
BITMAP	*display_buffer	= NULL; // off-screen buffer for final compositing
// --------------------------------


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

const int   angle_step	= 1;		// deg
const float MAXrange	= 5;		//[m]
const float distance_resolution = 0.01; // [m] = 1 cm

const int   N_angles  = (int)(360 / angle_step);
const int   distance_steps = (int)(MAXrange / distance_resolution);

// Visualization parameters (perception view)
const int 	sliding_window 	= 30; // number of rays to plot in the perception view
const float	ignore_distance = 0.2; // ignore cones closer than this distance [m]
// --------------------------------

// --------------------------------
// AUXILIARY FUNCTIONS
// --------------------------------
float 	angle_rotation_sprite( float angle );
void	init_cones( cone *cones, int max_cones );
void	load_cones_positions( const char *filename, cone *cones, int max_cones );
void	lidar( float car_x, float car_y, detection *measures );
void	keyboard_control( float *car_x, float *car_y, int *car_angle );
void	vehicle_model( float *car_x, float *car_y, int *car_angle, float speed, float steering );


// --------------------------------
// MAIN FUNCTION
// --------------------------------
int main()
{
	printf("Starting sim...\n");

	// Initializing Allegro
	allegro_init(); // initialize graphics data structures
	install_keyboard(); // initialize the keyboard
	install_mouse(); // initialize the mouse
	
	// Initialize the graphics mode
	set_color_depth(32); // set the color depth to 8 bits for each of the RGB channels and 8 bits for the alpha channel (faster than 24 bit since it is aligned to 32 bits)

// Colors
const int grass_green = makecol(78,91,49); // army_green
const int asphalt_gray = makecol(128,126,120); // asphalt
const int white = makecol(255, 255, 255); // white
const int pink = makecol(255, 0, 255); // pink
	
	set_gfx_mode(GFX_AUTODETECT_WINDOWED, XMAX, YMAX, 0, 0); // enters the graphics mode (windowed) with resolution 1920x1080
	clear_to_color(screen, white); // clear the screen making all pixels to white

	// Create the off-screen display buffer
display_buffer = create_bitmap(XMAX, YMAX);
	clear_to_color(display_buffer, pink);

	draw_sprite(screen, display_buffer, 0, 0);
	clear_keybuf();
	readkey();

	// background
background = create_bitmap(XMAX, YMAX); 
	clear_bitmap(background);
	clear_to_color(background, grass_green);

	draw_sprite(screen, background, 0, 0);
	clear_keybuf();
	readkey();

	// track
track = create_bitmap(XMAX, YMAX);
	clear_bitmap(track);
	clear_to_color(track, asphalt_gray);
	
const int 	max_cones = 1000;
cone		cones[max_cones];
const char	filename[100] = "track/cones.yaml";

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
	clear_keybuf();
	readkey();

	car = load_bitmap("bitmaps/f1_car_05x.bmp", NULL); // load a bitmap image
	if (car == NULL) {
		printf("Error loading sprite\n");
		exit(1);
	}  	

	// car_x and car_y are the center of the car
	// so the top-left corner of the car bitmap is at (car_x - car->w/2, car_y - car->h/2)
	// those are needed to make the center of the sprite coincide with the car position
int car_x_px = (int)(car_x * px_per_meter);
int car_y_px = (int)(car_y * px_per_meter);

int car_bitmap_x = car_x_px - (car->w / 2);
int car_bitmap_y = car_y_px - (car->h / 2);


	rotate_scaled_sprite(
		screen, car, 
		(int)(car_bitmap_x), 
		(int)(car_bitmap_y), 
		ftofix(angle_rotation_sprite(car_angle)),  // deg to fixed point
		ftofix(1)
	);

	circlefill(screen, car_x_px, car_y_px, 3, makecol(0,255,0));
	circlefill(screen, (int)(car_bitmap_x), (int)(car_bitmap_y), 3, makecol(0,255,255));
	clear_keybuf();
	readkey();


	// Perception bitmap 
	// the bitmap size need to be 2x the max range of the LiDAR to 
	// contain all the possible detections
perception = create_bitmap(2*MAXrange*px_per_meter, 2*MAXrange*px_per_meter);
	clear_bitmap(perception);
	clear_to_color(perception, pink); // pink color to make it transparent (True color notation)
	
	// Drawing initial simulation
	pthread_mutex_lock(&draw_mutex);
		clear_to_color(display_buffer, white);
		
		draw_sprite(display_buffer, background, 0, 0);
		draw_sprite(display_buffer, track, 0, 0);

		// draw the car (use rotate_scaled_sprite to make the background transparent,
		// and to rotate the car according to the car_angle)
		rotate_scaled_sprite(
			screen, car, 
			car_bitmap_x, 
			car_bitmap_y, 
			ftofix(angle_rotation_sprite(car_angle)),  // deg to fixed point
			ftofix(1)
		);

		draw_sprite(
			display_buffer, 
			perception, 
			car_x_px - MAXrange*px_per_meter,
			car_y_px - MAXrange*px_per_meter
		);
	pthread_mutex_unlock(&draw_mutex);

	// Initialize the periodic task system (using SCHED_OTHER)
	// [TODO] Maybe we need to change the scheduling policy 
	ptask_init(SCHED_OTHER);

	// Create periodic tasks:
	// Task 1: Perception task
	// Task 2: Control task
	// Task 3: Display task

	if (task_create(1, perception_task, PERCEPTION_PERIOD, PERCEPTION_DEADLINE, PERCEPTION_PRIORITY, ACT) != 0) {
		fprintf(stderr, "Failed to create Perception Task\n");
		exit(EXIT_FAILURE);
	}

	if (task_create(2, control_task, CONTROL_PERIOD, CONTROL_DEADLINE, CONTROL_PRIORITY, ACT) != 0) {
		fprintf(stderr, "Failed to create Control Task\n");
		exit(EXIT_FAILURE);
	}

	if (task_create(3, display_task, DISPLAY_PERIOD, DISPLAY_DEADLINE, DISPLAY_PRIORITY, ACT) != 0) {
		fprintf(stderr, "Failed to create Display Task\n");
		exit(EXIT_FAILURE);
	}

	// Wait for tasks to terminate (they will exit when ESC is pressed)
	wait_for_task_end(1);
	wait_for_task_end(2);
	wait_for_task_end(3);

    printf("Exiting simulation...\n");
    clear_keybuf();
    readkey();
    allegro_exit();
    return 0;
}


// Periodic task functions (using ptask.h notation)
void *perception_task(void *arg)
{
	int task_id = get_task_index(arg);
	wait_for_activation(task_id);
	
	detection measures[N_angles]; // contain measurements of the LiDAR corresponding to each angle
	static	int start_angle = 0; // start angle for the sliding window

	while (!key[KEY_ESC])
	{
		// LiDAR simulation
		// detection measures[N_angles];
		lidar(car_x, car_y, &measures);

		// render the perception view
		pthread_mutex_lock(&draw_mutex);
			clear_to_color(perception, makecol(255, 0, 255)); // pink color to make it transparent (True color notation)
			
			circlefill(
				perception, 
				perception->w / 2, 
				perception->h / 2, 
				3, 
				makecol(0,255,0)
			); 

			for (int i = 0; i < sliding_window; i += angle_step)
			{   
				int lidar_angle = (start_angle + i) % 360;
				// printf("Lidar angle: %d \t %f\n", lidar_angle, measures[lidar_angle].distance);

				// plot lines from car to detected point
				float cos_angle = cos(lidar_angle * deg2rad);
				float sin_angle = sin(lidar_angle * deg2rad);

				// There starts the graphical part:
				// since the perception window is centered on the car, we need to calculate the
				// coordinates of the detected point in the perception window
				// so the (car_x, car_y) coordinates in global frame 
				// are mapped to the center of the perception window (perception_cx, perception_cy)
				float perception_cx = (float)(perception->w/2)/px_per_meter;
				float perception_cy = (float)(perception->h/2)/px_per_meter;

				float x0 = perception_cx + cos_angle * ignore_distance;
				float y0 = perception_cy + sin_angle * ignore_distance;

				float x_detection = perception_cx + (measures[lidar_angle].distance * cos_angle);
				float y_detection = perception_cy + (measures[lidar_angle].distance * sin_angle);
				
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

					circle(
						perception, 
						(int)(x_detection * px_per_meter), 
						(int)(y_detection * px_per_meter), 
						3, 
						measures[lidar_angle].color
					);
				}
			}
		pthread_mutex_unlock(&draw_mutex);

		// update the start angle for the sliding window
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
			clear_to_color(display_buffer, makecol(255,0,255));
			
			draw_sprite(display_buffer, background, 0, 0);
			draw_sprite(display_buffer, track, 0, 0);

			// draw the car (use rotate_scaled_sprite to make the background transparent,
			// and to rotate the car according to the car_angle)
			rotate_scaled_sprite(
				display_buffer, car, 
				(int)(car_x * px_per_meter) - (int)(car->w / 2), 
				(int)(car_y * px_per_meter) - (int)(car->h / 2), 
				ftofix(angle_rotation_sprite(car_angle)),  // deg to fixed point
				ftofix(1)
			);

			draw_sprite(
				display_buffer, 
				perception, 
				(int)(car_x * px_per_meter) - (int)(MAXrange)*px_per_meter,
				(int)(car_y * px_per_meter) - (int)(MAXrange)*px_per_meter
			);

			blit(display_buffer, screen, 0, 0, 0, 0, XMAX, YMAX);
		pthread_mutex_unlock(&draw_mutex);

        wait_for_period(task_id);
    }
    return NULL;
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
		for (float distance = ignore_distance; distance < MAXrange; distance += distance_resolution)
		{
			// Calculate the x and y coordinates of the pixel at the current distance and angle
			float x = car_x + ( distance * cos((float)(lidar_angle) * deg2rad) );
			float y = car_y + ( distance * sin((float)(lidar_angle) * deg2rad) );

			int x_px = x * px_per_meter;
			int y_px = y * px_per_meter;

			//putpixel(screen, x_px, y_px, makecol(255, 0, 0));

			if (stop_distance != 1)
			{
				if (getpixel(track, x_px, y_px) == makecol(254, 221, 0)) // yellow
				{
					// printf("Cone detected\n");
					measures[lidar_angle].distance = distance; // convert to meters
					measures[lidar_angle].color = makecol(254, 221, 0); // yellow
					stop_distance = 1;  // cone detected, stop the loop
				}
				else if (getpixel(track, x_px, y_px) == makecol(46, 103, 248)) // blue
				{   
					// printf("Cone detected\n");
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

float angle_rotation_sprite(float angle)
{
    return 64.0f - 128.0f * angle / 180.0f;
}