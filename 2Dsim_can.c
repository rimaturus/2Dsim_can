/**
 * \file 2Dsim_can.c
 * \brief 2D simulation of a full autonomous driving FSAE vehicle with Allegro, PThreads, and YAML parsing.
 *
 * This file contains the main entry point of a small simulation program. It
 * uses Allegro for graphics, YAML for loading track cone data, and a
 * ptask library for periodic tasks created during the RTS course. The code simulates a simple FSAE vehicle (car)
 * that drives around a track populated with cones that delimits the track limits. Some tasks (Perception,
 * Control, Display) run periodically at a quasi-realistic rate.
 *
 * \author Edoardo Caciorgna
 * \date 2025-02-07
 */

/**
 * \mainpage "2Dsim_can"
 *
 * \section intro_sec Introduction
 * This documentation describes a small car simulation that uses:
 * - [Allegro](https://liballeg.org/) for graphics
 * - [LibYAML](https://github.com/yaml/libyaml) for parsing configuration files
 * - [pthread](https://man7.org/linux/man-pages/man7/pthreads.7.html) for concurrency
 * - A custom periodic tasks library (`ptask`)
 *
 * \section build_sec Build
 * Typical compilation might look like:
 * \code{.sh}
 * gcc -g -O3 -o 2Dsim_can 2Dsim_can.c ptask/ptask.c `allegro-config --libs` -lyaml -lm -lpthread
 * \endcode
 *
 * \section usage_sec Usage
 * Run the resulting executable using `sudo` to enable RT clock. Use arrow keys to control the car, and ESC to quit.
 *
 * \dot
 * digraph call_graph {
 *   rankdir=LR;
 *   "main()" -> "task_create()";
 *   "main()" -> "wait_for_task_end()";
 *   "task_create()" -> "perception_task()";
 *   "task_create()" -> "control_task()";
 *   "task_create()" -> "display_task()";
 *   "perception_task()" -> "lidar()";
 *   "perception_task()" -> "mapping()";
 *   "control_task()" -> "keyboard_control()";
 *   "keyboard_control()" -> "vehicle_model()";
 *   "display_task()" -> "Update screen";
 * }
 * \enddot
 */

#include <stdio.h>
#include <stdlib.h>
#include <allegro.h>
#include <math.h>

/* --------------------------------
 * YAML
 * -------------------------------- */
#include <yaml.h>
#include <string.h>

/* --------------------------------
 * PTHREADs
 * -------------------------------- */
#include <pthread.h>
#include <sched.h>
#include <time.h>
#include "ptask/ptask.h" // Custom functions for periodic task

/* TASK PERIODS (in milliseconds) */
#define PERCEPTION_PERIOD    1   /**< Period of Perception Task [ms] */
#define TRAJECTORY_PERIOD    10   /**< Period of Trajectory Planning Task [ms] */
#define CONTROL_PERIOD       20  /**< Period of Control Task [ms] */
#define DISPLAY_PERIOD       37  /**< Period of Display Task [ms] (~30 Hz) */

/* DEADLINES (in milliseconds) */
#define PERCEPTION_DEADLINE  PERCEPTION_PERIOD	/**< Deadline = Period for Perception Task */
#define TRAJECTORY_DEADLINE  TRAJECTORY_PERIOD	/**< Deadline = Period for Trajectory Planning Task */
#define CONTROL_DEADLINE     CONTROL_PERIOD		/**< Deadline = 2 * Period for Control Task */
#define DISPLAY_DEADLINE     DISPLAY_PERIOD		/**< Deadline = 2 * Period for Display Task */

/* PRIORITIES (lower number = higher priority) */
#define PERCEPTION_PRIORITY	15
#define TRAJECTORY_PRIORITY	20
#define CONTROL_PRIORITY    25
#define DISPLAY_PRIORITY    30

/** 
 * \brief Perception task (LiDAR simulation).
 *
 * This function simulates a LiDAR by periodically scanning the environment
 * around the car. It populates global arrays with the detected cone positions
 * and updates an off-screen bitmap that shows the perception view.
 *
 * \param arg Pointer to the task ID or any user-defined data (unused here).
 * \return A NULL pointer (no specific return value).
 */
void *perception_task(void *arg);

/**
 * @brief Calculates and manages object trajectory in a 2D simulation
 * 
 * This task is responsible for calculating and updating the trajectory of objects
 * in a 2-dimensional simulation environment. It runs in a separate thread to 
 * handle continuous trajectory computations.
 * 
 * @param arg Pointer to thread arguments (void pointer for flexibility)
 * @return void* Return value of the thread (typically NULL)
 */
void *trajectory_task(void *arg);

/**
 * \brief Control task (keyboard input).
 *
 * This function periodically reads the keyboard to adjust the speed and
 * steering of the car. It updates global state variables representing
 * the car's position and orientation.
 *
 * \param arg Pointer to the task ID or any user-defined data (unused here).
 * \return A NULL pointer (no specific return value).
 */
void *control_task(void *arg);

/**
 * \brief Display task (View simulation).
 *
 * This function periodically renders the scene to the screen by compositing
 * a background, track, the car sprite, and a LiDAR/perception overlay.
 *
 * \param arg Pointer to the task ID or any user-defined data (unused here).
 * \return A NULL pointer (no specific return value).
 */
void *display_task(void *arg);

/** 
 * \brief Global mutex for synchronizing drawing operations.
 *
 * Multiple tasks (Perception, Display) might attempt to draw at the same
 * time, so this mutex helps prevent race conditions and partial updates.
 */
pthread_mutex_t draw_mutex = PTHREAD_MUTEX_INITIALIZER;

/* --------------------------------
 * CONSTANTS
 * -------------------------------- */
#define px_per_meter 100       /**< Conversion scale: 100 px = 1 meter (1 px = 1 cm) */
#define deg2rad 0.0174533      /**< Conversion factor from degrees to radians */

/** 
 * \brief Screen resolution (in pixels).
 *
 * The window size is set to 10 meters x 10 meters in simulation coordinates.
 */
const char* text = "2D FSAE sim by rimaturus"; // Title of the window

const int XMAX = (19 * px_per_meter); /**< 10 m in x-direction, scaled */
const int YMAX = (12 * px_per_meter); /**< 10 m in y-direction, scaled */

/* CAR initial position */
float  car_x        = 4.5f;   /**< Car's initial X position [m] */
float  car_y        = 3.0f;   /**< Car's initial Y position [m] */
int    car_angle    = 0;     /**< Car's initial heading [deg] */

/** \brief Global bitmap for background (grass). */
BITMAP *background      = NULL;
/** \brief Global bitmap for track (asphalt + cones). */
BITMAP *track           = NULL;
/** \brief Global bitmap for car sprite. */
BITMAP *car             = NULL;
/** \brief Global bitmap for perception (LiDAR) view. */
BITMAP *perception      = NULL;
/** \brief Off-screen buffer for final compositing. */
BITMAP *display_buffer  = NULL;

/** 
 * \struct cone
 * \brief Represents a track cone on the field.
 */
typedef struct {
    float x;      /**< X position of the cone (in meters, converted to px when drawn) */
    float y;      /**< Y position of the cone */
    int   color;  /**< Color (in Allegro color format) */
} cone;

const float cone_radius = 0.05f; /**< Cone radius in meters (5 cm) */

int grass_green, asphalt_gray, white, pink;
int yellow, blue;

/* --------------------------------
 * LiDAR CONFIGURATION
 * -------------------------------- */

/** 
 * \struct pointcloud
 * \brief Represents a single LiDAR measurement for a particular angle.
 */
typedef struct {
    float point_x;   /**< X coordinate of the detected point */
    float point_y;   /**< Y coordinate of the detected point */
    float distance;  /**< Detected distance in meters */
    int   color;     /**< Allegro color if a cone is detected, -1 otherwise */
} pointcloud;

const int   angle_step           = 1;        /**< Angular resolution of LiDAR [deg] */
const float MAXrange             = 10.0f;     /**< Maximum LiDAR range [m] */
const float distance_resolution  = 0.01f;    /**< Distance step for LiDAR [m] */
const int   N_angles             = 360;      /**< Number of possible angles [0..359] */
const int   distance_steps       = (int)(MAXrange / distance_resolution);      /**< Discrete steps in the range (MAXrange / distance_resolution) */

pointcloud measures[360]; /**< Global array storing the current LiDAR scan data */

/**
 * \brief Starting angle for the "sliding window" of LiDAR visualization.
 *
 * Only a subset of rays near this angle are drawn in the perception view to
 * avoid clutter and reduce rendering cost.
 */
int start_angle = 0;
const int   sliding_window  = 360;  /**< Number of LiDAR rays to plot in the perception view */
const float ignore_distance = 0.2f; /**< Ignore cones closer than this distance [m] */

/* --------------------------------
 * CONE DETECTION
 * -------------------------------- */
#define MAX_DETECTED_CONES  360 /**< Maximum number of cones that can be detected in one LiDAR pass */
#define MAX_POINTS_PER_CONE 180 /**< Maximum number of LiDAR hits that can belong to one cone's border */

cone detected_cones[MAX_DETECTED_CONES]; /**< Global array for storing detected cones */

typedef struct {
	float x;
	float y;
} waypoint;

waypoint trajectory[MAX_DETECTED_CONES]; // trajectory

/** 
 * \struct cone_border
 * \brief Keeps track of LiDAR angles that belong to the same cone border.
 */
typedef struct {
    int angles[MAX_POINTS_PER_CONE]; /**< Indices (angles) in the LiDAR scan that map to the same cone border */
    int color;                       /**< Color of the cone */
} cone_border;

/* --------------------------------
 * AUXILIARY FUNCTIONS
 * -------------------------------- */

/**
 * \brief Converts a car angle from degrees to an Allegro fixed point for sprite rotation.
 * \param angle The angle in degrees.
 * \return The fixed-point angle for Allegro's `rotate_scaled_sprite` function.
 */
float angle_rotation_sprite(float angle);

/**
 * \brief Initializes a list of cones with default values (-1 color).
 * \param cones Array of `cone` structures to initialize.
 * \param max_cones Size of the array.
 */
void init_cones(cone *cones, int max_cones);

/**
 * \brief Loads cone positions from a YAML file.
 * \param filename Path to the YAML file.
 * \param cones Array of `cone` structures to store the loaded data.
 * \param max_cones Size of the array.
 */
void load_cones_positions(const char *filename, cone *cones, int max_cones);

/**
 * \brief Performs a LiDAR scan around the car and updates global `measures`.
 * \param car_x Car's X position in meters.
 * \param car_y Car's Y position in meters.
 * \param measures Global array that will be populated with measurement data.
 */
void lidar(float car_x, float car_y, pointcloud *measures);

/**
 * \brief Identifies cone borders from LiDAR `measures` and computes cone centers.
 * \param car_x Car's X position in meters.
 * \param car_y Car's Y position in meters.
 * \param car_angle Car's heading in degrees.
 * \param detected_cones Array of `cone` structures to store the newly detected cones.
 */
void mapping(float car_x, float car_y, int car_angle, cone *detected_cones);

/**
 * \brief Reads the current state of the keyboard and updates the car's position/orientation.
 * \param car_x Pointer to the car's X position.
 * \param car_y Pointer to the car's Y position.
 * \param car_angle Pointer to the car's heading in degrees.
 */
void keyboard_control(float *car_x, float *car_y, int *car_angle);

/**
 * \brief Updates the vehicle's position based on speed and steering, using a simple bicycle model.
 * \param car_x Pointer to car's X coordinate.
 * \param car_y Pointer to car's Y coordinate.
 * \param car_angle Pointer to car's heading (in degrees).
 * \param speed Current speed of the vehicle.
 * \param steering Current steering angle (in radians).
 */
void vehicle_model(float *car_x, float *car_y, int *car_angle, float speed, float steering);

/**
 * \brief Helper function to classify a newly detected LiDAR point into an existing cone or create a new cone.
 * \param angle The LiDAR angle index.
 * \param new_point_x X coordinate of the detected point.
 * \param new_point_y Y coordinate of the detected point.
 * \param color Color of the detected cone (Allegro format).
 * \param cone_borders Array of `cone_border` structures to update.
 */
void 	check_nearest_point(int angle, float new_point_x, float new_point_y, int color, cone_border *cone_borders);

/**
 * \brief Update trajectory planned based on detected cones.
 * \param car_x Pointer to car's X coordinate.
 * \param car_y Pointer to car's Y coordinate.
 * \param car_angle Pointer to car's heading (in degrees).
 * \param detected_cones Array of `cone` structures to store the newly detected cones.
 * \param trajectory Array of `waypoints` that represents trajectory.
 */
void	trajectory_planning(float car_x, float car_y, float car_angle, cone *detected_cones, waypoint *trajectory);

/* --------------------------------
 * MAIN FUNCTION
 * -------------------------------- */

/**
 * \brief Entry point of the simulation.
 *
 * Initializes Allegro, loads resources (background, track, cones, car sprite),
 * creates periodic tasks for Perception, Control, and Display, and waits
 * until the user presses ESC to exit.
 *
 * \return 0 on success, non-zero otherwise.
 */
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
	grass_green = makecol(78,91,49); // army_green
	asphalt_gray = makecol(128,126,120); // asphalt
	white = makecol(255, 255, 255); // white
	pink = makecol(255, 0, 255); // pink
	yellow = makecol(254, 221, 0); // yellow for cones
	blue = makecol(46, 103, 248); // blue for cones
		
		if (set_gfx_mode(GFX_AUTODETECT_WINDOWED, XMAX, YMAX, 0, 0) != 0) {
			// If windowed mode fails, try fullscreen
			if (set_gfx_mode(GFX_AUTODETECT, XMAX, YMAX, 0, 0) != 0) {
			allegro_message("Error setting graphics mode\n%s\n", allegro_error);
			return -1;
			}
		}
		set_window_title("2D FSAE Simulation"); // Optional: sets window title
		set_display_switch_mode(SWITCH_BACKGROUND); // Allows window to be minimized/switched

		clear_to_color(screen, white); // clear the screen making all pixels to white

		// Create the off-screen display buffer
	display_buffer = create_bitmap(XMAX, YMAX);
		clear_to_color(display_buffer, pink);

		draw_sprite(screen, display_buffer, 0, 0);
		clear_keybuf();
		//readkey();

		// background
	background = create_bitmap(XMAX, YMAX); 
		clear_bitmap(background);
		clear_to_color(background, grass_green);

		draw_sprite(screen, background, 0, 0);
		clear_keybuf();
		//readkey();

		// track
	track = create_bitmap(XMAX, YMAX);
		clear_bitmap(track);
		clear_to_color(track, asphalt_gray);
		
	const int 	max_cones = 1000;
	cone		cones[max_cones];
	const char	filename[100] = "track/cones.yaml";

		init_cones(cones, max_cones);
		load_cones_positions(filename, cones, max_cones);
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
					cone_radius * px_per_meter, // radius = 5 cm
					cones[i].color
				);
			}
		}

		draw_sprite(screen, track, 0, 0);
		clear_keybuf();
		//readkey();

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
		//readkey();


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

		if (task_create(2, trajectory_task, TRAJECTORY_PERIOD, TRAJECTORY_DEADLINE, TRAJECTORY_PRIORITY, ACT) != 0) {
			fprintf(stderr, "Failed to create Trajectory Task\n");
			exit(EXIT_FAILURE);
		}

		if (task_create(3, control_task, CONTROL_PERIOD, CONTROL_DEADLINE, CONTROL_PRIORITY, ACT) != 0) {
			fprintf(stderr, "Failed to create Control Task\n");
			exit(EXIT_FAILURE);
		}

		if (task_create(4, display_task, DISPLAY_PERIOD, DISPLAY_DEADLINE, DISPLAY_PRIORITY, ACT) != 0) {
			fprintf(stderr, "Failed to create Display Task\n");
			exit(EXIT_FAILURE);
		}

		// Wait for tasks to terminate (they will exit when ESC is pressed)
		wait_for_task_end(1);
		wait_for_task_end(2);
		wait_for_task_end(3);
		wait_for_task_end(4);

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
	
	// static	int start_angle = 0; // start angle for the sliding window

	while (!key[KEY_ESC])
	{
		// struct timespec iter_start, iter_end;
        // clock_gettime(CLOCK_MONOTONIC, &iter_start);

		// LiDAR simulation
		// detection measures[N_angles];
		lidar(car_x, car_y, measures);


		for (int i = 0; i < MAX_DETECTED_CONES; i++){
			detected_cones[i].x = -1;
			detected_cones[i].y = -1;
			detected_cones[i].color = -1;
		}

		mapping(car_x, car_y, car_angle, detected_cones); // Pass the address of first element

		trajectory_planning(car_x, car_y, car_angle, detected_cones, trajectory);

		// render the perception view
		pthread_mutex_lock(&draw_mutex);
			clear_to_color(perception, pink); // pink color to make it transparent (True color notation)

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
				}
			}

			int detected_cone_idx = 0;
			//while (detected_cones[detected_cone_idx].color != -1){
			while (detected_cone_idx < MAX_DETECTED_CONES-1){
				if (detected_cones[detected_cone_idx].color == -1){
					detected_cone_idx++;
					break;
				}
				else {
					// printf("Detected cone at: %f, %f\n", detected_cones[detected_cone_idx].x, detected_cones[detected_cone_idx].y);
					circlefill(
						perception, 
						(int)(detected_cones[detected_cone_idx].x * px_per_meter) - (int)(car_x * px_per_meter - MAXrange*px_per_meter), 
						(int)(detected_cones[detected_cone_idx].y * px_per_meter) - (int)(car_y * px_per_meter - MAXrange*px_per_meter), 
						3, 
						makecol(255, 0, 0) //detected_cones[detected_cone_idx].color
					);
					
					
					char* text = (char*)malloc(10);  // Allocate space for the string
					snprintf(text, 10, "%d", detected_cone_idx);  // Convert int to string

					textout_ex(
						perception, 
						font, 
						text, 
						(int)(trajectory[detected_cone_idx].x * px_per_meter) - (int)(car_x * px_per_meter - MAXrange*px_per_meter), 
						(int)(trajectory[detected_cone_idx].y * px_per_meter) - (int)(car_y * px_per_meter - MAXrange*px_per_meter), 
						makecol(255, 0, 0), 
						makecol(255, 255, 255)
					);

					free(text);  // Free the memory
					line(
						car, 
						(int)(car->w/2),
						(int)(car->h/2),
						(int)(car->w/2),
						(int)(car->h/2) - 1000,
						makecol(0, 255, 0)
					);

					circlefill(
						perception,
						(int)(trajectory[detected_cone_idx].x * px_per_meter) - (int)(car_x * px_per_meter - MAXrange*px_per_meter),
						(int)(trajectory[detected_cone_idx].y * px_per_meter) - (int)(car_y * px_per_meter - MAXrange*px_per_meter),
						3,
						makecol(0, 255, 0)
					);

					detected_cone_idx++;	
				}
			}
		pthread_mutex_unlock(&draw_mutex);

		// update the start angle for the sliding window
		start_angle = (start_angle + 1) % 360;


		// clock_gettime(CLOCK_MONOTONIC, &iter_end);
        // unsigned long iter_runtime_us = (iter_end.tv_sec - iter_start.tv_sec) * 1000000UL +
        //                                 (iter_end.tv_nsec - iter_start.tv_nsec) / 1000;

        // printf("[PERCEPTION] Runtime: %lu us\n", iter_runtime_us);

		wait_for_period(task_id);
	}
	return NULL;
}

void *trajectory_task(void *arg)
{
	int task_id = get_task_index(arg);
	wait_for_activation(task_id);

	while (!key[KEY_ESC])
	{
		// runtime(1);
		// trajectory_planning(car_x, car_y, car_angle, detected_cones, trajectory);
		// runtime(0);
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

// void runtime (int start_sgn)
// {
// 	static struct timespec iter_start, iter_end;
// 	if (start_sgn == 1){
// 		clock_gettime(CLOCK_MONOTONIC, &iter_start);
// 	}
// 	else{
// 		clock_gettime(CLOCK_MONOTONIC, &iter_end);
// 		unsigned long iter_runtime_us = (iter_end.tv_sec - iter_start.tv_sec) * 1000000UL +
// 										(iter_end.tv_nsec - iter_start.tv_nsec) / 1000;

// 		printf("[PERCEPTION] Runtime: %lu us\n", iter_runtime_us);
// 	}
// }

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
			clear_to_color(display_buffer, pink);
			
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
			
			int text_width = text_length(font, text);
			textout_ex(
				screen, 
				font, 
				text, 
				(screen->w - text_width) / 2, // centered x position 
				0, // y position at top
				makecol(255,0,0), // red text
				makecol(255,255,255) // white background
			);

		pthread_mutex_unlock(&draw_mutex);

        wait_for_period(task_id);
    }
    return NULL;
}


// Sensors
// LiDAR
void    lidar(float car_x, float car_y, pointcloud *measures)
{
	int stop_distance; 

	// Check for each angle in the range [0, 360] with a step of angle_step
	for (int i = 0; i < sliding_window; i += angle_step)
	{
		int		lidar_angle = (start_angle + i)%360;
		int 	current_distance = 0; // initialize distance at 0

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
				if (getpixel(track, x_px, y_px) == yellow) // yellow
				{
					// printf("Cone detected\n");
					measures[lidar_angle].distance = distance; // convert to meters
					measures[lidar_angle].color = yellow; // makecol(254, 221, 0); // yellow
					measures[lidar_angle].point_x = x;
					measures[lidar_angle].point_y = y;

					stop_distance = 1;  // cone detected, stop the loop
				}
				else if (getpixel(track, x_px, y_px) == blue) // blue
				{   
					// printf("Cone detected\n");
					measures[lidar_angle].distance = distance; // convert to meters
					measures[lidar_angle].color = blue; // blue
					measures[lidar_angle].point_x = x;
					measures[lidar_angle].point_y = y;

					stop_distance = 1;  // cone detected, stop the loop
				}
			}            
		}
	}
}

// Control
void 	vehicle_model(float *car_x, float *car_y, int *car_angle, float speed, float steering)
{
// Simulation parameters
const float     dt = 0.1;             // time step
const float     wheelbase = 1.0;      // distance between front and rear axles [m]

float theta;
	
	// Update vehicle position using a simple bicycle model (Ackermann steering)
	theta = (*car_angle) * deg2rad;              // convert current heading to radians
	*car_x += speed * cos(-theta) * dt;                   // update x position
	*car_y += speed * sin(-theta) * dt;                   // update y position
	theta += (speed / wheelbase) * tan(steering) * dt;   // update heading

	// Store updated heading in degrees
	*car_angle = (int)(theta / deg2rad);

}

// void 	autonomous_control(float *car_x, float *car_y, int *car_angle, waypoint *trajectory)
// {
// 	// reorder trajectory waypoints
// 	for (int point = 0; point < MAX_DETECTED_CONES; point++){
// 		if (trajectory[point].x == -1){
// 			printf("[AUTONOMOUS_CONTROL] Waypoints list is empty\n");
// 			break;
// 		}
// 		else {
// 			// calculate the angle between the car and the next waypoint
// 			float dx = trajectory[point].x - *car_x;
// 			float dy = trajectory[point].y - *car_y;
// 			float angle = atan2(dy, dx) * 180 / M_PI;

// 			// calculate the angle between the car and the next waypoint
// 			float angle_diff = angle - *car_angle;
// 			if (angle_diff > 180){
// 				angle_diff -= 360;
// 			}
// 			else if (angle_diff < -180){
// 				angle_diff += 360;
// 			}

// 			// if the angle is too big, rotate the car
// 			if (angle_diff > 5){
// 				// rotate the car to the right
// 				*car_angle += 5;
// 			}
// 			else if (angle_diff < -5){
// 				// rotate the car to the left
// 				*car_angle -= 5;
// 			}
// 			else {
// 				// move the car forward
// 				vehicle_model(car_x, car_y, car_angle, 0.1, 0);
// 			}
// 		}
// 	}
// }

void 	keyboard_control(float *car_x, float *car_y, int *car_angle)
{
const float     accel_step = 0.01;    // speed increment per key press
const float     steering_step = 0.05; // steering increment in radians per key press
const float     max_steering = 30 * deg2rad;     // maximum steering angle in radians

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

void 	mapping(float car_x, float car_y, int car_angle, cone *detected_cones)
{
#define KNOWN_RADIUS
cone_border cone_borders[MAX_DETECTED_CONES]; // maximum number of cones viewed at each position

// init cone borders
for (int i = 0; i < MAX_DETECTED_CONES; i++){
	for (int j = 0; j < MAX_POINTS_PER_CONE; j++){
		cone_borders[i].angles[j] = -1;
	}
	cone_borders[i].color = -1;
}

	// Circle Hough transformation of viewed points
#ifdef KNOWN_RADIUS
	for (int angle = 0; angle < 360; angle += angle_step)
	{
		// Group similar points
		if (measures[angle].color == -1)
		{
			// no cone seen at this angle, PASS
			continue;
		}
		else // a cone is detected at this angle
		{
			check_nearest_point(angle, measures[angle].point_x, measures[angle].point_y, measures[angle].color, cone_borders);
		}

	}

	// at this step the cone_borders contain the points of the pointcloud that are closer each other
	// classified by cone
	// now we need to calculate the center of the cones

	int cone_idx = 0;
	int detected_cone_idx = 0; // index where insert the new detected cone center

	typedef struct {
		float x;
		float y;
		float distance;
		int color;
	} Hough_circle_point;
	
	// for each cone detected
	while( (cone_idx < MAX_DETECTED_CONES-1) && (cone_borders[cone_idx].color != -1) ){
		int cone_cx = 0; // center of the cone
		int cone_cy = 0;

			// count the number of points in the border of the cone
			int N_border_points = 0;

			// count the number of points in the border of the cone
			while ( (N_border_points < MAX_POINTS_PER_CONE-1) && (cone_borders[cone_idx].angles[N_border_points] != -1) ){
				N_border_points++;
			}

		if (N_border_points > 2) // we need at least 3 points to calculate the center of the cone
		{
			// ----------------- LOCAL MINIMA VARIABLE -----------------
			// 			necessary to find the center of the cone
			float LMS_prev_dist, LMS_actual_dist;
			int LMS_first_min_idx = -1, LMS_second_min_idx = -1;
			int LMS_first_max_idx = -1, LMS_second_max_idx = -1; // LMS == Local Minima Search
			// ---------------------------------------------------------


 			// store the solution (w.c.s.: 2 intersection for each pair of circle referred to points)	
			Hough_circle_point possible_cone_centers[(N_border_points-1)*2];
			int possible_center_idx = 0;


			// for each points of the border of the cone
			for (int point_idx = 0; point_idx < N_border_points; point_idx++){
				/* Starting... [POINT_IDX == 0] */
				if (point_idx == 0){	// skip the first point (we need at least two point to compute intersection)
					continue; 
				}
				/* Endig... [POINT_IDX == 0] */

				// store all the points of the second circumference with the distance of the closest point of the first circumference/set of points
				Hough_circle_point circumference_points[360]; 

				/* Starting... [POINT_IDX == 1] */
				if (point_idx == 1){	// first iteration (second point)
										// consider all the 360 points of both the circumference to find the 2 intersections

					// The first point give us all the 360 possible centers of the cone (the circle that pass through the point)
					cone first_point_circle[360];

					for (int i = 0; i < 360; i++){
						first_point_circle[i].x = measures[cone_borders[cone_idx].angles[0]].point_x + cone_radius * cos(i * deg2rad);
						first_point_circle[i].y = measures[cone_borders[cone_idx].angles[0]].point_y + cone_radius * sin(i * deg2rad);
						first_point_circle[i].color = measures[cone_borders[cone_idx].angles[0]].color;
					}

					// for each point of the detected cone border 
				
					// Calculate the center of the cone
					// for all the angles that corresponds to a cone we need to calculate the center of the cone
					float new_x, new_y, new_distance;

					for (int i = 0; i < 360; i++){ // initialize the possible points
						circumference_points[i].distance = 2*MAXrange;
						circumference_points[i].x = 0;
						circumference_points[i].y = 0;
					}

					for (int i = 0; i < 360; i++){
						new_x = measures[cone_borders[cone_idx].angles[point_idx]].point_x + cone_radius * cos(i * deg2rad);
						new_y = measures[cone_borders[cone_idx].angles[point_idx]].point_y + cone_radius * sin(i * deg2rad);

						for (int j = 0; j < 360; j++){
							new_distance = sqrt(pow(new_x - first_point_circle[j].x, 2) + pow(new_y - first_point_circle[j].y, 2));

							if (new_distance < circumference_points[i].distance){
								circumference_points[i].distance = new_distance;
								circumference_points[i].x = new_x;
								circumference_points[i].y = new_y;
							}
						}
					}
				} 
				// at the end of this iteration we have the distance of each point of the second circumference that is closer to the first one
				// all of this points are stored in the possible_points array (Hough circle points type)

				/* Ending... [POINT_IDX == 1] */
				
				/* Starting... [POINT_IDX >= 2] */
				if (point_idx >= 2){	
					// we have 3 or more points so we can compute the min distance of each point of the circumference
					// from the points of intersection found in the previous iteration (less overhead)
					float new_x, new_y, new_distance;

					for (int i = 0; i < 360; i++){ // initialize the possible points
						circumference_points[i].distance = 2*MAXrange;
						circumference_points[i].x = 0;
						circumference_points[i].y = 0;
					}

					for (int i = 0; i < 360; i++){
						new_x = measures[cone_borders[cone_idx].angles[point_idx]].point_x + cone_radius * cos(i * deg2rad);
						new_y = measures[cone_borders[cone_idx].angles[point_idx]].point_y + cone_radius * sin(i * deg2rad);

						for (int j = 0; j < 360; j++){
							new_distance = sqrt(pow(new_x - possible_cone_centers[j].x, 2) + pow(new_y - possible_cone_centers[j].y, 2));

							if (new_distance < circumference_points[i].distance){
								circumference_points[i].distance = new_distance;
								circumference_points[i].x = new_x;
								circumference_points[i].y = new_y;
							}
						}
					}

				}
				/* Ending... [POINT_IDX >= 2] */

				// at this point, we have a 3rd order function that represent the min distances 
				// of each point of the new circumference from the first one while they intersecate
				// we need to find the two local minima of this function to find the two possible centers of the cone
				// float LMS_prev_dist, LMS_actual_dist;
				// int LMS_first_min_idx = -1, LMS_second_min_idx = -1;
				// int LMS_first_max_idx = -1, LMS_second_max_idx = -1; // LMS == Local Minima Search

				int prev_trend = 0, actual_trend = 0; // check the trend of the function [1: if crescent, 0: if constant, -1: if decrescent]
				
				for (int k = 1; k < 360; k++){
					if (circumference_points[k].distance < circumference_points[k-1].distance){
						actual_trend = -1; // decrescent
					}
					else if ( circumference_points[k].distance > circumference_points[k-1].distance){
						actual_trend = 1; // crescent
					}
					else{
						actual_trend = 0; // constant
					}

					// now we know the trend of the function
					// we can search for the local minima
					if (prev_trend == actual_trend){
						// we are in the same regione of the function
						continue;
					}
					else if (prev_trend == -1 && actual_trend == 1){
						// we are in a local minima
						if (LMS_first_min_idx == -1){
							LMS_first_min_idx = k-1;
						}
						else if (LMS_second_min_idx == -1){
							LMS_second_min_idx = k-1;
						}
					}
					else if (prev_trend == 1 && actual_trend == -1){
						// we are in a local maxima
						if (LMS_first_max_idx == -1){
							LMS_first_max_idx = k-1;
						}
						else if (LMS_second_max_idx == -1){
							LMS_second_max_idx = k-1;
						}
					}

					if (actual_trend == 0){
						// we are in a constant region (not update the state of the trend)
						continue;
					}
					else {	// we are in a region of the function that is not a local minima or maxima
						prev_trend = actual_trend;
					}
				} // [TODO] Mitigate the possible errors of bouncing of the function near the local minima or maxima			

				// at this point we have the two local minima of the function stored in LMS_first_min_idx and LMS_second_min_idx
				possible_cone_centers[possible_center_idx].x = circumference_points[LMS_first_min_idx].x;
				possible_cone_centers[possible_center_idx].y = circumference_points[LMS_first_min_idx].y;

				possible_cone_centers[possible_center_idx + 1].x = circumference_points[LMS_second_min_idx].x;
				possible_cone_centers[possible_center_idx + 1].y = circumference_points[LMS_second_min_idx].y;

				possible_center_idx += 2;
			}

			/* 
			At this point we have the possible centers of the cone.
			
			We need to choose the one that stays inside the cone 
			(the two intersection are one on the center of the cone and the other outside).
			*/
	
			// Cluster the candidate centers to select those that are closest
			// and compute their mean.
			const float CLUSTER_THRESHOLD = 0.01f; // distance threshold in meters
			int best_cluster_size = 0;
			float best_sum_x = 0.0f, best_sum_y = 0.0f;

			// Iterate over all candidate centers stored in possible_cone_centers
			for (int i = 0; i < possible_center_idx; i++) {
				int cluster_size = 1;
				float cluster_sum_x = possible_cone_centers[i].x;
				float cluster_sum_y = possible_cone_centers[i].y;

				for (int j = i + 1; j < possible_center_idx; j++) {
					float dx = possible_cone_centers[i].x - possible_cone_centers[j].x;
					float dy = possible_cone_centers[i].y - possible_cone_centers[j].y;
					float dist = sqrt(dx * dx + dy * dy);

					if (dist < CLUSTER_THRESHOLD) {
						cluster_sum_x += possible_cone_centers[j].x;
						cluster_sum_y += possible_cone_centers[j].y;
						cluster_size++;
					}
				}
				if (cluster_size > best_cluster_size) {
					best_cluster_size = cluster_size;
					best_sum_x = cluster_sum_x;
					best_sum_y = cluster_sum_y;
				}
			}

			if (best_cluster_size > 0) {
				float center_x = best_sum_x / best_cluster_size;
				float center_y = best_sum_y / best_cluster_size;

				detected_cones[detected_cone_idx].x = center_x;
				detected_cones[detected_cone_idx].y = center_y;
				detected_cones[detected_cone_idx].color = cone_borders[cone_idx].color;

				detected_cone_idx++;
			}
		}

		cone_idx++;
	}

#ifdef UNKNOWN_RADIUS
	// [TODO] Implement the trajectory planner for the case of unknown radius with the Hough transform
#endif /* UNKNOWN_RADIUS */	

	// Now I have the center of the cones in the detected_cones array
	// I can use this information to plan the trajectory

#endif /* KNOWN_RADIUS */
	
}

void 	trajectory_planning(float car_x, float car_y, float car_angle, cone *detected_cones, waypoint *trajectory)
{
	int N_detected_cones = 0;
	while (detected_cones[N_detected_cones].color != -1) N_detected_cones++; // count detected cones

	if (N_detected_cones < 3){
		return; // not enough cones to plan the trajectory
	}
	else {
		int connected_indices[N_detected_cones][2];
		const int B_idx = 0;
		const int Y_idx = 1;

		for (int i = 0; i < N_detected_cones; i++){
			for (int j = 0; j < 2; j++){
				connected_indices[i][j] = -1;
			}
		}

		for (int focus_idx = 0; focus_idx < N_detected_cones; focus_idx++)
		{
			float mDist_Y = 1000;
			float mDist_B = 1000;

			int mDist_Y_idx = -1;
			int mDist_B_idx = -1;

			// [TODO] Add a condition to check if the candidate index is free to be connected
			int focusColor = -1;

			if (detected_cones[focus_idx].color == yellow) 
				focusColor = Y_idx;
			else 
				focusColor = B_idx;

			if (connected_indices[focus_idx][B_idx] != -1 && connected_indices[focus_idx][Y_idx] != -1)
			{ 	// full connected
				continue;
			}
			
			if (connected_indices[focus_idx][Y_idx] == -1)
			{	// so search for the nearest yellow cone
				for (int candidate_idx = 0; candidate_idx < N_detected_cones; candidate_idx++){
					if (candidate_idx == focus_idx){
						continue;
					}
					
					if ((detected_cones[candidate_idx].color == yellow))// && (connected_indices[candidate_idx][focusColor] == -1))
					{
						float distance = sqrt(pow(detected_cones[candidate_idx].x - detected_cones[focus_idx].x, 2) + pow(detected_cones[candidate_idx].y - detected_cones[focus_idx].y, 2));
						
						if (distance < mDist_Y){
							mDist_Y = distance;
							mDist_Y_idx = candidate_idx;
						}
					}
				}
				// found the yellow one
				connected_indices[focus_idx][Y_idx] = mDist_Y_idx; 
				connected_indices[mDist_Y_idx][focusColor] = focus_idx;
			}

			if (connected_indices[focus_idx][B_idx] == -1)
			{	// so search for the nearest blue cone
				for (int candidate_idx = 0; candidate_idx < N_detected_cones; candidate_idx++){
					if (candidate_idx == focus_idx){
						continue;
					}

					if ((detected_cones[candidate_idx].color == blue))// && (connected_indices[candidate_idx][focusColor] == -1))
					{
						float distance = sqrt(pow(detected_cones[candidate_idx].x - detected_cones[focus_idx].x, 2) + pow(detected_cones[candidate_idx].y - detected_cones[focus_idx].y, 2));
						
						if (distance < mDist_B){
							mDist_B = distance;
							mDist_B_idx = candidate_idx;
						}
					}
				}
				// found the blue one
				connected_indices[focus_idx][B_idx] = mDist_B_idx; 
				connected_indices[mDist_B_idx][focusColor] = focus_idx;
			}

		}

#ifdef DEBUG
		for (int i = 0; i < N_detected_cones; i++){
			if (connected_indices[i][B_idx] != -1 && connected_indices[i][Y_idx] != -1){
				line(
					display_buffer,
					detected_cones[i].x * px_per_meter,
					detected_cones[i].y * px_per_meter,
					detected_cones[connected_indices[i][B_idx]].x * px_per_meter,
					detected_cones[connected_indices[i][B_idx]].y * px_per_meter,
					blue
				);
				line(
					display_buffer,
					detected_cones[i].x * px_per_meter,
					detected_cones[i].y * px_per_meter,
					detected_cones[connected_indices[i][Y_idx]].x * px_per_meter,
					detected_cones[connected_indices[i][Y_idx]].y * px_per_meter,
					yellow
				);
				line(
					display_buffer,
					detected_cones[connected_indices[i][B_idx]].x * px_per_meter,
					detected_cones[connected_indices[i][B_idx]].y * px_per_meter,
					detected_cones[connected_indices[i][Y_idx]].x * px_per_meter,
					detected_cones[connected_indices[i][Y_idx]].y * px_per_meter,
					makecol(0, 255, 0)
				);
			}
		}

		for (int i = 0; i < N_detected_cones; i++){
			char c = detected_cones[i].color == yellow ? 'Y' : 'B';
			printf("\nCone %d (%c): ", i, c);
			for (int j = 0; j < 2; j++){
				char c = detected_cones[j].color == yellow ? 'Y' : 'B';
				printf("%d (%c)\t", connected_indices[i][j], c);
			}
		}
#endif /* DEBUG */

		// Now I have in the connected_indices matrix the inidices of the connected cones
		// I have to use only the connection made by cones of different color 
		for (int i = 0; i < N_detected_cones; i++){
			int opposite_color_idx = detected_cones[i].color == yellow ? B_idx : Y_idx;

			trajectory[i].x = (detected_cones[i].x + detected_cones[connected_indices[i][opposite_color_idx]].x) / 2;
			trajectory[i].y = (detected_cones[i].y + detected_cones[connected_indices[i][opposite_color_idx]].y) / 2;

		}

 	}
}



void 	check_nearest_point(int angle, float new_point_x, float new_point_y, int color, cone_border *cone_borders)
{
	// check if the point is near to a cone
	for (int i = 0; i < MAX_DETECTED_CONES; i++)
	{
		if (cone_borders[i].color == -1)
		{
			// no cone detected at this position
			int insertion_point = 0;
			while ((insertion_point < MAX_POINTS_PER_CONE-1) && (cone_borders[i].angles[insertion_point] != -1)) insertion_point++;

			// populate the new cone border
			cone_borders[i].angles[insertion_point] = angle;
			cone_borders[i].color = color;
			break;
		}
		else // a cone is detected at this position
		{
			int insertion_point = 0;	// find the first free position to put this new point in the border array
			float distance = 2*MAXrange;	// max distance between two points detected

			int isPointOnCone = 0;		// flag to check if the point is near to a cone

			while ((insertion_point < MAX_POINTS_PER_CONE-1) && (cone_borders[i].angles[insertion_point] != -1)){
				float cone_point_x = measures[cone_borders[i].angles[insertion_point]].point_x;
				float cone_point_y = measures[cone_borders[i].angles[insertion_point]].point_y;

				float distance = sqrt(pow(new_point_x - cone_point_x, 2) + pow(new_point_y - cone_point_y, 2));

				if (distance < 2*cone_radius){
					// the point is near to the i-th cone
					isPointOnCone = 1;
				}

				insertion_point++; 
			}

			if (isPointOnCone){	// if the point is next to the i-th cone
				cone_borders[i].angles[insertion_point] = angle;
				break;
			}
		}

	}
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

#define tmp_scale  1.5/100 //1.5 
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
							cones[i].x = (float)( atof((char *)event.data.scalar.value) ) * tmp_scale * px_per_meter;
						} 
						else if (strcmp(current_key, "y") == 0) { 
							cones[i].y = (float)( atof((char *)event.data.scalar.value) ) * tmp_scale * px_per_meter;
						} 
						else if (strcmp(current_key, "color") == 0) {
							if (strcmp((char *)event.data.scalar.value, "yellow") == 0) {
								cones[i].color = yellow; //makecol(254, 221, 0); // Yellow color
							} 
							else if (strcmp((char *)event.data.scalar.value, "blue") == 0) {
								cones[i].color = blue; //makecol(46, 103, 248); // Blue color
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