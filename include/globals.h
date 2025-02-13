/**
 * @file globals.h
 * @brief Global constants, variables, and structures for the simulation and visualization system.
 *
 * This header file declares all global macros, extern variables, and data types required across multiple modules
 * for task management, visualization, and perception in a 2D simulation environment.
 *
 * @section tasks_constants Tasks Constants
 * - Defines task periods (in milliseconds):
 *   - PERCEPTION_PERIOD: Period for the perception task.
 *   - TRAJECTORY_PERIOD: Period for the trajectory planning task.
 *   - CONTROL_PERIOD: Period for the control task.
 *   - DISPLAY_PERIOD: Period for the display update task.
 * - Defines deadlines (in milliseconds), set equal to the corresponding task periods:
 *   - PERCEPTION_DEADLINE, TRAJECTORY_DEADLINE, CONTROL_DEADLINE, DISPLAY_DEADLINE.
 * - Defines task priorities (lower number indicates higher priority):
 *   - PERCEPTION_PRIORITY, TRAJECTORY_PRIORITY, CONTROL_PRIORITY, DISPLAY_PRIORITY.
 *
 * @section mutex Drawing Mutex
 * - Declares the external pthread_mutex_t variable draw_mutex used to synchronize drawing operations across threads.
 *
 * @section conversion_constants Conversion Constants
 * - px_per_meter: Conversion factor from meters to pixels.
 * - deg2rad: Constant to convert degrees to radians.
 *
 * @section visualization_constants Visualization Constants
 * - Declares external constants and variables for visualization:
 *   - title: Pointer to a string containing the window title.
 *   - X_MAX, Y_MAX: Maximum dimensions (in pixels) of the window or display.
 *   - cone_radius: Radius used for drawing cones.
 * - Declares external BITMAP pointers for various graphical assets:
 *   - control_panel: Bitmap for the control panel.
 *   - steering_wheel: Bitmap for the steering wheel graphic.
 *   - throttle_gauge: Bitmap for the throttle gauge.
 *   - background: Background image bitmap.
 *   - track: Bitmap representing the track.
 *   - car: Bitmap representing the car.
 *   - perception: Bitmap used for displaying perception data.
 *   - trajectory_bmp: Bitmap for trajectory visualization.
 *   - display_buffer: Bitmap used as a back buffer for display updates.
 * - Defines maxThrottleHeight: Maximum height for the throttle gauge.
 * - Declares external color variables for various color schemes used in the display:
 *   - grass_green, asphalt_gray, white, pink, yellow, blue.
 *
 * @section car_pose Car Pose
 * - Declares global variables representing the car's state:
 *   - car_x, car_y: The car's x and y positions in meters.
 *   - car_angle: The angle of the car in degrees.
 *
 * @section perception Perception Constants
 * - MAX_DETECTED_CONES: Macro defining the maximum number of cones that can be detected simultaneously.
 * - pointcloud_t: Struct representing a detected cone with the following fields:
 *   - point_x: x-coordinate of the detected point.
 *   - point_y: y-coordinate of the detected point.
 *   - distance: Distance from the sensor to the cone.
 *   - color: Color identifier for the cone.
 * - Declares an external array 'measures' of pointcloud_t to hold the detected cone data.
 * - Declares a semaphore 'lidar_sem' used to synchronize the passing of LiDAR data between tasks.
 */

/**
 * @struct pointcloud_t
 * @brief Contains the measures of distance made by the LiDAR at each angle.
 */

#ifndef GLOBALS_H
#define GLOBALS_H

#include <allegro.h>
#include <pthread.h>
#include <semaphore.h>

// #define DEBUG
#define PROFILING

// ------------------------
// 	TASKs COSTANTS
// ------------------------
/* Task periods (ms) */
#define PERCEPTION_PERIOD	100
#define TRAJECTORY_PERIOD	100
#define CONTROL_PERIOD		10
#define DISPLAY_PERIOD		17
#define SETTINGS_PERIOD		100

/* Deadlines (ms) */
#define PERCEPTION_DEADLINE		PERCEPTION_PERIOD
#define TRAJECTORY_DEADLINE  	TRAJECTORY_PERIOD
#define CONTROL_DEADLINE     	CONTROL_PERIOD
#define DISPLAY_DEADLINE     	DISPLAY_PERIOD
#define SETTINGS_DEADLINE     	SETTINGS_PERIOD

/* Priorities (lower number = higher priority) */
#define PERCEPTION_PRIORITY	15
#define TRAJECTORY_PRIORITY	20
#define CONTROL_PRIORITY	5
#define DISPLAY_PRIORITY	30
#define SETTINGS_PRIORITY	35

/* Drawing mutex */
extern pthread_mutex_t draw_mutex;

// ------------------------
//  CONVERSIONs COSTANTS
// ------------------------
#define px_per_meter 100
#define	deg2rad 0.017453292519943295769236907684886f

// ------------------------
// 	VISUALIZATION COSTANTS
// ------------------------
extern const char* 	title;
extern const int 	X_MAX; // pixels
extern const int 	Y_MAX; // pixels

/* Global bitmaps */
extern BITMAP *control_panel;
extern BITMAP *steering_wheel;
extern BITMAP *throttle_gauge;
#define maxThrottleHeight 100

extern BITMAP *background;
extern BITMAP *track;
extern BITMAP *car;
extern BITMAP *perception;
extern BITMAP *trajectory_bmp;
extern BITMAP *display_buffer;

/* Color */
extern int grass_green, asphalt_gray, white, pink;
extern int yellow, blue;
extern int red, green;

/* Car pose */
extern float car_x, car_y;	// meters
extern int car_angle; 		// degrees

/* Cones */
typedef struct {
    float x;      /**< X position of the cone (in meters, converted to px when drawn) */
    float y;      /**< Y position of the cone */
    int   color;  /**< Color (in Allegro color format) */
} cone;

extern const float cone_radius;
#define MAX_CONES_MAP 3000
extern cone	cones[MAX_CONES_MAP];
extern cone starting_cone_positions[MAX_CONES_MAP]; /**< Store the default cone positions before collisions */

// ------------------------
// 	PERCEPTION COSTANTS
// ------------------------
#define MAX_DETECTED_CONES 	360 // max number of cones simultaneously detected

typedef struct {
	float	point_x;
	float	point_y;
	float	distance;
	int		color;
} pointcloud_t;

extern pointcloud_t measures[MAX_DETECTED_CONES]; // array of detected cones

extern sem_t lidar_sem; // semaphore to pass lidar data to mapping task

#endif // GLOBALS_H