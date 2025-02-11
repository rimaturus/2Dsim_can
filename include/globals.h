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
#define PERCEPTION_PERIOD    50
#define TRAJECTORY_PERIOD    10
#define CONTROL_PERIOD       10
#define DISPLAY_PERIOD       17

/* Deadlines (ms) */
#define PERCEPTION_DEADLINE		PERCEPTION_PERIOD
#define TRAJECTORY_DEADLINE  	TRAJECTORY_PERIOD
#define CONTROL_DEADLINE     	CONTROL_PERIOD
#define DISPLAY_DEADLINE     	DISPLAY_PERIOD

/* Priorities (lower number = higher priority) */
#define PERCEPTION_PRIORITY	15
#define TRAJECTORY_PRIORITY	20
#define CONTROL_PRIORITY	25
#define DISPLAY_PRIORITY    30

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

extern const float cone_radius;

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

/* Car pose */
extern float car_x, car_y;	// meters
extern int car_angle; 		// degrees

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