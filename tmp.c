#include <stdio.h>
#include <stdlib.h>
#include <allegro.h>
#include <math.h>
#include <yaml.h>
#include <string.h>
#include <pthread.h>
#include <sched.h>
#include <time.h>
#include "ptask/ptask.h"
#include <semaphore.h>

sem_t lidar_sem;

/* Task periods (ms) */
#define PERCEPTION_PERIOD    10
#define TRAJECTORY_PERIOD    10
#define CONTROL_PERIOD       10
#define DISPLAY_PERIOD       17

/* Deadlines (ms) */
#define PERCEPTION_DEADLINE  PERCEPTION_PERIOD
#define TRAJECTORY_DEADLINE  TRAJECTORY_PERIOD
#define CONTROL_DEADLINE     CONTROL_PERIOD
#define DISPLAY_DEADLINE     DISPLAY_PERIOD

/* Priorities (lower number = higher priority) */
#define PERCEPTION_PRIORITY	15
#define TRAJECTORY_PRIORITY	20
#define CONTROL_PRIORITY     25
#define DISPLAY_PRIORITY     30

void *perception_task(void *arg);
void *trajectory_task(void *arg);
void *control_task(void *arg);
void *display_task(void *arg);

pthread_mutex_t draw_mutex = PTHREAD_MUTEX_INITIALIZER;

#define px_per_meter 100
#define deg2rad 0.0174533

const char* text = "2D FSAE sim by rimaturus";
const int XMAX = (19 * px_per_meter);
const int YMAX = (12 * px_per_meter);

/* Car initial position */
float car_x = 4.5f;
float car_y = 3.0f;
int   car_angle = 0;

/* Global bitmaps */
BITMAP *background = NULL;
BITMAP *track = NULL;
BITMAP *car = NULL;
BITMAP *perception = NULL;
BITMAP *display_buffer = NULL;

typedef struct {
	float x;
	float y;
	int   color;
} cone;

const float cone_radius = 0.05f;

int grass_green, asphalt_gray, white, pink;
int yellow, blue;

/* LiDAR configuration */
typedef struct {
	float point_x;
	float point_y;
	float distance;
	int   color;
} pointcloud;

const int angle_step = 1;
const float MAXrange = 10.0f;
const float distance_resolution = 0.01f;
const int N_angles = 360;
const int distance_steps = (int)(MAXrange / distance_resolution);

pointcloud measures[360];

int start_angle = 0;
const int sliding_window = 360;
const float ignore_distance = 0.2f;

/* Cone detection */
#define MAX_DETECTED_CONES 360
#define MAX_POINTS_PER_CONE 180

cone detected_cones[MAX_DETECTED_CONES];
cone track_map[MAX_DETECTED_CONES];
int track_map_idx = 0;

typedef struct {
	float x;
	float y;
} waypoint;

waypoint trajectory[2*MAX_DETECTED_CONES];
int trajectory_idx = 0;

typedef struct {
	int angles[MAX_POINTS_PER_CONE];
	int color;
} cone_border;

void init_cones(cone *cones, int max_cones);
void load_cones_positions(const char *filename, cone *cones, int max_cones);
void lidar(float car_x, float car_y, pointcloud *measures);
void mapping(float car_x, float car_y, int car_angle, cone *detected_cones);
void keyboard_control(float *car_x, float *car_y, int *car_angle);
void vehicle_model(float *car_x, float *car_y, int *car_angle, float speed, float steering);
void check_nearest_point(int angle, float new_point_x, float new_point_y, int color, cone_border *cone_borders);
void trajectory_planning(float car_x, float car_y, float car_angle, cone *detected_cones, waypoint *trajectory);
void runtime(int stop_signal, char* task_name);
void draw_dir_arrow();
void update_map(cone *detected_cones);
void autonomous_control(float *car_x, float *car_y, int *car_angle, waypoint *trajectory);
float angle_rotation_sprite(float angle);

int main()
{
	return 0;
}