#include "globals.h"

const char* title = "2D FSAE sim by rimaturus";

const int X_MAX = (19 * px_per_meter);
const int Y_MAX = (12 * px_per_meter);

const float cone_radius = 0.05f;

/* CAR initial position */
float  car_x        = 4.5f;
float  car_y        = 3.0f;
int    car_angle    = 0;

/* Global bitmaps */
BITMAP *control_panel   = NULL;
BITMAP *steering_wheel  = NULL;
BITMAP *throttle_gauge  = NULL;

BITMAP *background      = NULL;
BITMAP *track           = NULL;
BITMAP *car             = NULL;
BITMAP *perception      = NULL;
BITMAP *trajectory_bmp	= NULL;
BITMAP *display_buffer  = NULL;


int grass_green, asphalt_gray, white, pink;
int yellow, blue;

/* LiDAR */
pointcloud_t measures[360];

sem_t lidar_sem;
pthread_mutex_t draw_mutex = PTHREAD_MUTEX_INITIALIZER;