/**
 * @file globals.c
 * @brief Global definitions and state variables for the 2D FSAE simulation.
 *
 * This file declares and initializes global constants, variables, and resources used by the simulation.
 *
 * @details
 * - Simulation Information:
 *   - @var title
 *     The simulation title ("2D FSAE sim by rimaturus").
 *
 * - Simulation Dimensions:
 *   - @var X_MAX
 *     The maximum horizontal dimension of the simulation in pixels, computed as 19 * px_per_meter.
 *   - @var Y_MAX
 *     The maximum vertical dimension of the simulation in pixels, computed as 12 * px_per_meter.
 *
 * - Physics and Geometry:
 *   - @var cone_radius
 *     The radius (in meters) for cone-related calculations, set to 0.05.
 *
 * - Car Initial State:
 *   - @var car_x
 *     The initial X-coordinate position of the car within the simulation.
 *   - @var car_y
 *     The initial Y-coordinate position of the car within the simulation.
 *   - @var car_angle
 *     The initial orientation angle of the car (in degrees).
 *
 * - Graphical Resources:
 *   - @var control_panel
 *   - @var steering_wheel
 *   - @var throttle_gauge
 *
 *     Bitmap pointers for UI elements including the control panel, steering wheel, and throttle gauge.
 *
 *   - @var background
 *   - @var track
 *   - @var car
 *   - @var perception
 *   - @var trajectory_bmp
 *   - @var display_buffer
 *
 *     Bitmap pointers used for rendering various components of the simulation such as the background, track markings,
 *     car image, perception overlays, trajectory visualizations, and the overall display buffer.
 *
 * - Color Definitions:
 *   - @var grass_green, asphalt_gray, white, pink, yellow, blue
 *
 *     Integer values representing various colors used for rendering the simulation's elements.
 *
 * - LiDAR Data:
 *   - @var measures
 *     An array of 360 LiDAR measurements (one per degree) representing the simulated point cloud data.
 *
 * - Thread Synchronization:
 *   - @var lidar_sem
 *     A POSIX semaphore used to synchronize access to LiDAR data.
 *   - @var draw_mutex
 *     A POSIX mutex that ensures mutual exclusion during drawing operations.
 *
 * @note The multiplier 'px_per_meter' used for computing X_MAX and Y_MAX is defined elsewhere.
 */
#include "globals.h"

const char* title = "2D FSAE sim by rimaturus";

const int X_MAX = (19 * px_per_meter);
const int Y_MAX = (10 * px_per_meter);

const float cone_radius = 0.05f;

/* CAR initial position */
float  car_x        = 4.0f;
float  car_y        = 3.0f;
int    car_angle    = 270;

cone	cones[MAX_CONES_MAP];
cone	starting_cone_positions[MAX_CONES_MAP];

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
int red, green;

/* LiDAR */
pointcloud_t measures[360];

sem_t lidar_sem;
pthread_mutex_t draw_mutex = PTHREAD_MUTEX_INITIALIZER;