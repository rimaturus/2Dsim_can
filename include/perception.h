/**
 * @file perception.h
 * @brief Header file for perception-related types, constants, and function declarations.
 *
 * This file contains the datatypes and function prototypes used for perception in the 2D simulation. 
 * It includes structures to represent cones, cone borders, candidate cones, and mapping functionalities such as LiDAR scanning, real-time mapping, and track map updating.
 *
 * Global constants and configurable parameters:
 *  - MAX_POINTS_PER_CONE: Maximum number of points (angles) that can define the border of a cone.
 *  - MAX_CONES_MAP: Maximum number of cones that can be stored in the track map.
 *  - maxRange: Maximum detection range (in meters) for the LiDAR.
 *  - ignore_distance: Distance threshold below which detected points may be ignored.
 *  - sliding_window, angle_step, start_angle: Parameters that control the LiDAR scanning and cone detection algorithm.
 *
 * Global arrays:
 *  - detected_cones: Array containing the cones detected by perception.
 *  - track_map: Array representing the global track map of cones.
 *  - track_map_idx: Current index in the track_map array.
 *
 * Other constants:
 *  - MAX_CANDIDATES: Maximum number of candidate cones tracked.
 *  - DETECTIONS_THRESHOLD: Minimum number of detections required to consider a candidate cone valid.
 */

/**
 * @struct cone
 * @brief Represents a cone detected in the environment.
 *
 * This structure stores the position and color information for a cone. The positions are stored in floating point 
 * values in meters and can be converted to pixels when rendering. The color is represented using the Allegro color format.
 *
 * Members:
 *  - x: X position of the cone.
 *  - y: Y position of the cone.
 *  - color: Color of the cone (using Allegro color format).
 */

/**
 * @struct cone_border
 * @brief Represents a border of a cone as seen in a LiDAR scan.
 *
 * This structure collects indices from the LiDAR scan that correspond to the boundaries of a cone. It also stores the 
 * color information associated with that cone border.
 *
 * Members:
 *  - angles: An array of indices (angles) in the LiDAR scan corresponding to the same cone border.
 *  - color: Color of the cone border.
 */

/**
 * @struct candidate_cone
 * @brief Represents a candidate cone that might be validated after multiple detections.
 *
 * This structure is used for maintaining a candidate cone before it is confirmed. It stores the position, color, and 
 * the number of detections that have contributed to this candidate.
 *
 * Members:
 *  - x: X position of the candidate cone.
 *  - y: Y position of the candidate cone.
 *  - color: Color of the candidate cone.
 *  - detections: Count of how many times this candidate has been detected.
 */

/**
 * @struct Hough_circle_point_t
 * @brief Represents a point in the Hough circle transformation space.
 * 
 * This structure is used to store information about a point in the Hough circle transformation space, including its
 * position, distance, and color.
 */

/**
 * @brief Processes the LiDAR measurements.
 *
 * This function simulates or processes a LiDAR scan, updating the point cloud information based on the car's current position.
 *
 * @param car_x The X coordinate of the car's position.
 * @param car_y The Y coordinate of the car's position.
 * @param measures Pointer to the point cloud structure to be updated with the LiDAR measurements.
 */

/**
 * @brief Performs real-time mapping using detected cones.
 *
 * This function updates the array of detected cones by correlating the car’s current position and orientation with cone positions.
 *
 * @param car_x The X coordinate of the car's position.
 * @param car_y The Y coordinate of the car's position.
 * @param car_angle The orientation angle of the car (in degrees or radians as defined elsewhere).
 * @param detected_cones Pointer to the array where detected cones are stored.
 */

/**
 * @brief Checks and updates the nearest point for a given LiDAR scan angle.
 *
 * This function compares a new detected point against existing data to determine if it belongs to a known cone border,
 * updating the cone border data accordingly.
 *
 * @param angle The scan angle index from the LiDAR measurement.
 * @param new_point_x The X coordinate of the new detected point.
 * @param new_point_y The Y coordinate of the new detected point.
 * @param color The color associated with the new detected point.
 * @param cone_borders Pointer to the cone_border structure to be updated.
 */

/**
 * @brief Updates the global track map with the latest detected cones.
 *
 * This function takes the newly detected cones and integrates them into the overall track map, which represents the
 * accumulated perception of the environment.
 *
 * @param detected_cones Pointer to the array of cones that have been newly detected.
 */
#ifndef PERCEPTION_H
#define PERCEPTION_H

#include <math.h>
#include "globals.h"

typedef struct {
    float x;      /**< X position of the cone (in meters, converted to px when drawn) */
    float y;      /**< Y position of the cone */
    int   color;  /**< Color (in Allegro color format) */
} cone;

#define MAX_POINTS_PER_CONE 180
#define MAX_CONES_MAP 3000

#define maxRange 10.0f
extern const float ignore_distance;

extern const int sliding_window;
extern const int angle_step;
extern int start_angle;

extern cone detected_cones[MAX_DETECTED_CONES];

typedef struct {
    int angles[MAX_POINTS_PER_CONE]; /**< Indices (angles) in the LiDAR scan that map to the same cone border */
    int color;                       /**< Color of the cone */
} cone_border;

extern cone track_map[MAX_CONES_MAP];
extern int track_map_idx;

#define MAX_CANDIDATES 100000
#define DETECTIONS_THRESHOLD 10

typedef struct {
	float x;
	float y;
	int color;
	int detections;  // Number of times this candidate has been detected
} candidate_cone;

// Real-time mapping
// Helper struct definitions
typedef struct {
	float x;
	float y;
	float distance;
	int color;
} Hough_circle_point_t;

// LiDAR measures
void lidar(float car_x, float car_y, pointcloud_t *measures);

// Real-time mapping
void mapping(float car_x, float car_y, int car_angle, cone *detected_cones);
void check_nearest_point(int angle, float new_point_x, float new_point_y, int color, cone_border *cone_borders);

// Update the map
void update_map(cone *detected_cones); 

#endif // PERCEPTION_H