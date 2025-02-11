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

// LiDAR measures
void lidar(float car_x, float car_y, pointcloud_t *measures);

// Real-time mapping
void mapping(float car_x, float car_y, int car_angle, cone *detected_cones);
void check_nearest_point(int angle, float new_point_x, float new_point_y, int color, cone_border *cone_borders);

// Update the map
void update_map(cone *detected_cones); 

#endif // PERCEPTION_H