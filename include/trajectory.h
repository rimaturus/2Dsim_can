#ifndef TRAJECTORY_H
#define TRAJECTORY_H

#include "perception.h"
#include "globals.h"

typedef struct {
	float x;
	float y;
} waypoint;

extern waypoint trajectory[2*MAX_DETECTED_CONES];
extern int trajectory_idx;

void trajectory_planning(float car_x, float car_y, float car_angle, cone *detected_cones, waypoint *trajectory);

#endif // TRAJECTORY_H