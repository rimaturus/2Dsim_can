/**
 * @file trajectory.h
 * @brief Provides declarations for trajectory planning based on cone detection.
 *
 * This header defines the data structures and function prototypes necessary for generating
 * trajectories (waypoints) that guide the car through detected cones.
 */

/**
 * @struct waypoint
 * @brief Represents a point in 2D space.
 *
 * This structure is used to define a waypoint with x and y coordinates.
 */

/**
 * @var trajectory
 * @brief Global array of waypoints forming the planned trajectory.
 *
 * This array stores the computed trajectory based on the detected cones. It is sized to
 * accommodate up to twice the number of maximum detectable cones (2 * MAX_DETECTED_CONES).
 */

/**
 * @var trajectory_idx
 * @brief Global index used to track the number of waypoints currently stored in the trajectory.
 *
 * This variable indicates the current position in the trajectory array, facilitating
 * the addition of new waypoints.
 */

/**
 * @brief Plans the trajectory for the car based on its current state and detected obstacles.
 *
 * This function calculates the trajectory by considering the car's position and orientation,
 * as well as the positions of detected cones. The resultant waypoints are stored in the provided
 * trajectory array.
 *
 * @param car_x The x-coordinate of the car's current position.
 * @param car_y The y-coordinate of the car's current position.
 * @param car_angle The heading angle of the car in radians.
 * @param detected_cones Pointer to an array of cone structures representing detected cones.
 * @param trajectory Pointer to an array of waypoint structures where the planned trajectory will be stored.
 */
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