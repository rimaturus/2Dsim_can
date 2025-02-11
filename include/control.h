/**
 * @file control.h
 * @brief Declarations for vehicle control functions and variables in the 2D simulator.
 *
 * This header file is part of the 2D simulation project and provides the interface
 * for controlling the simulated vehicle. It supports both manual (keyboard-driven)
 * and autonomous (trajectory-based) modes.
 *
 * Global Variables:
 * - steering: Current steering angle in radians.
 * - pedal: Throttle input as a normalized value in the range [0, 1].
 *
 * Dependencies:
 * - trajectory.h: Used for waypoint and trajectory definitions.
 * - globals.h: Contains global variables and settings.
 * - perception.h: Provides functionalities related to situational awareness.
 */

/**
 * @brief Controls the vehicle using keyboard input.
 *
 * Processes user keyboard inputs to modify the vehicle's position
 * and orientation. The function updates the car's x and y coordinates,
 * as well as its angular orientation (in degrees).
 *
 * @param car_x Pointer to the car's x-coordinate.
 * @param car_y Pointer to the car's y-coordinate.
 * @param car_angle Pointer to the car's current angle (in degrees).
 */
 

/**
 * @brief Controls the vehicle autonomously along a given trajectory.
 *
 * Uses the provided trajectory (a sequence of waypoints) to automatically
 * adjust the vehicle's steering and pedal inputs. This function updates the 
 * car's position and orientation by computing the necessary control actions
 * based on the navigation path.
 *
 * @param car_x Pointer to the car's x-coordinate.
 * @param car_y Pointer to the car's y-coordinate.
 * @param car_angle Pointer to the car's current orientation angle (in degrees).
 * @param trajectory Pointer to the waypoint structure that defines the trajectory.
 */
#ifndef CONTROL_H
#define CONTROL_H

#include "trajectory.h"
#include "globals.h"
#include "perception.h"

extern float 	steering; // in radians
extern float	pedal; // [0 -> 1]


void keyboard_control(float *car_x, float *car_y, int *car_angle);
void autonomous_control(float *car_x, float *car_y, int *car_angle, waypoint *trajectory);

#endif // CONTROL_H