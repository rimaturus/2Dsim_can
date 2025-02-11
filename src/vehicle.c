/**
 * @defgroup VehicleModule Vehicle model and control
 * @{
 */

/**
 * @brief Updates the vehicle's position and orientation based on pedal and steering inputs.
 *
 * This function simulates the movement of a vehicle using a simple bicycle model (Ackermann steering).
 * It updates the current position (car_x, car_y) and the heading (car_angle) of the vehicle according
 * to the pedal input (acceleration or braking) and the steering angle over a fixed time step.
 *
 * The simulation parameters include:
 * - A time step (dt) for the simulation.
 * - A mass of the vehicle, which is used to compute acceleration from the force.
 * - A fixed wheelbase, representing the distance between the front and rear axles.
 * - A maximum achievable speed and a maximum braking force.
 *
 * When the pedal input is positive, the vehicle accelerates towards a speed that is a proportion of maxSpeed.
 * If the pedal input is not positive, the vehicle is assumed to be braking, and the deceleration is computed
 * using the current speed and a maximum braking factor.
 *
 * The vehicle's position is updated based on its current speed and orientation. The steering angle influences
 * the change in the vehicle's heading only if the vehicle is moving above a small threshold speed.
 * The angle conversion between degrees and radians is used to interface with the trigonometric functions.
 *
 * @param[in,out] car_x Pointer to the vehicle's x-coordinate. Updated based on the vehicle's motion.
 * @param[in,out] car_y Pointer to the vehicle's y-coordinate. Updated based on the vehicle's motion.
 * @param[in,out] car_angle Pointer to the vehicle's heading in degrees. Updated based on the computed orientation.
 * @param[in] pedal The pedal input; a positive value corresponds to acceleration while a non-positive value
 *                  initiates braking.
 * @param[in] steering The steering input that determines the rate of change of the vehicle's heading.
 */
#include <math.h>
#include <stdio.h>
#include "vehicle.h"
#include "globals.h"

void 	vehicle_model(float *car_x, float *car_y, int *car_angle, float pedal, float steering)
{
// Simulation parameters
const float     dt = 0.1;			// time step

const float		mass = 100;      	// mass of the vehicle [kg]
const float     wheelbase = 2.0;	// distance between front and rear axles [m]
const float		maxSpeed = 1.0;		// maximum speed in m/s
const float     maxBraking = 50.0;	// maximum braking in m/s^2

float theta;

static float current_speed = 0.0; 	// persist speed between calls
float speed, acceleration;

	if (pedal > 0.0)
	{
		speed = pedal * maxSpeed;	// speed in m/s
		acceleration = (speed - current_speed) / mass;  // F = ma -> a = F/m
	}
	else
	{	// Brake
		acceleration = (pedal * maxBraking * current_speed) / mass;  // F = ma
	}

	current_speed += acceleration * dt;
	
	speed = (current_speed < 0.0) ? 0.0 : current_speed;  // use the actual speed with inertia

	// Update vehicle position using a simple bicycle model (Ackermann steering)
	theta = (*car_angle) * deg2rad;              // convert current heading to radians
	*car_x += speed * cos(-theta) * dt;                   // update x position
	*car_y += speed * sin(-theta) * dt;                   // update y position

	if (speed < 0.01 * maxSpeed)
	{
		// If the car is stopped, the steering angle is irrelevant
		return;
	}
	theta += (1.0 / wheelbase) * tan(steering) * dt;   // update heading independent of speed

	// Store updated heading in degrees
	*car_angle = (int)(theta / deg2rad);

}

/** @} */