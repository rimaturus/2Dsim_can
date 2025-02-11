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