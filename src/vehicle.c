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
#include "control.h"

#define M_PI 3.14159265358979323846
static float rad_angle = 270 * deg2rad;

void 	vehicle_model(float *car_x, float *car_y, int *car_angle, float pedal, float target_steering)
{
#define CINEMATIC_MODEL

#ifndef CINEMATIC_MODEL
// Simulation parameters
const float     dt = (float)(CONTROL_PERIOD)/100;			// time step

const float		mass = 100;      	// mass of the vehicle [kg]
const float     wheelbase = 3.0;	// distance between front and rear axles [m]
const float		maxSpeed = 1.0;		// maximum speed in m/s
const float     maxBraking = 50.0;	// maximum braking in m/s^2
const float     maxSteering = 30;

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

    // function to follow steering target
    target_steering = (fabs(target_steering) > maxSteering) ? maxSteering : target_steering;

    if (steering < target_steering)
    {
        steering += 0.1;
    }
    else if (steering > target_steering)
    {
        steering -= 0.1;
    }
    printf("steering: %.2f\n", steering);

	theta += (1.0 / wheelbase) * tan(steering) * dt;   // update heading independent of speed
    theta = (theta > 360.0) ? theta - 360.0 : theta;   // normalize angle to [0, 360)
    theta = (theta < -360.0) ? theta + 360.0 : theta;  // normalize angle to [-360, 0) 

	// Store updated heading in degrees
	*car_angle = (int)(theta / deg2rad);
#endif /* NON CINEMATIC_MODEL */

#ifdef CINEMATIC_MODEL
    // Example single-track model
    const float dt = (float)(CONTROL_PERIOD)/100;
    const float wheelbase = 0.5f;
    const float maxSpeed = 1.0f;
    const float maxBraking = 50.0f;
    const float maxSteering = 30.0f * deg2rad; // in rad

    static float current_speed_cinematic = 0.0f;
    float speed_cinematic, accel_cinematic;

    // Compute speed from pedal
    if (pedal > 0.0f) {
        speed_cinematic = pedal * maxSpeed;
        accel_cinematic = speed_cinematic - current_speed_cinematic;
    } else {
        accel_cinematic = (pedal * maxBraking * current_speed_cinematic);
    }
    current_speed_cinematic += accel_cinematic * dt;
    current_speed_cinematic = (current_speed_cinematic < 0.0f) ? 0.0f : current_speed_cinematic;

    // Clamp the steering target
    target_steering = (target_steering > maxSteering) ? maxSteering : ((target_steering < -maxSteering) ? -maxSteering : target_steering);
    
    // Smoothly follow target steering with proper boundary conditions
    float steering_rate = 0.1f;  // Rate of steering change
    if (fabs(steering - target_steering) < steering_rate) {
        steering = target_steering;  // Snap to target if very close
    } else if (steering < target_steering) {
        steering += steering_rate;
    } else if (steering > target_steering) {
        steering -= steering_rate;
    }
    // Update heading using angular rate
    float angular_rate = (current_speed_cinematic / wheelbase) * tan(steering);
    rad_angle += angular_rate * dt;

    // Convert to degrees and normalize
    float deg_angle = rad_angle / deg2rad;
    deg_angle = fmod(deg_angle, 360.0);
    if (deg_angle < 0) deg_angle += 360.0;

    // Store the final angle
    *car_angle = (int)deg_angle;

    // Update position
    *car_x += current_speed_cinematic * cos(-deg_angle*deg2rad) * dt;
    *car_y += current_speed_cinematic * sin(-deg_angle*deg2rad) * dt;

#endif /* CINEMATIC_MODEL */
}















void check_collisions()
{
	int detectedCollision = 0; // Flag to indicate a collision

	// Check for collisions with cones
	for (int i = 0; i < MAX_DETECTED_CONES; i++)
	{
		if (cones[i].color != -1)
		{
			float dx = car_x - cones[i].x/px_per_meter;
			float dy = car_y - cones[i].y/px_per_meter;
			float distance = sqrt(dx * dx + dy * dy);

			// Calculate angle between car and cone
			float angle_to_cone = atan2(dy, dx) / deg2rad;  // Convert to degrees
			float relative_angle = fmod(angle_to_cone - car_angle + 360, 360);  // Normalize to [0,360]
			
			// Set threshold distance based on angle
			float threshold;

			if (relative_angle <= 30 || relative_angle >= 330 ||  // Front cone [-30,+30]
				(relative_angle >= 210 && relative_angle <= 360)) { // Back cone [210,360]
				threshold = cone_radius + 0.25;  // Add 0.5 meter for front/back collisions (simulation car length)
			}
			else {
				threshold = cone_radius;  // Side cone
			}

			if (distance < threshold)
			{
				detectedCollision = 1; // Collision detected
				float car_heading = car_angle * deg2rad;

				// Push the cone in the direction of the car's movement
				float push_distance = threshold;  // How far to push the cone
				cones[i].x += (push_distance * cos(-car_heading)) * px_per_meter;
				cones[i].y += (push_distance * sin(-car_heading)) * px_per_meter;

				#ifdef DEBUG
				printf("Collision detected with cone %d", i);
				printf(" - Cone pushed to (%.2f, %.2f)\n", cones[i].x, cones[i].y);
				#endif /* DEBUG */
			}
		}
	}

	if (detectedCollision)
	{
		clear_bitmap(track);
		clear_to_color(track, asphalt_gray);
		
		for (int i = 0; i < MAX_CONES_MAP; i++)
		{
			if (cones[i].color != -1) // plot only track cones
			{
				circlefill(
					track, 
					(int)(cones[i].x), 
					(int)(cones[i].y), 
					cone_radius * px_per_meter, // radius = 5 cm
					cones[i].color
				);
			}
		}
	}
}

/** @} */