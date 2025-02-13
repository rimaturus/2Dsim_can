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
#define NON_CINEMATIC_MODEL

#ifndef CINEMATIC_MODEL
// Simulation parameters
const float     dt = (float)(CONTROL_PERIOD)/100;			// time step

const float		mass = 100;      	// mass of the vehicle [kg]
const float     wheelbase = 3.0;	// distance between front and rear axles [m]
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
#endif /* NON CINEMATIC_MODEL */

#ifdef CINEMATIC_MODEL
    //=========================================================================
    // Simulation parameters and realistic vehicle constants
    //=========================================================================
    const float dt = 0.1;  // simulation time step [s]

    // Vehicle parameters (typical for a small car)
    const float mass = 1200.0;      // vehicle mass [kg]
    const float wheelbase = 2.5;    // wheelbase [m]
    // For a bicycle model we partition the wheelbase:
    const float l_r = 1.0;          // distance from the center-of-mass to the rear axle [m]
    // const float l_f = wheelbase - l_r;  // front axle distance [m]

    // Engine / brake characteristics:
    // Assume pedal is in the range [-1, 1] (negative for braking)
    const float max_engine_force = 2000.0;  // maximum engine force [N]
    const float max_brake_force = 8000.0;     // maximum braking force [N]
	const float max_speed = 1.50;  // maximum speed [m/s]

    // Aerodynamic and rolling resistances
    const float air_density = 1.225;     // [kg/m^3]
    const float drag_coefficient = 0.32; // typical drag coefficient
    const float frontal_area = 2.2;      // [m^2]
    const float rolling_resistance = 0.015;  // rolling resistance coefficient
    const float g = 9.81;  // gravitational acceleration [m/s^2]

    //=========================================================================
    // Persistent state (across calls)
    //=========================================================================
    static float speed = 0.0;  // vehicle speed [m/s]

    //=========================================================================
    // Compute the drive/brake force from the pedal input.
    // (A positive pedal applies engine force; a negative pedal applies braking.)
    //=========================================================================
    float drive_force = 0.0;
    if (pedal >= 0.0) {
        drive_force = pedal * max_engine_force;
    } else {
        drive_force = pedal * max_brake_force;  // note: pedal is negative
    }

    //=========================================================================
    // Compute forces opposing motion:
    //   - Aerodynamic drag (proportional to speed²)
    //   - Rolling resistance (approximately constant)
    //=========================================================================
    float drag_force = 0.5f * air_density * drag_coefficient *
                       frontal_area * speed * speed;
    float rolling_force = rolling_resistance * mass * g;

    // The drag and rolling forces always oppose the direction of motion.
    float net_force = drive_force;
    if (speed > 0)
        net_force -= (drag_force + rolling_force);
    else if (speed < 0)
        net_force += (drag_force + rolling_force);

    //=========================================================================
    // Update the longitudinal dynamics (Newton’s 2nd law: F = m*a)
    //=========================================================================
    float acceleration = net_force / mass;
    speed += acceleration * dt;
    if (speed < 0) {
        speed = 0;
    }
	else if (speed > max_speed) {
		speed = max_speed;
	}

    //=========================================================================
    // Update the vehicle’s position and orientation using a kinematic
    // bicycle model.
    //=========================================================================
    // Convert current heading (stored in degrees) to radians.
    float theta = (*car_angle) * deg2rad;

    // The kinematic bicycle model can include a slip angle β, which is the angle
    // between the vehicle’s velocity vector and its heading. A simple way to
    // compute β is:
    //
    //      β = arctan((l_r / wheelbase) * tan(steering))
    //
    // Here, steering is assumed to be given in radians.
    float beta = atan((l_r / wheelbase) * tan(steering));

    // Update the (x,y) position.
    // The vehicle moves in the direction (theta + beta).
    *car_x += speed * cos(theta + beta) * dt;
    *car_y -= speed * sin(theta + beta) * dt;

    // Update the heading angle.
    // A common approximation is:
    //      dθ/dt = (speed / wheelbase) * tan(steering)
    // (For a more refined model one could use yaw dynamics, but this is often
    // sufficient for simulation at low-to-moderate speeds.)
    theta += (speed / wheelbase) * tan(steering) * dt;

    // Store the updated heading back in degrees.
    *car_angle = (theta / deg2rad);

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