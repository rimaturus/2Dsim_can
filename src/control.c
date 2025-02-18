/**
 * @file control.c
 * @brief Implements vehicle control functions including both manual (keyboard) and autonomous (centerline based) control.
 *
 * This file contains methods to update the vehicle's motion state. The manual control routine (keyboard_control)
 * processes user keystrokes to adjust speed (pedal) and steering angle, while the autonomous control routine uses
 * a centerline of waypoints to compute a reference trajectory and derive the appropriate steering correction.
 */

#include <stdio.h>
#include <math.h>
#include <allegro.h>

#include "trajectory.h"
#include "perception.h"
#include "globals.h"
#include "control.h"
#include "vehicle.h"

/**
 * @brief Adjusts vehicle controls based on keyboard input.
 *
 * This function processes the state of keyboard keys to adjust the vehicle's pedal (speed) and steering angle.
 * It increments or decrements the pedal value and steering angle within pre-defined limits depending on the keys
 * pressed (e.g., KEY_UP, KEY_DOWN, KEY_LEFT, KEY_RIGHT). After updating these control signals, the vehicle's state
 * is updated by calling the vehicle_model function.
 */
void keyboard_control();

/**
 * @brief Autonomous control routine using centerline waypoints.
 *
 * Implements an autonomous control strategy based on a provided centerline represented by an array of waypoints.
 * The function follows these steps:
 *  - Counts the number of valid centerline waypoints (terminated when a waypoint with x < 0.0f is found).
 *  - Filters the centerline to extract waypoints that are located ahead of the vehicle using the
 *    is_in_front helper.
 *  - Identifies the closest valid waypoint ahead, and depending on the availability of neighboring points,
 *    computes a reference trajectory vector. This is done in one of three ways:
 *      * If at least three valid ahead waypoints exist, the reference vector is computed from the previous to the
 *        next waypoint surrounding the closest ahead waypoint.
 *      * If there are only one or two ahead points (but at least two total waypoints), the reference vector is
 *        derived by combining the vector from the vehicle to the last waypoint and the segment between the last
 *        two waypoints.
 *      * If only one waypoint is available, the vector from the vehicle to that waypoint is used as the reference.
 *  - The computed reference trajectory is normalized. If normalization fails, a default forward direction is used.
 *  - The vehicle's current heading is computed as a unit vector based on local_car_angle.
 *  - The required steering correction (delta) is obtained by computing the sine of the angle difference through the 2D cross
 *    product between the normalized reference vector and the heading vector.
 *  - A constant pedal value is applied.
 *  - Finally, the updated control signals (pedal and delta for steering) are applied to the vehicle by calling vehicle_model.

 * @param[in]     center_waypoints Pointer to an array of waypoints representing the desired centerline trajectory.
 *                                 The array should be terminated by a waypoint with x < 0.0f.
 */
void autonomous_control(waypoint *center_waypoints);

waypoint reordered_ahead[MAX_WAYPOINTS];
int wp_ahead_idx = 0;

float pedal = 0.0f;         // current speed in m per step
float steering = 0.0f;      // current steering angle in radians









void keyboard_control()
{
const float     accel_step = 0.01;    // speed increment per key press
const float     brake_step = 0.1;     // brake increment per key press

const float     steering_step = 30 * deg2rad; // steering increment in radians per key press
const float     max_steering = 30 * deg2rad;     // maximum steering angle in radians

	// Adjust speed
	if (key[KEY_UP])
	{
		pedal = (pedal + accel_step) > 1.0 ? 1.0 : (pedal + accel_step);
	}
	if (key[KEY_DOWN])
	{
		pedal = (pedal - accel_step) < -1.0 ? -1.0 : (pedal - brake_step);
	}

	// Adjust steering angle
	if (key[KEY_LEFT])
	{
		steering += steering_step;
		if (steering > max_steering)
			steering = max_steering;
	}
	else if (key[KEY_RIGHT])
	{
		steering -= steering_step;
		if (steering < -max_steering)
			steering = -max_steering;
	}
    else
    {
        steering = 0.0f;
    }

	// Motion model of the vehicle
	vehicle_model(pedal, steering);
}

float bound_steering(float steering_rads)
{
    const float     max_steering = 30 * deg2rad;     // maximum steering angle in radians

    if (steering_rads > max_steering)
    {
        steering_rads = max_steering;
    }
    else if (steering_rads < -max_steering)
    {
        steering_rads = -max_steering;
    }
    return steering_rads;
}

void autonomous_control(waypoint *center_waypoints)
{   
    pthread_mutex_lock(&map_mutex); // Begin critical section
	float local_car_x = car_x;
	float local_car_y = car_y;
    float local_car_angle = car_angle;
	pthread_mutex_unlock(&map_mutex); // End critical section

    if (center_waypoints == NULL){
        return; // the trajectory is not computed yet
    }

    float car_versor_x = cos(-local_car_angle * deg2rad);
    float car_versor_y = sin(-local_car_angle * deg2rad);

    waypoint waypoints_ahead[MAX_WAYPOINTS];
    wp_ahead_idx = 0;

    int wp_idx = 0;    
    while ((wp_idx < MAX_WAYPOINTS) && (center_waypoints[wp_idx].x >= 0.0f))
    {
        float car2wp_x = center_waypoints[wp_idx].x - local_car_x;
        float car2wp_y = center_waypoints[wp_idx].y - local_car_y;

        float norm_car2wp = sqrt(car2wp_x * car2wp_x + car2wp_y * car2wp_y);

        float car2wp_versor_x = car2wp_x/norm_car2wp;
        float car2wp_versor_y = car2wp_y/norm_car2wp;

        float dot_product = car2wp_versor_x * car_versor_x + car2wp_versor_y * car_versor_y;
        // dot product = norm of the vectors * cos of the angle between them
        // since the norm is 1 == cos

        // if cos > 0 ==> angle car - waypoint < 90° (abs)
        int ctrl_ahead = (dot_product > 0) ? 1 : 0;

        if (!ctrl_ahead){
            wp_idx++;
            continue;
        }
        else
        {
            // append to waypoints_ahead 
            waypoints_ahead[wp_ahead_idx].x = center_waypoints[wp_idx].x;
            waypoints_ahead[wp_ahead_idx].y = center_waypoints[wp_idx].y;
            wp_ahead_idx++;
        } 

        wp_idx++;
    }

    if (wp_ahead_idx == 0) return;

    // here I have more than 1 waypoint ahead
    int visited_idx[wp_ahead_idx]; // map of visited idx [i-th visited if =1, else =0]

    for (int i = 0; i < wp_ahead_idx; i++) // initialize
    {
        visited_idx[i] = 0;
        reordered_ahead[i].x = -1;
        reordered_ahead[i].y = -1;
    }

    float current_x = local_car_x;
    float current_y = local_car_y;

    for (int j = 0; j < wp_ahead_idx; j++)
    {
        float minDist = maxRange;
        int minDist_idx = -1;

        // start reordering
        for (int i = 0; i < wp_ahead_idx; i++)
        {
            if (visited_idx[i] == 1) continue;

            float distance = sqrt(pow(waypoints_ahead[i].x - current_x, 2) + pow(waypoints_ahead[i].y - current_y, 2));

            if (distance < minDist)
            {
                minDist = distance;
                minDist_idx = i;
            }
        }

        current_x = waypoints_ahead[minDist_idx].x;
        current_y = waypoints_ahead[minDist_idx].y;

        reordered_ahead[j].x = waypoints_ahead[minDist_idx].x;
        reordered_ahead[j].y = waypoints_ahead[minDist_idx].y;

        visited_idx[minDist_idx] = 1;
    }

    // reordered_ahead is ordered
    float car2wp2_x = reordered_ahead[3].x - local_car_x;
    float car2wp2_y = reordered_ahead[3].y - local_car_y;

    float norm_car2wp2 = sqrt(car2wp2_x * car2wp2_x + car2wp2_y * car2wp2_y);

    float car2wp2_versor_x = car2wp2_x/norm_car2wp2;
    float car2wp2_versor_y = car2wp2_y/norm_car2wp2;

    float cross_product = car_versor_y * car2wp2_versor_x - car_versor_x * car2wp2_versor_y;

    float steer_target = bound_steering(asinf(cross_product));
    // printf("cross_product: %f\n", cross_product);
    // printf("asinf(cross_product): %f\n", asinf(cross_product)/deg2rad );

    // steering = (steer_target > 0) ? 1 : -1;
    pedal = 0.05;

    vehicle_model(pedal, steer_target);
}
