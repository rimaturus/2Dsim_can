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
 *
 * @param[in,out] car_x   Pointer to the vehicle's x-coordinate (in meters).
 * @param[in,out] car_y   Pointer to the vehicle's y-coordinate (in meters).
 * @param[in,out] car_angle Pointer to the vehicle's orientation angle (in degrees).
 */
void keyboard_control(float *car_x, float *car_y, int *car_angle);

/**
 * @brief Normalizes a 2D vector.
 *
 * Computes the Euclidean norm of the vector specified by the pointers x and y. If the norm is above a
 * small threshold (to avoid division by zero), the vector components are divided by the norm to yield a unit
 * vector.
 *
 * @param[in,out] x Pointer to the x component of the vector. The normalized value is written back.
 * @param[in,out] y Pointer to the y component of the vector. The normalized value is written back.
 *
 * @return 1 if the vector is non-zero and has been normalized; 0 if the vector length is negligible.
 */
static int nomalize(float *x, float *y);

/**
 * @brief Determines if a given waypoint is ahead of the vehicle.
 *
 * Evaluates whether the waypoint (defined by wx and wy) is in front of the vehicle by computing the
 * angle between the vehicle's heading (in degrees) and the vector from the vehicle (cx, cy) to the waypoint.
 * The waypoint is considered "ahead" if the absolute angular difference is within ±100°.
 *
 * @param[in] car_angle_deg The current heading of the vehicle in degrees.
 * @param[in] wx            The x-coordinate of the waypoint.
 * @param[in] wy            The y-coordinate of the waypoint.
 * @param[in] cx            The x-coordinate of the vehicle.
 * @param[in] cy            The y-coordinate of the vehicle.
 *
 * @return Non-zero value if the waypoint is in front of the vehicle; 0 otherwise.
 */
static int is_in_front(float car_angle_deg, float wx, float wy, float cx, float cy);

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
 *  - The vehicle's current heading is computed as a unit vector based on car_angle.
 *  - The required steering correction (delta) is obtained by computing the sine of the angle difference through the 2D cross
 *    product between the normalized reference vector and the heading vector.
 *  - A constant pedal value is applied.
 *  - Finally, the updated control signals (pedal and delta for steering) are applied to the vehicle by calling vehicle_model.
 *
 * @param[in,out] car_x          Pointer to the vehicle's x-coordinate (in meters).
 * @param[in,out] car_y          Pointer to the vehicle's y-coordinate (in meters).
 * @param[in,out] car_angle      Pointer to the vehicle's orientation angle (in degrees).
 * @param[in]     center_waypoints Pointer to an array of waypoints representing the desired centerline trajectory.
 *                                 The array should be terminated by a waypoint with x < 0.0f.
 */
void autonomous_control(float *car_x, float *car_y, int *car_angle, waypoint *center_waypoints);

float pedal = 0.0f;         // current speed in m per step
float steering = 0.0f;      // current steering angle in radians

void keyboard_control(float *car_x, float *car_y, int *car_angle)
{
const float     accel_step = 0.01;    // speed increment per key press

const float     steering_step = 1.5 * deg2rad; // steering increment in radians per key press
const float     max_steering = 30 * deg2rad;     // maximum steering angle in radians

	// Adjust speed
	if (key[KEY_UP])
	{
		pedal = (pedal + accel_step) > 1.0 ? 1.0 : (pedal + accel_step);
	}
	if (key[KEY_DOWN])
	{
		pedal = (pedal - accel_step) < -1.0 ? -1.0 : (pedal - accel_step);
	}

	// Adjust steering angle
	if (key[KEY_LEFT])
	{
		steering += steering_step;
		if (steering > max_steering)
			steering = max_steering;
	}
	if (key[KEY_RIGHT])
	{
		steering -= steering_step;
		if (steering < -max_steering)
			steering = -max_steering;
	}

	// Motion model of the vehicle
	vehicle_model(car_x, car_y, car_angle, pedal, steering);
}

/* 

// Helper function to normalize angle to [-180, 180] range (in degrees).
static float normalize_angle_deg(float angle_deg) {
    while (angle_deg > 180.0f)  angle_deg -= 360.0f;
    while (angle_deg < -180.0f) angle_deg += 360.0f;
    return angle_deg;
}

// Helper to check if a waypoint is within +/- 90° of car’s heading (i.e., “in front”).
static int is_in_front(float car_angle_deg, float wx, float wy, float cx, float cy) {
    // dx, dy to the waypoint
    float dx = wx - cx;
    float dy = wy - cy;

    // Angle from car to waypoint, in degrees
    float angle_to_point = (atan2f(dy, dx) / deg2rad);
    float angle_diff = normalize_angle_deg(angle_to_point - car_angle_deg);

    // "In front" if within +/- 90 deg
    return (fabs(angle_diff) <= 100.0f);
}

#define LOOKAHEAD_DISTANCE 3.0f

// Main autonomous control function
void autonomous_control(float *car_x, float *car_y, int *car_angle, waypoint *trajectory)
{
    // 1) Collect all front waypoints
    waypoint front_waypoints[MAX_DETECTED_CONES];
    float    front_dists[MAX_DETECTED_CONES];
    int front_count = 0;

    for (int i = 0; i < MAX_DETECTED_CONES; i++) {
        if (trajectory[i].x == -1.0f) break; // no more valid points
        if (is_in_front((float)*car_angle, trajectory[i].x, trajectory[i].y,
                        *car_x, *car_y))
        {
            // record the waypoint
            front_waypoints[front_count] = trajectory[i];
            // record distance to car
            float dx = trajectory[i].x - *car_x;
            float dy = trajectory[i].y - *car_y;
            front_dists[front_count] = sqrtf(dx*dx + dy*dy);
            front_count++;
        }
    }

    // If no front waypoints, do fallback
    if (front_count == 0) {
        steering = 0.0f;
        vehicle_model(car_x, car_y, car_angle, 0.0f, steering);
        return;
    }

    // 2) Sort the front waypoints by ascending distance (simple insertion or bubble sort for clarity)
    //    You can swap in a standard library sort or a faster approach
    for (int i = 0; i < front_count - 1; i++) {
        for (int j = i + 1; j < front_count; j++) {
            if (front_dists[j] < front_dists[i]) {
                // swap distances
                float tmpDist = front_dists[i];
                front_dists[i] = front_dists[j];
                front_dists[j] = tmpDist;
                // swap waypoints
                waypoint tmpWp = front_waypoints[i];
                front_waypoints[i] = front_waypoints[j];
                front_waypoints[j] = tmpWp;
            }
        }
    }

    // 3) Find the waypoint whose distance >= LOOKAHEAD_DISTANCE
    int chosen_idx = front_count - 1; // default to the farthest if none are >=
    for (int i = 0; i < front_count; i++) {
        if (front_dists[i] >= LOOKAHEAD_DISTANCE) {
            chosen_idx = i;
            break;
        }
    }

    waypoint chosen_wp = front_waypoints[chosen_idx];

#ifdef DEBUG
    // Draw all front waypoints in green
    for (int i = 0; i < front_count; i++) {
        int px = (int)(front_waypoints[i].x * px_per_meter);
        int py = (int)(front_waypoints[i].y * px_per_meter);
        circlefill(screen, px, py, 3, makecol(0, 255, 0)); 
    }
    // Draw chosen waypoint in black
    {
        int px = (int)(chosen_wp.x * px_per_meter);
        int py = (int)(chosen_wp.y * px_per_meter);
        circlefill(screen, px, py, 3, makecol(0, 0, 0));
    }
#endif

    // 4) Compute angle diff to chosen waypoint
    float dx = chosen_wp.x - *car_x;
    float dy = chosen_wp.y - *car_y;
    float target_angle_deg = (atan2f(dy, dx) / deg2rad);
    float angle_diff_deg = normalize_angle_deg(target_angle_deg - (float)*car_angle);

    // 5) Steering: scale angle diff in radians; adjust factor as needed
    steering = +(angle_diff_deg * deg2rad);// * 0.5f;

    // 6) Pedal: a simple constant, or do something more complex (like slow down for sharp turns)
    float pedal = 0.1f;

    // 7) Update vehicle
    vehicle_model(car_x, car_y, car_angle, pedal, steering);
} */


// Helper: Normalize an (x,y) vector; returns 0 if vector length is 0.
static int nomalize(float *x, float *y) {
    float norm = sqrtf((*x) * (*x) + (*y) * (*y));
    if (norm < 1e-6f) return 0;
    *x /= norm;
    *y /= norm;
    return 1;
}

//---------------------------------------------------------------------
// Helper: Check if a waypoint is "ahead" of the car. 
// Here we use the provided idea: compute the angle between the vector from
// the car to the waypoint and the car's heading. (Assumes car_angle is in degrees.)
static int is_in_front(float car_angle_deg, float wx, float wy, float cx, float cy) {
    float dx = wx - cx;
    float dy = wy - cy;
    float angle_to_point = (atan2f(dy, dx) / deg2rad);  // convert to degrees
    // Normalize the angle difference to [-180, 180]
    float angle_diff = angle_to_point - car_angle_deg;
    while (angle_diff > 180.0f)  angle_diff -= 360.0f;
    while (angle_diff < -180.0f) angle_diff += 360.0f;
    // Consider "ahead" if within +/- 100° (adjust threshold as needed)
    return (fabs(angle_diff) <= 100.0f);
}

//---------------------------------------------------------------------
// The new autonomous control routine that uses the centerline logic.
void autonomous_control(float *car_x, float *car_y, int *car_angle, 
                                     waypoint *center_waypoints)
{
	int num_waypoints = 0;
	while (center_waypoints[num_waypoints].x >= 0.0f) num_waypoints++;

    // Make sure we have at least one waypoint.
    if (num_waypoints <= 0) {
        // Fallback: do nothing.
        vehicle_model(car_x, car_y, car_angle, 0.0f, 0.0f);
        return;
    }
    
    // 1) Filter the center_waypoints to obtain only the ones "ahead" of the car.
    // Also, find the one closest to the car (in front).
    int n_waypoint_index = -1;  // index (in center_waypoints) of the closest ahead point
    float min_dist = 1e6;
    int count_ahead = 0;
    // (Optional: if you need the list of ahead waypoints, you can store them in an array.)
    waypoint ahead_waypoints[100];

    for (int i = 0; i < num_waypoints; i++) {
        if (is_in_front((float)*car_angle, center_waypoints[i].x, center_waypoints[i].y, *car_x, *car_y)) {
            ahead_waypoints[count_ahead++] = center_waypoints[i];
            float dx = center_waypoints[i].x - *car_x;
            float dy = center_waypoints[i].y - *car_y;
            float d = sqrtf(dx * dx + dy * dy);
            if (d < min_dist) {
                min_dist = d;
                n_waypoint_index = i;
            }
        }
    }

    // If no ahead waypoints are found, then use a fallback (e.g., go straight)
    if (count_ahead == 0 || n_waypoint_index < 0) {
        vehicle_model(car_x, car_y, car_angle, 0.0f, 0.0f);
        return;
    }

    // 2) Determine the desired (reference) trajectory vector.
    float ref_vec_x = 0.0f, ref_vec_y = 0.0f;

    if ((count_ahead > 2) && (n_waypoint_index > 0) && (n_waypoint_index + 1 < num_waypoints)) {
        // Case A: We have at least 3 points (previous, current, next).
        // Use the points [n_waypoint_index-1, n_waypoint_index, n_waypoint_index+1].
        // One simple way is to define the desired direction as the vector from the previous
        // waypoint to the next one (i.e. “smoothing” the trajectory).
        waypoint prev = center_waypoints[n_waypoint_index - 1];
        waypoint next = center_waypoints[n_waypoint_index + 1];
        ref_vec_x = next.x - prev.x;
        ref_vec_y = next.y - prev.y;
    }
    else {
        if (num_waypoints > 2) {
            // Case B: If there is only one or two ahead points, use the last two waypoints of the full centerline.
            waypoint last_wp = center_waypoints[num_waypoints - 1];
            waypoint second_last = center_waypoints[num_waypoints - 2];
            // Here we add the vector from the car to the last waypoint with the segment between the last two points.
            ref_vec_x = (last_wp.x - *car_x) + (last_wp.x - second_last.x);
            ref_vec_y = (last_wp.y - *car_y) + (last_wp.y - second_last.y);
        }
        else {
            // Case C: Only one waypoint is available.
            waypoint last_wp = center_waypoints[num_waypoints - 1];
            ref_vec_x = last_wp.x - *car_x;
            ref_vec_y = last_wp.y - *car_y;
        }
    }
    
    // Normalize the reference trajectory vector.
    if (!nomalize(&ref_vec_x, &ref_vec_y)) {
        // If normalization fails, set a default forward direction.
        ref_vec_x = 1.0f;
        ref_vec_y = 0.0f;
    }
    
    // 3) Compute the car’s current heading unit vector.
    // Here, car_angle is in degrees so we convert it to radians.
    float car_angle_rad = (*car_angle) * deg2rad;
    float dir_x = cosf(car_angle_rad);
    float dir_y = sinf(car_angle_rad);
    
    // 4) Compute the cross product between the reference vector and the current direction.
    // In 2D the cross product (scalar) is: ref_x*dir_y - ref_y*dir_x.
    float cross = ref_vec_x * dir_y - ref_vec_y * dir_x;
    
    // Since both vectors are unit length, sin(theta) = cross. Compute the angle difference.
    float delta = asinf(cross);  // desired steering correction (in radians)
    
    // (Optional: If you have a PID controller, you can process delta here.)
    // delta = pid_compute(desired, delta);

    
    // 6) (Optional) Choose a pedal value – here we use a constant.
    float pedal = 0.1f;   // adjust as needed
    
#ifdef DEBUG
    printf("Car pos=(%.2f, %.2f), car_angle=%d\n", *car_x, *car_y, *car_angle);
    printf("Ref vector: (%.2f, %.2f), Delta=%.2f rad\n", ref_vec_x, ref_vec_y, delta);
#endif

    // 7) Finally, update the vehicle’s model with the computed control signals.
    vehicle_model(car_x, car_y, car_angle, pedal, delta);
}
