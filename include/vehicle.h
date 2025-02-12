/**
 * @brief Simulates the vehicle's movement by updating its position and orientation.
 *
 * This function computes and updates the vehicle's state based on the input controls.
 * The vehicle's x and y coordinates, as well as its orientation angle, are modified
 * to reflect the effect of the applied pedal (acceleration/brake) and steering inputs.
 *
 * @param car_x Pointer to the vehicle's x-coordinate. The updated x position is stored here.
 * @param car_y Pointer to the vehicle's y-coordinate. The updated y position is stored here.
 * @param car_angle Pointer to the vehicle's current orientation angle (in degrees or radians).
 *                  The updated angle is stored here.
 * @param pedal Control input for acceleration (or braking). A non-zero value indicates a change in speed.
 * @param steering Control input for the steering direction. A non-zero value modifies the vehicle's heading.
 *
 * @note Ensure that the pointers provided (car_x, car_y, car_angle) are valid and point to allocated memory.
 *       The scaling and meaning of the 'pedal' and 'steering' inputs should correspond to the mechanics of the
 *       vehicle model being simulated.
 */
#ifndef VEHICLE_H
#define VEHICLE_H

void vehicle_model(float *car_x, float *car_y, int *car_angle, float pedal, float steering);
void check_collisions();

#endif // VEHICLE_H