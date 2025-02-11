#ifndef CONTROL_H
#define CONTROL_H

#include <trajectory.h>
#include "globals.h"
#include "perception.h"

extern float 	steering; // in radians
extern float	pedal; // [0 -> 1]


void keyboard_control(float *car_x, float *car_y, int *car_angle);
void autonomous_control(float *car_x, float *car_y, int *car_angle, waypoint *trajectory);

#endif // CONTROL_H