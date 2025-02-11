#ifndef UTILITIES_H
#define UTILITIES_H

#include <perception.h>
#include <globals.h>

void	init_cones(cone *cones);
void	load_cones_positions(const char *filename, cone *cones, int max_cones);
float	angle_rotation_sprite(float angle);
void	runtime(int stop_signal, char* task_name);

#endif // UTILITIES_H