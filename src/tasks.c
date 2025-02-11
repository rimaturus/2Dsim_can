#include <stdio.h>

#include "globals.h"	// for shared globals
#include "tasks.h"
#include "perception.h"	// for LiDAR and mapping functions
#include "trajectory.h"	// for trajectory planning
#include "vehicle.h"	// to control the vehicle + vehicle model
#include "display.h"	// to draw on screen
#include "ptask.h"		// for periodic tasks

// Periodic task functions (using ptask.h notation)
void *perception_task(void *arg)
{
	int task_id = get_task_index(arg);
	wait_for_activation(task_id);

	while (!key[KEY_ESC])
	{
		runtime(0, "PERCEPTION");
		
		lidar(car_x, car_y, measures);

		for (int i = 0; i < MAX_DETECTED_CONES; i++){
			detected_cones[i].x = -1;
			detected_cones[i].y = -1;
			detected_cones[i].color = -1;
		}

		mapping(car_x, car_y, car_angle, detected_cones); // Pass the address of first element
		sem_post(&lidar_sem);

		runtime(1, "PERCEPTION");

		wait_for_period(task_id);
	}
	return NULL;
}

void *trajectory_task(void *arg)
{
	int task_id = get_task_index(arg);
	wait_for_activation(task_id);

	while (!key[KEY_ESC])
	{
		runtime(0, "TRAJ_PLANNING");

		sem_wait(&lidar_sem);
		trajectory_planning(car_x, car_y, car_angle, detected_cones, trajectory);

		runtime(1, "TRAJ_PLANNING");

		wait_for_period(task_id);
	}
	return NULL;
}

void *control_task(void *arg)
{
    int task_id = get_task_index(arg);
    wait_for_activation(task_id);

    while (!key[KEY_ESC])
    {
		runtime(0, "CONTROL");

		if (!key[KEY_A]){
        	keyboard_control(&car_x, &car_y, &car_angle);
		}
		else
			autonomous_control(&car_x, &car_y, &car_angle, trajectory);
		

		runtime(1, "CONTROL");

        wait_for_period(task_id);
    }
    return NULL;
}

void *display_task(void *arg)
{
	int task_id = get_task_index(arg);
	wait_for_activation(task_id);

	while (!key[KEY_ESC])
	{
		runtime(0, "DISPLAY");

		update_display();
		
		runtime(1, "DISPLAY");

		wait_for_period(task_id);

	}
	return NULL;
}
