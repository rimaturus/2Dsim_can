/**
 * @file tasks.c
 * @brief Contains periodic task functions for LiDAR perception, trajectory planning,
 *        vehicle control, and display update in a 2D simulation environment.
 *
 * This file implements tasks that run periodically in separate threads. Each task is synchronized
 * with others through semaphores and managed by a periodic task scheduler.
 */

/**
 * @brief Periodic perception task.
 *
 * This task acquires LiDAR measurements, resets the array of detected cones,
 * and performs mapping based on the current position and orientation of the vehicle.
 * After processing the sensor data, it signals the trajectory planning task using a semaphore.
 * The task executes continuously until the ESC key is pressed.
 *
 * @param[in] arg Pointer used to determine the task's index.
 *
 * @return Always returns NULL.
 *
 * @note The function uses a runtime measurement call to monitor its execution time.
 */

/**
 * @brief Periodic trajectory planning task.
 *
 * This task waits for a signal from the perception task to ensure updated sensor data is available.
 * It then calculates a new trajectory for the vehicle based on its current position, orientation,
 * and the detected cones from the environment.
 * The task runs periodically and terminates when the ESC key is pressed.
 *
 * @param[in] arg Pointer used to determine the task's index.
 *
 * @return Always returns NULL.
 *
 * @note The task's operation is encapsulated between runtime measurement calls.
 */

/**
 * @brief Periodic control task.
 *
 * This task is responsible for controlling the vehicle's movement.
 * It chooses between keyboard-based control and autonomous control based on the state of the 'A'
 * key. If autonomous mode is active (i.e., 'A' is pressed), it uses the computed trajectory,
 * otherwise it relies on user input to drive the vehicle.
 * Execution continues until the ESC key is pressed.
 *
 * @param[in] arg Pointer used to determine the task's index.
 *
 * @return Always returns NULL.
 *
 * @note The function uses runtime measurement calls to monitor execution time per cycle.
 */

/**
 * @brief Periodic display task.
 *
 * This task updates the graphical display of the simulation environment.
 * It refreshes the on-screen information such as the vehicle's position, sensor data,
 * and other simulation elements.
 * The task continues running until the ESC key is pressed.
 *
 * @param[in] arg Pointer used to determine the task's index.
 *
 * @return Always returns NULL.
 *
 * @note Like the other tasks, execution time is tracked using runtime measurement calls.
 */
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

	sem_post(&lidar_sem);
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

	sem_post(&lidar_sem);
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

	sem_post(&lidar_sem);
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

	sem_post(&lidar_sem);
	return NULL;
}
