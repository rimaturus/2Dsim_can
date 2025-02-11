/**
 * @file utilities.h
 * @brief Utility function prototypes for cone management and runtime tasks.
 *
 * This header declares functions for initializing cone structures, loading cone positions
 * from a file, calculating sprite rotations, and handling runtime processes.
 */

/**
 * @brief Initializes an array of cone structures.
 *
 * This function sets up the specified array of cones so that each cone is properly initialized.
 *
 * @param cones Pointer to the first element of an array of cone structures.
 */
 
/**
 * @brief Loads cone positions from a file.
 *
 * Reads cone position data from the provided file and populates the cone array accordingly.
 * Only up to @p max_cones entries are loaded, ensuring the array bounds are respected.
 *
 * @param filename Path to the file containing cone position data.
 * @param cones Pointer to an array of cone structures where the positions will be stored.
 * @param max_cones The maximum number of cones to load from the file.
 */
 
/**
 * @brief Computes the sprite rotation angle.
 *
 * Calculates and returns a new rotation angle for a sprite based on the input @p angle.
 * The calculation may involve normalization or other transformations suitable for sprite handling.
 *
 * @param angle The initial angle value (in degrees or radians, as defined by the application's convention).
 * @return The adjusted angle after applying the rotation transformation.
 */
 
/**
 * @brief Handles runtime processes based on a stop signal.
 *
 * Executes runtime tasks, potentially including logging or task management, until a stop condition
 * defined by @p stop_signal is met. The associated task description is provided via @p task_name.
 *
 * @param stop_signal Signal value indicating when to terminate the runtime process.
 * @param task_name A string identifier for the task being executed.
 */
#ifndef UTILITIES_H
#define UTILITIES_H

#include <perception.h>
#include <globals.h>

void	init_cones(cone *cones);
void	load_cones_positions(const char *filename, cone *cones, int max_cones);
float	angle_rotation_sprite(float angle);
void	runtime(int stop_signal, char* task_name);

#endif // UTILITIES_H