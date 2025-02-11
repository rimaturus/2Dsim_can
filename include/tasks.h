/**
 * @file tasks.h
 * @brief Declaration of task thread functions for the simulation.
 *
 * This header file declares thread functions for various tasks in the simulation:
 * - Perception: Processes sensor or simulation data to interpret the environment.
 * - Trajectory: Computes and updates planned paths or trajectories.
 * - Control: Implements control logic that may handle both manual and autonomous inputs.
 * - Display: Manages rendering of the simulation's visual output.
 *
 * Each task is designed to be executed in its own thread, and the functions
 * follow the prototype required by POSIX thread routines.
 */

/**
 * @brief Perception task.
 *
 * This function executes perception related operations, such as processing sensor
 * data or simulation input to understand the surrounding environment.
 *
 * @param arg Pointer to the arguments required by the task. The actual type and usage
 *            of this parameter are defined by the implementation context.
 * @return Pointer representing the function's outcome, typically not used.
 */
 
/**
 * @brief Trajectory planning task.
 *
 * This function is responsible for calculating and updating the movement trajectory 
 * for the entity in simulation. It handles the logic for path planning based on
 * environmental data and simulation dynamics.
 *
 * @param arg Pointer to the arguments specific to the trajectory task.
 * @return Pointer representing the result of the function, usually unused.
 */

/**
 * @brief Control task.
 *
 * This function manages the control logic, determining inputs for either keyboard
 * control or autonomous behavior. It integrates various control signals and decisions,
 * ensuring the entity reacts appropriately within the simulation.
 *
 * @param arg Pointer to a structure or value containing control parameters.
 * @return Pointer to the result of the control processing, typically not utilized.
 */

/**
 * @brief Display update task.
 *
 * This function handles the rendering process for the simulation's trajectory display.
 * It updates the visual output based on the current state of the simulation data.
 *
 * @param arg Pointer to display specific parameters or context information.
 * @return Pointer indicating the outcome of the display operation, generally unused.
 */
#ifndef TASKS_H
#define TASKS_H

#include "utilities.h"  // For runtime
#include "control.h"    // For keyboard_control and autonomous_control
#include "display.h"    // For draw_trajectory

void *perception_task(void *arg);
void *trajectory_task(void *arg);
void *control_task(void *arg);
void *display_task(void *arg);

#endif // TASKS_H