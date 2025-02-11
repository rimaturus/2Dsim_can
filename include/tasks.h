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