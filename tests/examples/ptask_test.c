#include <stdio.h>
#include <stdlib.h>
#include "ptask/ptask.h"   // Ensure this file is in your include path or the same directory
#include <pthread.h>
#include <sched.h>
#include <time.h>

/* ------------------------------------- */
/*           TASK CONSTANTS              */
/* ------------------------------------- */
#define PER1    1000   // 500 ms
#define DREL1   1000   // 500 ms
#define PRIO1   20

#define PER2    2000  // 2000 ms = 2 s
#define DREL2   2000  // 2000 ms = 2 s
#define PRIO2   10

/* ------------------------------------- */
/*           PERIODIC TASKs              */
/* ------------------------------------- */
void *task(void *arg)
{
    int i, pmax;
    int count = 1; // periodic counter

    i = get_task_index(arg);   // get the task index
    wait_for_activation(i);    // wait for the activation signal

    printf("Task %d activated\n", i);

    if (i == 1)
        pmax = 10;
    else
        pmax = 2;

    while (count <= pmax) {
        printf("Task %d, period %d\n", i, count);
        count++;
        wait_for_period(i); // wait for the next period
    }
    return NULL;
}

/* ------------------------------------- */
/*                 MAIN                  */
/* ------------------------------------- */
int main(void)
{
    // Use the default scheduling policy SCHED_OTHER instead of SCHED_FIFO
    ptask_init(SCHED_FIFO);

    if (task_create(1, task, PER1, DREL1, PRIO1, ACT) != 0) {
        fprintf(stderr, "Failed to create Task 1\n");
        exit(EXIT_FAILURE);
    }
    printf("MAIN: Task 1 created\n");

    if (task_create(2, task, PER2, DREL2, PRIO2, ACT) != 0) {
        fprintf(stderr, "Failed to create Task 2\n");
        exit(EXIT_FAILURE);
    }
    printf("MAIN: Task 2 created\n");

    printf("MAIN: waiting for Task 1 to finish\n");
    wait_for_task_end(1);
    printf("MAIN: Task 1 finished\n");

    printf("MAIN: waiting for Task 2 to finish\n");
    wait_for_task_end(2);
    printf("MAIN: Task 2 finished\n");

    return 0;
}

