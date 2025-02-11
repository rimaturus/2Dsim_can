#ifndef PTASK_H
#define PTASK_H

#ifdef __cplusplus
extern "C" {
#endif

#include <pthread.h>
#include <semaphore.h>
#include <stdio.h>
#include <time.h>
#include <string.h>  // For strerror()

/* ---------------------------*/
/*       Configuration        */
/* ---------------------------*/

#define MAX_TASKS 32

/* Time units */
#define MICRO 1
#define NANO  2

/* Activation flag for tasks */
#define ACT   1
#define DEACT 0

/* ---------------------------*/
/*     Data Type Definitions  */
/* ---------------------------*/

/*
 * We define our own time structure.
 * (It has the same layout as struct timespec, so we can cast its address
 * to (struct timespec *) when calling clock_gettime(), clock_nanosleep(), etc.)
 */
typedef struct timespec_custom {
    time_t tv_sec;
    long   tv_nsec;
} timespec_custom;

/*
 * Task parameter structure.
 * Each periodic task is described by one instance of this structure.
 */
typedef struct task_par {
    int arg;           /* Task index or identifier */
    int period;        /* Period (in milliseconds) */
    int deadline;      /* Relative deadline (in milliseconds) */
    int prio;          /* Scheduling priority */
    int dmiss;         /* Deadline miss counter */
    timespec_custom at;/* Next activation time */
    timespec_custom dl;/* Current deadline time */
    pthread_t tid;     /* Thread identifier */
    sem_t asem;        /* Semaphore used for task activation */
} task_par;

/* ---------------------------*/
/*       Global Variables     */
/* ---------------------------*/

/*
 * The following globals are defined in the implementation section.
 * In user files that include ptask.h (but do not define PTASK_IMPLEMENTATION)
 * these symbols are declared as extern.
 */
extern task_par tp[MAX_TASKS];
extern timespec_custom ptask_t0;  /* Reference “zero‐time” for the system */
extern int ptask_policy;          /* Scheduling policy to be used */

/* ---------------------------*/
/*      Function Prototypes   */
/* ---------------------------*/

/* Time utility functions */
void time_copy(timespec_custom *td, timespec_custom ts);
void time_add_ms(timespec_custom *t, int ms);
int  time_cmp(timespec_custom t1, timespec_custom t2);

/* Initialization and system time */
void ptask_init(int policy);
long get_systime(int unit);

/* Periodic task support functions */
int  task_create(int i,
                 void *(*task)(void *),
                 int period,
                 int drel,
                 int prio,
                 int aflag);
int  get_task_index(void *arg);
void wait_for_activation(int i);
void task_activate(int i);
int  deadline_miss(int i);
void wait_for_period(int i);
void task_set_period(int i, int per);
void task_set_deadline(int i, int drel);
int  task_period(int i);
int  task_deadline(int i);
int  task_dmiss(int i);
void task_atime(int i, timespec_custom *at);
void task_adline(int i, timespec_custom *dl);
void wait_for_task_end(int i);

#ifdef __cplusplus
}
#endif

/* -----------------------------------------------------------------*/
/*             Implementation Section                             */
/*     (Define PTASK_IMPLEMENTATION in one source file to compile     */
/*      the definitions.  Other files will see only the declarations.)*/
/* -----------------------------------------------------------------*/
#ifdef PTASK_IMPLEMENTATION

/* Global variable definitions */
task_par tp[MAX_TASKS];
timespec_custom ptask_t0;
int ptask_policy;

/* Copies the content of a timespec_custom into another. */
void time_copy(timespec_custom *td, timespec_custom ts)
{
    td->tv_sec  = ts.tv_sec;
    td->tv_nsec = ts.tv_nsec;
}

/* Adds a given number of milliseconds to a timespec_custom value. */
void time_add_ms(timespec_custom *t, int ms)
{
    t->tv_sec += ms / 1000;
    t->tv_nsec += (ms % 1000) * 1000000;
    if (t->tv_nsec > 1000000000) {
        t->tv_nsec -= 1000000000;
        t->tv_sec++;
    }
}

/* Compares two timespec_custom values.
 * Returns 1 if t1 > t2, -1 if t1 < t2, and 0 if equal.
 */
int time_cmp(timespec_custom t1, timespec_custom t2)
{
    if (t1.tv_sec > t2.tv_sec)
        return 1;
    if (t1.tv_sec < t2.tv_sec)
        return -1;
    if (t1.tv_nsec > t2.tv_nsec)
        return 1;
    if (t1.tv_nsec < t2.tv_nsec)
        return -1;
    return 0;
}

/*
 * Initializes the periodic task system.
 * This function sets the scheduling policy and gets a reference system time.
 * It also initializes the activation semaphores for each task.
 */
void ptask_init(int policy)
{
    int i;
    ptask_policy = policy;
    /* Use a cast here because ptask_t0 is timespec_custom but clock_gettime expects a struct timespec*.
       (This is safe provided timespec_custom has the same layout as struct timespec.) */
    clock_gettime(CLOCK_MONOTONIC, (struct timespec *)&ptask_t0);
    for (i = 0; i < MAX_TASKS; i++) {
        sem_init(&tp[i].asem, 0, 0);
    }
}

/*
 * Returns the elapsed time (since ptask_init) in the specified unit.
 * Use MICRO for microseconds and NANO for nanoseconds.
 */
long get_systime(int unit)
{
    timespec_custom t;
    long tu, mul, div;
    switch (unit) {
        case MICRO: mul = 1000000; div = 1000; break;
        case NANO:  mul = 1000;    div = 1000000; break;
        default:    mul = 1000;    div = 1000000; break;
    }
    clock_gettime(CLOCK_MONOTONIC, (struct timespec *)&t);
    tu = (t.tv_sec - ptask_t0.tv_sec) * mul;
    tu += (t.tv_nsec - ptask_t0.tv_nsec) / div;
    return tu;
}

/*
 * Creates a periodic task.
 *
 * Parameters:
 *   i      - Task index (0 to MAX_TASKS-1)
 *   task   - Pointer to the task function (which must have signature void *func(void *))
 *   period - Period of the task in milliseconds
 *   drel   - Relative deadline (in milliseconds)
 *   prio   - Scheduling priority for the task
 *   aflag  - If ACT (1), the task is activated immediately; otherwise (DEACT)
 *
 * Returns:
 *   0 on success, nonzero on error.
 */
int task_create(int i,
                void *(*task)(void *),
                int period,
                int drel,
                int prio,
                int aflag)
{
    pthread_attr_t myatt;
    struct sched_param mypar;
    int tret;

    if (i >= MAX_TASKS) {
        printf("MAX_TASKS reached\n");
        return -1;
    }
    tp[i].arg      = i;
    tp[i].period   = period;
    tp[i].deadline = drel;
    tp[i].prio     = prio;
    tp[i].dmiss    = 0;

    pthread_attr_init(&myatt);
    pthread_attr_setinheritsched(&myatt, PTHREAD_EXPLICIT_SCHED);
    pthread_attr_setschedpolicy(&myatt, ptask_policy);

    mypar.sched_priority = tp[i].prio;
    pthread_attr_setschedparam(&myatt, &mypar);

    tret = pthread_create(&tp[i].tid, &myatt, task, (void *)&tp[i]);
    if (tret != 0) {
        fprintf(stderr, "pthread_create error for task %d: %s\n", i, strerror(tret));
        return tret;
    }

    if (aflag == ACT)
        task_activate(i);

    return tret;
}

/* Returns the task index (its “arg” field) given the task’s argument pointer. */
int get_task_index(void *arg)
{
    task_par *tpar = (task_par *)arg;
    return tpar->arg;
}

/*
 * Blocks the calling thread until the task is activated.
 * After activation, it saves the current time into the task’s activation and
 * deadline times and adds the period/deadline offsets.
 */
void wait_for_activation(int i)
{
    timespec_custom t;
    sem_wait(&tp[i].asem);
    clock_gettime(CLOCK_MONOTONIC, (struct timespec *)&t);
    time_copy(&tp[i].at, t);
    time_copy(&tp[i].dl, t);
    time_add_ms(&tp[i].at, tp[i].period);
    time_add_ms(&tp[i].dl, tp[i].deadline);
}

/* Releases (activates) the task i by posting its semaphore. */
void task_activate(int i)
{
    sem_post(&tp[i].asem);
}

/*
 * Checks whether the current time is past the task’s deadline.
 * If so, increments the deadline miss counter.
 * Returns 1 if a deadline miss occurred, 0 otherwise.
 */
int deadline_miss(int i)
{
    timespec_custom now;
    clock_gettime(CLOCK_MONOTONIC, (struct timespec *)&now);
    if (time_cmp(now, tp[i].dl) > 0) {
        tp[i].dmiss++;
        return 1;
    }
    return 0;
}

/*
 * Waits for the next period.
 * Uses absolute time (via clock_nanosleep) to avoid cumulative drift.
 */
void wait_for_period(int i)
{
    clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME,
                    (struct timespec *)&tp[i].at, NULL);
    time_add_ms(&tp[i].at, tp[i].period);
    time_add_ms(&tp[i].dl, tp[i].period);
}

/* Setters and getters for task parameters */
void task_set_period(int i, int per)
{
    tp[i].period = per;
}

void task_set_deadline(int i, int drel)
{
    tp[i].deadline = drel;
}

int task_period(int i)
{
    return tp[i].period;
}

int task_deadline(int i)
{
    return tp[i].deadline;
}

int task_dmiss(int i)
{
    return tp[i].dmiss;
}

void task_atime(int i, timespec_custom *at)
{
    at->tv_sec  = tp[i].at.tv_sec;
    at->tv_nsec = tp[i].at.tv_nsec;
}

void task_adline(int i, timespec_custom *dl)
{
    dl->tv_sec  = tp[i].dl.tv_sec;
    dl->tv_nsec = tp[i].dl.tv_nsec;
}

/*
 * Waits for the given task to complete (join its thread).
 */
void wait_for_task_end(int i)
{
    pthread_join(tp[i].tid, NULL);
}

#endif /* PTASK_IMPLEMENTATION */
#endif /* PTASK_H */
