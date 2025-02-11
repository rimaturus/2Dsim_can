#include <stdio.h>
#include <stdlib.h>

// -------------------------------------
// THREADs
// -------------------------------------
#include <pthread.h>
#include <sched.h>
#include <time.h>

void *task1(void *p);

int main()
{
	pthread_t	tid1, tid2;
	int 		err1, err2;
	int 		param1 = 1, param2 = 2; 

		err1 = pthread_create(&tid1, NULL, task1, (void *)&param1);
		err2 = pthread_create(&tid2, NULL, task1, (void *)&param2);	

		if ((err1 != 0) || (err2 != 0))
			printf("Error creating threads\n");
		else
			printf("Threads created\n");

		pthread_join(tid1, NULL);
		pthread_join(tid2, NULL);


	return 0;
}

void *task1(void *p)
{
int 	*pi;
int 	a;
int 	i;

	pi = (int *)p;
	a = *pi;

	if (a == 1)	
		printf("Task 1\n");
	else
		printf("Task 2\n");
	
	return NULL;
}