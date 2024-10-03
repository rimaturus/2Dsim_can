#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <pthread.h>

#include "src/visualization/visualization.h"
#include "src/control/keyboard_control.h"
#include "include/configStruct.h"


void	*visualizationThread(void *arg);
void	*controlThread(void *arg);


typedef struct {
	Config config;
	char	track_file[256];
} SimParams;


volatile int	finish = 0;
pthread_mutex_t	simMutex = PTHREAD_MUTEX_INITIALIZER;


int	main(int argc, char *argv[])
{
	SimParams	simParams;
	int		i;	// used in "for" loop

	char config_file[256];

	// Initialize default simulation parameters
	strcpy(config_file, "config/config.yaml");
	strcpy(simParams.track_file, "config/cones.yaml");

	// Take the input arguments 
	for (i = 1; i < argc; i++)
	{
		if (strcmp(argv[i], "--config") == 0 && i + 1 < argc)
		{
			strncpy(config_file, argv[i + 1], sizeof(config_file) - 1);
			config_file[sizeof(config_file) - 1] = '\0';
			i++;
		}
		else if (strcmp(argv[i], "--track") == 0 && i + 1 < argc)
		{
			strncpy(simParams.track_file, argv[i + 1], sizeof(simParams.track_file) - 1);
			simParams.track_file[sizeof(simParams.track_file) - 1] = '\0';
			i++;
		}
		else
		{
			printf("Usage: %s [--config <config_file>] [--track <track_file>]\n", argv[0]);
			return -1;
		}
	}

	loadConfig(config_file, &simParams.config);

	// Thread section 
	pthread_t	visThreadId;
	pthread_t	controlThreadId;

	if (pthread_create(&visThreadId, NULL, visualizationThread, &simParams) != 0)
	{
		perror("Failed to create visualization thread");
		return -1;
	}

	if (pthread_create(&controlThreadId, NULL, controlThread, &simParams.config) != 0)
    {
        perror("Failed to create control thread");
        return -1;
    }

	// Wait for thread to finish
	pthread_join(visThreadId, NULL);
	pthread_join(controlThreadId, NULL);

	return 0;
}

void	*visualizationThread(void *arg)
{
	SimParams	*simParams = (SimParams *)arg;
	int		result;

	result = visualization_main(&simParams->config, simParams->track_file);

	if (result != 0)
	{
		fprintf(stderr, "Visualization exited with error code %d\n", result);
	}

	finish = 1;  // Signal that the simulation is end
	return NULL;
}

void	*controlThread(void *arg)
{
	Config	*config = (Config *)arg;
	int		result;

	result = keyboard_control_main(config);

	if (result != 0)
	{
		fprintf(stderr, "Control exited with error code %d\n", result);
	}

	finish = 1;  // Signal that the control is end
	return NULL;
}
