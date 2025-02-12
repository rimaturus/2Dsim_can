#include <stdio.h>
#include <math.h>
#include <string.h>
#include <time.h>
#include <yaml.h>

#include "globals.h"
#include "perception.h"

void init_cones(cone *cones)
{
	for (int i = 0; i < MAX_CONES_MAP; i++)
	{
		cones[i].x = 0.0f;
		cones[i].y = 0.0f;
		cones[i].color = -1;
	}
}

float tmp_scale = 1.5 / 100;

void load_cones_positions(const char *filename, cone *cones, int max_cones)
{
	printf("Loading cones from file: %s\n", filename);
	FILE *file = fopen(filename, "r");
	if (!file) {
		fprintf(stderr, "Error: Unable to open file %s\n", filename);
		return;
	}

	yaml_parser_t parser;
	yaml_event_t event;

	// Initialize YAML parser
	if (!yaml_parser_initialize(&parser)) {
		fprintf(stderr, "Error: Failed to initialize YAML parser.\n");
		fclose(file);
		return;
	}
	yaml_parser_set_input_file(&parser, file);

	int i = 0; // Current cone index
	int state = 0; // Parser state
	char current_key[256] = {0}; // Buffer to store the current key

	/*
		Parser States:
		0 - Start (expecting 'cones' key)
		1 - Found 'cones' key
		2 - Inside 'cones' sequence
		3 - Inside a cone mapping
	*/

	while (1) {
		if (!yaml_parser_parse(&parser, &event)) {
			fprintf(stderr, "Error: Failed to parse YAML file.\n");
			break;
		}

		if (event.type == YAML_STREAM_END_EVENT) {
			yaml_event_delete(&event);
			break;
		}

		switch (event.type) {
			case YAML_SCALAR_EVENT:
				if (state == 0) {
					// Expecting the 'cones' key
					if (strcmp((char *)event.data.scalar.value, "cones") == 0) {
						state = 1;
					}
				}
				else if (state == 3) {
					// Inside a cone mapping, expecting key or value
					if (strlen(current_key) == 0) {
						// Expecting a key
						strncpy(current_key, (char *)event.data.scalar.value, sizeof(current_key) - 1);
						current_key[sizeof(current_key) - 1] = '\0'; // Ensure null-termination
					}
					else {
						// Expecting a value corresponding to the current key
						if (strcmp(current_key, "x") == 0) {
							cones[i].x = (float)( atof((char *)event.data.scalar.value) ) * tmp_scale * px_per_meter;
						} 
						else if (strcmp(current_key, "y") == 0) { 
							cones[i].y = (float)( atof((char *)event.data.scalar.value) ) * tmp_scale * px_per_meter;
						} 
						else if (strcmp(current_key, "color") == 0) {
							if (strcmp((char *)event.data.scalar.value, "yellow") == 0) {
								cones[i].color = yellow; //makecol(254, 221, 0); // Yellow color
							} 
							else if (strcmp((char *)event.data.scalar.value, "blue") == 0) {
								cones[i].color = blue; //makecol(46, 103, 248); // Blue color
							}
							// Add more colors as needed
						}
						// Reset current_key for the next key-value pair
						current_key[0] = '\0';
					}
				}
				break;

			case YAML_SEQUENCE_START_EVENT:
				if (state == 1) {
					// Found the start of the 'cones' sequence
					state = 2;
				}
				break;

			case YAML_MAPPING_START_EVENT:
				if (state == 2) {
					// Starting a new cone mapping
					if (i >= max_cones) {
						fprintf(stderr, "Warning: Maximum number of cones (%d) reached. Additional cones will be ignored.\n", max_cones);
						yaml_event_delete(&event);
						goto cleanup;
					}
					// Initialize the current cone
					cones[i].x = 0.0f;
					cones[i].y = 0.0f;
					cones[i].color = 0; // Reset color to indicate it's being set
					state = 3;
				}
				break;

			case YAML_MAPPING_END_EVENT:
				if (state == 3) {
					// Finished reading a cone's data
					state = 2; // Return to 'cones' sequence
					i++; // Move to the next cone
				}
				break;

			default:
				break;
		}

		yaml_event_delete(&event);
	}

cleanup:
	yaml_parser_delete(&parser);
	fclose(file);
	
	printf("Total cones loaded: %d\n", i);
}

float angle_rotation_sprite(float angle)
{
    return 64.0f - 128.0f * angle / 180.0f;
}


// Helper functions to measure code performance
void runtime (int stop_signal, char* task_name)
{
#ifdef PROFILING

	static struct timespec iter_start, iter_end;
	if (stop_signal == 0){
		clock_gettime(CLOCK_MONOTONIC, &iter_start);
		unsigned long iter_start_time_us = (iter_start.tv_sec) * 1000000UL +
										(iter_start.tv_nsec) / 1000;
		printf("[%s],START,%lu\n", task_name, iter_start_time_us);
	}
	else{
		clock_gettime(CLOCK_MONOTONIC, &iter_end);
		unsigned long iter_runtime_us = (iter_end.tv_sec - iter_start.tv_sec) * 1000000UL +
										(iter_end.tv_nsec - iter_start.tv_nsec) / 1000;

		unsigned long iter_end_time_us = (iter_end.tv_sec) * 1000000UL +
										(iter_end.tv_nsec) / 1000;
		printf("[%s],END,%lu\n", task_name, iter_end_time_us);
		// printf("[%s]\t Runtime: %lu us\n", task_name, iter_runtime_us);
	}

#endif /* PROFILING */
}