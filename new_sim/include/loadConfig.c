#include "configStruct.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <yaml.h>

// ---------------------------------------------------------------------
//					LOAD CONFIG and TRACK FILES (libyaml)
// 	Take in input the path of the .yaml file, check if everything is correct
// 	and assign the read values to specific variables or arrays
//	
//	Tutorial followed to parse the yaml file: https://www.wpsoftware.net/andrew/pages/libyaml.html
//	PARSE to ASSIGNATION: 
//		STREAM_START_TOKEN -> KEY_TOKEN -> VALUE_TOKEN -> SCALAR_TOKEN -> KEY... -> STREAM_END_TOKEN
//	e.g.		SOF			 "wheelbase"       ":"            "2.5"     "STEER..."		 EOF
// ---------------------------------------------------------------------

void	loadConfig(const char *config_file, Config *config){
	FILE *file;
    yaml_parser_t parser;
    yaml_token_t token;
    char key[256];
    char value[256];
    int state = 0;  // 0: Expect key, 1: Expect value

    file = fopen(config_file, "r");
    if (!file) {
        fprintf(stderr, "Failed to open config file: %s\n", config_file);
        return;
    }

    if (!yaml_parser_initialize(&parser)) {
        fprintf(stderr, "Failed to initialize YAML parser\n");
        fclose(file);
        return;
    }

    yaml_parser_set_input_file(&parser, file);

    while (1) {
        yaml_parser_scan(&parser, &token);
        if (token.type == YAML_STREAM_END_TOKEN) {
            yaml_token_delete(&token);
            break;
        }

        if (token.type == YAML_KEY_TOKEN) {
            state = 0;  // Expecting a key
        } else if (token.type == YAML_VALUE_TOKEN) {
            state = 1;  // Expecting a value
        } else if (token.type == YAML_SCALAR_TOKEN) {
            if (state == 0) {
                strncpy(key, (char *)token.data.scalar.value, sizeof(key) - 1);
                key[sizeof(key) - 1] = '\0';
            } else if (state == 1) {
                strncpy(value, (char *)token.data.scalar.value, sizeof(value) - 1);
                value[sizeof(value) - 1] = '\0';

                // visualization
                if (strcmp(key, "PIXELS_PER_METER") == 0) 
				{
                    config->visualization_params.pixel_per_meter = atof(value);
                }
				// car
				else if (strcmp(key, "wheelbase") == 0) 
				{
                    config->car_params.wheelbase = atof(value);
                } 
				else if (strcmp(key, "weight") == 0) 
				{
                    config->car_params.weight = atof(value);
                } 
				// actuation
				else if (strcmp(key, "max_speed") == 0) 
				{
                    config->actuation_params.max_speed = atof(value);
                } 
				else if (strcmp(key, "max_throttle") == 0) 
				{
                    config->actuation_params.max_throttle = atof(value);
                } 
				else if (strcmp(key, "max_brake") == 0) 
				{
                    config->actuation_params.max_brake = atof(value);
                } 
				else if (strcmp(key, "max_torque") == 0) 
				{
                    config->actuation_params.max_torque = atof(value);
                } 
				else if (strcmp(key, "max_steering") == 0) 
				{
                    config->actuation_params.max_steering = atof(value);
                } 
				else if (strcmp(key, "min_steering") == 0) 
				{
                    config->actuation_params.min_steering = atof(value);
                } 
				else if (strcmp(key, "steer_step") == 0) 
				{
                    config->actuation_params.steer_step = atof(value);
                } 
				else if (strcmp(key, "pps_step") == 0) 
				{
                    config->actuation_params.pps_step = atof(value);
                } 
				// perception
				else if (strcmp(key, "range_noise_std_dev") == 0) 
				{
                    config->perception_params.range_std_dev = atof(value);
                } 
				else if (strcmp(key, "bearing_noise_std_dev") == 0) 
				{
                    config->perception_params.bearing_std_dev = atof(value);
                } 
				else if (strcmp(key, "detection_range") == 0) 
				{
                    config->perception_params.detection_range = atof(value);
                } 
				// PID
				else if (strcmp(key, "Kp") == 0) 
				{
                    config->pid_params.kp = atof(value);
                } 
				else if (strcmp(key, "Ki") == 0) 
				{
                    config->pid_params.ki = atof(value);
                } 
				else if (strcmp(key, "Kd") == 0) 
				{
                    config->pid_params.kd = atof(value);
                } 
				else if (strcmp(key, "setpoint") == 0) 
				{
                    config->pid_params.setpoint = atof(value);
                } 
				// CAN ids
				else if (strcmp(key, "STEERING_CAN_ID") == 0) 
				{
                    config->ids.steer_id = (int)strtol(value, NULL, 0);
                } 
				else if (strcmp(key, "THROTTLE_CAN_ID") == 0) 
				{
                    config->ids.throttle_id = (int)strtol(value, NULL, 0);
                } 
				else if (strcmp(key, "CAR_X_CAN_ID") == 0) 
				{
                    config->ids.carX_id = (int)strtol(value, NULL, 0);
                } 
				else if (strcmp(key, "CAR_Y_CAN_ID") == 0) 
				{
                    config->ids.carY_id = (int)strtol(value, NULL, 0);
                } 
				else if (strcmp(key, "CAR_ANGLE_CAN_ID") == 0) 
				{
                    config->ids.carAngle_id = (int)strtol(value, NULL, 0);
                } 
				else if (strcmp(key, "CONE_CAN_ID_START") == 0) 
				{
                    config->ids.cone_start_id = (int)strtol(value, NULL, 0);
                }
            }
        }
        yaml_token_delete(&token);
    }

    yaml_parser_delete(&parser);
    fclose(file);
}