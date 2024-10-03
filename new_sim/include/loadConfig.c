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

void loadConfig(const char *config_file, Config *config) {
    FILE *file;
    yaml_parser_t parser;
    yaml_document_t document;
    yaml_node_t *root_node;

    // Initialize default values (optional but recommended)
    memset(config, 0, sizeof(Config));

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

    if (!yaml_parser_load(&parser, &document)) {
        fprintf(stderr, "Failed to parse YAML file\n");
        yaml_parser_delete(&parser);
        fclose(file);
        return;
    }

    root_node = yaml_document_get_root_node(&document);
    if (!root_node || root_node->type != YAML_MAPPING_NODE) {
        fprintf(stderr, "Invalid YAML format in config file\n");
        yaml_document_delete(&document);
        yaml_parser_delete(&parser);
        fclose(file);
        return;
    }

    // Parse the root mapping
    for (yaml_node_pair_t *root_pair = root_node->data.mapping.pairs.start;
         root_pair < root_node->data.mapping.pairs.top; root_pair++) {
        yaml_node_t *key_node = yaml_document_get_node(&document, root_pair->key);
        yaml_node_t *value_node = yaml_document_get_node(&document, root_pair->value);

        const char *section = (const char *)key_node->data.scalar.value;

        if (strcmp(section, "visualization") == 0) {
            // Parse visualization section
            for (yaml_node_pair_t *pair = value_node->data.mapping.pairs.start;
                 pair < value_node->data.mapping.pairs.top; pair++) {
                yaml_node_t *k = yaml_document_get_node(&document, pair->key);
                yaml_node_t *v = yaml_document_get_node(&document, pair->value);

                if (strcmp((char *)k->data.scalar.value, "PIXELS_PER_METER") == 0) {
                    config->visualization_params.pixel_per_meter = atof((char *)v->data.scalar.value);
                }
            }
        } else if (strcmp(section, "car") == 0) {
            // Parse car section
            for (yaml_node_pair_t *pair = value_node->data.mapping.pairs.start;
                 pair < value_node->data.mapping.pairs.top; pair++) {
                yaml_node_t *k = yaml_document_get_node(&document, pair->key);
                yaml_node_t *v = yaml_document_get_node(&document, pair->value);

                if (strcmp((char *)k->data.scalar.value, "wheelbase") == 0) {
                    config->car_params.wheelbase = atof((char *)v->data.scalar.value);
                } else if (strcmp((char *)k->data.scalar.value, "weight") == 0) {
                    config->car_params.weight = atof((char *)v->data.scalar.value);
                }
            }
        } else if (strcmp(section, "actuation") == 0) {
            // Parse actuation section
            for (yaml_node_pair_t *pair = value_node->data.mapping.pairs.start;
                 pair < value_node->data.mapping.pairs.top; pair++) {
                yaml_node_t *k = yaml_document_get_node(&document, pair->key);
                yaml_node_t *v = yaml_document_get_node(&document, pair->value);

                const char *key = (char *)k->data.scalar.value;
                const char *val = (char *)v->data.scalar.value;

                if (strcmp(key, "max_speed") == 0) {
                    config->actuation_params.max_speed = atof(val);
                } else if (strcmp(key, "max_throttle") == 0) {
                    config->actuation_params.max_throttle = atof(val);
                } else if (strcmp(key, "max_brake") == 0) {
                    config->actuation_params.max_brake = atof(val);
                } else if (strcmp(key, "max_torque") == 0) {
                    config->actuation_params.max_torque = atof(val);
                } else if (strcmp(key, "max_steering") == 0) {
                    config->actuation_params.max_steering = atof(val);
                } else if (strcmp(key, "min_steering") == 0) {
                    config->actuation_params.min_steering = atof(val);
                } else if (strcmp(key, "steer_step") == 0) {
                    config->actuation_params.steer_step = atof(val);
                } else if (strcmp(key, "pps_step") == 0) {
                    config->actuation_params.pps_step = atof(val);
                }
            }
        } else if (strcmp(section, "perception") == 0) {
            // Parse perception section
            for (yaml_node_pair_t *pair = value_node->data.mapping.pairs.start;
                 pair < value_node->data.mapping.pairs.top; pair++) {
                yaml_node_t *k = yaml_document_get_node(&document, pair->key);
                yaml_node_t *v = yaml_document_get_node(&document, pair->value);

                if (strcmp((char *)k->data.scalar.value, "range_noise_std_dev") == 0) {
                    config->perception_params.range_std_dev = atof((char *)v->data.scalar.value);
                } else if (strcmp((char *)k->data.scalar.value, "bearing_noise_std_dev") == 0) {
                    config->perception_params.bearing_std_dev = atof((char *)v->data.scalar.value);
                } else if (strcmp((char *)k->data.scalar.value, "detection_range") == 0) {
                    config->perception_params.detection_range = atof((char *)v->data.scalar.value);
                }
            }
        } else if (strcmp(section, "pid_controller") == 0) {
            // Parse PID controller section
            for (yaml_node_pair_t *pair = value_node->data.mapping.pairs.start;
                 pair < value_node->data.mapping.pairs.top; pair++) {
                yaml_node_t *k = yaml_document_get_node(&document, pair->key);
                yaml_node_t *v = yaml_document_get_node(&document, pair->value);

                if (strcmp((char *)k->data.scalar.value, "Kp") == 0) {
                    config->pid_params.kp = atof((char *)v->data.scalar.value);
                } else if (strcmp((char *)k->data.scalar.value, "Ki") == 0) {
                    config->pid_params.ki = atof((char *)v->data.scalar.value);
                } else if (strcmp((char *)k->data.scalar.value, "Kd") == 0) {
                    config->pid_params.kd = atof((char *)v->data.scalar.value);
                } else if (strcmp((char *)k->data.scalar.value, "setpoint") == 0) {
                    config->pid_params.setpoint = atof((char *)v->data.scalar.value);
                }
            }
        } else if (strcmp(section, "CAN_IDS") == 0) {
            // Parse CAN IDs section
            for (yaml_node_pair_t *pair = value_node->data.mapping.pairs.start;
                 pair < value_node->data.mapping.pairs.top; pair++) {
                yaml_node_t *k = yaml_document_get_node(&document, pair->key);
                yaml_node_t *v = yaml_document_get_node(&document, pair->value);

                const char *key = (char *)k->data.scalar.value;
                const char *val = (char *)v->data.scalar.value;

                if (strcmp(key, "STEERING_CAN_ID") == 0) {
                    config->ids.steer_id = (int)strtol(val, NULL, 0);
                } else if (strcmp(key, "THROTTLE_CAN_ID") == 0) {
                    config->ids.throttle_id = (int)strtol(val, NULL, 0);
                } else if (strcmp(key, "CAR_X_CAN_ID") == 0) {
                    config->ids.carX_id = (int)strtol(val, NULL, 0);
                } else if (strcmp(key, "CAR_Y_CAN_ID") == 0) {
                    config->ids.carY_id = (int)strtol(val, NULL, 0);
                } else if (strcmp(key, "CAR_ANGLE_CAN_ID") == 0) {
                    config->ids.carAngle_id = (int)strtol(val, NULL, 0);
                } else if (strcmp(key, "CONE_CAN_ID_START") == 0) {
                    config->ids.cone_start_id = (int)strtol(val, NULL, 0);
                }
            }
        }
    }

    // Cleanup
    yaml_document_delete(&document);
    yaml_parser_delete(&parser);
    fclose(file);

    // Add print statements to display loaded values
    printf("Configuration Loaded:\n");
    printf("Visualization:\n");
    printf("  PIXELS_PER_METER: %f\n", config->visualization_params.pixel_per_meter);

    printf("Car Parameters:\n");
    printf("  Wheelbase: %f\n", config->car_params.wheelbase);
    printf("  Weight: %f\n", config->car_params.weight);

    printf("Actuation Parameters:\n");
    printf("  Max Speed: %f\n", config->actuation_params.max_speed);
    printf("  Max Throttle: %f\n", config->actuation_params.max_throttle);
    printf("  Max Brake: %f\n", config->actuation_params.max_brake);
    printf("  Max Torque: %f\n", config->actuation_params.max_torque);
    printf("  Max Steering: %f\n", config->actuation_params.max_steering);
    printf("  Min Steering: %f\n", config->actuation_params.min_steering);
    printf("  Steer Step: %f\n", config->actuation_params.steer_step);
    printf("  PPS Step: %f\n", config->actuation_params.pps_step);

    printf("Perception Parameters:\n");
    printf("  Range Noise Std Dev: %f\n", config->perception_params.range_std_dev);
    printf("  Bearing Noise Std Dev: %f\n", config->perception_params.bearing_std_dev);
    printf("  Detection Range: %f\n", config->perception_params.detection_range);

    printf("PID Controller Parameters:\n");
    printf("  Kp: %f\n", config->pid_params.kp);
    printf("  Ki: %f\n", config->pid_params.ki);
    printf("  Kd: %f\n", config->pid_params.kd);
    printf("  Setpoint: %f\n", config->pid_params.setpoint);

    printf("CAN IDs:\n");
    printf("  Steering CAN ID: 0x%X\n", config->ids.steer_id);
    printf("  Throttle CAN ID: 0x%X\n", config->ids.throttle_id);
    printf("  Car X CAN ID: 0x%X\n", config->ids.carX_id);
    printf("  Car Y CAN ID: 0x%X\n", config->ids.carY_id);
    printf("  Car Angle CAN ID: 0x%X\n", config->ids.carAngle_id);
    printf("  Cone CAN ID Start: 0x%X\n", config->ids.cone_start_id);
}
