#ifndef CONFIGSTRUCT_H
#define CONFIGSTRUCT_H

typedef struct {
	float	pixel_per_meter;
} Visualization;

typedef struct {
	float	wheelbase;
	float	weight;
} Car;

typedef struct {
	float	max_speed;
	float 	max_throttle;
	float	max_brake;
	float	max_torque;
	float	max_steering;
	float	min_steering;
	float 	steer_step;
	float 	pps_step;
} Actuation;

typedef struct {
	float	range_std_dev;
	float	bearing_std_dev;
	float 	detection_range;
} Perception;

typedef struct {
	float	kp;
	float	ki;
	float 	kd;
	float	setpoint;
} PID;

typedef struct {
	int	steer_id;
	int	throttle_id;
	int	carX_id;
	int	carY_id;
	int	carAngle_id;
	int	cone_start_id;
} CAN_ids;

typedef struct {
    Visualization	visualization_params;
    Car				car_params;
    Actuation		actuation_params;
    Perception		perception_params;
    PID				pid_params;
    CAN_ids			ids;
} Config;

void	loadConfig(const char *config_file, Config *config);
#endif