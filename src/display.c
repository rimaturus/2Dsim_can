#include <allegro.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include "globals.h"
#include "perception.h"
#include "trajectory.h"
#include "utilities.h"
#include "control.h"


void draw_dir_arrow()
{
	// Draw direction arrow
	int arrow_length = 50;  // Length in pixels
	float arrow_x = car_x * px_per_meter;
	float arrow_y = car_y * px_per_meter;
	float rad_angle = -car_angle * deg2rad;  // Convert to radians and adjust direction

	// Calculate arrow endpoint
	float end_x = arrow_x + arrow_length * cos(rad_angle);
	float end_y = arrow_y + arrow_length * sin(rad_angle);

	// Draw thick green arrow shaft
	for(int i=-2; i<=2; i++) {
		for(int j=-2; j<=2; j++) {
			line(display_buffer, 
				arrow_x + i,
				arrow_y + j, 
				end_x + i,
				end_y + j,
				makecol(0, 255, 0));
		}
	}

	// Draw thick arrow head
	float head_size = 15.0;
	float head_angle = 0.5;  // ~30 degrees in radians
	for(int i=-2; i<=2; i++) {
		for(int j=-2; j<=2; j++) {
			line(display_buffer, 
				end_x + i,
				end_y + j,
				end_x - head_size * cos(rad_angle - head_angle) + i,
				end_y - head_size * sin(rad_angle - head_angle) + j,
				makecol(0, 255, 0));
			line(display_buffer,
				end_x + i,
				end_y + j,
				end_x - head_size * cos(rad_angle + head_angle) + i,
				end_y - head_size * sin(rad_angle + head_angle) + j,
				makecol(0, 255, 0));
		}
	}
}

void draw_car(float car_x, float car_y, int car_angle)
{
	rotate_scaled_sprite(
				display_buffer, car, 
				(int)(car_x * px_per_meter) - (int)(car->w / 2), 
				(int)(car_y * px_per_meter) - (int)(car->h / 2), 
				ftofix(angle_rotation_sprite(car_angle)),  // deg to fixed point
				ftofix(1)
			);
	// draw_dir_arrow();

#ifdef DEBUG
	// Draw view angle lines
	float view_length = 500.0;
	float rad_angle_left = (-car_angle - 90.0) * deg2rad;
	float rad_angle_right = (-car_angle + 90.0) * deg2rad;
	line(display_buffer,
		car_x * px_per_meter,
		car_y * px_per_meter,
		car_x * px_per_meter + view_length * cos(rad_angle_left),
		car_y * px_per_meter + view_length * sin(rad_angle_left),
		makecol(0, 255, 0));
	line(display_buffer,
		car_x * px_per_meter,
		car_y * px_per_meter,
		car_x * px_per_meter + view_length * cos(rad_angle_right),
		car_y * px_per_meter + view_length * sin(rad_angle_right),
		makecol(0, 255, 0));
#endif /* DEBUG */
}

void draw_track()
{
	if (track != NULL) 
	{
		if (track->w == 0 || track->h == 0) {
			allegro_message("Track bitmap has invalid dimensions");
			return;
		}
		draw_sprite(display_buffer, track, 0, 0);
	}
}

void draw_lidar(pointcloud_t *measures)
{

	clear_to_color(perception, pink); // pink color to make it transparent (True color notation)

	circlefill( // draw the car center point
		perception, 
		perception->w / 2, 
		perception->h / 2, 
		3, 
		makecol(0,255,0)
	); 

int lidar_angle = (start_angle % 360);

	for (lidar_angle; lidar_angle < sliding_window; lidar_angle += angle_step)
	{   
		lidar_angle = (lidar_angle) % 360;
		// printf("Lidar angle: %d \t %f\n", lidar_angle, measures[lidar_angle].distance);

		// plot lines from car to detected point
		float cos_angle = cos(lidar_angle * deg2rad);
		float sin_angle = sin(lidar_angle * deg2rad);

		// There starts the graphical part:
		// since the perception window is centered on the car, we need to calculate the
		// coordinates of the detected point in the perception window
		// so the (car_x, car_y) coordinates in global frame 
		// are mapped to the center of the perception window (perception_cx, perception_cy)
		float perception_cx = (float)(perception->w/2)/px_per_meter;
		float perception_cy = (float)(perception->h/2)/px_per_meter;

		float x0 = perception_cx + cos_angle * ignore_distance;
		float y0 = perception_cy + sin_angle * ignore_distance;

		float x_detection = perception_cx + (measures[lidar_angle].distance * cos_angle);
		float y_detection = perception_cy + (measures[lidar_angle].distance * sin_angle);
		
		if (measures[lidar_angle].color == -1) // no cone detected
		{
			line(
				perception, 
				(int)(x0 * px_per_meter), 
				(int)(y0 * px_per_meter), 
				(int)(x_detection * px_per_meter), 
				(int)(y_detection * px_per_meter), 
				makecol(255, 0, 0)
			);
		}
		else
		{
			line(
				perception, 
				(int)(x0 * px_per_meter), 
				(int)(y0 * px_per_meter), 
				(int)(x_detection * px_per_meter), 
				(int)(y_detection * px_per_meter), 
				measures[lidar_angle].color);
		}
	}
}

void draw_detected_cones(cone *detected_cones){
	int detected_cone_idx = 0;
	
	while (detected_cone_idx < MAX_DETECTED_CONES-1){
		if (detected_cones[detected_cone_idx].color == -1){
			break;
		}
		else {
			circlefill(
				perception, 
				(int)(detected_cones[detected_cone_idx].x * px_per_meter) - (int)(car_x * px_per_meter - maxRange*px_per_meter), 
				(int)(detected_cones[detected_cone_idx].y * px_per_meter) - (int)(car_y * px_per_meter - maxRange*px_per_meter), 
				3, 
				makecol(255, 0, 0) //detected_cones[detected_cone_idx].color
			);

			detected_cone_idx++;	
		}
	}
}

void draw_cone_map(cone *track_map, int track_map_idx)
{
int map_idx = 0;
	while (map_idx < track_map_idx)
	{
		circlefill(
			perception, 
			(int)(track_map[map_idx].x * px_per_meter) - (int)(car_x * px_per_meter - maxRange*px_per_meter), 
			(int)(track_map[map_idx].y * px_per_meter) - (int)(car_y * px_per_meter - maxRange*px_per_meter), 
			3, 
			makecol(255, 255, 255) //detected_cones[detected_cone_idx].color
		);

		map_idx++;	
	}
}

void draw_perception()
{
	draw_lidar(measures);

	draw_detected_cones(detected_cones);

	draw_cone_map(track_map, track_map_idx);

	draw_sprite(
			display_buffer, 
			perception, 
			(int)(car_x * px_per_meter) - (int)(maxRange)*px_per_meter,
			(int)(car_y * px_per_meter) - (int)(maxRange)*px_per_meter
		);
}

void draw_trajectory(waypoint *trajectory)
{
	clear_bitmap(trajectory_bmp);
	clear_to_color(trajectory_bmp, pink);
	for (int traj_point_idx = 0; traj_point_idx < trajectory_idx; traj_point_idx++)
	{
		// printf("Trajectory point %d: (%f, %f)\n", traj_point_idx, trajectory[traj_point_idx].x, trajectory[traj_point_idx].y);
		circlefill(
			trajectory_bmp,
			(int)(trajectory[traj_point_idx].x * px_per_meter),// - (int)(car_x * px_per_meter - maxRange*px_per_meter),
			(int)(trajectory[traj_point_idx].y * px_per_meter),// - (int)(car_y * px_per_meter - maxRange*px_per_meter),
			3,
			makecol(0, 255, 0)
		);

#ifdef DEBUG
		char* text = (char*)malloc(10);  // Allocate space for the string
		snprintf(text, 10, "%d", traj_point_idx);  // Convert int to string

		textout_ex(
			trajectory_bmp, 
			font, 
			text, 
			(int)(trajectory[traj_point_idx].x * px_per_meter), // - (int)(car_x * px_per_meter - maxRange*px_per_meter), 
			(int)(trajectory[traj_point_idx].y * px_per_meter), // - (int)(car_y * px_per_meter - maxRange*px_per_meter), 
			makecol(255, 0, 0), 
			makecol(255, 255, 255)
		);

		free(text);  // Free the memory

		line(
			car, 
			(int)(car->w/2),
			(int)(car->h/2),
			(int)(car->w/2),
			(int)(car->h/2) - 1000,
			makecol(0, 255, 0)
		);
#endif /* DEBUG */

	}
	draw_sprite(display_buffer, trajectory_bmp, 0, 0);
}

void draw_controls(){
	rotate_scaled_sprite(
		display_buffer, 
		steering_wheel, 
		100, 
		100, 
		ftofix(angle_rotation_sprite(( ( (int)(steering/deg2rad) + 90 ) %360 ))) ,  // deg to fixed point
		ftofix(1)
	);


	int gauge_x = 50;
	int gauge_y = 200;
	int gauge_width = 30;
	int gauge_height = 100;
	int fill_height = (int)(pedal * gauge_height);

	// Draw gauge outline
	rect(
		display_buffer, 
		gauge_x, 
		gauge_y, 
		gauge_x + gauge_width, 
		gauge_y + 2 * gauge_height, 
		makecol(255, 255, 255)
	);

	// Fill gauge based on pedal level
	if (pedal > 0.0)
	{
		rectfill(
			display_buffer, 
			gauge_x, 
			gauge_y + gauge_height - fill_height,
			gauge_x + gauge_width, 
			gauge_y + gauge_height,
			makecol(0, 255, 0)
		);
	}
	else
	{
		rectfill(
			display_buffer, 
			gauge_x, 
			gauge_y + gauge_height - fill_height,
			gauge_x + gauge_width, 
			gauge_y + gauge_height,
			makecol(255, 0, 0)
		);
	}
}

void update_display()
{
	pthread_mutex_lock(&draw_mutex);
		clear_to_color(display_buffer, pink);
    
		// Draw layers in order
		draw_sprite(display_buffer, background, 0, 0);
		draw_track();
		draw_car(car_x, car_y, car_angle);

		draw_perception();
		draw_trajectory(trajectory);

		int text_width = text_length(font, title);
		textout_ex(
			display_buffer, 
			font, 
			title, 
			(display_buffer->w - text_width) / 2, // centered x position 
			0, // y position at top
			makecol(255,0,0), // red text
			makecol(255,255,255) // white background
		);

		draw_controls();

		// Draw final buffer to screen
		blit(display_buffer, screen, 0, 0, 0, 0, X_MAX, Y_MAX);

	pthread_mutex_unlock(&draw_mutex);
}
