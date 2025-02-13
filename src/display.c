/**
 * @file display.c
 * @brief Rendering and display update functions for the 2D simulation using Allegro.
 *
 * This file contains functions responsible for drawing various elements of the simulation,
 * including the car sprite, direction indicator, track, lidar/perception display, trajectory, 
 * and user controls (e.g., steering wheel and pedal gauge). It also handles the final composition
 * of the display buffer and its transfer to the screen.
 */
#include <allegro.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include "globals.h"
#include "perception.h"
#include "trajectory.h"
#include "utilities.h"
#include "control.h"
#include "vehicle.h"
#include "display.h"

// Create buttons
int btn_state_cones = 0;
int btn_state_perception = 1;
int btn_state_map = 1;
int btn_state_traj = 1;
int btn_state_autonomous = 0;

// (They stay in the left half of control panel)
Button buttons[num_buttons] = {
	{20, 20, 150, 30, "Clean track", &btn_state_cones},
	{20, 60, 150, 30, "Lidar", &btn_state_perception},
	{20, 100, 150, 30, "Mapping", &btn_state_map},
	{20, 140, 150, 30, "Trajectory", &btn_state_traj},
	{20, 180, 150, 30, "Full Autonomous", &btn_state_autonomous}
};

/**
 * @brief Draws a directional arrow representing the car's orientation.
 *
 * Computes the arrow's start (at the car's position) and end points based on the car angle,
 * using cosine and sine functions. The arrow shaft and head are drawn as thick green lines,
 * with the head drawn at two angles to form a head shape.
 */
void draw_dir_arrow();

/**
 * @brief Renders the car sprite rotated to its current angle.
 *
 * Calculates the proper position for the car sprite based on its coordinates in the world space
 * (scaled by pixels per meter) and draws it using Allegro's rotate_scaled_sprite function.
 * In DEBUG mode, the function also draws view angle lines to indicate the car's field of view.
 *
 * @param car_x The car's x-coordinate in world units.
 * @param car_y The car's y-coordinate in world units.
 * @param car_angle The car's angle in degrees.
 */
void draw_car(float car_x, float car_y, int car_angle);

/**
 * @brief Draws the track.
 *
 * Checks that the track bitmap has valid dimensions, and if so, draws the track image onto 
 * the display buffer. If the bitmap dimensions are invalid, an error message is shown.
 */
void draw_track();

/**
 * @brief Draws the lidar/perception view.
 *
 * Clears the perception bitmap and centers it on the car's position. For each lidar measurement,
 * computes the global-to-perception window mapping for the detected point and draws a line from the
 * car's center to that point. The color of the line depends on whether a cone was detected or not.
 *
 * @param measures Pointer to an array of lidar measurements.
 */
void draw_lidar(pointcloud_t *measures);

/**
 * @brief Renders the detected cones on the perception bitmap.
 *
 * Iterates through the array of detected cones and draws a filled circle at each cone's location,
 * after mapping the coordinates from the simulation world to the perception window.
 *
 * @param detected_cones Pointer to an array of detected cone structures.
 */
void draw_detected_cones(cone *detected_cones);

/**
 * @brief Draws the cone map overlay.
 *
 * Iterates through the track map data and draws a white filled circle at each cone's position,
 * properly mapping simulation world coordinates to the perception window.
 *
 * @param track_map Pointer to an array of cones representing the track map.
 * @param track_map_idx The number of valid entries in the track map array.
 */
void draw_cone_map(cone *track_map, int track_map_idx);

/**
 * @brief Renders the complete perception layer.
 *
 * Combines the lidar lines, detected cones, and cone map visualizations to create the perception
 * overlay. Afterwards, the perception bitmap is blended onto the main display buffer offset in 
 * relation to the car's position.
 */
void draw_perception();

/**
 * @brief Draws the trajectory of the car.
 *
 * Clears and sets the background for the trajectory bitmap and iterates over each trajectory waypoint,
 * drawing each as a filled circle. In DEBUG mode, the waypoint index is also displayed and an extra line
 * is drawn from the car sprite center.
 *
 * @param trajectory Pointer to an array of waypoint structures representing the car's trajectory.
 */
void draw_trajectory(waypoint *trajectory);

/**
 * @brief Draws user control indicators.
 *
 * Renders the steering wheel sprite with the correct rotation and draws a gauge for pedal control.
 * The gauge's fill color indicates the pedal level (green for positive, red for non-positive).
 */
void draw_controls();

/**
 * @brief Updates the entire display.
 *
 * Locks the drawing mutex and sequentially renders the background, track, car,
 * perception, trajectory, title text, and controls on the display buffer.
 * Once all elements are rendered, the display buffer is blitted to the screen,
 * and the mutex is subsequently unlocked.
 */
void update_display();


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
#ifdef DEBUG
	
	draw_dir_arrow();

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
	if (btn_state_cones)
	{
		clear_bitmap(track);
		clear_to_color(track, pink);

		memcpy(cones, starting_cone_positions, sizeof(cone) * MAX_CONES_MAP);

		for (int i = 0; i < MAX_CONES_MAP; i++)
		{
			if (starting_cone_positions[i].color != -1) // plot only track cones
			{
				circlefill(
					track, 
					(int)(starting_cone_positions[i].x), 
					(int)(starting_cone_positions[i].y), 
					cone_radius * px_per_meter, // radius = 5 cm
					cones[i].color
				);
			}
		}
	}

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

int lidar_angle;

	for (int i = 0; i < sliding_window; i += angle_step)
	{   
		lidar_angle = (start_angle + i) % 360;
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
	if (btn_state_perception)
	{
		draw_lidar(measures);
		draw_detected_cones(detected_cones);
	}
	else {
		clear_bitmap(perception);
		clear_to_color(perception, pink);
	}

	if (btn_state_map) draw_cone_map(track_map, track_map_idx);

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

void display_pedal(BITMAP *bitmap)
{
    int gauge_width = 30;
    int gauge_height = 2 * maxThrottleHeight;
    
    // Position in right section
    int gauge_x = bitmap->w * 5/8;  
    int gauge_y = (bitmap->h - gauge_height) / 2;
    
    int fill_height = (int)(pedal * gauge_height/2);

    // Draw gauge outline
    rect(
        bitmap, 
        gauge_x, 
        gauge_y, 
        gauge_x + gauge_width, 
        gauge_y + gauge_height, 
        makecol(255, 255, 255)
    );

    // Fill gauge based on pedal level
    int middle_y = gauge_y + gauge_height/2;
    if (pedal > 0) {
        rectfill(
            bitmap,
            gauge_x,
            middle_y - fill_height,  // Start from middle, go up
            gauge_x + gauge_width,
            middle_y,                // Stop at middle
            makecol(0, 255, 0)      // Green for positive
        );
    } else {
        rectfill(
            bitmap,
            gauge_x,
            middle_y,               // Start from middle
            gauge_x + gauge_width,
            middle_y + abs(fill_height),  // Go down from middle
            makecol(255, 0, 0)     // Red for negative
        );
    }
}

void draw_button(Button* btn, BITMAP* buffer) {
	rect(buffer, btn->x, btn->y, btn->x + btn->width, btn->y + btn->height, (*(btn->value))? green : red);
	textout_centre_ex(buffer, font, btn->text, 
					 btn->x + btn->width/2, 
					 btn->y + btn->height/2 - 4, 
					 makecol(255, 255, 255), -1);
	
	char value_str[32];
	sprintf(value_str, "%d", *(btn->value));
	textout_ex(buffer, font, value_str, 
			   btn->x + btn->width + 10, 
			   btn->y + btn->height/2 - 4, 
			   makecol(255, 255, 255), -1);
}

void draw_controls()
{
    clear_bitmap(control_panel);
    clear_to_color(control_panel, pink);

    // Draw buttons in left section
    for (int i = 0; i < num_buttons; i++) {
        draw_button(&buttons[i], control_panel);
    }

    display_pedal(control_panel);

    // Position steering wheel in right section
    rotate_scaled_sprite(
        control_panel, 
        steering_wheel, 
        control_panel->w * 3/4, // Right section of the right section of the control panel
        control_panel->h/2 - steering_wheel->h/2,      // Vertical center
        ftofix(angle_rotation_sprite(((int)(steering/deg2rad) + 90) % 360)),
        ftofix(1)
    );

    // Draw the control panel at the bottom of the screen
    masked_blit(
        control_panel, display_buffer, 
        0, 0, 
        X_MAX - control_panel->w, Y_MAX - control_panel->h, 
        control_panel->w, control_panel->h
    );    
}

void update_display()
{
	check_collisions();

	pthread_mutex_lock(&draw_mutex);
		clear_to_color(display_buffer, pink);
    
		// Draw layers in order
		draw_sprite(display_buffer, background, 0, 0);
		draw_track();
		draw_car(car_x, car_y, car_angle);

		draw_perception();

		if (btn_state_traj) draw_trajectory(trajectory);
		else {
			clear_bitmap(trajectory_bmp);
			clear_to_color(trajectory_bmp, pink);
		}

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
