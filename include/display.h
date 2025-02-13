/**
 * @file display.h
 * @brief Header file declaring display rendering functions.
 *
 * This file contains declarations for functions that handle the drawing of various
 * display elements including car representation, lidar measurements, track, perception,
 * trajectory, and more. These functions provide an interface for visualizing simulation
 * data.
 *
 * @note This header relies on types and globals defined in "globals.h".
 */
#ifndef DISPLAY_H
#define DISPLAY_H

typedef struct {
	int x;
	int y;
	int width;
	int height;
	const char* text;
	int* value;
} Button;

// Create buttons
#define num_buttons 5

extern int btn_state_cones;
extern int btn_state_perception;
extern int btn_state_map;
extern int btn_state_traj;
extern int btn_state_autonomous;


/**
 * @brief Draws a directional arrow.
 *
 * This function is used to render an arrow representing direction on the display.
 * The arrow can be used as an indicator for direction in the simulation.
 */
void draw_dir_arrow();

/**
 * @brief Draws a car shape on the display.
 *
 * @param car_x      X-coordinate of the car's position.
 * @param car_y      Y-coordinate of the car's position.
 * @param car_angle  Orientation angle of the car (typically in degrees).
 *
 * Renders a graphical representation of the car using its position and orientation.
 */
void draw_car(float car_x, float car_y, int car_angle);

/**
 * @brief Draws the track.
 *
 * This function draws the track layout on the display, which might include boundaries,
 * lanes, or other information necessary for simulation or visualization.
 */
void draw_track();

/**
 * @brief Draws lidar measurements.
 *
 * @param measures Pointer to a pointcloud_t structure containing lidar data.
 *
 * Processes and graphically displays the lidar data as a set of points or a cloud,
 * allowing visualization of the detected obstacles or environment.
 */
void draw_lidar(pointcloud_t *measures);

/**
 * @brief Draws detected cones.
 *
 * @param detected_cones Pointer to a cone structure that holds information about detected cones.
 *
 * Renders the detected cones on the display, which could represent position markers
 * or obstacles in the simulation environment.
 */
void draw_detected_cones(cone *detected_cones);

/**
 * @brief Draws the cone map (track map).
 *
 * @param track_map Pointer to an array of cone structures representing the map.
 * @param track_map_idx Index representing the current position or number of cones on the track map.
 *
 * This function displays the cone-based track map, helping the user to visualize the layout and key points along the track.
 */
void draw_cone_map(cone *track_map, int track_map_idx);

/**
 * @brief Draws perception results.
 *
 * This function is responsible for visualizing the outcomes of the perception process,
 * which might include sensor fusion results or object detection overlays.
 */
void draw_perception();

/**
 * @brief Draws the planned trajectory.
 *
 * @param trajectory Pointer to an array of waypoint structures representing the planned path.
 *
 * Visualizes the computed trajectory on the display, providing insight into planned routes or maneuvers.
 */
void draw_trajectory(waypoint *trajectory);

/**
 * @brief Updates the display.
 *
 * This function refreshes the display, ensuring that all drawn elements are updated
 * according to the latest simulation data.
 */
void update_display();


#endif // DISPLAY_H