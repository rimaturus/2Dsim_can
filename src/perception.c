/**
 * @defgroup PerceptionModule Cone Detection and Mapping
 * @{
 */

/**
 * @file perception.c
 * @brief Implements LiDAR-based cone detection and mapping using circle Hough transform.
 *
 * This file contains functions to process LiDAR measurements, cluster points belonging
 * to the same cone, compute cone centers with a Hough circle parameterization, and update
 * a global map of detected cones.
 *
 * Global variables:
 *  - sliding_window: Number of angles scanned (360 degrees).
 *  - angle_step: Step in degrees for each LiDAR scan.
 *  - start_angle: Starting angle of the LiDAR scan.
 *  - ignore_distance: Minimum distance threshold for detection.
 *  - distance_resolution: Increment step for distances along LiDAR rays.
 *  - detected_cones: Array that holds final detected cone positions and colors.
 *  - n_candidates: Number of candidate cones currently tracked.
 *  - candidates: Array that holds candidate cone detections.
 *  - track_map_idx: Index for the global map where detected cones are stored.
 *  - track_map: Global map of cone positions and colors.
 *
 * @defgroup Perception_Functions Cone Detection and Mapping
 * @{
 */

/**
 * @brief Checks if a given LiDAR point is near an already detected cone.
 *
 * The function loops over the cone borders array and finds a suitable cone where the new
 * point (defined by angle and (x,y) coordinates) can be assigned. If the cone border is empty,
 * it is initialized; if the cone already exists, the function compares distances to decide
 * if the new point should be added to that cone's border.
 *
 * @param angle Angle index at which the measurement was taken.
 * @param new_point_x X coordinate of the new LiDAR point.
 * @param new_point_y Y coordinate of the new LiDAR point.
 * @param color Color identifier of the detected point (e.g., yellow or blue).
 * @param cone_borders Pointer to an array of cone_border structures that hold border points.
 */
 
/**
 * @brief Simulates a LiDAR scan around the vehicle to detect cones.
 *
 * For each angle in a 360 degree window, the function increments distances starting from 
 * a minimum ignore distance until an obstacle of interest (yellow or blue pixel) is reached.
 * The found distance and corresponding point coordinates are stored in the measures array.
 *
 * @param car_x Vehicle's x coordinate in world space.
 * @param car_y Vehicle's y coordinate in world space.
 * @param measures Pointer to a pointcloud_t array where LiDAR measurements are stored.
 */

/**
 * @brief Initializes an array of cone borders.
 *
 * Each cone border element is cleared by setting its angle indexes to -1 and color to -1,
 * representing an uninitialized state.
 *
 * @param cone_borders Pointer to the cone_border array to be initialized.
 */

/**
 * @brief Calculates and stores the circumference points for a cone.
 *
 * Given a cone center (center_x, center_y) and its color, the function computes 360 points
 * around the circle using the predefined cone_radius, and stores them in the provided array.
 *
 * @param center_x X coordinate of the cone center.
 * @param center_y Y coordinate of the cone center.
 * @param color Color of the cone.
 * @param circle_points Pointer to an array of cone structures that will receive the circle points.
 */

/**
 * @brief Finds the closest intersection points between candidate circles.
 *
 * For a circle (centered at point_x, point_y) with a predefined cone radius, the function calculates
 * its circumference points (360 possible points) and determines the minimum distance to points provided
 * in the reference_points array. The resulting closest points are stored in the circumference_points array.
 *
 * @param circumference_points Array of Hough_circle_point_t where the processed points will be stored.
 * @param point_x X coordinate used as the origin for the circumference.
 * @param point_y Y coordinate used as the origin for the circumference.
 * @param reference_points Array of Hough_circle_point_t used as a reference to compute distances.
 * @param ref_size Number of elements in the reference_points array.
 */

/**
 * @brief Identifies two local minima in a set of circle intersection points.
 *
 * The function analyzes the trends of the distances stored in the provided points array and finds
 * the indexes of the two local minima. These minima correspond to candidate centers for the cone.
 *
 * @param points Array of Hough_circle_point_t representing distances along a circumference.
 * @param first_min Pointer to an integer that will store the index of the first local minimum.
 * @param second_min Pointer to an integer that will store the index of the second local minimum.
 */

/**
 * @brief Performs cluster analysis on candidate cone centers.
 *
 * The function examines an array of candidate centers (possible_centers) and groups those that
 * are spatially close using a clustering threshold. The coordinates of the best candidate cluster
 * (i.e., the one with the most points) are averaged and returned as the computed cone center.
 *
 * @param possible_centers Array of Hough_circle_point_t containing potential cone centers.
 * @param center_count The number of candidate centers in the array.
 * @return Pointer to a static float array of two elements where [0] is the x coordinate and [1] is the y coordinate of the computed cone center.
 */

/**
 * @brief Processes LiDAR measures to detect and map cones using a circle Hough transformation.
 *
 * The mapping function groups LiDAR points detected as cones into clusters (cone borders) using 
 * the check_nearest_point helper. It then calculates circle intersections for points forming each
 * cone border, finds local minima corresponding to candidate cone centers, clusters these candidate centers,
 * and finally selects the best center for each cone. The resulting cone centers are stored, with their
 * color information preserved.
 *
 * @param car_x Vehicle's x coordinate in world space.
 * @param car_y Vehicle's y coordinate in world space.
 * @param car_angle Vehicle's heading angle in degrees.
 * @param detected_cones Pointer to an array of cones where the detected cone centers will be stored.
 */

/**
 * @brief Updates the global map with newly detected cones.
 *
 * For each detected cone in the detected_cones array, the function checks against existing candidates.
 * If the newly detected cone is close to a candidate, a moving average is computed for its position. If a candidate
 * reaches a detection threshold (DETECTIONS_THRESHOLD), it is committed to the global map (track_map).
 * Otherwise, if no nearby candidate is found, a new candidate is created.
 *
 * @param detected_cones Pointer to the array of cones with detected centers and colors ready for map update.
 *
 * @}
 */
#include <stdio.h>
#include <math.h>
#include <string.h>

#include "globals.h"
#include "perception.h"
#include "trajectory.h"

const int sliding_window = 360;
const int angle_step = 1;
int start_angle = 0;

const float ignore_distance = 0.5f;
const float distance_resolution = 0.01f;
cone detected_cones[MAX_DETECTED_CONES];

int n_candidates = 0;
candidate_cone candidates[MAX_CANDIDATES];

int track_map_idx = 0;
cone track_map[MAX_CONES_MAP];

// LiDAR measures
void 	check_nearest_point(int angle, float new_point_x, float new_point_y, int color, cone_border *cone_borders)
{
	// check if the point is near to a cone
	for (int i = 0; i < MAX_DETECTED_CONES; i++)
	{
		if (cone_borders[i].color == -1)
		{
			// no cone detected at this position
			int insertion_point = 0;
			while ((insertion_point < MAX_POINTS_PER_CONE-1) && (cone_borders[i].angles[insertion_point] != -1)) insertion_point++;

			// populate the new cone border
			cone_borders[i].angles[insertion_point] = angle;
			cone_borders[i].color = color;
			break;
		}
		else // a cone is detected at this position
		{
			int insertion_point = 0;	// find the first free position to put this new point in the border array
			float distance = 2*maxRange;	// max distance between two points detected

			int isPointOnCone = 0;		// flag to check if the point is near to a cone

			while ((insertion_point < MAX_POINTS_PER_CONE-1) && (cone_borders[i].angles[insertion_point] != -1)){
				float cone_point_x = measures[cone_borders[i].angles[insertion_point]].point_x;
				float cone_point_y = measures[cone_borders[i].angles[insertion_point]].point_y;

				distance = sqrt(pow(new_point_x - cone_point_x, 2) + pow(new_point_y - cone_point_y, 2));

				if (distance < 2*cone_radius){
					// the point is near to the i-th cone
					isPointOnCone = 1;
				}

				insertion_point++; 
			}

			if (isPointOnCone){	// if the point is next to the i-th cone
				cone_borders[i].angles[insertion_point] = angle;
				break;
			}
		}

	}
}

void    lidar(float car_x, float car_y, pointcloud_t *measures)
{
	int stop_distance; 

	// Check for each angle in the range [0, 360] with a step of angle_step
	for (int i = 0; i < sliding_window; i += angle_step)
	{
		int		lidar_angle = (start_angle + i)%360;
		// int 	current_distance = 0; // initialize distance at 0

		// Initialize the measure with the maximum range and no color
		measures[lidar_angle].distance = maxRange;
		measures[lidar_angle].color = -1; // cone not detected

		stop_distance = 0;

		// Check each pixel in the range [0, maxRange] with a step of distance_resolution
		for (float distance = ignore_distance; distance < maxRange; distance += distance_resolution)
		{
			// Calculate the x and y coordinates of the pixel at the current distance and angle
			float x = car_x + ( distance * cos((float)(lidar_angle) * deg2rad) );
			float y = car_y + ( distance * sin((float)(lidar_angle) * deg2rad) );

			int x_px = x * px_per_meter;
			int y_px = y * px_per_meter;

			//putpixel(screen, x_px, y_px, makecol(255, 0, 0));

			if (stop_distance != 1)
			{
				if (getpixel(track, x_px, y_px) == yellow) // yellow
				{
					// printf("Cone detected\n");
					measures[lidar_angle].distance = distance; // convert to meters
					measures[lidar_angle].color = yellow; // makecol(254, 221, 0); // yellow
					measures[lidar_angle].point_x = x;
					measures[lidar_angle].point_y = y;

					stop_distance = 1;  // cone detected, stop the loop
				}
				else if (getpixel(track, x_px, y_px) == blue) // blue
				{   
					// printf("Cone detected\n");
					measures[lidar_angle].distance = distance; // convert to meters
					measures[lidar_angle].color = blue; // blue
					measures[lidar_angle].point_x = x;
					measures[lidar_angle].point_y = y;

					stop_distance = 1;  // cone detected, stop the loop
				}
			}            
		}
	}
}

// Initialize cone borders array
void	init_cone_borders(cone_border *cone_borders) 
{
	for (int i = 0; i < MAX_DETECTED_CONES; i++) 
	{
		for (int j = 0; j < MAX_POINTS_PER_CONE; j++) 
		{
			cone_borders[i].angles[j] = -1;
		}
		cone_borders[i].color = -1;
	}
}

// Calculate circle points for a given center and radius
void 	calculate_circle_points(float center_x, float center_y, int color, cone *circle_points) 
{
	for (int i = 0; i < 360; i++) 
	{
		circle_points[i].x = center_x + cone_radius * cos(i * deg2rad);
		circle_points[i].y = center_y + cone_radius * sin(i * deg2rad);
		circle_points[i].color = color;
	}
}

// Find closest points between two circles
void 	find_closest_points(Hough_circle_point_t *circumference_points, float point_x, float point_y, 
						Hough_circle_point_t *reference_points, int ref_size) 
{
	for (int i = 0; i < 360; i++) 
	{
		circumference_points[i].distance = 2*maxRange;
		float new_x = point_x + cone_radius * cos(i * deg2rad);
		float new_y = point_y + cone_radius * sin(i * deg2rad);

		for (int j = 0; j < ref_size; j++) 
		{
			float new_distance = sqrt(pow(new_x - reference_points[j].x, 2) + 
									pow(new_y - reference_points[j].y, 2));

			if (new_distance < circumference_points[i].distance) 
			{
				circumference_points[i].distance = new_distance;
				circumference_points[i].x = new_x;
				circumference_points[i].y = new_y;
			}
		}
	}
}

// Find local minima in circle intersection points
void find_local_minima(Hough_circle_point_t *points, int *first_min, int *second_min) 
{
	int prev_trend = 0, actual_trend = 0;
	*first_min = -1;
	*second_min = -1;

	for (int k = 1; k < 360; k++) 
	{
		if (points[k].distance < points[k-1].distance) actual_trend = -1;
		else if (points[k].distance > points[k-1].distance) actual_trend = 1;
		else actual_trend = 0;

		if (prev_trend == -1 && actual_trend == 1) 
		{
			if (*first_min == -1) *first_min = k-1;
			else if (*second_min == -1) *second_min = k-1;
		}

		if (actual_trend != 0) prev_trend = actual_trend;
	}
}

// Cluster analysis to find cone center
float* find_cone_center(Hough_circle_point_t *possible_centers, int center_count) 
{
	const float CLUSTER_THRESHOLD = 0.01f;
	static float result[2];

	int best_cluster_size = 0;
	float best_sum_x = 0.0f, best_sum_y = 0.0f;

	for (int i = 0; i < center_count; i++) 
	{
		int cluster_size = 1;
		float cluster_sum_x = possible_centers[i].x;
		float cluster_sum_y = possible_centers[i].y;

		for (int j = i + 1; j < center_count; j++) 
		{
			float dx = possible_centers[i].x - possible_centers[j].x;
			float dy = possible_centers[i].y - possible_centers[j].y;

			if (sqrt(dx * dx + dy * dy) < CLUSTER_THRESHOLD) 
			{
				cluster_sum_x += possible_centers[j].x;
				cluster_sum_y += possible_centers[j].y;
				cluster_size++;
			}
		}
		if (cluster_size > best_cluster_size) 
		{
			best_cluster_size = cluster_size;
			best_sum_x = cluster_sum_x;
			best_sum_y = cluster_sum_y;
		}
	}

	if (best_cluster_size > 0) 
	{
		result[0] = best_sum_x / best_cluster_size;
		result[1] = best_sum_y / best_cluster_size;
	}
	return result;
}


void 	mapping(float car_x, float car_y, int car_angle, cone *detected_cones)
{
(void)car_x;       // Silence unused parameter warnings
(void)car_y;
(void)car_angle; 
cone_border cone_borders[MAX_DETECTED_CONES]; // maximum number of cones viewed at each position

// init cone borders
for (int i = 0; i < MAX_DETECTED_CONES; i++){
	for (int j = 0; j < MAX_POINTS_PER_CONE; j++){
		cone_borders[i].angles[j] = -1;
	}
	cone_borders[i].color = -1;
}

	// Circle Hough transformation of viewed points
	for (int angle = 0; angle < 360; angle += angle_step)
	{
		// Group similar points
		if (measures[angle].color == -1)
		{
			// no cone seen at this angle, PASS
			continue;
		}
		else // a cone is detected at this angle
		{
			check_nearest_point(angle, measures[angle].point_x, measures[angle].point_y, measures[angle].color, cone_borders);
		}

	}

	// at this step the cone_borders contain the points of the pointcloud_t that are closer each other
	// classified by cone
	// now we need to calculate the center of the cones

	int cone_idx = 0;
	int detected_cone_idx = 0; // index where insert the new detected cone center

	typedef struct {
		float x;
		float y;
		float distance;
		int color;
	} Hough_circle_point;
	
	// for each cone detected
	while( (cone_idx < MAX_DETECTED_CONES-1) && (cone_borders[cone_idx].color != -1) ){
			// count the number of points in the border of the cone
			int N_border_points = 0;

			// count the number of points in the border of the cone
			while ( (N_border_points < MAX_POINTS_PER_CONE-1) && (cone_borders[cone_idx].angles[N_border_points] != -1) ){
				N_border_points++;
			}

		if (N_border_points > 2) // we need at least 3 points to calculate the center of the cone
		{
			// ----------------- LOCAL MINIMA VARIABLE -----------------
			// 			necessary to find the center of the cone
			// float LMS_prev_dist, LMS_actual_dist;
			int LMS_first_min_idx = -1, LMS_second_min_idx = -1;
			int LMS_first_max_idx = -1, LMS_second_max_idx = -1; // LMS == Local Minima Search
			// ---------------------------------------------------------


 			// store the solution (w.c.s.: 2 intersection for each pair of circle referred to points)	
			Hough_circle_point possible_cone_centers[(N_border_points-1)*2];
			int possible_center_idx = 0;


			// for each points of the border of the cone
			for (int point_idx = 0; point_idx < N_border_points; point_idx++){
				/* Starting... [POINT_IDX == 0] */
				if (point_idx == 0){	// skip the first point (we need at least two point to compute intersection)
					continue; 
				}
				/* Endig... [POINT_IDX == 0] */

				// store all the points of the second circumference with the distance of the closest point of the first circumference/set of points
				Hough_circle_point circumference_points[360]; 

				/* Starting... [POINT_IDX == 1] */
				if (point_idx == 1){	// first iteration (second point)
										// consider all the 360 points of both the circumference to find the 2 intersections

					// The first point give us all the 360 possible centers of the cone (the circle that pass through the point)
					cone first_point_circle[360];

					for (int i = 0; i < 360; i++){
						first_point_circle[i].x = measures[cone_borders[cone_idx].angles[0]].point_x + cone_radius * cos(i * deg2rad);
						first_point_circle[i].y = measures[cone_borders[cone_idx].angles[0]].point_y + cone_radius * sin(i * deg2rad);
						first_point_circle[i].color = measures[cone_borders[cone_idx].angles[0]].color;
					}

					// for each point of the detected cone border 
				
					// Calculate the center of the cone
					// for all the angles that corresponds to a cone we need to calculate the center of the cone
					float new_x, new_y, new_distance;

					for (int i = 0; i < 360; i++){ // initialize the possible points
						circumference_points[i].distance = 2*maxRange;
						circumference_points[i].x = 0;
						circumference_points[i].y = 0;
					}

					for (int i = 0; i < 360; i++){
						new_x = measures[cone_borders[cone_idx].angles[point_idx]].point_x + cone_radius * cos(i * deg2rad);
						new_y = measures[cone_borders[cone_idx].angles[point_idx]].point_y + cone_radius * sin(i * deg2rad);

						for (int j = 0; j < 360; j++){
							new_distance = sqrt(pow(new_x - first_point_circle[j].x, 2) + pow(new_y - first_point_circle[j].y, 2));

							if (new_distance < circumference_points[i].distance){
								circumference_points[i].distance = new_distance;
								circumference_points[i].x = new_x;
								circumference_points[i].y = new_y;
							}
						}
					}
				} 
				// at the end of this iteration we have the distance of each point of the second circumference that is closer to the first one
				// all of this points are stored in the possible_points array (Hough circle points type)

				/* Ending... [POINT_IDX == 1] */
				
				/* Starting... [POINT_IDX >= 2] */
				if (point_idx >= 2){	
					// we have 3 or more points so we can compute the min distance of each point of the circumference
					// from the points of intersection found in the previous iteration (less overhead)
					float new_x, new_y, new_distance;

					for (int i = 0; i < 360; i++){ // initialize the possible points
						circumference_points[i].distance = 2*maxRange;
						circumference_points[i].x = 0;
						circumference_points[i].y = 0;
					}

					for (int i = 0; i < 360; i++){
						new_x = measures[cone_borders[cone_idx].angles[point_idx]].point_x + cone_radius * cos(i * deg2rad);
						new_y = measures[cone_borders[cone_idx].angles[point_idx]].point_y + cone_radius * sin(i * deg2rad);

						for (int j = 0; j < 360; j++){
							new_distance = sqrt(pow(new_x - possible_cone_centers[j].x, 2) + pow(new_y - possible_cone_centers[j].y, 2));

							if (new_distance < circumference_points[i].distance){
								circumference_points[i].distance = new_distance;
								circumference_points[i].x = new_x;
								circumference_points[i].y = new_y;
							}
						}
					}

				}
				/* Ending... [POINT_IDX >= 2] */

				// at this point, we have a 3rd order function that represent the min distances 
				// of each point of the new circumference from the first one while they intersecate
				// we need to find the two local minima of this function to find the two possible centers of the cone
				// float LMS_prev_dist, LMS_actual_dist;
				// int LMS_first_min_idx = -1, LMS_second_min_idx = -1;
				// int LMS_first_max_idx = -1, LMS_second_max_idx = -1; // LMS == Local Minima Search

				int prev_trend = 0, actual_trend = 0; // check the trend of the function [1: if crescent, 0: if constant, -1: if decrescent]
				
				for (int k = 1; k < 360; k++){
					if (circumference_points[k].distance < circumference_points[k-1].distance){
						actual_trend = -1; // decrescent
					}
					else if ( circumference_points[k].distance > circumference_points[k-1].distance){
						actual_trend = 1; // crescent
					}
					else{
						actual_trend = 0; // constant
					}

					// now we know the trend of the function
					// we can search for the local minima
					if (prev_trend == actual_trend){
						// we are in the same regione of the function
						continue;
					}
					else if (prev_trend == -1 && actual_trend == 1){
						// we are in a local minima
						if (LMS_first_min_idx == -1){
							LMS_first_min_idx = k-1;
						}
						else if (LMS_second_min_idx == -1){
							LMS_second_min_idx = k-1;
						}
					}
					else if (prev_trend == 1 && actual_trend == -1){
						// we are in a local maxima
						if (LMS_first_max_idx == -1){
							LMS_first_max_idx = k-1;
						}
						else if (LMS_second_max_idx == -1){
							LMS_second_max_idx = k-1;
						}
					}

					if (actual_trend == 0){
						// we are in a constant region (not update the state of the trend)
						continue;
					}
					else {	// we are in a region of the function that is not a local minima or maxima
						prev_trend = actual_trend;
					}
				} // [TODO] Mitigate the possible errors of bouncing of the function near the local minima or maxima			

				// at this point we have the two local minima of the function stored in LMS_first_min_idx and LMS_second_min_idx
				possible_cone_centers[possible_center_idx].x = circumference_points[LMS_first_min_idx].x;
				possible_cone_centers[possible_center_idx].y = circumference_points[LMS_first_min_idx].y;

				possible_cone_centers[possible_center_idx + 1].x = circumference_points[LMS_second_min_idx].x;
				possible_cone_centers[possible_center_idx + 1].y = circumference_points[LMS_second_min_idx].y;

				possible_center_idx += 2;
			}

			/* 
			At this point we have the possible centers of the cone.
			
			We need to choose the one that stays inside the cone 
			(the two intersection are one on the center of the cone and the other outside).
			*/
	
			// Cluster the candidate centers to select those that are closest
			// and compute their mean.
			const float CLUSTER_THRESHOLD = 0.01f; // distance threshold in meters
			int best_cluster_size = 0;
			float best_sum_x = 0.0f, best_sum_y = 0.0f;

			// Iterate over all candidate centers stored in possible_cone_centers
			for (int i = 0; i < possible_center_idx; i++) {
				int cluster_size = 1;
				float cluster_sum_x = possible_cone_centers[i].x;
				float cluster_sum_y = possible_cone_centers[i].y;

				for (int j = i + 1; j < possible_center_idx; j++) {
					float dx = possible_cone_centers[i].x - possible_cone_centers[j].x;
					float dy = possible_cone_centers[i].y - possible_cone_centers[j].y;
					float dist = sqrt(dx * dx + dy * dy);

					if (dist < CLUSTER_THRESHOLD) {
						cluster_sum_x += possible_cone_centers[j].x;
						cluster_sum_y += possible_cone_centers[j].y;
						cluster_size++;
					}
				}
				if (cluster_size > best_cluster_size) {
					best_cluster_size = cluster_size;
					best_sum_x = cluster_sum_x;
					best_sum_y = cluster_sum_y;
				}
			}

			if (best_cluster_size > 0) {
				float center_x = best_sum_x / best_cluster_size;
				float center_y = best_sum_y / best_cluster_size;

				detected_cones[detected_cone_idx].x = center_x;
				detected_cones[detected_cone_idx].y = center_y;
				detected_cones[detected_cone_idx].color = cone_borders[cone_idx].color;

				detected_cone_idx++;
			}
		}

		cone_idx++;
	}

	update_map(detected_cones);	
}

// Update the map
void update_map(cone *detected_cones) 
{
	int N_new_detections = 0;
	while (detected_cones[N_new_detections].color != -1) N_new_detections++; // count new detections

	// Process each new detection
	for (int new_idx = 0; new_idx < N_new_detections; new_idx++) 
	{
		int found = 0;
		
		// Check if detection matches any existing candidate
		for (int i = 0; i < n_candidates; i++) 
		{
			float distance = sqrt(pow(detected_cones[new_idx].x - candidates[i].x, 2) + 
								pow(detected_cones[new_idx].y - candidates[i].y, 2));
			
			if (distance < 3 * cone_radius) 
			{	
				// Only update if we haven't reached the threshold yet
				if (candidates[i].detections < DETECTIONS_THRESHOLD) 
				{	// Update candidate position with moving average
					candidates[i].x = (candidates[i].x * candidates[i].detections + detected_cones[new_idx].x) / (candidates[i].detections + 1);
					candidates[i].y = (candidates[i].y * candidates[i].detections + detected_cones[new_idx].y) / (candidates[i].detections + 1);
					candidates[i].detections++;
					
					// If threshold reached, add to map
					if (candidates[i].detections == DETECTIONS_THRESHOLD) 
					{
						track_map[track_map_idx].x = candidates[i].x;
						track_map[track_map_idx].y = candidates[i].y;
						track_map[track_map_idx].color = candidates[i].color;
						track_map_idx++;
					}
				}
				
				found = 1;
				break;
			}
		}
		
		// If no matching candidate found, create new one
		if (!found && n_candidates < MAX_CANDIDATES) 
		{
			candidates[n_candidates].x = detected_cones[new_idx].x;
			candidates[n_candidates].y = detected_cones[new_idx].y;
			candidates[n_candidates].color = detected_cones[new_idx].color;
			candidates[n_candidates].detections = 1;
			n_candidates++;
		}
	}
}

/** @} */