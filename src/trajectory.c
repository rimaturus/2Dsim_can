#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include "trajectory.h"
#include "globals.h"
#include "perception.h"

int trajectory_idx = 0;
waypoint trajectory[2*MAX_DETECTED_CONES];

void 	trajectory_planning(float car_x, float car_y, float car_angle, cone *detected_cones, waypoint *trajectory)
{
	// Initialize trajectory points to invalid values
	for (int i = 0; i < MAX_DETECTED_CONES; i++) {
		trajectory[i].x = -1;
		trajectory[i].y = -1;
	}
	
	if (track_map_idx < 3) {
		return; // Not enough cones in map to plan trajectory
	}

	int connected_indices[track_map_idx][2];
	const int B_idx = 0;
	const int Y_idx = 1;

	// Initialize connection matrix
	for (int i = 0; i < track_map_idx; i++) {
		for (int j = 0; j < 2; j++) {
			connected_indices[i][j] = -1;
		}
	}

	// Find nearest neighbors for each cone in track map
	for (int focus_idx = 0; focus_idx < track_map_idx; focus_idx++) {
		float mDist_Y = 1000;
		float mDist_B = 1000;
		int mDist_Y_idx = -1;
		int mDist_B_idx = -1;

		int focusColor = (track_map[focus_idx].color == yellow) ? Y_idx : B_idx;

		if (connected_indices[focus_idx][B_idx] != -1 && connected_indices[focus_idx][Y_idx] != -1) {
			continue; // Skip if cone already fully connected
		}

		// Find nearest yellow cone if needed
		if (connected_indices[focus_idx][Y_idx] == -1) {
			for (int candidate_idx = 0; candidate_idx < track_map_idx; candidate_idx++) {
				if (candidate_idx != focus_idx && track_map[candidate_idx].color == yellow) {
					float distance = sqrt(pow(track_map[candidate_idx].x - track_map[focus_idx].x, 2) + 
									   pow(track_map[candidate_idx].y - track_map[focus_idx].y, 2));
					
					if (distance < mDist_Y) {
						mDist_Y = distance;
						mDist_Y_idx = candidate_idx;
					}
				}
			}
			if (mDist_Y_idx >= 0) {
				connected_indices[focus_idx][Y_idx] = mDist_Y_idx;
				connected_indices[mDist_Y_idx][focusColor] = focus_idx;
			}
		}

		// Find nearest blue cone if needed
		if (connected_indices[focus_idx][B_idx] == -1) {
			for (int candidate_idx = 0; candidate_idx < track_map_idx; candidate_idx++) {
				if (candidate_idx != focus_idx && track_map[candidate_idx].color == blue) {
					float distance = sqrt(pow(track_map[candidate_idx].x - track_map[focus_idx].x, 2) + 
									   pow(track_map[candidate_idx].y - track_map[focus_idx].y, 2));
					
					if (distance < mDist_B) {
						mDist_B = distance;
						mDist_B_idx = candidate_idx;
					}
				}
			}
			if (mDist_B_idx >= 0) {
				connected_indices[focus_idx][B_idx] = mDist_B_idx;
				connected_indices[mDist_B_idx][focusColor] = focus_idx;
			}
		}
	}

#ifdef DEBUG
	// Draw connections between cones
	for (int i = 0; i < track_map_idx; i++) {
		if (connected_indices[i][B_idx] != -1 && connected_indices[i][Y_idx] != -1) {

		pthread_mutex_lock(&draw_mutex);	
			line(trajectory_bmp,
				track_map[i].x * px_per_meter,
				track_map[i].y * px_per_meter,
				track_map[connected_indices[i][B_idx]].x * px_per_meter,
				track_map[connected_indices[i][B_idx]].y * px_per_meter,
				blue);
			
			line(trajectory_bmp,
				track_map[i].x * px_per_meter,
				track_map[i].y * px_per_meter,
				track_map[connected_indices[i][Y_idx]].x * px_per_meter,
				track_map[connected_indices[i][Y_idx]].y * px_per_meter,
				yellow);
			
			line(trajectory_bmp,
				track_map[connected_indices[i][B_idx]].x * px_per_meter,
				track_map[connected_indices[i][B_idx]].y * px_per_meter,
				track_map[connected_indices[i][Y_idx]].x * px_per_meter,
				track_map[connected_indices[i][Y_idx]].y * px_per_meter,
				makecol(0, 255, 0));
		}
		pthread_mutex_unlock(&draw_mutex);
	}
#endif

	// Generate trajectory points from cone connections
	trajectory_idx = 0;
	for (int i = 0; i < track_map_idx && trajectory_idx < MAX_DETECTED_CONES; i++) {
		int opposite_color_idx = track_map[i].color == yellow ? B_idx : Y_idx;
		
		if (connected_indices[i][opposite_color_idx] != -1) {
			trajectory[trajectory_idx].x = (track_map[i].x + track_map[connected_indices[i][opposite_color_idx]].x) / 2;
			trajectory[trajectory_idx].y = (track_map[i].y + track_map[connected_indices[i][opposite_color_idx]].y) / 2;
			trajectory_idx++;
		}
	}

	// Reorder trajectory points based on proximity
	if (trajectory_idx > 1) 
	{
		waypoint temp[MAX_DETECTED_CONES];
		int used[MAX_DETECTED_CONES] = {0};
		
		// Copy first point
		temp[0] = trajectory[0];
		used[0] = 1;

		// Find nearest points iteratively
		for (int i = 1; i < trajectory_idx; i++) 
		{
			float min_dist = INFINITY;
			int min_idx = -1;

			for (int j = 0; j < trajectory_idx; j++) 
			{
				if (!used[j]) 
				{
					float dist = sqrt(pow(temp[i-1].x - trajectory[j].x, 2) + pow(temp[i-1].y - trajectory[j].y, 2));
					if (dist < min_dist) 
					{
						min_dist = dist;
						min_idx = j;
					}
				}
			}

			temp[i] = trajectory[min_idx];
			used[min_idx] = 1;
		}

		// Copy back to original array
		for (int i = 0; i < trajectory_idx; i++) 
		{
			trajectory[i] = temp[i];
		}
	}
	// printf("Trajectory points: %d\n", trajectory_idx);
}
