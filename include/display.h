#ifndef DISPLAY_H
#define DISPLAY_H

#include <globals.h>

void draw_dir_arrow();

void draw_car(float car_x, float car_y, int car_angle);

void draw_track();

void draw_lidar(pointcloud_t *measures); 

void draw_detected_cones(cone *detected_cones);

void draw_cone_map(cone *track_map, int track_map_idx);

void draw_perception();

void draw_trajectory(waypoint *trajectory);

void update_display();


#endif // DISPLAY_H