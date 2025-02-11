#include <stdio.h>
#include <allegro.h>

#include "control.h"
#include "display.h"
#include "globals.h"
#include "perception.h"
#include "tasks.h"
#include "trajectory.h"
#include "utilities.h"
#include "vehicle.h"
#include "ptask.h"

const char	filename[100] = "track/cones.yaml";
int car_x_px, car_y_px;
int car_bitmap_x, car_bitmap_y;

void init_allegro();

void init_track();
void init_car();
void init_perception();
void init_visual_controls();

void init_bitmaps();
void update_screen();

int main(void)
{
	init_allegro();

	init_bitmaps();
	update_screen();

	// Initialize the periodic task system (using SCHED_OTHER)
	ptask_init(SCHED_OTHER);

	// Create periodic tasks: perception, trajectory, control, display

	if (task_create(1, perception_task, PERCEPTION_PERIOD, PERCEPTION_DEADLINE, PERCEPTION_PRIORITY, ACT) != 0) {
		fprintf(stderr, "Failed to create Perception Task\n");
		exit(EXIT_FAILURE);
	}

	if (task_create(2, trajectory_task, TRAJECTORY_PERIOD, TRAJECTORY_DEADLINE, TRAJECTORY_PRIORITY, ACT) != 0) {
		fprintf(stderr, "Failed to create Trajectory Task\n");
		exit(EXIT_FAILURE);
	}

	if (task_create(3, control_task, CONTROL_PERIOD, CONTROL_DEADLINE, CONTROL_PRIORITY, ACT) != 0) {
		fprintf(stderr, "Failed to create Control Task\n");
		exit(EXIT_FAILURE);
	}

	if (task_create(4, display_task, DISPLAY_PERIOD, DISPLAY_DEADLINE, DISPLAY_PRIORITY, ACT) != 0) {
		fprintf(stderr, "Failed to create Display Task\n");
		exit(EXIT_FAILURE);
	}

	// Wait for tasks to terminate (they will exit when ESC is pressed)
	wait_for_task_end(1);
	wait_for_task_end(2);
	wait_for_task_end(3);
	wait_for_task_end(4);

	printf("Exiting simulation...\n");
	clear_keybuf();
	readkey();
	allegro_exit();
	return 0;
}


void init_allegro()
{
	allegro_init(); // initialize graphics data structures
	install_keyboard(); // initialize the keyboard
	install_mouse(); // initialize the mouse
	
	// Initialize the graphics mode
	set_color_depth(32); // set the color depth to 8 bits for each of the RGB channels and 8 bits for the alpha channel (faster than 24 bit since it is aligned to 32 bits)

	// Colors
	grass_green = makecol(78,91,49); // army_green
	asphalt_gray = makecol(128,126,120); // asphalt
	white = makecol(255, 255, 255); // white
	pink = makecol(255, 0, 255); // pink
	yellow = makecol(254, 221, 0); // yellow for cones
	blue = makecol(46, 103, 248); // blue for cones
	
	set_gfx_mode(GFX_AUTODETECT_WINDOWED, X_MAX, Y_MAX, 0, 0);
	
	set_window_title("2D FSAE Simulation"); // Optional: sets window title
	set_display_switch_mode(SWITCH_BACKGROUND); // Allows window to be minimized/switched

	clear_to_color(screen, pink); // clear the screen making all pixels to white
}

void init_track()
{
	track = create_bitmap(X_MAX, Y_MAX);
		clear_bitmap(track);
		clear_to_color(track, asphalt_gray);
		
	cone	cones[MAX_CONES_MAP];


		init_cones(cones);
		load_cones_positions(filename, cones, MAX_CONES_MAP);
		printf("Cones loaded\n");

		// plot cones
		for (int i = 0; i < MAX_CONES_MAP; i++)
		{
			if (cones[i].color != -1) // plot only track cones
			{
				// printf("Cone %d: x = %f, y = %f, color = %d\n", i, cones[i].x, cones[i].y, cones[i].color);
				// circlefill(screen, i * px_per_meter, i * px_per_meter, 5, makecol(255,0,0));
				circlefill(
					track, 
					cones[i].x, 
					cones[i].y, 
					cone_radius * px_per_meter, // radius = 5 cm
					cones[i].color
				);
			}
		}
}

void init_car()
{
	car = load_bitmap("bitmaps/f1_car_05x.bmp", NULL); // load a bitmap image
		if (car == NULL) {
			printf("Error loading sprite\n");
			exit(1);
		}  	

	// car_x and car_y are the center of the car
	// so the top-left corner of the car bitmap is at (car_x - car->w/2, car_y - car->h/2)
	// those are needed to make the center of the sprite coincide with the car position
	car_x_px = (int)(car_x * px_per_meter);
	car_y_px = (int)(car_y * px_per_meter);

	car_bitmap_x = car_x_px - (car->w / 2);
	car_bitmap_y = car_y_px - (car->h / 2);

#ifdef DEBUG
		circlefill(screen, car_x_px, car_y_px, 3, makecol(0,255,0));
		circlefill(screen, (int)(car_bitmap_x), (int)(car_bitmap_y), 3, makecol(0,255,255));
#endif /* DEBUG */
}

void init_perception()
{
	perception = create_bitmap(2*maxRange*px_per_meter, 2*maxRange*px_per_meter);
		clear_bitmap(perception);
		clear_to_color(perception, pink); // pink color to make it transparent (True color notation)
}

void init_trajectory()
{
	trajectory_bmp = create_bitmap(X_MAX, Y_MAX);
		clear_bitmap(trajectory_bmp);
		clear_to_color(trajectory_bmp, pink);
}

void init_visual_controls()
{
	steering_wheel = load_bitmap("bitmaps/f1_steer.bmp", NULL); // load a bitmap image
	if (steering_wheel == NULL) {
		printf("Error loading sprite\n");
		exit(1);
	}  	
	// Visual pedal gauge
	throttle_gauge = create_bitmap(50, 2 * maxThrottleHeight);
		clear_bitmap(throttle_gauge);
		clear_to_color(throttle_gauge, pink);
}

void init_bitmaps()
{
	display_buffer = create_bitmap(X_MAX, Y_MAX);
		clear_bitmap(display_buffer);
		clear_to_color(display_buffer, pink);

	
	background = create_bitmap(X_MAX, Y_MAX); 
		clear_bitmap(background);
		clear_to_color(background, grass_green);

	init_track();
	init_car();
	init_perception();
	init_trajectory();

	init_visual_controls();
}

void update_screen()
{
	pthread_mutex_lock(&draw_mutex);
		
		clear_to_color(display_buffer, pink);

		rotate_scaled_sprite(
			display_buffer, 
			steering_wheel, 
			100, 
			100, 
			ftofix(angle_rotation_sprite((int)(steering) / deg2rad)),  // deg to fixed point
			ftofix(0.5)
		);

		draw_sprite(display_buffer, background, 0, 0);
		draw_sprite(display_buffer, track, 0, 0);

		rotate_scaled_sprite(
			display_buffer, 
			car, 
			(int)(car_bitmap_x), 
			(int)(car_bitmap_y), 
			ftofix(angle_rotation_sprite(car_angle)),  // deg to fixed point
			ftofix(1)
		);

		draw_sprite(
			display_buffer, 
			perception, 
			car_x_px - maxRange*px_per_meter,
			car_y_px - maxRange*px_per_meter
		);

		draw_sprite(display_buffer, trajectory_bmp, 0, 0);

		// Draw the display buffer to the screen
		blit(display_buffer, screen, 0, 0, 0, 0, X_MAX, Y_MAX);

	pthread_mutex_unlock(&draw_mutex);
}