#include <stdio.h>
#include <stdlib.h>
#include <allegro.h>
#include <math.h>
#include <yaml.h>
#include <string.h>

// Constants
#define px_per_meter 1e2 // 100 pixels = 1 meters (1 px = 1 cm)
#define px2m_minimap 1 // pixels to meters (1 px = 1 m)
#define deg2rad 0.0174533 // degrees to radians

// Structures
typedef struct {
	float x;
	float y;
	int color;
} cone;

typedef struct {
        int distance;
        int color;
} detection;

// LiDAR
const int   angle_step = 1; // deg
const int   MAXrange = (int)(10 * px_per_meter); // m --> 10 m
const int   distance_resolution = (int)(0.01 * px_per_meter); // m --> 1 cm

const int   N_angles  = (int)(360 / angle_step);
const int   distance_steps = (int)(MAXrange / distance_resolution);

// Auxiliary functions
void    init_cones( cone *cones, int max_cones );
void    load_cones_positions( const char *filename, cone *cones, int max_cones );

// sensors
void    lidar( int car_x, int car_y, detection *measures );


int main()
{
	printf("Starting sim...\n");

	// Initializing Allegro
	allegro_init(); // initialize graphics data structures
	install_keyboard(); // initialize the keyboard
	install_mouse(); // initialize the mouse
	
	// Initialize the graphics mode
	set_color_depth(32); // set the color depth to 8 bits for each of the RGB channels and 8 bits for the alpha channel (faster than 24 bit since it is aligned to 32 bits)
	const int XMAX = ( 10 * px_per_meter );
    const int YMAX = ( 10 * px_per_meter ); // screen resolution


	// Colors
	const int grass_green = makecol(78,91,49); // army_green
	const int asphalt_gray = makecol(128,126,120); // asphalt
	const int white = makecol(255, 255, 255); // white
	
	set_gfx_mode(GFX_AUTODETECT_WINDOWED, XMAX, YMAX, 0, 0); // enters the graphics mode (windowed) with resolution 1920x1080
	clear_to_color(screen, white); // clear the screen making all pixels to white

	// Initialize the scenes
	BITMAP *background;
	BITMAP *track;
    BITMAP *car;

	// background
	background = create_bitmap(XMAX, YMAX);
	clear_bitmap(background);
	clear_to_color(background, grass_green);
    draw_sprite(screen, background, 0, 0);

	// track
	track = create_bitmap(XMAX, YMAX);
	clear_bitmap(track);
	clear_to_color(track, asphalt_gray);
	
    const int max_cones = 1000;
	cone 	cones[max_cones];
    const char    filename[100] = "track/cones.yaml";
			init_cones( &cones, max_cones );
			load_cones_positions( &filename, &cones, max_cones );
            printf("Cones loaded\n");

            // plot cones
            for (int i = 0; i < 1000; i++)
            {
                if (cones[i].color != -1) // plot only track cones
                {
                    circlefill(
                        track, 
                        cones[i].x, 
                        cones[i].y, 
                        0.05 * px_per_meter, // radius = 5 cm
                        cones[i].color
                    );
                }
            }

    draw_sprite(screen, track, 0, 0);

    // Vehicle
    int car_x = 1 * px_per_meter;
    int car_y = 1 * px_per_meter;
    int car_angle = 0; // deg

    car = load_bitmap("bitmaps/f1_car_pink.bmp", NULL); // load a bitmap image
    if (car == NULL) {
        printf("Error loading sprite\n");
        exit(1);
    }  

    // // make trasnparent background for the car
    // [TODO] Need to fix the mask to have more clean borders (may try HSV)

    // int black = makecol(0, 0, 0);
    // int pink = makecol(255, 0, 255);
    // BITMAP *car_mask = create_bitmap(car->w, car->h);

    // for (int x = 0; x < car->w; x++)
    //     for (int y = 0; y < car->h; y++)
    //     {
    //         int c = getpixel(car, x, y);
    //         if (c == white) c = black;
    //         putpixel(car_mask, x, y, c);
    //     }
    
    // save_bitmap("bitmaps/f1_car_pink.bmp", car_mask, NULL);


    //blit(car, screen, 0, 0, carpos_x, carpos_y, car->w, car->h); // draw the sprite on the screen at (300, 300)
    //draw_sprite(screen, car, carpos_x, carpos_y); // draw the sprite on the screen at (300, 300)
    stretch_sprite(screen, car, car_x, car_y, car->w/2, car->h/2); // draw the sprite on the screen at (300, 300)
    // choose this one to have the transparent background


    // Sensors run
    detection     measures[N_angles]; // results


    // Simulation loop
    do {
        printf("Running sim...\n");
        // Update the car position
        int center_car_x = car_x + (int)(car->w / 4);
        int center_car_y = car_y + (int)(car->h / 4);

        // Perception
        lidar(center_car_x, center_car_y, &measures);

        // Normalize angles in [ -180° ,  180° ] range
                            //   left    right
        for (int lidar_angle = 0; lidar_angle < 360; lidar_angle += angle_step)
        {   
            int total_angle = (lidar_angle + car_angle);
            float lidar2car_angle = total_angle > 180 ? (float)total_angle - 360.0 : (float)total_angle; // deg

            // plot lines from car to detected point
            int x_detection = center_car_x + (int)(measures[lidar_angle].distance * px_per_meter * cos(lidar2car_angle * deg2rad));
            int y_detection = center_car_y + (int)(measures[lidar_angle].distance * px_per_meter * sin(lidar2car_angle * deg2rad));

            if (measures[lidar_angle].color == -1) // no cone detected
            {
                line(screen, center_car_x, center_car_y, x_detection, y_detection, makecol(255, 0, 0));
            }
            else
            {
                line(screen, center_car_x, center_car_y, x_detection, y_detection, measures[lidar_angle].color);
            }
            // printf("Lidar angle: %d, Distance: %d, Color: %d\n", lidar_angle, measures[lidar_angle].distance, measures[lidar_angle].color);

        }
        clear_keybuf();
        readkey(); // wait for a key press to close the window

    } while (!key[KEY_ESC]); // exit the loop when the ESC key is pressed








	printf("Press any key to exit...\n");
	clear_keybuf();
	readkey(); // wait for a key press to close the window
	allegro_exit(); // deallocate graphics data structures

	return 0;
}

// Sensors
// LiDAR
void    lidar(int car_x, int car_y, detection *measures)
{
    int stop_distance; 

    // Check for each angle in the range [0, 360] with a step of angle_step
    for (int lidar_angle = 0; lidar_angle < 360; lidar_angle++)
    {
        int   current_distance = 0; // initialize distance at 0

        // Initialize the measure with the maximum range and no color
        measures[lidar_angle].distance = MAXrange / px_per_meter;
        measures[lidar_angle].color = -1; // cone not detected

        stop_distance = 0;

        // Check each pixel in the range [0, MAXrange] with a step of distance_resolution
        for (int distance = 0; distance < MAXrange; distance++)
        {
            // Calculate the x and y coordinates of the pixel at the current distance and angle
            int x = car_x + (int)( distance * cos((float)(lidar_angle) * deg2rad) );
            int y = car_y + (int)( distance * sin((float)(lidar_angle) * deg2rad) );

            putpixel(screen, x, y, makecol(255, 0, 0));

            if (stop_distance != 1)
            {
                if (getpixel(screen, x, y) == makecol(254, 221, 0)) // yellow
                {
                    printf("Cone detected\n");
                    measures[lidar_angle].distance = distance / px_per_meter; // convert to meters
                    measures[lidar_angle].color = makecol(254, 221, 0); // yellow
                    stop_distance = 1;  // cone detected, stop the loop
                }
                else if (getpixel(screen, x, y) == makecol(46, 103, 248)) // blue
                {   
                    printf("Cone detected\n");
                    measures[lidar_angle].distance = distance / px_per_meter; // convert to meters
                    measures[lidar_angle].color = makecol(46, 103, 248); // blue
                    stop_distance = 1;  // cone detected, stop the loop
                }
            }            
        }
    }
}



// Auxiliary functions

void init_cones(cone* cones, int max_cones)
{
	for (int i = 0; i < max_cones; i++)
	{
		cones[i].x = 0;
		cones[i].y = 0;
		cones[i].color = -1;
	}
}

#define tmp_scale  1.5 
// [TODO] Need to be substituted with corrected px_per_meter
// Load cone positions (the YAML file must contain x,y in meters considering that
// the vehicle is 2 x 1 meters and the cones are 0.1 x 0.1 meters)

void load_cones_positions(const char *filename, cone *cones, int max_cones) 
{   
    printf("Loading cones from file: %s\n", filename);
    FILE *file = fopen(filename, "r");
    if (!file) {
        fprintf(stderr, "Error: Unable to open file %s\n", filename);
        return;
    }

    yaml_parser_t parser;
    yaml_event_t event;

    // Initialize YAML parser
    if (!yaml_parser_initialize(&parser)) {
        fprintf(stderr, "Error: Failed to initialize YAML parser.\n");
        fclose(file);
        return;
    }
    yaml_parser_set_input_file(&parser, file);

    int i = 0; // Current cone index
    int state = 0; // Parser state
    char current_key[256] = {0}; // Buffer to store the current key

    /*
        Parser States:
        0 - Start (expecting 'cones' key)
        1 - Found 'cones' key
        2 - Inside 'cones' sequence
        3 - Inside a cone mapping
    */

    while (1) {
        if (!yaml_parser_parse(&parser, &event)) {
            fprintf(stderr, "Error: Failed to parse YAML file.\n");
            break;
        }

        if (event.type == YAML_STREAM_END_EVENT) {
            yaml_event_delete(&event);
            break;
        }

        switch (event.type) {
            case YAML_SCALAR_EVENT:
                if (state == 0) {
                    // Expecting the 'cones' key
                    if (strcmp((char *)event.data.scalar.value, "cones") == 0) {
                        state = 1;
                    }
                }
                else if (state == 3) {
                    // Inside a cone mapping, expecting key or value
                    if (strlen(current_key) == 0) {
                        // Expecting a key
                        strncpy(current_key, (char *)event.data.scalar.value, sizeof(current_key) - 1);
                        current_key[sizeof(current_key) - 1] = '\0'; // Ensure null-termination
                    }
                    else {
                        // Expecting a value corresponding to the current key
                        if (strcmp(current_key, "x") == 0) {
                            cones[i].x = ( atof((char *)event.data.scalar.value) ) * tmp_scale;
                        } 
                        else if (strcmp(current_key, "y") == 0) { 
                            cones[i].y = ( atof((char *)event.data.scalar.value) ) * tmp_scale;
                        } 
                        else if (strcmp(current_key, "color") == 0) {
                            if (strcmp((char *)event.data.scalar.value, "yellow") == 0) {
                                cones[i].color = makecol(254, 221, 0); // Yellow color
                            } 
                            else if (strcmp((char *)event.data.scalar.value, "blue") == 0) {
                                cones[i].color = makecol(46, 103, 248); // Blue color
                            }
                            // Add more colors as needed
                        }
                        // Reset current_key for the next key-value pair
                        current_key[0] = '\0';
                    }
                }
                break;

            case YAML_SEQUENCE_START_EVENT:
                if (state == 1) {
                    // Found the start of the 'cones' sequence
                    state = 2;
                }
                break;

            case YAML_MAPPING_START_EVENT:
                if (state == 2) {
                    // Starting a new cone mapping
                    if (i >= max_cones) {
                        fprintf(stderr, "Warning: Maximum number of cones (%d) reached. Additional cones will be ignored.\n", max_cones);
                        yaml_event_delete(&event);
                        goto cleanup;
                    }
                    // Initialize the current cone
                    cones[i].x = 0.0f;
                    cones[i].y = 0.0f;
                    cones[i].color = 0; // Reset color to indicate it's being set
                    state = 3;
                }
                break;

            case YAML_MAPPING_END_EVENT:
                if (state == 3) {
                    // Finished reading a cone's data
                    state = 2; // Return to 'cones' sequence
                    i++; // Move to the next cone
                }
                break;

            default:
                break;
        }

        yaml_event_delete(&event);
    }

cleanup:
    yaml_parser_delete(&parser);
    fclose(file);
    
    printf("Total cones loaded: %d\n", i);
}