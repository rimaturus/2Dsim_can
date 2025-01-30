#include <stdio.h>
#include <allegro.h>

// example function for key pressed check
void get_keycodes(char *scan, char *ascii);
void get_string(char *str, int x, int y, int color, int bg);

int main(){
	printf("Starting sim...\n");

	// Allegro part
	allegro_init(); // initialize graphics data structures
	install_keyboard(); // initialize the keyboard

	set_color_depth(32); // set the color depth to 8 bits for each of the RGB channels and 8 bits for the alpha channel (faster than 24 bit since it is aligned to 32 bits)

	int XMAX = 640, YMAX = 480; // screen resolution
	set_gfx_mode(GFX_AUTODETECT_WINDOWED, XMAX, YMAX, 0, 0); // enters the graphics mode (windowed) with resolution 640x480
	// the general command is: set_gfx_mode(GFX_AUTODETECT, w, h, vw, vh)
	

	// Color part
	int r, g, b; // must be integers in the range 0-255
	int color; 
		r = 255; g = 0; b = 0;
		color = makecol(r, g, b);	// makecol(r, g, b) returns a color value that can be used in drawing functions

		int px_color = getpixel(screen, 100, 100); // get the color of the pixel at (100, 100)
		
		// if you want to decompose color returned by getpixel into its RGB components
		int r_px, g_px, b_px;
		r_px = getr(px_color);
		g_px = getg(px_color);
		b_px = getb(px_color);

		clear_to_color(screen, makecol(255, 255, 255)); // clear the screen making all pixels to white

	// Drawing part
	int x, y;
	int col;
		x = 100; y = 100;
		putpixel(screen, x, y, color); // draw a pixel at (100, 100) with the color defined above
		col = getpixel(screen, x, y); // get the color of the pixel at (100, 100)

	int x1, y1, x2, y2;
		x1 = 100; y1 = 100; x2 = 200; y2 = 200;

		line(screen, x1, y1, x2, y2, color); // draw a line from (100, 100) to (200, 200) with the color defined above
		rect(screen, x1, y1, x2, y2, color); // draw a rectangle with the top-left corner at (100, 100) and the bottom-right corner at (200, 200) with the color defined above
		rectfill(screen, x1, y1, x2, y2, color); // draw a filled rectangle with the top-left corner at (100, 100) and the bottom-right corner at (200, 200) with the color defined above
		
		int radius = 50;
		circle(screen, x1, y1, radius, color); // draw a circle with the center at (100, 100) and radius 50 with the color defined above
		circlefill(screen, x1, y1, radius, color); // draw a filled circle with the center at (100, 100) and radius 50 with the color defined above

		int rx, ry;
		rx = 50; ry = 100;
		ellipse(screen, x1, y1, rx, ry, color); // draw an ellipse with the center at (100, 100), horizontal radius 50, vertical radius 100 with the color defined above
		ellipsefill(screen, x1, y1, rx, ry, color); // draw a filled ellipse with the center at (100, 100), horizontal radius 50, vertical radius 100 with the color defined above

		triangle(screen, x1, y1, x2, y1, x2, y2, color); // draw a triangle with vertices at (100, 100), (200, 100), (200, 200) with the color defined above

	int points[10] = {100, 100, 200, 100, 200, 200, 100, 200, 100, 100}; // {P1, P2, ..., Pn}
		polygon(screen, 5, points, color); // draw a polygon with 5 vertices with the color defined above
		polygon(screen, 4, points, color); // draw a polygon with 5 vertices with the color defined above

	// Text part
	int bg = makecol(0, 255, 255); // background color (if set to -1 it will be transparent)
	int text_col = makecol(255, 0, 0); // text color

	// if you want a new font [TODO]

	// 1) Download from the web with (.pcx) extension
	// 2) Define a pointer to the Allegro FONT structure: 
	//struct FONT		*myfont;
	// 3) Load the font from the file:
	//myfont = load_font("fonts/Montserrat.pcx", NULL, NULL); // load a font from a file
	// [TODO] Fix this font error: "Shutting down Allegro due to signal #11 // Segmentation fault (core dumped)"
	// 4) Use the font in the text functions:
	//textout_ex(screen, myfont, "Hello, World!", 100, 200, text_col, bg); // draw the text "Hello, World!" at (400, 200) with the color defined above

	
	// if you want to put a text with top-left corner at (400, 200)
	textout_ex(screen, font, "Hello, World!", 400, 200, text_col, bg); // draw the text "Hello, World!" at (400, 200) with the color defined above

	// if you want to put a text with center at (300, 300)
	textout_centre_ex(screen, font, "Hello, World!", 300, 300, text_col, bg); // draw the text "Hello, World!" at (300, 300) with the color defined above

	// using sprintf to convert a number to a string
	char s[100];
	float pi = 3.14159;

	sprintf(s, "pi = %5.2f", pi); // convert the number x to a string "s" printing only 2 decimal places
	textout_ex(screen, font, s, 100, 100, text_col, bg); // draw the text "x = 3.14" at (100, 100) with the color defined above

	// keyboard 
	// after install_keyboard() you can use the following functions:
	keypressed(); // returns true if a key is pressed (this function is non-blocking)

	int a = readkey(); // returns the ASCII code of the key pressed (this function is blocking)
	// now in a is stored:  | high byte: scancode  |  low byte: ASCII code |
	// to access the ASCII code: a & 0xFF
	char a_ascii, a_scancode;
		a_ascii = a & 0xFF;
		a_scancode = a >> 8; // the MSB is 0 if key is pressed, 1 if key is released

		// example function
		get_keycodes(&a_scancode, &a_ascii);

		// to check if a specific key is pressed
		// there is a list of all scancodes in the file allegro/keyboard.h 
		// [KEY_A, KEY_B, ..., KEY_Z, KEY_0, KEY_1, ..., KEY_9, KEY_ESC, KEY_ENTER, ...]
		if (key[KEY_A]) // check if the key 'A' is pressed
		{
			sprintf(s, "A is pressed\n");
			textout_ex(screen, font, s, 120, 120, text_col, bg); // draw the text "A is pressed" at (120, 120) with the color defined above
		}

	// example code for a game loop
	printf("Starting simple loop:\nDrawing random pixel with random color until ESC is pressed...\n");
	int 	sx, sy, scol;

			srand(time(NULL)); // seed the random number generator

			do	{
				sx = rand() % XMAX; // random x coordinate
				sy = rand() % YMAX; // random y coordinate
				scol = makecol(rand() % 256, rand() % 256, rand() % 256); // random color (oppure anche semplicemente: scol = rand() % 16;)
				putpixel(screen, sx, sy, scol); // draw a pixel at (sx, sy) with the random color
			}	while (!key[KEY_ESC]); // exit the loop if the key 'ESC' is pressed

	// example code for text input
	clear_to_color(screen, makecol(255, 255, 255)); // clear the screen making all pixels to white
	printf("Starting text input...\n");
	clear_keybuf(); // clear the keyboard buffer

	char stringa[1000];
	int x_stringa = 100, y_stringa = 100;
	get_string(&stringa, x_stringa, y_stringa, color, bg);

	// take in input floats
	char float_string[100];	// string to store the data input
	float fx;	// float to store the converted string

		textout_ex(screen, font, "Insert a float: ", 10, 30, 3, 0); // prompt

		get_string(&float_string, 34, 30, 3, 0); // get the string from the user
		sscanf(float_string, "%f", &fx); // convert the string to a float
		printf("The float is: %f\n", fx); // print the float

	// Mouse functions
	install_mouse(); // initialize the mouse
		//  this functions initialize the updated of the mouse position and buttons
		//  -) mouse_x: returns the x coordinate of the mouse
		//  -) mouse_y: returns the y coordinate of the mouse
		// 	-) mouse_b: returns the state of the mouse buttons (1: key pressed, 0: key released)
			// 			bit 0: left button, 
			//			bit 1: right button,
			//			bit 2: middle button

	// possible added functions

	// enable_hardware_cursor(); // enable the hardware cursor (faster, BUT COULD NOT WORK ON SOME ALLEGRO FUNCTIONS)
	show_mouse(screen); // show the mouse cursor
	show_mouse(NULL); // hide the mouse cursor

	position_mouse(100, 100); // set the mouse cursor position at (100, 100) [NOT WORK IF HARDWARE CURSOR IS ENABLED]

	// [ACHTUNG!!!] the following functions could not work if the hardware cursor is enabled AHD THE MOUSE IS HIDDEN
	scare_mouse(); // hide mouse pointer prior to drawing
	// example code closed by scare/unscare:
		// putpixel(screen, mouse_x, mouse_y, 14);
		// circlefill(screen, mouse_x, mouse_y, 10, 14);
	unscare_mouse(); // show mouse pointer after drawing 

	// change mouse icon from a bmp file
	BITMAP *mic = load_bitmap("icon/mouse.bmp", NULL); // load a bitmap image
	set_mouse_sprite(mic);
	set_mouse_sprite_focus(0, 0); // set the focus of the mouse sprite
	position_mouse(100, 100);
	show_mouse(screen);
	
	printf("Move mouse while pressing left button to draw until ESC is pressed...\n");
	int x_mouse, y_mouse;
	int col_mouse = 14;

		do {
			if (mouse_b & 1) {
				x_mouse = mouse_x;
				y_mouse = mouse_y;
				putpixel(screen, x_mouse, y_mouse, col_mouse);
			}
		} while (!key[KEY_ESC]);

	

	// How to work with bitmaps
	BITMAP *bmp;
	int 	width = 100;
	int 	height = 100;

	bmp = create_bitmap(width, height); // create a bitmap with resolution 100x100
	// but this bitmap is not visible on the screen and not empty
	clear_bitmap(bmp); // clear the bitmap making all pixels to black
	clear_to_color(bmp, makecol(255, 255, 255)); // clear the bitmap making all pixels to white
	// the functions above needed to be used one or the other

	// to draw on the bitmap
	putpixel(bmp, 10, 10, makecol(255, 0, 0)); // draw a pixel at (10, 10) with the color red
	circle(bmp, 50, 50, 20, makecol(0, 255, 0)); // draw a circle with the center at (50, 50) and radius 20 with the color green
	line(bmp, 10, 10, 90, 90, makecol(0, 0, 255)); // draw a line from (10, 10) to (90, 90) with the color blue

	// once createdm the dimensions of a bitmap can be accessed through its ponter by reading the corrisponding fields of the BITMAP structure:
	int w = bmp->w; // width
	int h = bmp->h; // height

	// to access the dimensions of the "screen" bitmap:
	int w_screen = SCREEN_W;
	int h_screen = SCREEN_H;

	// to draw the bitmap on the screen
	// blit(source, dest, source_x, source_y, dest_x, dest_y, width, height)
	blit(bmp, screen, 0, 0, 200, 200, w, h); // draw the bitmap on the screen at (200, 200)

	/*
			ACHTUNG!!!
			COPYING BITMAPS --> DOUBLE BUFFERING (to avoid flickering)
			1) create a bitmap with the same resolution of the screen
			2) draw on the bitmap
			3) blit the bitmap on the screen
			4) destroy the bitmap
	*/

	// after use the bitmap must be destroyed
	destroy_bitmap(bmp); // it is important to deallocate the memory used by the bitmap 
	// (at the end its done automatically by the allegro_exit() function)

	// load sprites
	BITMAP *sprite;
	int		x_sprite = 300;
	int		y_sprite = 300;

		sprite = load_bitmap("bitmaps/mouse.bmp", NULL); // load a bitmap image

		if (sprite == NULL) {
			printf("Error loading sprite\n");
			exit(1);
		}
		
		blit(sprite, screen, 0, 0, x_sprite, y_sprite, sprite->w, sprite->h); // draw the sprite on the screen at (300, 300)

	
	// draw the sprite on the screen at (300, 300)
	draw_sprite(screen, sprite, x_sprite, y_sprite); 
	// hint: the sprite is drawn with the top-left corner at (x_sprite, y_sprite) and give a more parlante name to the sprite
	// this is similar to blint() but it is optimized for drawing sprites (it uses a masked drawing mode where transparent pixels are not drawn)
	//	in 8-bit (VGA) mode, the color 0 is considered transparent
	//	in 32-bit mode, the color 0x000000 is considered transparent
	//  in truecolor mode, the color makecol(255, 0, 255) (bright pink) is considered transparent


	// how to handles piani di disegno

	scare_mouse(); // hide mouse pointer prior to drawing

	BITMAP *background;
	BITMAP *topo;
	int 	x_topo = 300;
	int		y_topo = 50;

			background = load_bitmap("bitmaps/tom&Jerry.bmp", NULL); // load a bitmap image
			topo = load_bitmap("bitmaps/mouse.bmp", NULL); // load a bitmap image
			if ( (background == NULL) || (topo == NULL) ) {
				printf("Error loading sprite\n");
				exit(1);
			}

			blit(background, screen, 0, 0, 0, 0, background->w, background->h); // draw the background on the screen at (0, 0)
			blit(topo, screen, 0, 0, x_topo, y_topo, topo->w, topo->h); // draw the sprite on the screen at (300, 50)
			draw_sprite(screen, topo, x_topo, y_topo+200); // draw the sprite on the screen at (300, 250)

			// how to transform the drawn sprite
			draw_sprite_h_flip(screen, topo, x_topo, y_topo+300); // draw the sprite on the screen at (300, 450) flipped horizontally
			draw_sprite_v_flip(screen, topo, x_topo-100, y_topo+200); // draw the sprite on the screen at (300, 650) flipped vertically
			draw_sprite_vh_flip(screen, topo, x_topo-100, y_topo+300); // draw the sprite on the screen at (300, 850) flipped both horizontally and vertically

			stretch_sprite(screen, topo, x_topo, y_topo+100, topo->w/2, topo->h); // draw the sprite on the screen at (300, 650) stretched by a factor of 2

			// ROTATION IS STRANGE!!!!!
			/*
				The image is first placed at the specified position in its top-left corner, then rotated around its center by the specified angle.

				Angle == fixed point angle (0-255) where:
					0 is 0 degrees 
					64 is 90 degrees
					128 is 180 degrees
					192 is 270 degrees
					256 is 360 degrees == 0 degrees
			*/
			rotate_sprite(screen, topo, x_topo+100, y_topo+200, itofix(64)); // draw the sprite on the screen at (300, 750) rotated by 64 degrees

			rotate_scaled_sprite(screen, topo, x_topo+100, y_topo+200, itofix(64), ftofix(1.5)); // draw the sprite on the screen at (300, 750) rotated by 64 degrees and scaled by a factor of 1.5

		int pivot_x = x_topo + 100;
		int pivot_y = y_topo + 100;

		int pos_x = x_topo + 100;
		int pos_y = y_topo + 200;

			pivot_scaled_sprite(screen, topo, pos_x, pos_y, pivot_x, pivot_y, itofix(64), ftofix(1.5)); // draw the sprite on the screen at (300, 750) rotated by 64 degrees and scaled by a factor of 1.5 around the point (300, 750)

	unscare_mouse(); // show mouse pointer after drawing 


	destroy_bitmap(sprite); // deallocate the memory used by the sprite


	// End of the program
	printf("Press any key to exit...\n");
	clear_keybuf();
	readkey(); // wait for a key press to close the window
	allegro_exit(); // deallocate graphics data structures

	return 0;
}

// example function for key pressed check
void get_keycodes(char *scan, char *ascii)
{
int 	k;

		k = readkey();	// block until a key is pressed
		*ascii = k;		// get ascii code 
		*scan = k >> 8;	// get scan code
}

// example function for text input
void get_string(char *str, int x, int y, int color, int bg)
{
char 	ascii, scan, s[2];
int		i = 0;

		do {
			get_keycodes(&scan, &ascii);
			if (scan != KEY_ENTER)	{
				s[0] = ascii;	// put ascii in s for echoing
				s[1] = '\0';
				textout_ex(screen, font, s, x, y, color, bg);	// echo the character
				x = x + 8;
				str[i++] = ascii;	// put ascii in str
			}
		}	while (scan != KEY_ENTER);
		str[i] = '\0';	// terminate the string

}