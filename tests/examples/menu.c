#include <allegro.h>
#include <stdio.h>

// Structure for button
typedef struct {
	int x, y;
	int width, height;
	const char* text;
	int* value;
} Button;

// Function to check if mouse is over button
int is_mouse_over_button(Button* btn, int mouse_x, int mouse_y) {
	return (mouse_x >= btn->x && mouse_x <= btn->x + btn->width &&
			mouse_y >= btn->y && mouse_y <= btn->y + btn->height);
}

// Function to draw button
void draw_button(Button* btn, BITMAP* buffer) {
	rect(buffer, btn->x, btn->y, btn->x + btn->width, btn->y + btn->height, makecol(255, 255, 255));
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

int main() {
	// Initialize Allegro
	allegro_init();
	install_keyboard();
	install_mouse();
	set_color_depth(32);
	set_gfx_mode(GFX_AUTODETECT_WINDOWED, 800, 600, 0, 0);

	// Mouse cursor settings
	enable_hardware_cursor();
	set_mouse_range(0, 0, SCREEN_W-1, SCREEN_H-1);
	
	// Add cursor visibility flag
	int cursor_visible = 1;
	show_mouse(cursor_visible ? screen : NULL);

	// Create buffer
	BITMAP* buffer = create_bitmap(SCREEN_W, SCREEN_H);

	// Example variables to modify
	int var1 = 0;
	int var2 = 10;

	// Create buttons
	Button buttons[] = {
		{100, 100, 150, 30, "Increase Var1", &var1},
		{100, 150, 150, 30, "Increase Var2", &var2}
	};
	int num_buttons = 2;

	// Main loop
	while (!key[KEY_ESC]) {
		// Clear buffer
		clear_to_color(buffer, makecol(0, 0, 0));

		 // Toggle cursor visibility with 'C' key
        if (key[KEY_C]) {
            cursor_visible = !cursor_visible;
            show_mouse(cursor_visible ? screen : NULL);
            rest(100); // Debounce key press
        }

		// Handle mouse input
		if (mouse_b & 1) { // Left click
			for (int i = 0; i < num_buttons; i++) {
				if (is_mouse_over_button(&buttons[i], mouse_x, mouse_y)) {
					(*buttons[i].value)++;
					rest(100); // Debounce
				}
			}
		}

		// Draw buttons
		for (int i = 0; i < num_buttons; i++) {
			draw_button(&buttons[i], buffer);
		}

		// Draw buffer to screen
		blit(buffer, screen, 0, 0, 0, 0, SCREEN_W, SCREEN_H);
		rest(10);
	}

	// Cleanup
	show_mouse(NULL);  // Hide mouse cursor
	destroy_bitmap(buffer);
	allegro_exit();
	return 0;
}
END_OF_MAIN()