#ifndef KEYBOARD_CONTROL_H
#define KEYBOARD_CONTROL_H

#include "configStruct.h"

#include <allegro5/allegro.h>
#include <allegro5/keyboard.h>
#include <allegro5/events.h>

void send_float(int can_socket, int can_id, float value);
int setup_can_socket(const char *interface_name);
void signal_handler(int sig);

void *keyboard_control_main(Config *config);


#endif
