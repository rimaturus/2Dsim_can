#ifndef CONTROL_H
#define CONTROL_H

void send_float(int can_socket, int can_id, float value);
void *keyboard_listener(void *arg);
void reset_terminal_mode();
void set_conio_terminal_mode();
int kbhit();
int getch();
void signal_handler(int sig);
void load_config(const char *config_file);
int setup_can_socket(const char *interface_name);

void *keyboard_control_main(const char *config_file);


#endif
