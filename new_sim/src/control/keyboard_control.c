#include "keyboard_control.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <pthread.h>
#include <termios.h>
#include <fcntl.h>
#include <signal.h>
#include <errno.h>
#include <ctype.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <sys/socket.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <yaml.h>

// VARIABLES
// Limits for steering and throttle
#define MAX_STEERING_ANGLE 30.0f  // degrees
#define MIN_STEERING_ANGLE -30.0f // degrees
#define MAX_THROTTLE       100.0f // arbitrary units
#define MIN_THROTTLE       0.0f   // arbitrary units

// Increment steps
#define STEERING_STEP 1.0f  // degrees per key press
#define THROTTLE_STEP 1.0f  // units per key press

// Current control states
volatile float current_steering = 0.0f; // degrees
volatile float current_throttle = 0.0f; // units



// CAN IDs for steering and throttle (default values)
int STEERING_CAN_ID = 0x300;
int THROTTLE_CAN_ID = 0x301;

// Terminal control variables
struct termios orig_termios;

// Lock for thread-safe updates
volatile int stop_flag = 0;
pthread_mutex_t control_mutex = PTHREAD_MUTEX_INITIALIZER;


// FUNCTIONS



void send_float(int can_socket, int can_id, float value)
{
    struct can_frame frame;
    int             nbytes;

    frame.can_id  = can_id;
    frame.can_dlc = sizeof(float);
    memcpy(frame.data, &value, sizeof(float));

    nbytes = write(can_socket, &frame, sizeof(struct can_frame));
    if (nbytes != sizeof(struct can_frame))
    {
        perror("CAN write error");
    }
}

void *keyboard_listener(void *arg)
{
    int can_socket = *(int *)arg;
    int ch;

    while (!stop_flag)
    {
        if (kbhit())
        {
            ch = getch();
            ch = tolower(ch);
            pthread_mutex_lock(&control_mutex);
            switch (ch)
            {
                case 'a':
                    current_steering -= STEERING_STEP;
                    if (current_steering < MIN_STEERING_ANGLE)
                        current_steering = MIN_STEERING_ANGLE;
                    send_float(can_socket, STEERING_CAN_ID, current_steering);
                    break;
                case 'd':
                    current_steering += STEERING_STEP;
                    if (current_steering > MAX_STEERING_ANGLE)
                        current_steering = MAX_STEERING_ANGLE;
                    send_float(can_socket, STEERING_CAN_ID, current_steering);
                    break;
                case 'w':
                    current_throttle += THROTTLE_STEP;
                    if (current_throttle > MAX_THROTTLE)
                        current_throttle = MAX_THROTTLE;
                    send_float(can_socket, THROTTLE_CAN_ID, current_throttle);
                    break;
                case 's':
                    current_throttle -= THROTTLE_STEP;
                    if (current_throttle < MIN_THROTTLE)
                        current_throttle = MIN_THROTTLE;
                    send_float(can_socket, THROTTLE_CAN_ID, current_throttle);
                    break;
                case 'q':
                    printf("Quitting control script...\n");
                    stop_flag = 1;
                    break;
                default:
                    break;
            }
            pthread_mutex_unlock(&control_mutex);
        }
        usleep(10000); // Sleep for 10 ms
    }

    return NULL;
}

// Terminal control functions

void reset_terminal_mode()
{
    tcsetattr(0, TCSANOW, &orig_termios);
}

void set_conio_terminal_mode()
{
    struct termios new_termios;

    // Take two copies - one for now, one for later
    tcgetattr(0, &orig_termios);
    memcpy(&new_termios, &orig_termios, sizeof(new_termios));

    // Disable canonical mode, and set buffer size to 1 byte
    new_termios.c_lflag &= ~(ICANON | ECHO);
    new_termios.c_cc[VMIN]  = 0;
    new_termios.c_cc[VTIME] = 0;
    tcsetattr(0, TCSANOW, &new_termios);
}

int kbhit()
{
    int bytes_available;
    ioctl(0, FIONREAD, &bytes_available);
    return bytes_available;
}

int getch()
{
    unsigned char ch;
    if (read(0, &ch, 1) < 0)
    {
        return -1;
    }
    return ch;
}

void signal_handler(int sig)
{
    stop_flag = 1;
}

// Function to load configuration from a YAML file
void load_config(const char *config_file)
{
    FILE          *file;
    yaml_parser_t  parser;
    yaml_event_t   event;
    int            done       = 0;
    int            in_can_ids = 0;
    char           key[256];
    char           value[256];
    int            state      = 0; // 0: Expect key, 1: Expect value

    file = fopen(config_file, "r");
    if (!file)
    {
        fprintf(stderr, "Failed to open config file: %s\n", config_file);
        return;
    }

    if (!yaml_parser_initialize(&parser))
    {
        fprintf(stderr, "Failed to initialize YAML parser\n");
        fclose(file);
        return;
    }

    yaml_parser_set_input_file(&parser, file);

    while (!done)
    {
        if (!yaml_parser_parse(&parser, &event))
        {
            fprintf(stderr, "YAML parsing error\n");
            break;
        }

        switch (event.type)
        {
            case YAML_MAPPING_START_EVENT:
                if (state == 1 && strcmp(key, "CAN_IDS") == 0)
                {
                    in_can_ids = 1;
                }
                break;
            case YAML_MAPPING_END_EVENT:
                if (in_can_ids)
                {
                    in_can_ids = 0;
                }
                break;
            case YAML_SCALAR_EVENT:
                if (state == 0)
                {
                    strncpy(key, (char *)event.data.scalar.value, sizeof(key) - 1);
                    key[sizeof(key) - 1] = '\0';
                    state = 1;
                }
                else if (state == 1)
                {
                    strncpy(value, (char *)event.data.scalar.value, sizeof(value) - 1);
                    value[sizeof(value) - 1] = '\0';
                    state = 0;

                    if (in_can_ids)
                    {
                        if (strcmp(key, "STEERING_CAN_ID") == 0)
                        {
                            STEERING_CAN_ID = (int)strtol(value, NULL, 0);
                        }
                        else if (strcmp(key, "THROTTLE_CAN_ID") == 0)
                        {
                            THROTTLE_CAN_ID = (int)strtol(value, NULL, 0);
                        }
                        // TODO ADD ALL THE NEW ACTUATION VARIABLES
                    }
                }
                break;
            case YAML_STREAM_END_EVENT:
                done = 1;
                break;
            default:
                break;
        }
        yaml_event_delete(&event);
    }

    yaml_parser_delete(&parser);
    fclose(file);
}

// Function to set up CAN socket
int setup_can_socket(const char *interface_name)
{
    int                 can_socket;
    struct sockaddr_can addr;
    struct ifreq        ifr;

    if ((can_socket = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0)
    {
        perror("Error while opening CAN socket");
        return -1;
    }

    strcpy(ifr.ifr_name, interface_name);
    if (ioctl(can_socket, SIOCGIFINDEX, &ifr) < 0)
    {
        perror("Error getting interface index");
        close(can_socket);
        return -1;
    }

    addr.can_family  = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (bind(can_socket, (struct sockaddr *)&addr, sizeof(addr)) < 0)
    {
        perror("Error in socket bind");
        close(can_socket);
        return -1;
    }

    return can_socket;
}



void *keyboard_control_main(const char *config_file)
{
    char **argv = (char **)arg;
    int    argc = 0;
    int    i;

    // Count the number of arguments
    while (argv[argc] != NULL)
    {
        argc++;
    }

    int      can_socket;
    pthread_t keyboard_thread;

    // Load configuration
    load_config(config_file);

    // Initialize CAN socket
    can_socket = setup_can_socket("vcan0");
    if (can_socket < 0)
    {
        return NULL;
    }

    // Set up terminal in non-blocking mode
    set_conio_terminal_mode();

    // Handle SIGINT to reset terminal settings
    signal(SIGINT, signal_handler);

    printf("Control Script Started.\n");
    printf("Controls:\n");
    printf("  A: Steer Left\n");
    printf("  D: Steer Right\n");
    printf("  W: Throttle Up\n");
    printf("  S: Throttle Down\n");
    printf("  Q: Quit\n");

    // Start the keyboard listener thread
    if (pthread_create(&keyboard_thread, NULL, keyboard_listener, (void *)&can_socket) != 0)
    {
        perror("Failed to create keyboard listener thread");
        reset_terminal_mode();
        close(can_socket);
        return NULL;
    }

    // Main loop
    while (!stop_flag)
    {
        usleep(100000); // Sleep for 100 ms
    }

    // Wait for keyboard thread to finish
    pthread_join(keyboard_thread, NULL);

    // Reset steering and throttle to neutral before exiting
    send_float(can_socket, STEERING_CAN_ID, 0.0f);
    send_float(can_socket, THROTTLE_CAN_ID, 0.0f);

    // Cleanup
    reset_terminal_mode();
    close(can_socket);

    printf("Control Script Terminated.\n");

    return NULL;
}
