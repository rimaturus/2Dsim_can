#include "keyboard_control.h"
#include "configStruct.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <pthread.h>

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

#include <allegro5/allegro.h>
#include <allegro5/keyboard.h>
#include <allegro5/events.h>

// VARIABLES

// Current control states
volatile float current_steering = 0.0f; // [degrees]
volatile float current_throttle = 0.0f; // [units]

// Lock for thread-safe updates
volatile int stop_flag = 0;
pthread_mutex_t control_mutex = PTHREAD_MUTEX_INITIALIZER;


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

void signal_handler(int sig)
{
    stop_flag = 1;
}

void *keyboard_control_main(Config *config)
{
    int      can_socket;

    // Initialize CAN socket
    can_socket = setup_can_socket("vcan0");
    if (can_socket < 0)
    {
        fprintf(stderr, "VCAN interface not configured\n");
        fprintf(stderr, "Please run 'bash setup_vcan.sh' before this\n");
        return NULL;
    }

    // Initialize Allegro 5
    if (!al_init())
    {
        fprintf(stderr, "Failed to initialize Allegro.\n");
        close(can_socket);
        return NULL;
    }

    if (!al_install_keyboard())
    {
        fprintf(stderr, "Failed to install keyboard.\n");
        al_uninstall_system();
        close(can_socket);
        return NULL;
    }

    // Create event queue
    ALLEGRO_EVENT_QUEUE *event_queue = al_create_event_queue();
    if (!event_queue)
    {
        fprintf(stderr, "Failed to create event queue.\n");
        al_uninstall_keyboard();
        al_uninstall_system();
        close(can_socket);
        return NULL;
    }

    // Register keyboard events
    al_register_event_source(event_queue, al_get_keyboard_event_source());

    // Handle SIGINT to reset terminal settings
    signal(SIGINT, signal_handler);

    printf("Control Script Started.\n");
    printf("Controls:\n");
    printf("  A: Steer Left\n");
    printf("  D: Steer Right\n");
    printf("  W: Throttle Up\n");
    printf("  S: Throttle Down\n");
    printf("  Q: Quit\n");

    // Main loop
    while (!stop_flag)
    {
        ALLEGRO_EVENT ev;
        while (al_get_next_event(event_queue, &ev))
        {
            if (ev.type == ALLEGRO_EVENT_KEY_DOWN)
            {
                printf("Key Pressed\n");
                
                int keycode = ev.keyboard.keycode;
                pthread_mutex_lock(&control_mutex);
                switch (keycode)
                {
                    case ALLEGRO_KEY_A:
                        current_steering -= config->actuation_params.steer_step;
                        if (current_steering < config->actuation_params.min_steering)
                            current_steering = config->actuation_params.min_steering;
                        send_float(can_socket, config->ids.steer_id, current_steering);
                        break;
                    case ALLEGRO_KEY_D:
                        current_steering += config->actuation_params.steer_step;
                        if (current_steering > config->actuation_params.max_steering)
                            current_steering = config->actuation_params.max_steering;
                        send_float(can_socket, config->ids.steer_id, current_steering);
                        break;
                    case ALLEGRO_KEY_W:
                        current_throttle += config->actuation_params.pps_step;
                        if (current_throttle > config->actuation_params.max_throttle)
                            current_throttle = config->actuation_params.max_throttle;
                        send_float(can_socket, config->ids.throttle_id, current_throttle);
                        break;
                    case ALLEGRO_KEY_S:
                        current_throttle -= config->actuation_params.pps_step;
                        if (current_throttle < 0.0f)
                            current_throttle = 0.0f;
                        send_float(can_socket, config->ids.throttle_id, current_throttle);
                        break;
                    case ALLEGRO_KEY_Q:
                        printf("Quitting control script...\n");
                        stop_flag = 1;
                        break;
                    
                    default:
                        break;
                }
                pthread_mutex_unlock(&control_mutex);
            }
        }
        usleep(10000); // Sleep for 10 ms
    }

    // Reset steering and throttle to neutral before exiting
    send_float(can_socket, config->ids.steer_id, 0.0f);
    send_float(can_socket, config->ids.throttle_id, 0.0f);

    // Cleanup
    al_destroy_event_queue(event_queue);
    al_uninstall_keyboard();
    al_uninstall_system();
    close(can_socket);

    printf("Control Script Terminated.\n");

    return NULL;
}