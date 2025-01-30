#include "can_utils.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <pthread.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <sys/socket.h>
#include <stdint.h>

#define MAX_CAN_IDS 100 // Maximum number of CAN IDs to filter

// Structure to hold CAN ID and associated value
typedef struct {
    uint32_t can_id;
    float value;
    pthread_mutex_t mutex;
    pthread_cond_t cond;
    int new_data;
    int valid;
} can_data_t;

// Array to store CAN IDs and values
static can_data_t can_data_array[MAX_CAN_IDS];
static int can_data_count = 0;
static pthread_mutex_t can_data_array_mutex = PTHREAD_MUTEX_INITIALIZER;

// Structure for cansend thread arguments
struct cansend_args {
    int can_id;
    float data;
};

// Function to send a float data over CAN with a given ID
void cansend(int can_id, float data) {
    int s; // SocketCAN socket
    struct sockaddr_can addr;
    struct can_frame frame;
    struct ifreq ifr;

    // Open SocketCAN socket
    if ((s = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
        perror("Error while opening socket");
        return;
    }

    // Specify the CAN interface to use ("vcan0")
    strcpy(ifr.ifr_name, "vcan0");
    ioctl(s, SIOCGIFINDEX, &ifr);

    // Bind the socket to the CAN interface
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (bind(s, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        perror("Error in socket bind");
        close(s);
        return;
    }

    // Prepare the CAN frame
    frame.can_id = can_id; // Set CAN ID
    frame.can_dlc = sizeof(float); // Data length code

    // Copy the float data into the CAN frame data (little endian)
    memcpy(frame.data, &data, sizeof(float));

    // Send the CAN frame
    if (write(s, &frame, sizeof(struct can_frame)) != sizeof(struct can_frame)) {
        perror("Error in CAN write");
    }

    close(s);
}

// Function to receive CAN messages and store float data
void canreceive() {
    int s; // SocketCAN socket
    struct sockaddr_can addr;
    struct can_frame frame;
    struct ifreq ifr;
    int nbytes;

    // Open SocketCAN socket
    if ((s = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
        perror("Error while opening socket");
        return;
    }

    // Specify the CAN interface to use ("vcan0")
    strcpy(ifr.ifr_name, "vcan0");
    ioctl(s, SIOCGIFINDEX, &ifr);

    // Bind the socket to the CAN interface
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (bind(s, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        perror("Error in socket bind");
        close(s);
        return;
    }

    while (1) {
        // Read a CAN frame
        nbytes = read(s, &frame, sizeof(struct can_frame));
        if (nbytes < 0) {
            perror("Error in CAN read");
            break;
        }

        if (nbytes < sizeof(struct can_frame)) {
            fprintf(stderr, "Incomplete CAN frame\n");
            continue;
        }

        // Lock the array mutex to access can_data_array
        pthread_mutex_lock(&can_data_array_mutex);

        // Check if the CAN ID is in the filter list
        for (int i = 0; i < can_data_count; i++) {
            if (can_data_array[i].can_id == frame.can_id && can_data_array[i].valid) {
                // Extract the float data (assuming little endian)
                float data;
                memcpy(&data, frame.data, sizeof(float));

                // Lock the individual CAN ID mutex
                pthread_mutex_lock(&can_data_array[i].mutex);

                // Store the value and set new_data flag
                can_data_array[i].value = data;
                can_data_array[i].new_data = 1;

                // Signal any thread waiting on this CAN ID
                pthread_cond_signal(&can_data_array[i].cond);

                // Unlock the individual CAN ID mutex
                pthread_mutex_unlock(&can_data_array[i].mutex);

                // Optional: Output the CAN ID and data
                // printf("Received CAN ID: 0x%X Data: %f\n", frame.can_id, data);

                break; // Exit the for loop
            }
        }

        // Unlock the array mutex
        pthread_mutex_unlock(&can_data_array_mutex);
    }

    close(s);
}

// Thread function for sending CAN data
void* cansend_thread(void* arg) {
    struct cansend_args* args = (struct cansend_args*)arg;

    // Call the cansend function with the provided arguments
    cansend(args->can_id, args->data);

    free(arg); // Free the allocated memory
    return NULL;
}

// Thread function for receiving CAN data
void* canreceive_thread(void* arg) {
    (void)arg; // Unused parameter
    // Call the canreceive function
    canreceive();
    return NULL;
}

// Function to start the cansend thread
void cansend_start(int can_id, float data) {
    pthread_t thread_id;

    struct cansend_args* args = malloc(sizeof(struct cansend_args));
    if (args == NULL) {
        perror("Failed to allocate memory for cansend arguments");
        return;
    }
    args->can_id = can_id;
    args->data = data;

    if (pthread_create(&thread_id, NULL, cansend_thread, args) != 0) {
        perror("Failed to create cansend thread");
        free(args);
        return;
    }

    // Detach the thread so it cleans up after itself
    pthread_detach(thread_id);
}

// Function to start the canreceive thread
void canreceive_start() {
    pthread_t thread_id;
    if (pthread_create(&thread_id, NULL, canreceive_thread, NULL) != 0) {
        perror("Failed to create canreceive thread");
        return;
    }

    // Detach the thread so it cleans up after itself
    pthread_detach(thread_id);
}

// Function to add a CAN ID to the filter list
int can_add_filter(uint32_t can_id) {
    pthread_mutex_lock(&can_data_array_mutex);

    // Check if the CAN ID is already in the array
    for (int i = 0; i < can_data_count; i++) {
        if (can_data_array[i].can_id == can_id && can_data_array[i].valid) {
            pthread_mutex_unlock(&can_data_array_mutex);
            return 0; // Already exists
        }
    }

    if (can_data_count >= MAX_CAN_IDS) {
        pthread_mutex_unlock(&can_data_array_mutex);
        return -1; // No space left
    }

    // Initialize the new can_data_t entry
    can_data_array[can_data_count].can_id = can_id;
    can_data_array[can_data_count].value = 0.0f;
    can_data_array[can_data_count].new_data = 0;
    can_data_array[can_data_count].valid = 1;
    pthread_mutex_init(&can_data_array[can_data_count].mutex, NULL);
    pthread_cond_init(&can_data_array[can_data_count].cond, NULL);

    can_data_count++;

    pthread_mutex_unlock(&can_data_array_mutex);
    return 0;
}

// Function to get the latest value for a given CAN ID
float can_get_value(uint32_t can_id) {
    can_data_t* data_entry = NULL;

    // Lock the array mutex to search for the CAN ID
    pthread_mutex_lock(&can_data_array_mutex);

    // Find the CAN ID in the array
    for (int i = 0; i < can_data_count; i++) {
        if (can_data_array[i].can_id == can_id && can_data_array[i].valid) {
            data_entry = &can_data_array[i];
            break;
        }
    }

    pthread_mutex_unlock(&can_data_array_mutex);

    if (data_entry == NULL) {
        fprintf(stderr, "CAN ID 0x%X not found. Please add it using can_add_filter().\n", can_id);
        exit(EXIT_FAILURE); // Or handle the error as needed
    }

    // Lock the individual CAN ID mutex
    pthread_mutex_lock(&data_entry->mutex);

    // Wait until new data is available
    while (!data_entry->new_data) {
        pthread_cond_wait(&data_entry->cond, &data_entry->mutex);
    }

    // Retrieve the value and reset new_data flag
    float value = data_entry->value;
    data_entry->new_data = 0;

    // Unlock the individual CAN ID mutex
    pthread_mutex_unlock(&data_entry->mutex);

    return value;
}
