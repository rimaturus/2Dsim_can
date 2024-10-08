#include <stdio.h>
#include <stdlib.h>
#include <unistd.h> // For sleep function
#include "params.h"

// Function to simulate control inputs (for demonstration purposes)
void compute_controls(double *steering, double *wheel_torque, Pose current_pose) {
    // Simple control logic:
    // Alternate steering angle between -0.3 and 0.3 radians
    // Alternate torque between throttle and brake

    static int toggle = 0;
    if (toggle == 0) {
        *steering = 0.3;
        *wheel_torque = 200.0; // Throttle
    }
    else {
        *steering = -0.3;
        *wheel_torque = -200.0; // Brake
    }
    toggle = 1 - toggle;
}

int main() {
    // Initialize Vehicle Model
    VehicleModel vehicle;
    if (!init_vehicle_model("config.yaml", &vehicle)) {
        fprintf(stderr, "Failed to initialize vehicle model.\n");
        return EXIT_FAILURE;
    }

    // Simulation parameters
    int simulation_steps = 1000; // Number of simulation iterations
    double steering_input = 0.0;
    double wheel_torque_input = 0.0;
    Acceleration accel;
    Pose new_pose;

    printf("Starting simulation...\n");

    for(int step = 0; step < simulation_steps; step++) {
        // Compute control inputs based on current pose
        compute_controls(&steering_input, &wheel_torque_input, vehicle.pose);

        // Call IMU function to get accelerations and update pose
        if (!IMU(&vehicle, steering_input, wheel_torque_input, &accel, &new_pose)) {
            fprintf(stderr, "IMU computation failed at step %d.\n", step);
            return EXIT_FAILURE;
        }

        // Print the results
        printf("Step %d:\n", step+1);
        printf("  Control Inputs:\n");
        printf("    Steering (rad): %.4f\n", steering_input);
        printf("    Wheel Torque (Nm): %.2f\n", wheel_torque_input);
        printf("  Accelerations:\n");
        printf("    ax: %.4f m/s²\n", accel.ax);
        printf("    ay: %.4f m/s²\n", accel.ay);
        printf("    Yaw Rate: %.4f rad/s\n", accel.yaw_rate);
        printf("  Pose:\n");
        printf("    x: %.4f m\n", new_pose.x);
        printf("    y: %.4f m\n", new_pose.y);
        printf("    yaw: %.4f rad\n", new_pose.yaw);
        printf("---------------------------------------\n");

        // Sleep for a short duration to simulate real-time (optional)
        // usleep(10000); // Sleep for 10ms
    }

    printf("Simulation completed.\n");
    return EXIT_SUCCESS;
}
