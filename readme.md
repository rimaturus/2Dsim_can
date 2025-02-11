# 2D FSAE Simulation

## Project Overview

This project is a 2D simulation of a Formula SAE (FSAE) vehicle, designed to model vehicle dynamics, perception, trajectory planning, and control systems. It provides a platform for testing and visualizing autonomous driving algorithms in a simplified environment.

## Key Features

-   **Vehicle Dynamics:** Simulates basic vehicle movement based on pedal and steering inputs.
-   **LiDAR-based Perception:** Implements cone detection using simulated LiDAR measurements and a circle Hough transform.
-   **Trajectory Planning:** Generates a driving trajectory by connecting detected cones.
-   **Autonomous Control:** Includes an autonomous control system to follow the planned trajectory.
-   **Manual Control:** Allows keyboard control of the vehicle.
-   **Visualization:** Uses the Allegro library to provide a 2D graphical representation of the simulation, including the car, track, cones, and planned trajectory.
-   **Periodic Tasks:** Utilizes a periodic task scheduler for managing different simulation components.

## Project Structure

The project is organized into several modules:

-   **`src/main.c`:** Initializes the simulation, creates tasks, and manages the main loop.
-   **`src/vehicle.c`:** Implements the vehicle dynamics model.
-   **`src/perception.c`:** Handles LiDAR data processing and cone detection.
-   **`src/trajectory.c`:** Plans the driving trajectory based on detected cones.
-   **`src/control.c`:** Implements control algorithms for both manual and autonomous driving.
-   **`src/display.c`:** Manages the graphical display of the simulation.
-   **`src/tasks.c`:** Defines the periodic tasks for perception, trajectory planning, control, and display.
-   **`src/utilities.c`:** Provides utility functions for cone management and runtime measurement.
-   **`src/globals.c`:** Defines global variables and constants used throughout the simulation.
-   **`src/ptask.c`:** Implements the periodic task scheduler.
-   **`include/*.h`:** Header files declaring the interfaces for each module.

## Global Call Graph
![Global Call Graph](/home/edo/unipi/1anno/istr/2Dsim_can/graphs/global_graph.png)


## Task Descriptions
 - **Perception Task**: Acquires LiDAR measurements, detects cones, and updates the map.
 - **Trajectory Task**: Plans the trajectory based on the detected cones.
 - **Control Task**: Controls the vehicle either manually or autonomously.
 - **Display Task**: Updates the graphical display of the simulation.

## Building and Running the Simulation
 1) Ensure you have Allegro 4 installed *(On Ubuntu/Debian:* `sudo apt-get install liballegro4.2 liballegro4.2-dev`*)*.

 2) Other dependencies for this project: 
    - *libyaml*: `git clone https://github.com/yaml/libyaml.git` 

 3) Use the provided Makefile to build the project.

 4) Run the executable `make`.