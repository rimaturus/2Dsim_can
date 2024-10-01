# Makefile for 2D_sim project

# Compiler and flags
CC = gcc
CFLAGS = -Wall -O2 -I./src/visualization -I./include
LDFLAGS = -lallegro -lallegro_font -lallegro_ttf -lallegro_primitives -lpthread -lyaml -lm

# Directories
SRC_DIR = src
BUILD_DIR = build
VISUALIZATION_DIR = $(SRC_DIR)/visualization
#FUTURE_DIR = $(SRC_DIR)/future_module

# Source files
MAIN_SRC = 2D_sim.c
VIS_SRC = $(VISUALIZATION_DIR)/visualization.c
#FUTURE_SRC = $(FUTURE_DIR)/future_module.c

# Object files
OBJS = $(BUILD_DIR)/2D_sim.o $(BUILD_DIR)/visualization.o

# Executable name
TARGET = 2D_sim

# Default target
all: $(BUILD_DIR) $(TARGET)

# Create build directory
$(BUILD_DIR):
	mkdir -p $(BUILD_DIR)

# Compile main program
$(BUILD_DIR)/2D_sim.o: $(MAIN_SRC)
	$(CC) $(CFLAGS) -c $< -o $@

# Compile visualization module
$(BUILD_DIR)/visualization.o: $(VIS_SRC) $(VISUALIZATION_DIR)/visualization.h
	$(CC) $(CFLAGS) -c $< -o $@

# Compile future module (uncomment when you add it)
# $(BUILD_DIR)/future_module.o: $(FUTURE_SRC) $(FUTURE_DIR)/future_module.h
# 	$(CC) $(CFLAGS) -c $< -o $@

# Link object files into executable
$(TARGET): $(OBJS)
	$(CC) -o $@ $^ $(LDFLAGS)

# Clean up build files
clean:
	rm -rf $(BUILD_DIR) $(TARGET)

# Phony targets
.PHONY: all clean
