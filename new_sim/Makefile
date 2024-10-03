CC = gcc
CFLAGS = -Wall -O2 -I$(VISUALIZATION_DIR) -I$(CONTROL_DIR) -I./$(INCLUDE_DIR)
LDFLAGS = -lallegro -lallegro_font -lallegro_ttf -lallegro_primitives -lallegro -lpthread -lyaml -lm

# Directories
SRC_DIR = src
BUILD_DIR = build
VISUALIZATION_DIR = $(SRC_DIR)/visualization
CONTROL_DIR = $(SRC_DIR)/control
INCLUDE_DIR = include

# Source files
MAIN_SRC = 2D_sim.c
VIS_SRC = $(VISUALIZATION_DIR)/visualization.c
CONTROL_SRC = $(CONTROL_DIR)/keyboard_control.c
CONFIG_SRC = $(INCLUDE_DIR)/loadConfig.c

# Object files
OBJS = $(BUILD_DIR)/2D_sim.o $(BUILD_DIR)/visualization.o $(BUILD_DIR)/keyboard_control.o $(BUILD_DIR)/loadConfig.o

# Executable name
TARGET = 2D_sim

# Default target
all: $(BUILD_DIR) $(TARGET)

$(BUILD_DIR):
	mkdir -p $(BUILD_DIR)

# Build 
$(BUILD_DIR)/2D_sim.o: $(MAIN_SRC) $(CONTROL_DIR)/keyboard_control.h $(VISUALIZATION_DIR)/visualization.h $(INCLUDE_DIR)/configStruct.h
	$(CC) $(CFLAGS) -c $< -o $@

$(BUILD_DIR)/visualization.o: $(VIS_SRC) $(VISUALIZATION_DIR)/visualization.h $(INCLUDE_DIR)/configStruct.h
	$(CC) $(CFLAGS) -c $< -o $@ 

$(BUILD_DIR)/keyboard_control.o: $(CONTROL_SRC) $(CONTROL_DIR)/keyboard_control.h $(INCLUDE_DIR)/configStruct.h
	$(CC) $(CFLAGS) -c $< -o $@

$(BUILD_DIR)/loadConfig.o: $(CONFIG_SRC) $(INCLUDE_DIR)/configStruct.h
	$(CC) $(CFLAGS) -c $< -o $@

# Link object files into executable
$(TARGET): $(OBJS)
	$(CC) -o $@ $^ $(LDFLAGS)

# Clean up build files
clean:
	rm -rf $(BUILD_DIR) $(TARGET)

# Phony targets
.PHONY: all clean
