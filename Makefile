TARGET    = 2D_sim
CC        = gcc
CFLAGS    = -g -O3 -Wall #-Wextra -Werror -std=c99
LIBS      = `allegro-config --libs` -lyaml -lm -lpthread

SRCS      = $(wildcard src/*.c)
BUILD_DIR = build
OBJS      = $(SRCS:src/%.c=$(BUILD_DIR)/%.o)

INC_DIR   = include
CFLAGS   += -I$(INC_DIR)

# Default target
all: $(TARGET)

# Link the target using object files from the build directory
$(TARGET): $(OBJS)
	$(CC) $(CFLAGS) -o $@ $^ $(LIBS)

# Compile source files to object files in the build directory.
# The order-only prerequisite ensures that BUILD_DIR exists before compilation.
$(BUILD_DIR)/%.o: src/%.c | $(BUILD_DIR)
	$(CC) $(CFLAGS) -c $< -o $@ $(LIBS)

# Create the build directory if it doesn't exist.
$(BUILD_DIR):
	mkdir -p $(BUILD_DIR)

# Clean up built files.
clean:
	rm -f $(OBJS) $(TARGET)