CXX = g++
CXXFLAGS = -Wall -Wextra -O1

# Libraries
YAML_LIB = -lyaml
PTHREAD_LIB = -lpthread
FS_LIB = -lstdc++fs
ALLEGRO_LIBS = -lallegro -lallegro_main -lallegro_font -lallegro_ttf -lallegro_primitives

# Source files and output binaries
PERCEPTION_SRC = perception_/perception.cpp
PID_CONTROL_SRC = control_/pid/pid_control.cpp
VISUALIZATION_SRC = visualization_/visualization.c

# Target binaries
PERCEPTION_BIN = perception
PID_CONTROL_BIN = pid_control
VISUALIZATION_BIN = visualization

all: $(PERCEPTION_BIN) $(PID_CONTROL_BIN) $(VISUALIZATION_BIN)

# Build rules
$(PERCEPTION_BIN): $(PERCEPTION_SRC)
	$(CXX) $(CXXFLAGS) $< -o $@ $(YAML_LIB)

$(PID_CONTROL_BIN): $(PID_CONTROL_SRC)
	$(CXX) $(CXXFLAGS) $< -o $@ $(YAML_LIB) $(PTHREAD_LIB) $(FS_LIB)

$(VISUALIZATION_BIN): $(VISUALIZATION_SRC)
	$(CXX) $(CXXFLAGS) $< -o $@ $(ALLEGRO_LIBS) $(YAML_LIB) -pthread

clean:
	rm -f $(PERCEPTION_BIN) $(PID_CONTROL_BIN) $(VISUALIZATION_BIN)

