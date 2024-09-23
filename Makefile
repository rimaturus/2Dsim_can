CXX = g++
CXXFLAGS = -Wall -Wextra -O1

YAML_LIB = -lyaml-cpp
PTHREAD_LIB = -lpthread
FS_LIB = -lstdc++fs
ALLEGRO_LIBS = -lallegro -lallegro_main -lallegro_font -lallegro_ttf -lallegro_primitives

all: perception pid_control visualization

perception: perception.cpp
	$(CXX) $(CXXFLAGS) perception.cpp -o perception $(YAML_LIB)

pid_control: pid_control.cpp
	$(CXX) $(CXXFLAGS) pid_control.cpp -o pid_control $(YAML_LIB) $(PTHREAD_LIB) $(FS_LIB)

visualization: visualization.cpp
	$(CXX) $(CXXFLAGS) visualization.cpp -o visualization $(ALLEGRO_LIBS) $(YAML_LIB) -pthread

clean:
	rm -f perception pid_control visualization
