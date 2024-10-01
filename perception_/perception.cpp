#include "perception.h"
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <cstring>
#include <fcntl.h>
#include <errno.h>
#include <random>
#include <thread>

// Constants
float	PIXELS_PER_METER = 10.0f;  // Default value, will be loaded from config

// CAN IDs
int	CAR_X_CAN_ID     = 0x200;
int	CAR_Y_CAN_ID     = 0x201;
int	CAR_ANGLE_CAN_ID = 0x202;

// Noise parameters
double	RANGE_NOISE_STD_DEV   = 0.1;  // Default value, will be loaded from config
double	BEARING_NOISE_STD_DEV = 1.0;  // Default value, will be loaded from config
double	DETECTION_RANGE       = 5.0;  // Default detection range in meters

// Function prototypes
void			loadConfig(const std::string &filename);
std::vector<Cone>	loadCones(const std::string &filename);
int			setupCANSocket(const std::string &interface_name);
void			sendCANData(int send_can_socket, std::queue<CANData> &dataQueue, std::mutex &queueMutex, std::condition_variable &cv);
void			processCANFrame(const struct can_frame &frame, float &car_x, float &car_y, float &car_angle, bool &has_car_x, bool &has_car_y, bool &has_car_angle);
void			computeAndSendConeData(const std::vector<Cone> &cones, float car_x_m, float car_y_m, float car_angle, std::queue<CANData> &dataQueue, std::mutex &queueMutex, std::condition_variable &cv);
int			main();

// Function definitions

void	loadConfig(const std::string &filename)
{
	YAML::Node	config;
	try
	{
		config = YAML::LoadFile(filename);

		// Load PIXELS_PER_METER
		if (config["PIXELS_PER_METER"])
		{
			PIXELS_PER_METER = config["PIXELS_PER_METER"].as<float>();
		}

		// Load noise parameters and detection range
		if (config["perception"])
		{
			RANGE_NOISE_STD_DEV   = config["perception"]["range_noise_std_dev"].as<double>();
			BEARING_NOISE_STD_DEV = config["perception"]["bearing_noise_std_dev"].as<double>();
			DETECTION_RANGE       = config["perception"]["detection_range"].as<double>();
		}

		// Load CAN IDs
		if (config["CAN_IDS"])
		{
			CAR_X_CAN_ID     = config["CAN_IDS"]["CAR_X_CAN_ID"].as<int>();
			CAR_Y_CAN_ID     = config["CAN_IDS"]["CAR_Y_CAN_ID"].as<int>();
			CAR_ANGLE_CAN_ID = config["CAN_IDS"]["CAR_ANGLE_CAN_ID"].as<int>();
		}

		std::cout << "Configuration loaded from " << filename << std::endl;
	}
	catch (const std::exception &e)
	{
		std::cerr << "Error loading configuration from " << filename << ": " << e.what() << std::endl;
		std::cerr << "Using default configuration values." << std::endl;
	}
}

std::vector<Cone>	loadCones(const std::string &filename)
{
	std::vector<Cone>	cones;
	try
	{
		YAML::Node	conesNode = YAML::LoadFile(filename)["cones"];
		if (!conesNode)
		{
			std::cerr << "Error: 'cones' key not found in " << filename << std::endl;
			return cones;
		}
		for (const auto &node : conesNode)
		{
			Cone	cone;
			cone.x_pixels = node["x"].as<float>();  // Positions in pixels
			cone.y_pixels = node["y"].as<float>();
			cone.color    = node["color"].as<std::string>();
			cones.push_back(cone);
		}
		std::cout << "Loaded " << cones.size() << " cones from " << filename << std::endl;
	}
	catch (const std::exception &e)
	{
		std::cerr << "Error loading cones from " << filename << ": " << e.what() << std::endl;
	}
	return cones;
}

int	setupCANSocket(const std::string &interface_name)
{
	int			can_socket;
	struct sockaddr_can	addr;
	struct ifreq		ifr;
	int			flags;

	// Create a socket
	if ((can_socket = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0)
	{
		perror("Error while opening socket");
		return -1;
	}

	strcpy(ifr.ifr_name, interface_name.c_str());
	if (ioctl(can_socket, SIOCGIFINDEX, &ifr) < 0)
	{
		perror("Error getting interface index");
		close(can_socket);
		return -1;
	}

	addr.can_family  = AF_CAN;
	addr.can_ifindex = ifr.ifr_ifindex;

	// Bind the socket to the CAN interface
	if (bind(can_socket, (struct sockaddr *)&addr, sizeof(addr)) < 0)
	{
		perror("Error in socket bind");
		close(can_socket);
		return -2;
	}

	// Set socket to non-blocking mode
	flags = fcntl(can_socket, F_GETFL, 0);
	if (fcntl(can_socket, F_SETFL, flags | O_NONBLOCK) < 0)
	{
		perror("Error setting non-blocking mode");
		close(can_socket);
		return -3;
	}

	return can_socket;
}

void	sendCANData(int send_can_socket, std::queue<CANData> &dataQueue, std::mutex &queueMutex, std::condition_variable &cv)
{
	struct can_frame	out_frame;
	CANData			data;
	int			bytes_sent;

	while (true)
	{
		std::unique_lock<std::mutex>	lock(queueMutex);
		cv.wait(lock, [&] { return !dataQueue.empty(); });

		while (!dataQueue.empty())
		{
			data = dataQueue.front();
			dataQueue.pop();

			// Prepare CAN frame to send
			out_frame.can_id  = data.id;
			out_frame.can_dlc = 8;  // 4 bytes for range, 4 bytes for bearing

			// Pack range and bearing into data
			memcpy(out_frame.data, &data.range, sizeof(float));
			memcpy(out_frame.data + 4, &data.bearing, sizeof(float));

			// Send the CAN frame
			bytes_sent = write(send_can_socket, &out_frame, sizeof(struct can_frame));
			if (bytes_sent != sizeof(struct can_frame))
			{
				perror("Failed to send CAN frame for cone");
			}
			else
			{
				// Uncomment the following line for debugging
				// std::cout << "Sent CAN frame with ID: " << std::hex << data.id << ", Range: " << data.range << " m, Bearing: " << data.bearing << " degrees." << std::endl;
			}
		}
	}
}

void	processCANFrame(const struct can_frame &frame, float &car_x, float &car_y, float &car_angle, bool &has_car_x, bool &has_car_y, bool &has_car_angle)
{
	float	value;
	if (frame.can_dlc != sizeof(float))
	{
		// Invalid data length
		return;
	}

	memcpy(&value, frame.data, sizeof(float));

	if (frame.can_id == CAR_X_CAN_ID)
	{
		car_x     = value;
		has_car_x = true;
	}
	else if (frame.can_id == CAR_Y_CAN_ID)
	{
		car_y     = value;
		has_car_y = true;
	}
	else if (frame.can_id == CAR_ANGLE_CAN_ID)
	{
		car_angle     = value;
		has_car_angle = true;
	}
	// Unknown CAN ID is ignored
}

void	computeAndSendConeData(const std::vector<Cone> &cones, float car_x_m, float car_y_m, float car_angle, std::queue<CANData> &dataQueue, std::mutex &queueMutex, std::condition_variable &cv)
{
	static std::default_random_engine	generator(std::random_device{}());
	std::normal_distribution<double>	rangeNoiseDist(0.0, RANGE_NOISE_STD_DEV);
	std::normal_distribution<double>	bearingNoiseDist(0.0, BEARING_NOISE_STD_DEV);
	unsigned int			yellowIndex = 0;
	unsigned int			blueIndex   = 0;
	float					cone_x_m;
	float					cone_y_m;
	float					dx;
	float					dy;
	float					cos_angle;
	float					sin_angle;
	float					x_rel;
	float					y_rel;
	float					range;
	float					bearing;
	float					bearing_deg;
	double					noisyRange;
	double					noisyBearingDeg;
	unsigned int			canID;

	std::cout << "Received car data: X=" << car_x_m << " m, Y=" << car_y_m << " m, Angle=" << car_angle * 180.0 / M_PI << " degrees" << std::endl;

	// Compute range and bearing to each cone
	for (const auto &cone : cones)
	{
		// Convert cone position from pixels to meters
		cone_x_m = cone.x_pixels / PIXELS_PER_METER;
		cone_y_m = cone.y_pixels / PIXELS_PER_METER;

		dx = cone_x_m - car_x_m;
		dy = cone_y_m - car_y_m;

		// Rotate into car's coordinate frame (negative angle)
		cos_angle = cos(-car_angle);
		sin_angle = sin(-car_angle);
		x_rel     = dx * cos_angle - dy * sin_angle;
		y_rel     = dx * sin_angle + dy * cos_angle;

		// Compute range and bearing
		range   = sqrt(x_rel * x_rel + y_rel * y_rel);
		bearing = atan2(y_rel, x_rel);  // Bearing in radians

		// Convert bearing to degrees
		bearing_deg = bearing * 180.0 / M_PI;

		// Add Gaussian noise to range and bearing
		noisyRange      = range + rangeNoiseDist(generator);
		noisyBearingDeg = bearing_deg + bearingNoiseDist(generator);

		// Ensure range is not negative
		if (noisyRange < 0.0)
		{
			noisyRange = 0.0;
		}

		// Filter out cones beyond detection range
		if (noisyRange > DETECTION_RANGE)
		{
			continue;  // Skip this cone
		}

		// Output the result
		std::cout << "Cone at (" << cone_x_m << " m, " << cone_y_m << " m): Range = " << noisyRange << " m, Bearing = " << noisyBearingDeg << " degrees, Color = " << cone.color << std::endl;

		// Determine CAN ID based on cone color
		if (cone.color == "yellow")
		{
			if (yellowIndex > 0x7F)
			{
				// Handle overflow
				std::cerr << "Warning: Too many yellow cones, exceeding CAN ID range." << std::endl;
				continue;
			}
			canID = 0x400 + yellowIndex;
			yellowIndex++;
		}
		else if (cone.color == "blue")
		{
			if (blueIndex > 0x7F)
			{
				// Handle overflow
				std::cerr << "Warning: Too many blue cones, exceeding CAN ID range." << std::endl;
				continue;
			}
			canID = 0x480 + blueIndex;
			blueIndex++;
		}
		else
		{
			// Handle other colors or unknown color
			std::cerr << "Warning: Unknown cone color '" << cone.color << "'. Skipping cone." << std::endl;
			continue;
		}

		// Push data to the queue
		{
			std::lock_guard<std::mutex>	lock(queueMutex);
			dataQueue.push({canID, static_cast<float>(noisyRange), static_cast<float>(noisyBearingDeg)});
		}
		cv.notify_one();
	}

	std::cout << "-----\n" << std::endl;
}

int	main()
{
	std::vector<Cone>	cones;
	int					can_socket;
	int					send_can_socket;
	float				car_x          = 0.0f;
	float				car_y          = 0.0f;
	float				car_angle      = 0.0f;
	bool				has_car_x      = false;
	bool				has_car_y      = false;
	bool				has_car_angle  = false;
	struct can_frame	frame;
	std::queue<CANData>	dataQueue;
	std::mutex			queueMutex;
	std::condition_variable	cv;
	std::thread			senderThread;

	// Load cones from YAML file
	cones = loadCones("../track_/cones.yaml");

	// Load configuration
	loadConfig("../config.yaml");

	// Set up CAN socket for receiving (car data)
	can_socket = setupCANSocket("vcan0");
	if (can_socket < 0)
	{
		return -1;
	}

	// Set up CAN socket for sending (cone data)
	send_can_socket = setupCANSocket("vcan0");
	if (send_can_socket < 0)
	{
		close(can_socket);
		return -1;
	}

	std::cout << "Cone sensor script is running. Listening for car data over CAN bus..." << std::endl;

	// Start sending thread
	senderThread = std::thread(sendCANData, send_can_socket, std::ref(dataQueue), std::ref(queueMutex), std::ref(cv));

	while (true)
	{
		int	nbytes = read(can_socket, &frame, sizeof(struct can_frame));

		if (nbytes < 0)
		{
			if (errno != EAGAIN && errno != EWOULDBLOCK)
			{
				perror("CAN read error");
				break;
			}
			// No data available, sleep briefly
			usleep(1000);
			continue;
		}

		if (nbytes < sizeof(struct can_frame))
		{
			std::cerr << "Incomplete CAN frame received." << std::endl;
			continue;
		}

		// Process the received CAN frame
		processCANFrame(frame, car_x, car_y, car_angle, has_car_x, has_car_y, has_car_angle);

		// If all data received, compute range and bearing
		if (has_car_x && has_car_y && has_car_angle)
		{
			// Car position is already in meters
			computeAndSendConeData(cones, car_x, car_y, car_angle, dataQueue, queueMutex, cv);

			// Reset flags to wait for new data
			has_car_x     = false;
			has_car_y     = false;
			has_car_angle = false;
		}
	}

	// Close CAN sockets (This line will only be reached if the loop breaks)
	close(can_socket);
	close(send_can_socket);

	// Join the sender thread (Not actually needed in this infinite loop, but good practice)
	senderThread.join();

	return 0;
}
