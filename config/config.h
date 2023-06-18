#ifndef CONFIG_H
#define CONFIG_H

// window name 
#define WINDOW_NAME "Display"
#define SUB_WINDOW_NAME "Trajectory"

// Your Camera Path
#define CAMERA_PATH "nvarguscamerasrc ! video/x-raw(memory:NVMM), width=(int)1280, height=(int)720, framerate=(fraction)30/1 ! nvvidconv flip-method=2 ! video/x-raw, width=(int)1226, height=(int)370, format=(string)BGRx ! videoconvert ! video/x-raw, format=(string)BGR ! appsink"

#define REAL_FOCAL_LENGTH 1316.9918

// #define SIMULATION_DATA_PATH "../Data/07/image_0/%06d.png"
// #define SIMULATION_DATA_PATH "../Data/dataset/%05d.png"
#define SIMULATION_DATA_PATH "../Data/turn_dataset1/%05d.png"
#define SIMUL_FOCAL_LENGTH 707.0912

// Your lidar port in dev 
#define LIDAR_PORT "/dev/ttyUSB0"
#define SCALE 0.5
// screen size 
#define SCREENX 1000
#define SCREENY 1000
// start location 
#define STARTX 100
#define STARTY 100
// baudrate 
#define LIDAR_BAUDRATE 115200
// PI 
#define PI 3.14
// transform ratio
#define PIXEL_RATIO 0.02
#define OUR_LIDAR_MAX_DISTANCE 2000

#define _countof( _Array ) (int)(sizeof(_Array) / sizeof(_Array[0]))

// UART PORT /dev/ttyUSB1
#define UART_PORT "/dev/ttyUSB0"
#define UART_BAUDRATE B115200
#define VMINX 1 
#define NSERIAL_CHAR 256

#define START_FLAG 's'
#define UP_FLAG 'u'
#define GO_FORWARD 'f'
#define TURN_RIGHT 'r'
#define TURN_LEFT 'l'
#define END_FLAG 'e'

#endif // CONFIG_H
