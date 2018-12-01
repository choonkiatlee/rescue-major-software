#include <Arduino.h>
//#include <SoftwareSerial.h>
#include <ros.h>
#include "z_odrive.h"

#define RAD_TO_CM 100000
#define ODRIVE_VEL_LIMIT 100000.0f
#define ODRIVE_CURRENT_LIMIT 10.0f
#define TRACKS_WIDTH 40.0f
#define DIRECTION_RIGHT 1
#define MOTOR_LEFT 0
#define MOTOR_RIGHT 1
#define MOTOR_LEFT_DIR 1
#define MOTOR_RIGHT_DIR 1

#define odrive_serial Serial1
//extern SoftwareSerial odrive_serial; //RX (ODrive TX), TX (ODrive RX)

#define MOTOR_NUM 1

extern ros::NodeHandle nh;
extern ODriveArduino* odrives[];
extern int motors[];
extern float positions[];
