#include "globals.h"

//SoftwareSerial odrive_serial(10, 11); //RX (ODrive TX), TX (ODrive RX)
ros::NodeHandle nh;
ODriveArduino odrive1(odrive_serial);
ODriveArduino* odrives[MOTOR_NUM] = {&odrive1};
int motors[MOTOR_NUM] = {0};
float positions[MOTOR_NUM] = {0};
