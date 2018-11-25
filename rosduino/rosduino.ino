/*
 * rosserial Servo Control Example
 *
 * This sketch demonstrates the control of hobby R/C servos
 * using ROS and the arduiono
 * 
 * For the full tutorial write up, visit
 * www.ros.org/wiki/rosserial_arduino_demos
 *
 * For more information on the Arduino Servo Library
 * Checkout :
 * http://www.arduino.cc/en/Reference/Servo
 */

#include "rosduino.h"

void setup(){
  pinMode(13, OUTPUT);

  nh.initNode();
  handlers_init();
}

void loop(){
  nh.spinOnce();
  delay(1);
}
