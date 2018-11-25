/*
 * external includes
 */

#include "Arduino.h"
#include <ros.h>
#include <std_msgs/String.h>

/*
 * internal includes
 */

#include "globals.h"

/*
 * PUBLISHERS
 */

// subscribers

extern ros::Subscriber<std_msgs::String> send_back_sub;

// publishers

extern std_msgs::String back_msg;
extern ros::Publisher back_sub;

// functions

void handlers_init();
