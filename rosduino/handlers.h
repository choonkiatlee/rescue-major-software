/*
 * external includes
 */

#include "std_msgs/Char.h" //should not be necessary but something seems to be broken
#include "std_msgs/Float32.h" //should not be necessary but something seems to be broken
#include "std_msgs/Float32MultiArray.h" //should not be necessary but something seems to be broken
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

extern ros::Subscriber<std_msgs::Float32> position_sub;

// publishers

extern std_msgs::String rosout_msg;
extern ros::Publisher rosout_pub;

// functions

void handlers_init();
