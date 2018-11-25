/*
 * This is a place for ros subscribers and publishers
 * Please keep larger handler functions outside this file
 */

#include "handlers.h"

/*
 * SUBSCRIBERS
 */

// example

void send_back_cb( const std_msgs::String& cmd_msg){
  back_msg = cmd_msg;
  back_sub.publish( &back_msg );
  digitalWrite(13, HIGH-digitalRead(13));  //toggle led  
}

ros::Subscriber<std_msgs::String> send_back_sub("send_back", send_back_cb);

/*
 * PUBLISHERS
 */

// example

std_msgs::String back_msg;
ros::Publisher back_sub("back", &back_msg);

/*
 * FUNCTIONS
 */

void handlers_init() {
  nh.subscribe(send_back_sub); // example
  nh.advertise(back_sub); // example
}
