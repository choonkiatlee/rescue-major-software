/*
 * This is a place for ros subscribers and publishers
 * Please keep larger handler functions outside this file
 */
#include "handlers.h"
#include "odrive.h"

/*
 * SUBSCRIBERS
 */

// example

//void send_back_cb( const std_msgs::String& cmd_msg){
//  back_msg = cmd_msg;
//  back_sub.publish( &back_msg );
//  digitalWrite(13, HIGH-digitalRead(13));  //toggle led  
//}
//
//ros::Subscriber<std_msgs::String> send_back_sub("send_back", send_back_cb);

void _position_cb( const std_msgs::Float32& distance){
  odrive_run_fixed(0, distance.data);

//      for (float ph = 0.0f; ph < 6.28318530718f; ph += 0.01f) {
//        float pos_m0 = 20000.0f * cos(ph);
////        float pos_m1 = 20000.0f * sin(ph);
//        odrives[0]->SetPosition(0, pos_m0);
////        odrive.SetPosition(1, pos_m1);
//        delay(5);
//      }
  digitalWrite(13, HIGH-digitalRead(13));  //toggle led
}

ros::Subscriber<std_msgs::Float32> position_sub("position", _position_cb);
//
//void _drive_limits_cb( const std_msgs::Float32MultiArray& limits){
//  odrive_set_limits(0, limits.data[0], limits.data[1]);
//  digitalWrite(13, HIGH-digitalRead(13));  //toggle led
//}
//
//ros::Subscriber<std_msgs::Float32MultiArray> drive_limits_sub("drive/limits", _drive_limits_cb);

void _drive_commands_cb( const std_msgs::Char& command){
  if(command.data == 'r') odrive_init();
  digitalWrite(13, HIGH-digitalRead(13));  //toggle led
}

ros::Subscriber<std_msgs::Char> drive_commands_sub("drive/commands", _drive_commands_cb);
//
//void _drive_position_cb( const std_msgs::Float32MultiArray& positions){
//  odrive_drive_fixed(positions.data[0], positions.data[1]);
//  digitalWrite(13, HIGH-digitalRead(13));  //toggle led
//}
//
//ros::Subscriber<std_msgs::Float32MultiArray> drive_position_sub("drive/position", _drive_position_cb);

/*
 * PUBLISHERS
 */

//std_msgs::String rosout_msg;
//ros::Publisher rosout_pub("rosout", &rosout_msg);

/*
 * FUNCTIONS
 */

void handlers_init() {
//  nh.subscribe(send_back_sub); // example
//  nh.advertise(rosout_pub);
  nh.subscribe(position_sub);
//  nh.subscribe(drive_limits_sub);
//  nh.subscribe(drive_position_sub);
  nh.subscribe(drive_commands_sub);
//  nh.subscribe(reset_sub);
  
  
  
}
