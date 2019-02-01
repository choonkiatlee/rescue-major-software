import rospy  # in PyCharm shows error, solution: https://gavazzi.us/2017/07/31/pycharm-ros/
from std_msgs import msg

import drive
from config import *

# SUBSCRIBER CALLBACKS

def _position_test_cb(position):
    drive.set_axis_position(drive.odrives[ODRIVE_DRIVE_ID].axis0, position.data)
    rospy.loginfo("Recieved on position_test: " + str(position.data))


def _drive_commands_cb(command):
    if command.data == 'r':
        drive.full_reset_and_calibrate_all()
    rospy.loginfo("Recieved on drive/commands: " + str(command.data))


def _drive_distance_cb(distances):
    drive.drive_distance(distances.data[0], distances.data[1])
    rospy.loginfo("Recieved on drive/distance: " + ' '.join(distances.data))

# PUBLISHERS

def debug_publish(msg_str):
    try:
        debug_pub.publish(msg_str)
        rospy.loginfo("Sent on rosout: " + str(msg_str))
    except rospy.ROSInterruptException:
        pass

# TEST

def _send_back(msg_str):
    debug_publish(msg_str.data)

# FUNCTIONS

def init_handlers():
    rospy.init_node('nuc_main', anonymous=True)

    # SUBSCRIBERS

    rospy.Subscriber("position_test", msg.Float32, _position_test_cb)
    rospy.Subscriber("drive/commands", msg.Char, _drive_commands_cb)
    rospy.Subscriber("drive/distance", msg.Float32MultiArray, _drive_distance_cb)

    rospy.Subscriber("send_back", msg.String, _send_back) # TEST

    global debug_pub
    debug_pub = rospy.Publisher('debug', msg.String, queue_size=10)


# TODO ?
#   void _drive_limits_cb( const std_msgs::Float32MultiArray& limits){
#     odrive_set_limits(0, limits.data[0], limits.data[1]);
#     digitalWrite(13, HIGH-digitalRead(13));  //toggle led
#   }
#   ros::Subscriber<std_msgs::Float32MultiArray> drive_limits_sub("drive/limits", _drive_limits_cb);
