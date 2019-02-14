import rospy  # in PyCharm shows error, solution: https://gavazzi.us/2017/07/31/pycharm-ros/
from std_msgs import msg

import drive
from config import *

# SUBSCRIBER CALLBACKS

def _position_test_cb(position):
    drive.set_axis_position(drive.odrives[ODRIVE_DRIVE_ID].axis0, position.data)
    rospy.loginfo("Recieved on position_test: " + str(position.data))


def _drive_commands_cb(command):
    if chr(command.data) == 'r':
        drive.full_reset_and_calibrate_all()
    rospy.loginfo("Recieved on drive/commands: " + str(command.data))


def _drive_distance_cb(distances):
    drive.drive_distance(distances.data[0], distances.data[1])
    rospy.loginfo("Recieved on drive/distance: " + ' '.join(map(str, distances.data)))


def _drive_velocity_cb(velocities):
    drive.drive_velocity(velocities.data[0], velocities.data[1])
    rospy.loginfo("Recieved on drive/velocity: " + ' '.join(map(str, velocities.data)))


def _drive_vel_limits_cb(velocities):
    drive.set_vel_limits(velocities.data)
    rospy.loginfo("Recieved on drive/velocity_limits: " + ' '.join(map(str, velocities.data)))


def _drive_acc_limits_cb(accs):
    drive.set_acc_limits(accs.data)
    rospy.loginfo("Recieved on drive/velocity_limits: " + ' '.join(map(str, accs.data)))

# PUBLISHERS

def debug_publish(msg_str):
    try:
        debug_pub.publish(msg_str)
        rospy.loginfo("Sent on debug: " + str(msg_str.data))
    except rospy.ROSInterruptException:
        pass

def axis_states_publish(axes):
    try:
        msg_arr = msg.Float32MultiArray()
        msg_arr.data = axes
        msg_arr.layout.dim = [msg.MultiArrayDimension(), msg.MultiArrayDimension(), msg.MultiArrayDimension()]
        msg_arr.layout.dim[0].size = len(axes)
        msg_arr.layout.dim[1].size = len(axes[0])
        msg_arr.layout.dim[2].size = len(axes[0][0])
        msg_arr.layout.dim[0].stride = len(axes) * len(axes[0]) * len(axes[0][0])
        msg_arr.layout.dim[1].stride = len(axes[0]) * len(axes[0][0])
        msg_arr.layout.dim[2].stride = len(axes[0][0])
        msg_arr.layout.dim[0].label = "odrives"
        msg_arr.layout.dim[1].label = "axes"
        msg_arr.layout.dim[2].label = "values"

        print(msg_arr.data)

        print(msg_arr)

        axis_state_pub.publish(msg_arr)
        # rospy.loginfo("Sent on debug: " + str(axes))
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
    rospy.Subscriber("drive/velocity", msg.Float32MultiArray, _drive_velocity_cb)
    rospy.Subscriber("drive/vel_limits", msg.Float32MultiArray, _drive_vel_limits_cb)
    rospy.Subscriber("drive/acc_limits", msg.Float32MultiArray, _drive_acc_limits_cb)

    rospy.Subscriber("send_back", msg.String, _send_back) # TEST

    global debug_pub
    debug_pub = rospy.Publisher('debug', msg.String, queue_size=10)

    global axis_state_pub
    axis_state_pub = rospy.Publisher('drive/axis_states', msg.Float32MultiArray, queue_size=10)


# TODO ?
#   void _drive_limits_cb( const std_msgs::Float32MultiArray& limits){
#     odrive_set_limits(0, limits.data[0], limits.data[1]);
#     digitalWrite(13, HIGH-digitalRead(13));  //toggle led
#   }
#   ros::Subscriber<std_msgs::Float32MultiArray> drive_limits_sub("drive/limits", _drive_limits_cb);
