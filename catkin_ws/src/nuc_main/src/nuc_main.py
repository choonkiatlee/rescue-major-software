#!/usr/bin/env python

import rospy
import os
import time

import handlers
import drive
from config import *

if __name__ == '__main__':
    drive.init()
    handlers.init_handlers()

    axis_log_time = -10000

    if not rospy.core.is_initialized():
        raise rospy.exceptions.ROSInitException("client code must call rospy.init_node() first")
    rospy.logdebug("node[%s, %s] entering spin(), pid[%s]", rospy.core.get_caller_id(),
                   rospy.core.get_node_uri(), os.getpid())
    try:
        while not rospy.core.is_shutdown():

            if time.time() > axis_log_time + AXIS_LOG_PERIOD:
                states = drive.get_states()
                print(states)
                # handlers.axis_states_publish(states)
            rospy.rostime.wallsleep(0.5)
    except KeyboardInterrupt:
        rospy.logdebug("keyboard interrupt, shutting down")
        rospy.core.signal_shutdown('keyboard interrupt')
