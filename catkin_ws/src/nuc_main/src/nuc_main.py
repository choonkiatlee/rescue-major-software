#!/usr/bin/env python

import rospy

import handlers
import drive

if __name__ == '__main__':
    # drive.init()
    handlers.init_handlers()

    rospy.spin()
