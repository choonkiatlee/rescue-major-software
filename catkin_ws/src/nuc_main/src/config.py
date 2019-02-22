import numpy as np

# drive related dimensions and gearing

WHEEL_RADIUS = 10
DRIVE_GEARING = 64  # 64 means a gearing of 64 to one
FLIPPER_GEARING = 64
TRACKS_SEPARATION = 50

# control settings

UNIT_ANGLE = 1 / 360  # multiplication factor of unit into 1 rotation
UNIT_DISTANCE = 1  # multiplication factor of unit into cm
UNIT_SPEED = 1  # multiplication factor of unit into cm/s
RADIAN = 8192

# odrives config

ODRIVE_DRIVE_ID = 0
ODRIVE_LEFT_FLIPPER_ID = 1
ODRIVE_RIGHT_FLIPPER_ID = 2
DIRECTION_RIGHT = 1
DRIVE_LEFT_AXIS = 0
DRIVE_RIGHT_AXIS = 1
DRIVE_LEFT_DIR = 1  # 1 or -1 multiplicative factor
DRIVE_RIGHT_DIR = 1
CONNECTED_AXES = [(True, False), (False, False), (False, False)]
# TODO: use some identifier to avoid mismach of odrives

AXIS_LOG_PERIOD = 0.5
AXIS_LOG_ID_STATE = 0
AXIS_LOG_ID_VEL = 1
AXIS_LOG_ID_VEL_SET = 2
AXIS_LOG_ID_POS_SET = 3
AXIS_LOG_ID_POS = 4
AXIS_LOG_STATE_UNCALIBRATED = 0
AXIS_LOG_STATE_IDLE = 1
AXIS_LOG_STATE_MOV_VEL = 2
AXIS_LOG_STATE_MOV_POS = 3
AXIS_LOG_STATE_UNCONNECTED = 4
