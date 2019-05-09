import numpy as np

# drive related dimensions and gearing

WHEEL_RADIUS = 0.07
DRIVE_GEARING = 26 * 3  # 64 means a gearing of 64 to one
FLIPPER_GEARING = 16
TRACKS_SEPARATION = 0.3

# control settings

UNIT_ANGLE = 1 / 360  # multiplication factor of unit into 1 rotation
UNIT_DISTANCE = 1  # multiplication factor of unit into m
UNIT_SPEED = 1  # multiplication factor of unit into m/s
RADIAN = 8192
MAX_DRIVE_SPEED = 0.23
MAX_FLIPPER_SPEED = 2.55
MAX_CURRENT = 5

# odrives config

ODRIVE_DRIVE_ID = 0
ODRIVE_LEFT_FLIPPER_ID = 1
ODRIVE_RIGHT_FLIPPER_ID = 2
DIRECTION_RIGHT = 1
DRIVE_LEFT_AXIS = 0
DRIVE_RIGHT_AXIS = 1
DRIVE_LEFT_DIR = -1  # 1 or -1 multiplicative factor
DRIVE_RIGHT_DIR = 1
FLIPPER_LF = (1, 0)  # odrive ID, axis ID
FLIPPER_LB = (1, 1)
FLIPPER_RF = (2, 0)
FLIPPER_RB = (2, 1)
FLIPPER_LF_DIR = 1
FLIPPER_LB_DIR = 1
FLIPPER_RF_DIR = 1
FLIPPER_RB_DIR = 1
CONNECTED_AXES = [(True, True), (True, True), (True, True)]
ODRIVE_SERIAL_NUMS = ["209137933548", "205337863548", "2080376D3548"]

# axis log config

AXIS_LOG_PERIOD = 5
AXIS_LOG_ID_STATE = 0
AXIS_LOG_ID_VEL = 1
AXIS_LOG_ID_VEL_SET = 2
AXIS_LOG_ID_POS_SET = 3
AXIS_LOG_ID_POS = 4
