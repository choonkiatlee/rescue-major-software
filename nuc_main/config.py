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

