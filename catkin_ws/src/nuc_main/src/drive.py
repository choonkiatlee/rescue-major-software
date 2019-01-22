from __future__ import print_function, division

import odrive
from odrive.enums import *
import time

from config import *

odrives = []
positions = []


def full_reset_and_calibrate_all():
    global odrives
    global positions
    """Completely resets all odrives, calibrates axis0 and configures axis0 to only encoder index search
    on startup and be ready in AXIS_STATE_CLOSED_LOOP_CONTROL"""
    for my_drive in odrives:
        my_drive.erase_configuration()
        print("Erased.")
        try:  # Reboot causes loss of connection, use try to supress errors
            my_drive.reboot()
        except Exception as e:
            print("Suppressed error during reboot: " + str(e))
            pass
    positions = []
    print("Rebooted.")
    odrives = list(odrive.find_all())  # Reconnect to the Odrives
    print("Connected.")
    for axes, my_drive in zip(CONNECTED_AXES, odrives):
        positions.append((0, 0))
        for axis_id in range(2):
            if axes[axis_id] == False:
                continue
            if axis_id == 0:
                axis = my_drive.axis0
            else:
                axis = my_drive.axis1
            axis.motor.config.pre_calibrated = True  # Set all the flags required for pre calibration
            axis.encoder.config.pre_calibrated = True
            axis.encoder.config.use_index = True
            axis.config.startup_encoder_index_search = True  # Change startup sequence
            axis.config.startup_closed_loop_control = True
            axis.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE  # Calibrate
            print("Started calibration.")
            while axis.current_state != AXIS_STATE_IDLE:  # Wait for calibration to be done
                time.sleep(0.1)
            print("Calibration complete.")
            my_drive.save_configuration()
            axis.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL


def position_test():
    while True:
        for my_drive in odrives:
            for axis in (my_drive.axis0, my_drive.axis1):
                set_axis_position(axis, 0)
                time.sleep(0.5)
                set_axis_position(axis, 4096)
                time.sleep(0.5)
                set_axis_position(axis, 0)
                time.sleep(0.5)
                set_axis_position(axis, -4096)
                time.sleep(0.5)


def set_axis_position(axis, pos):
    axis.controller.config.control_mode = CTRL_MODE_POSITION_CONTROL
    axis.controller.pos_setpoint = pos


def add_axis_position(axis, pos):
    axis.controller.config.control_mode = CTRL_MODE_POSITION_CONTROL
    original_pos = axis.controller.pos_feedback
    axis.controller.pos_setpoint = pos + original_pos


def set_axis_rps(axis, rps):
    axis.controller.config.control_mode = CTRL_MODE_VELOCITY_CONTROL
    axis.controller.vel_setpoint = rps * RADIAN


def set_axis_velocity(axis, v):
    rps = v * DRIVE_GEARING / (2 * np.pi * WHEEL_RADIUS)
    set_axis_rps(axis, rps)


def drive_distance(distance, angular_distance):
    if abs(angular_distance) < 1e-6:
        radius = distance / angular_distance
        radiusL = radius + TRACKS_SEPARATION / 2
        radiusR = radius - TRACKS_SEPARATION / 2
        distanceL = radiusL * angular_distance
        distanceR = radiusR * angular_distance
    else:
        distanceL = distance
        distanceR = distance
    axes = ()
    add_axis_position(axes[DRIVE_LEFT_AXIS], distanceL * DRIVE_LEFT_DIR)
    add_axis_position(axes[DRIVE_LEFT_AXIS], distanceR * DRIVE_RIGHT_DIR)


def drive_velocity(speed, angular_speed):
    if abs(angular_speed) < 1e-6:
        radius = speed / angular_speed
        radiusL = radius + TRACKS_SEPARATION / 2
        radiusR = radius - TRACKS_SEPARATION / 2
        speedL = radiusL * angular_speed
        speedR = radiusR * angular_speed
    else:
        speedL = speed
        speedR = speed
    axes = (odrives[ODRIVE_DRIVE_ID].axis0, odrives[ODRIVE_DRIVE_ID].axis1)
    add_axis_position(axes[DRIVE_LEFT_AXIS], speedL * DRIVE_LEFT_DIR)
    add_axis_position(axes[DRIVE_LEFT_AXIS], speedR * DRIVE_RIGHT_DIR)


def init():
    global odrives
    odrives = list(odrive.find_all())
    for my_drive in odrives:
        print("Bus voltage is " + str(my_drive.vbus_voltage) + "V")


# Find a connected ODrive (this will block until you connect one)
# print("Finding an odrive...")
# my_drive = odrive.find_any()

# my_drive = full_reset_and_calibrate(my_drive)

# To read a value, simply read the property
# print("Bus voltage is " + str(my_drive.vbus_voltage) + "V")

# set_rps(my_drive, 1)
