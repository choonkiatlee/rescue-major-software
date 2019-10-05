from __future__ import print_function, division

import math
import odrive
from odrive.enums import *
import time
import config

odrives = {}
axis_states = []


def full_reset_and_calibrate_all():
    global odrives
    """Completely resets all odrives, calibrates axis0 and configures axis0 to only encoder index search
    on startup and be ready in AXIS_STATE_CLOSED_LOOP_CONTROL"""

    print("Starting full reset and calibrate")
    for drive_name, drive in odrives:
        drive_cfg = config.ODRIVES[drive_name]
        drive.erase_configuration()
        print("Erased odrive " + drive_name + "(" + drive_cfg["SERIAL_NO"] + ")")
        try:  # Reboot causes loss of connection, use try to supress errors
            drive.reboot()
        except Exception as e:
            print("Suppressed error during reboot of " + drive_name + "(" + drive_cfg["SERIAL_NO"] + "): " + str(e))
            pass
        print("Rebooted odrive " + drive_name + "(" + drive_cfg["SERIAL_NO"] + ")")
    connect_all()

    for drive_name, drive in odrives:
        drive_cfg = config.ODRIVES[drive_name]
        for axis_id in range(2):
            if axis_id == 0:
                axis = drive.axis0
            else:
                axis = drive.axis1
            axis.motor.config.pre_calibrated = True  # Set all the flags required for pre calibration
            axis.encoder.config.pre_calibrated = True
            axis.encoder.config.use_index = True
            axis.config.startup_encoder_index_search = True  # Change startup sequence
            axis.config.startup_closed_loop_control = True
            axis.motor.config.current_lim = 50
            axis.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE  # Calibrate
            print("Started calibration of odrive " + drive_name + "(" + drive_cfg["SERIAL_NO"] + ") axis " + str(axis_id), end="")
            while axis.current_state != AXIS_STATE_IDLE:  # Wait for calibration to be done
                time.sleep(0.5)
                print(".", end="")
            print("\n Calibration of odrive " + drive_name + "(" + drive_cfg["SERIAL_NO"] + ") axis " + str(axis_id) + " complete")
            drive.save_configuration()
            axis.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
    print("Calibrations complete")
    set_all_limits()


def stop_all():
    for drive_name, drive in odrives:
        set_axis_rps(drive.axis0, 0)
        set_axis_rps(drive.axis1, 0)
    print("Stopped all odrives")


def set_axis_position(axis, pos):
    axis.controller.config.control_mode = CTRL_MODE_POSITION_CONTROL
    axis.controller.pos_setpoint = pos


def add_axis_position(axis, pos):
    axis.controller.config.control_mode = CTRL_MODE_POSITION_CONTROL
    original_pos = axis.encoder.pos_estimate
    axis.controller.pos_setpoint = pos + original_pos
    return pos + original_pos


def add_axis_distance(axis, dis):
    pos = dis * config.DRIVE["DRIVE_GEARING"] / (2 * math.pi * config.DRIVE["WHEEL_RADIUS"]) * config.DRIVE["RADIAN"]
    pos = add_axis_position(axis, pos)
    return pos / config.DRIVE["DRIVE_GEARING"] * (2 * math.pi * config.DRIVE["WHEEL_RADIUS"]) / config.DRIVE["RADIAN"]


def set_axis_rps(axis, rps):
    axis.controller.config.control_mode = CTRL_MODE_VELOCITY_CONTROL
    axis.controller.vel_setpoint = rps * config.DRIVE["RADIAN"]


def set_axis_drive_velocity(axis, v):
    rps = v * config.DRIVE["DRIVE_GEARING"] / (2 * math.pi * config.DRIVE["WHEEL_RADIUS"])
    set_axis_rps(axis, rps)


def set_axis_flipper_velocity(axis, v):
    rps = v * config.DRIVE["FLIPPER_GEARING"]
    set_axis_rps(axis, rps)


def drive_distance(distance, angular_distance):
    if abs(angular_distance) > 1e-6:
        radius = distance / angular_distance
        radius_l = radius + config.DRIVE["TRACKS_SEPARATION"] / 2
        radius_r = radius - config.DRIVE["TRACKS_SEPARATION"] / 2
        distance_l = radius_l * angular_distance
        distance_r = radius_r * angular_distance
    else:
        distance_l = distance
        distance_r = distance
    axes = (odrives["DRIVE"].axis0, odrives["DRIVE"].axis1)

    add_axis_distance(axes[config.ODRIVES["DRIVE"]["LEFT"]["AXIS"]], distance_l * config.ODRIVES["DRIVE"]["LEFT"]["DIRECTION"])
    add_axis_distance(axes[config.ODRIVES["DRIVE"]["RIGHT"]["AXIS"]], distance_r * config.ODRIVES["DRIVE"]["RIGHT"]["DIRECTION"])


def drive_velocity(speed, angular_speed):
    if abs(angular_speed) > 1e-6:
        radius = speed / angular_speed
        radius_l = radius + config.DRIVE["TRACKS_SEPARATION"] / 2
        radius_r = radius - config.DRIVE["TRACKS_SEPARATION"] / 2
        speed_l = radius_l * angular_speed
        speed_r = radius_r * angular_speed
    else:
        speed_l = speed
        speed_r = speed
    axes = (odrives["DRIVE"].axis0, odrives["DRIVE"].axis1)

    set_axis_drive_velocity(axes[config.ODRIVES["DRIVE"]["LEFT"]["AXIS"]], speed_l * config.ODRIVES["DRIVE"]["LEFT"]["DIRECTION"])

    set_axis_drive_velocity(axes[config.ODRIVES["DRIVE"]["LEFT"]["AXIS"]], speed_r * config.ODRIVES["DRIVE"]["RIGHT"]["DIRECTION"])


def flipper_position(front, rear):
    axes = (odrives["FLIPPER"].axis0, odrives["FLIPPER"].axis1)
    set_axis_position(axes[config.ODRIVES["FLIPPER"]["FRONT"]["AXIS"]], front)
    set_axis_position(axes[config.ODRIVES["FLIPPER"]["REAR"]["AXIS"]], rear)


def flipper_velocity(front, rear):
    axes = (odrives["FLIPPER"].axis0, odrives["FLIPPER"].axis1)
    set_axis_flipper_velocity(axes[config.ODRIVES["FLIPPER"]["FRONT"]["AXIS"]], front)
    set_axis_flipper_velocity(axes[config.ODRIVES["FLIPPER"]["REAR"]["AXIS"]], rear)


def set_acc_limits(drive, flipper):
    odrives["DRIVE"].axis0.controller.config.accel_limit = drive * config.DRIVE["DRIVE_GEARING"] * config.DRIVE["CPR"]
    odrives["DRIVE"].axis0.controller.config.decel_limit = drive * config.DRIVE["DRIVE_GEARING"] * config.DRIVE["CPR"]

    odrives["DRIVE"].axis1.controller.config.accel_limit = drive * config.DRIVE["DRIVE_GEARING"] * config.DRIVE["CPR"]
    odrives["DRIVE"].axis1.controller.config.decel_limit = drive * config.DRIVE["DRIVE_GEARING"] * config.DRIVE["CPR"]

    odrives["DRIVE"].axis0.controller.config.accel_limit = flipper * config.DRIVE["FLIPPER_GEARING"] * config.DRIVE["CPR"]
    odrives["DRIVE"].axis0.controller.config.decel_limit = flipper * config.DRIVE["FLIPPER_GEARING"] * config.DRIVE["CPR"]

    odrives["DRIVE"].axis1.controller.config.accel_limit = flipper * config.DRIVE["FLIPPER_GEARING"] * config.DRIVE["CPR"]
    odrives["DRIVE"].axis1.controller.config.decel_limit = flipper * config.DRIVE["FLIPPER_GEARING"] * config.DRIVE["CPR"]


def set_vel_limits(drive, flipper):  # IN REVOLUTIONS PER SECOND
    odrives["DRIVE"].axis0.controller.config.vel_limit = drive * config.DRIVE["DRIVE_GEARING"] * config.DRIVE["CPR"]
    odrives["DRIVE"].axis1.controller.config.vel_limit = drive * config.DRIVE["DRIVE_GEARING"] * config.DRIVE["CPR"]

    odrives["FLIPPER"].axis0.controller.config.vel_limit = flipper * config.DRIVE["FLIPPER_GEARING"] * config.DRIVE["CPR"]
    odrives["FLIPPER"].axis1.controller.config.vel_limit = flipper * config.DRIVE["FLIPPER_GEARING"] * config.DRIVE["CPR"]


def set_curr_limits(drive, flipper):
    odrives["DRIVE"].axis0.motor.config.current_lim = drive
    odrives["DRIVE"].axis1.motor.config.current_lim = drive

    odrives["FLIPPER"].axis0.motor.config.current_lim = flipper
    odrives["FLIPPER"].axis1.motor.config.current_lim = flipper


def init():
    connect_all()
    for my_drive in odrives:
        print("Bus voltage is " + str(my_drive.vbus_voltage) + "V")
    set_all_limits()


def set_all_limits():
    set_vel_limits(config.DRIVE["MAX_DRIVE_SPEED"], config.DRIVE["MAX_FLIPPER_SPEED"])
    # set_acc_limits(1)
    set_curr_limits(config.DRIVE["MAX_CURRENT"], config.DRIVE["MAX_CURRENT"])


def connect_all():
    global odrives
    odrives = []
    print("Starting connect to all")
    for drive_name, drive_cfg in config.ODRIVES:
        print("Finding odrive " + drive_name + "(" + drive_cfg["SERIAL_NO"] + ")")
        odrives.append(odrive.find_any(serial_number=drive_cfg["SERIAL_NO"]))
        print("Found odrive " + drive_name + "(" + drive_cfg["SERIAL_NO"] + ")")
    print("Connected to all")
