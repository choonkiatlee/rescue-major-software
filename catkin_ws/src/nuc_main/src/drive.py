from __future__ import print_function, division

import odrive
from odrive.enums import *
import time
import os
from fibre.utils import Event, Logger
import time
import sys

from config import *

odrives = []
positions = []
axis_states = []
connected = False


def full_reset_and_calibrate_all():
    global odrives
    global positions
    global connected
    """Completely resets all odrives, calibrates axis0 and configures axis0 to only encoder index search
    on startup and be ready in AXIS_STATE_CLOSED_LOOP_CONTROL"""
    connected = False
    print("Erased [1/5]")
    for odrive_id, my_drive in enumerate(odrives):
        my_drive.erase_configuration()
        print("Erased odrive %d" % odrive_id)
        try:  # Reboot causes loss of connection, use try to supress errors
            my_drive.reboot()
        except Exception as e:
            print("Suppressed error during reboot: " + str(e))
            pass
    positions = []
    print("Rebooted [2/5]")
    connect_all()
    print("Connected [3/5]")

    for odrive_id, axes, my_drive in zip(range(len(odrives)), CONNECTED_AXES, odrives):
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
            axis.motor.config.current_lim = 50
            axis.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE  # Calibrate
            print("Started calibration of odrive %d axis %d" % (odrive_id, axis_id), end="")
            while axis.current_state != AXIS_STATE_IDLE:  # Wait for calibration to be done
                time.sleep(0.1)
                print(".", end="")
                sys.stdout.flush()
            print("\n Calibration of odrive %d axis %d complete" % (odrive_id, axis_id))
            my_drive.save_configuration()
            axis.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
    print("Calibrations complete [5/5]")
    init_axis_states()
    connected = True
    set_all_limits()


def stop_all():
    for axes, my_drive in zip(CONNECTED_AXES, odrives):
        for axis_id in range(2):
            if axes[axis_id] == False:
                continue
            if axis_id == 0:
                axis = my_drive.axis0
            else:
                axis = my_drive.axis1
            set_axis_rps(axis, 0)


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
    original_pos = axis.encoder.pos_estimate
    axis.controller.pos_setpoint = pos + original_pos
    return pos + original_pos


def add_axis_distance(axis, dis):
    pos = dis * DRIVE_GEARING / (2 * np.pi * WHEEL_RADIUS) * RADIAN
    pos = add_axis_position(axis, pos)
    return pos / DRIVE_GEARING * (2 * np.pi * WHEEL_RADIUS) / RADIAN


def set_axis_rps(axis, rps):
    axis.controller.config.control_mode = CTRL_MODE_VELOCITY_CONTROL
    axis.controller.vel_setpoint = rps * RADIAN


def set_axis_drive_velocity(axis, v):
    rps = v * DRIVE_GEARING / (2 * np.pi * WHEEL_RADIUS)
    print(v, rps)
    set_axis_rps(axis, rps)


def set_axis_flipper_velocity(axis, v):
    rps = v * FLIPPER_GEARING
    set_axis_rps(axis, rps)


def drive_distance(distance, angular_distance):
    if abs(angular_distance) > 1e-6:
        radius = distance / angular_distance
        radiusL = radius + TRACKS_SEPARATION / 2
        radiusR = radius - TRACKS_SEPARATION / 2
        distanceL = radiusL * angular_distance
        distanceR = radiusR * angular_distance
    else:
        distanceL = distance
        distanceR = distance
    axes = (odrives[ODRIVE_DRIVE_ID].axis0, odrives[ODRIVE_DRIVE_ID].axis1)

    axis_states[ODRIVE_DRIVE_ID][DRIVE_LEFT_AXIS][AXIS_LOG_ID_POS_SET] =\
        add_axis_distance(axes[DRIVE_LEFT_AXIS], distanceL * DRIVE_LEFT_DIR)
    axis_states[ODRIVE_DRIVE_ID][DRIVE_LEFT_AXIS][AXIS_LOG_ID_VEL_SET] =\
        axes[DRIVE_LEFT_AXIS].controller.config.acc_limit / DRIVE_GEARING * (2 * np.pi * WHEEL_RADIUS) / RADIAN

    axis_states[ODRIVE_DRIVE_ID][DRIVE_RIGHT_AXIS][AXIS_LOG_ID_POS_SET] =\
        add_axis_distance(axes[DRIVE_RIGHT_AXIS], distanceR * DRIVE_RIGHT_DIR)
    axis_states[ODRIVE_DRIVE_ID][DRIVE_RIGHT_AXIS][AXIS_LOG_ID_VEL_SET] =\
        axes[DRIVE_RIGHT_AXIS].controller.config.acc_limit / DRIVE_GEARING * (2 * np.pi * WHEEL_RADIUS) / RADIAN


def drive_velocity(speed, angular_speed):
    if abs(angular_speed) > 1e-6:
        radius = speed / angular_speed
        radiusL = radius + TRACKS_SEPARATION / 2
        radiusR = radius - TRACKS_SEPARATION / 2
        speedL = radiusL * angular_speed
        speedR = radiusR * angular_speed
    else:
        speedL = speed
        speedR = speed
    axes = (odrives[ODRIVE_DRIVE_ID].axis0, odrives[ODRIVE_DRIVE_ID].axis1)

    axis_states[ODRIVE_DRIVE_ID][DRIVE_LEFT_AXIS][AXIS_LOG_ID_VEL_SET] = speedL
    set_axis_drive_velocity(axes[DRIVE_LEFT_AXIS], speedL * DRIVE_LEFT_DIR)

    axis_states[ODRIVE_DRIVE_ID][DRIVE_RIGHT_AXIS][AXIS_LOG_ID_VEL_SET] = speedR
    set_axis_drive_velocity(axes[DRIVE_RIGHT_AXIS], speedR * DRIVE_RIGHT_DIR)


def flipper_position(pos_lb, pos_lf, pos_rf, pos_rb):
    axes = tuple((odrives[i].axis0, odrives[i].axis1) for i in range(len(odrives)))
    for FL, pos, dir in zip((FLIPPER_LB, FLIPPER_LF, FLIPPER_RF, FLIPPER_RB), (pos_lb, pos_lf, pos_rf, pos_rb),
                            (FLIPPER_LB_DIR, FLIPPER_LF_DIR, FLIPPER_RF_DIR, FLIPPER_RB_DIR)):
        axis_states[FL[0]][FL[1]][AXIS_LOG_ID_POS_SET] = pos
        set_axis_position(axes[FL[0]][FL[1]], pos * dir)
        axis_states[FL[0]][FL[1]][AXIS_LOG_ID_VEL_SET] =\
            axes[DRIVE_RIGHT_AXIS].controller.config.acc_limit / DRIVE_GEARING * (2 * np.pi * WHEEL_RADIUS) / RADIAN


def flipper_velocity(vel_lb, vel_lf, vel_rf, vel_rb):
    axes = tuple((odrives[i].axis0, odrives[i].axis1) for i in range(len(odrives)))
    for FL, vel, dir in zip((FLIPPER_LB, FLIPPER_LF, FLIPPER_RF, FLIPPER_RB), (vel_lb, vel_lf, vel_rf, vel_rb),
                            (FLIPPER_LB_DIR, FLIPPER_LF_DIR, FLIPPER_RF_DIR, FLIPPER_RB_DIR)):
        axis_states[FL[0]][FL[1]][AXIS_LOG_ID_VEL_SET] = vel
        set_axis_flipper_velocity(axes[FL[0]][FL[1]], vel * dir)


def init_axis_states():
    global axis_states
    axis_states = []
    for my_drive in odrives:
        axis_states.append([[0, 0, 0, 0, 0], [0, 0, 0, 0, 0]])


def get_states():
    if connected:
        for i in range(len(odrives)):
            if CONNECTED_AXES[i][0]:
                axis_states[i][0][AXIS_LOG_ID_STATE] = odrives[i].axis0.current_state
                axis_states[i][0][AXIS_LOG_ID_VEL] = odrives[i].axis0.encoder.vel_estimate / \
                                                     DRIVE_GEARING * (2 * np.pi * WHEEL_RADIUS) / RADIAN
                axis_states[i][0][AXIS_LOG_ID_POS] = odrives[i].axis0.encoder.pos_estimate / \
                                                     DRIVE_GEARING * (2 * np.pi * WHEEL_RADIUS) / RADIAN
            if CONNECTED_AXES[i][1]:
                axis_states[i][1][AXIS_LOG_ID_STATE] = odrives[i].axis1.current_state
                axis_states[i][1][AXIS_LOG_ID_VEL] = odrives[i].axis1.encoder.vel_estimate / \
                                                     DRIVE_GEARING * (2 * np.pi * WHEEL_RADIUS) / RADIAN
                axis_states[i][1][AXIS_LOG_ID_POS] = odrives[i].axis1.encoder.pos_estimate / \
                                                     DRIVE_GEARING * (2 * np.pi * WHEEL_RADIUS) / RADIAN
    return axis_states


def set_acc_limits(lims):
    for i, lim in enumerate(lims):
        if i // 2 >= len(odrives):
            break
        if CONNECTED_AXES[i // 2][i % 2]:
            axis = (odrives[i // 2].axis0, odrives[i // 2].axis1)[i % 2]
            axis.controller.config.accel_limit = lim * DRIVE_GEARING / (2 * np.pi * WHEEL_RADIUS) * RADIAN
            axis.controller.config.decel_limit = lim * DRIVE_GEARING / (2 * np.pi * WHEEL_RADIUS) * RADIAN


def set_vel_limits(lims):
    for i, lim in enumerate(lims):
        if i // 2 >= len(odrives):
            break
        if CONNECTED_AXES[i // 2][i % 2]:
            axis = (odrives[i // 2].axis0, odrives[i // 2].axis1)[i % 2]
            axis.controller.config.vel_limit = lim * DRIVE_GEARING / (2 * np.pi * WHEEL_RADIUS) * RADIAN
            print("set as " + str(lim * DRIVE_GEARING / (2 * np.pi * WHEEL_RADIUS) * RADIAN))


def set_curr_limits(lims):
    for i, lim in enumerate(lims):
        if i // 2 >= len(odrives):
            break
        if CONNECTED_AXES[i // 2][i % 2]:
            axis = (odrives[i // 2].axis0, odrives[i // 2].axis1)[i % 2]
            axis.motor.config.current_lim = lim


def init():
    connect_all()
    print("Connected")
    for my_drive in odrives:
        print("Bus voltage is " + str(my_drive.vbus_voltage) + "V")
    init_axis_states()
    set_all_limits()


def set_all_limits():
    set_vel_limits([MAX_DRIVE_SPEED, MAX_DRIVE_SPEED,
                    MAX_FLIPPER_SPEED, MAX_FLIPPER_SPEED, MAX_FLIPPER_SPEED, MAX_FLIPPER_SPEED])
    # set_acc_limits([5]*6)
    set_curr_limits([MAX_CURRENT] * 6)


def connect_all():
    global odrives
    global connected
    odrives = []
    for i in range(len(CONNECTED_AXES)):
        if CONNECTED_AXES[i][0] or CONNECTED_AXES[i][1]:
            odrives.append(odrive.find_any(serial_number=ODRIVE_SERIAL_NUMS[i]))
    connected = True
