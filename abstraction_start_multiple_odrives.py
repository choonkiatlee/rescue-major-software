from __future__ import print_function, division

import odrive
from odrive.enums import *
import time
import math
#import msvcrt
import os
from fibre.utils import Event, Logger
import time

config = {
	"wheel_radius" : 0.1,
	"drive_gearing" : 16, # 64 means a gearing of 64 to one
	"flipper_gearing" : 5
}

def full_reset_and_calibrate(targetSerial):
	"""Completely resets the Odrive, calibrates axis1 and configures axis1 to only encoder index search on startup and be ready in AXIS_STATE_CLOSED_LOOP_CONTROL"""

	odrv0 = odrive.find_any(serial_number=targetSerial)
	print("found odrive with serial target")

	odrv0.erase_configuration()
	print("Erased [1/7]")
	try: # Reboot causes loss of connection, use try to supress errors
		odrv0.reboot()
	except:
		pass

	odrv0 = odrive.find_any(serial_number=targetSerial)
	print("Rebooted [2/7]")

	print("Found odrive with serial " + str(odrv0.serial_number))
	print("Connected [3/7]")
	odrv0.axis0.motor.config.pre_calibrated = True # Set all the flags required for pre calibration
	odrv0.axis0.encoder.config.pre_calibrated = True
	odrv0.axis0.encoder.config.use_index = True
	odrv0.axis0.config.startup_encoder_index_search = True # Change startup sequence
	odrv0.axis0.config.startup_closed_loop_control = True
	odrv0.axis0.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE # Calibrate
	print("Started calibration 1 [4/7]", end="")
	while odrv0.axis0.current_state != AXIS_STATE_IDLE: # Wait for calibration to be done
		time.sleep(0.5)
		print(".", end="")
	print("\nCalibration 1 complete [5/7]")
	
	
	odrv0.axis1.motor.config.pre_calibrated = True # Set all the flags required for pre calibration
	odrv0.axis1.encoder.config.pre_calibrated = True
	odrv0.axis1.encoder.config.use_index = True
	odrv0.axis1.config.startup_encoder_index_search = True # Change startup sequence
	odrv0.axis1.config.startup_closed_loop_control = True
	odrv0.axis1.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE # Calibrate
	print("Started calibration 2 [6/7]", end="")
	while odrv0.axis1.current_state != AXIS_STATE_IDLE: # Wait for calibration to be done
		time.sleep(0.5)
		print(".", end="")

	print("\nCalibration 2 complete [7/7]")

	odrv0.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
	odrv0.axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
	odrv0.save_configuration()

	return odrv0

def set_position(axis, pos):
	axis.controller.config.control_mode = CTRL_MODE_POSITION_CONTROL
	axis.controller.pos_setpoint = pos

def position_test():
	while True:
		for pos in [0, 4096, 0, -4096]:
			for od in odrives:
				set_position(od.axis0, pos)
				set_position(od.axis1, pos)
			time.sleep(0.5)

def velocity_test():
	while True:
		for pos in [0, 1, 0, -1]:
			for od in odrives:
				set_velocity(od.axis0, pos)
				set_velocity(od.axis1, pos)
			time.sleep(0.01)

def set_rps(axis, rps):
	axis.controller.config.control_mode = CTRL_MODE_VELOCITY_CONTROL
	axis.controller.vel_setpoint = rps * 8192

def set_velocity(axis, v):
	rps = v * config["drive_gearing"] / (2 * 3.1415 * config["wheel_radius"])
	# print(rps * 8192)
	set_rps(axis, rps)



serialnum = ["209137933548", "205337863548", "2080376D3548"]
odrives = []

CALIBRATE = False

for number in serialnum:
	if CALIBRATE:
		odrives.append(full_reset_and_calibrate(number))
	else:
		odrives.append(odrive.find_any(serial_number=number))

velocity_test()
