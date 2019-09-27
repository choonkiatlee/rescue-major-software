from __future__ import print_function, division

import odrive
from odrive.enums import *
import time
import math
import argparse
from tkinter import *
from pynput import keyboard
import numpy as np
testing = False # testing variable
command = ["Stopped", 0]
incrament = 0.5


config = {
	"wheel_radius" : 0.1,
	"drive_gearing" : 28.55, # 64 means a gearing of 64 to one
	"flipper_gearing" : 81.37
}

serial_list_raw = ["209137933548", "336631563536"]

odrv = None


############################################
#### CALIBRATION PROCESS ###################
############################################


# Calibrate one odrive 
# Input: Serial id of the odrive as a string
# Output: ODrive object
def full_reset_and_calibrate(targetSerialID, i=0):
	"""Completely resets the Odrive, calibrates axis0 and configures axis0 to only encoder index search on startup and be ready in AXIS_STATE_CLOSED_LOOP_CONTROL"""
	print("------------- CALIBRATION ODRV " + str(i) + " -------------\n")

	odrv0 = odrive.find_any(serial_number="209137933548")

	odrv0.erase_configuration()
	print("ODRV NO." + str(i) + "  " + "Erased [1/7]")
	try: # Reboot causes loss of connection, use try to supress errors
		odrv0.reboot()
	except:
		pass
	print("ODRV NO." + str(i) + "  " + "Rebooted [2/7]")
	odrv0 = odrive.find_any() # Reconnect to the Odrive
	print("ODRV NO." + str(i) + "  " + "Connected [3/7]")
	odrv0.axis0.motor.config.pre_calibrated = True # Set all the flags required for pre calibration
	odrv0.axis0.encoder.config.pre_calibrated = True
	odrv0.axis0.encoder.config.use_index = True
	odrv0.axis0.config.startup_encoder_index_search = True # Change startup sequence
	odrv0.axis0.config.startup_closed_loop_control = True
	odrv0.axis0.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE # Calibrate
	print("ODRV NO." + str(i) + "  " + "Started calibration 1 [4/7]", end="")
	while odrv0.axis0.current_state != AXIS_STATE_IDLE: # Wait for calibration to be done
		time.sleep(0.1)
		print(".", end="")
	odrv0.save_configuration()

	print("ODRV NO." + str(i) + "  " + "AXIS 1. Calibration complete [5/7]\n")
	print("now will begin calibration sequence for second axis for ODRV NO." + str(i))
	time.sleep(3)
	odrv0.axis1.motor.config.pre_calibrated = True # Set all the flags required for pre calibration
	odrv0.axis1.encoder.config.pre_calibrated = True
	odrv0.axis1.encoder.config.use_index = True
	odrv0.axis1.config.startup_encoder_index_search = True # Change startup sequence
	odrv0.axis1.config.startup_closed_loop_control = True
	odrv0.axis1.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE # Calibrate
	print("ODRV NO." + str(i) + "  " + "Started calibration 2 [6/7]", end="")
	while odrv0.axis1.current_state != AXIS_STATE_IDLE: # Wait for calibration to be done
		time.sleep(0.5)
		print(".", end="")

	print("ODRV NO." + str(i) + "  " + "AXIS 2. Calibration 2 complete [7/7]")

	#closed loop control for both axis
	odrv0.save_configuration()
	odrv0.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
	odrv0.axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL

	return odrv0

# Calibrate all odrives. 
# Input: Serial ids of the odrives as a list of strings
# Output: List of all ODrive objects
def calibrateODrives(serialIDs):
	odrives = []
	for i, ID in enumerate(serialIDs):
		odrives.append(full_reset_and_calibrate(ID, i+1))

	return odrives

def obtainODriveObjects(serialIDs):
	odrives = []
	for i, ID in enumerate(serialIDs):
		odrives.append(odrive.find_any(ID))

	return odrives




############################################
#### ODRIVE CONTROL COMMANDS ###############
############################################


# Set rps of the output shaft
# Input: axis as ODrive object, desired rps of the output, gear ratio of output to motor
def set_rps(axis, rps, gear_ratio):
	axis.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
	axis.controller.config.control_mode = CTRL_MODE_VELOCITY_CONTROL
	axis.controller.vel_setpoint = rps * 2 * math.pi * gear_ratio * 8192

# Set current of a particular axis
# Input: axis as ODrive object, max current through that axis
def set_current_limit(axis, current_limit):
	axis.motor.config.current_lim = current_limit


# Set current of a particular axis
# Input: axis as ODrive object, max current through that axis
def set_velocity_limit(axis, velocity_limit):
	axis.controller.config.vel_limit = velocity_limit




############################################
#### PROGRAM NAVIGATION ####################
############################################

def initial_setting():
	global odrv
	print("-------------- ODRIVE SANITY CHECK --------------\n\n")
	print("Full Reset and Calibrate? (y/n)")

	while 1:
		if input() == "y":
			odrv = calibrateODrives(serial_list_raw)
			break
		elif input() == "n":
			odrv = obtainODriveObjects(serial_list_raw)
			break
		else:
			print("Enter either y or n to continue")

	return odrives

def control_loop():
	global odrv

	print("hi")
	
	'''
	previous = 0

	threshold = 10	
	sign = [0,0]

	while 1:
		speed = determineSpeed()
		Direction.config(text = "Direction = " + command[0])
		CurrentSpeed.config(text = "Current speed = left: " + str(speed[1][0]) + "right: " + str(speed[1][1]))
		Speed.config(text = "Spped setting = left: " + str(speed[0][0]) + "right: " + str(speed[0][1]))
		app.update()

		maxrps = 0.5
		if not testing:

			current = speed[1][0]
			
			if abs(previous - current) > threshold:
				current = previous + np.sign(current - previous)*threshold
				
			
			set_velocity(my_drive.axis0, sign[0]*abs(current)*maxrps*0.01)
			set_velocity(my_drive.axis1, sign[1]*abs(current)*maxrps*0.01)

			previous = current
			sign = determine_sign(sign, speed[1])
			print(current)
	'''


# MAIN PROGRAM 
def main():
	initial_setting()
	control_loop()

if __name__ == "__main__":
	main()


		