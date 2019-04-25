from __future__ import print_function, division

import odrive
from odrive.enums import *
import time
import math

config = {
	"wheel_radius" : 0.1,
	"drive_gearing" : 16, # 64 means a gearing of 64 to one
	"flipper_gearing" : 5
}

def full_reset_and_calibrate(odrv0):
	"""Completely resets the Odrive, calibrates axis0 and configures axis0 to only encoder index search on startup and be ready in AXIS_STATE_CLOSED_LOOP_CONTROL"""

	odrv0.erase_configuration()
	print("Erased.")
	try: # Reboot causes loss of connection, use try to supress errors
		odrv0.reboot()
	except:
		pass
	print("Rebooted.")
	odrv0 = odrive.find_any() # Reconnect to the Odrive
	print("Connected.")


	odrv0.axis0.motor.config.pre_calibrated = True # Set all the flags required for pre calibration
	odrv0.axis0.encoder.config.pre_calibrated = True
	odrv0.axis0.encoder.config.use_index = True
	odrv0.axis0.config.startup_encoder_index_search = True # Change startup sequence
	odrv0.axis0.config.startup_closed_loop_control = True
	odrv0.axis0.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE # Calibrate
	print("Started calibration.")
	while odrv0.axis0.current_state != AXIS_STATE_IDLE: # Wait for calibration to be done
		time.sleep(0.1)
	print("Calibration complete.")
	odrv0.save_configuration()
	odrv0.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
	
	return odrv0

def set_position(odrv0, pos):
	odrv0.axis0.controller.config.control_mode = CTRL_MODE_POSITION_CONTROL
	odrv0.axis0.controller.pos_setpoint = pos

def position_test():
	while True:
		set_position(my_drive, 0)
		time.sleep(0.5)
		set_position(my_drive, 4096)
		time.sleep(0.5)
		set_position(my_drive, 0)
		time.sleep(0.5)
		set_position(my_drive, -4096)
		time.sleep(0.5)

def set_rps(odrv0, rps):
	odrv0.axis0.controller.config.control_mode = CTRL_MODE_VELOCITY_CONTROL
	odrv0.axis0.controller.vel_setpoint = rps * 8192

def set_velocity(odrv0, v):
	rps = v * config["drive_gearing"] / (2 * 3.1415 * config["wheel_radius"])
	print(rps * 8192)
	set_rps(odrv0, rps)

# Find a connected ODrive (this will block until you connect one)
print("Finding an odrive...")
my_drive = odrive.find_any()

my_drive = full_reset_and_calibrate(my_drive)
# To read a value, simply read the property
# print("Bus voltage is " + str(my_drive.vbus_voltage) + "V")

my_drive.axis0.controller.config.vel_limit = 500000.0
my_drive.axis0.motor.config.current_lim = 5

print("set drive velocity in m/s or any non-number to quit")
while(True):
	velocity = input()
	try:
		velocity = float(velocity)
	except:
		break
	set_velocity(my_drive, float(velocity))

set_velocity(my_drive, 0)