'''
Run python Single_ODrive_MoveRobot.py -mode calibration  for calibration
RUn python Single_ODrive_MoveRobot.py					 for no calibration
'''

from __future__ import print_function, division

import odrive
from odrive.enums import *
import time
import math
import argparse
from tkinter import *
from pynput import keyboard
testing = False # testing variable
command = ["Stopped", 0]
incrament = 0.5


config = {
	"wheel_radius" : 0.1,
	"drive_gearing" : 16, # 64 means a gearing of 64 to one
	"flipper_gearing" : 5
}
def on_press(key):
	try:
		character = key.char

		if character == "a":
			command[0] = "Left"
		elif character == "w":
			command[0] = "Forward"
		elif character == "s":
			command[0] = "Reverse"
		elif character == "d":
			command[0] = "Right"
		
	except AttributeError:
		if key == keyboard.Key.up:
			command[1] += incrament
		elif key == keyboard.Key.down:
			command[1] -= incrament


def on_release(key):
	try: 
		key.char
		command[0] = "Stopped"

	except AttributeError:
		return 0

def parse_args():
	parser = argparse.ArgumentParser('Settings for ODrive')

	parser.add_argument(
		'-mode', default = 'no calibration', help='parsing in calibration will do the full reset and calibrate stage'
	)

	return parser.parse_args()


def determineSpeed():
	state = command[0]
	absSpeed = round(command[1],2)
	if absSpeed > 100:
		absSpeed = 100

	speedSetting = [absSpeed,absSpeed]
	realSpeed = [0,0]

	if state == "Left":
		speedSetting = [-1*absSpeed, absSpeed]
	elif state == "Forward":
		speedSetting = [absSpeed, absSpeed]
	elif state == "Reverse":
		speedSetting = [-1*absSpeed, -1*absSpeed]
	elif state == "Right":
		speedSetting = [absSpeed, -1*absSpeed]

	if state != "Stopped":
		realSpeed = speedSetting

	return speedSetting, realSpeed
		

#### ALL ODRIVE COMMANDS FROM HERE ####
def full_reset_and_calibrate(odrv0):
	"""Completely resets the Odrive, calibrates axis0 and configures axis0 to only encoder index search on startup and be ready in AXIS_STATE_CLOSED_LOOP_CONTROL"""
	odrv0.erase_configuration()
	print("Erased [1/7]")
	try: # Reboot causes loss of connection, use try to supress errors
		odrv0.reboot()
	except:
		pass
	print("Rebooted [2/7]")
	odrv0 = odrive.find_any() # Reconnect to the Odrive
	print("Connected [3/7]")
	odrv0.axis0.motor.config.pre_calibrated = True # Set all the flags required for pre calibration
	odrv0.axis0.encoder.config.pre_calibrated = True
	odrv0.axis0.encoder.config.use_index = True
	odrv0.axis0.config.startup_encoder_index_search = True # Change startup sequence
	odrv0.axis0.config.startup_closed_loop_control = True
	odrv0.axis0.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE # Calibrate
	print("Started calibration 1 [4/7]", end="")
	while odrv0.axis0.current_state != AXIS_STATE_IDLE: # Wait for calibration to be done
		time.sleep(0.1)
		print(".", end="")
	odrv0.save_configuration()

	print("\nCalibration 1 complete [5/7]")
	print("now will begin calibration sequence for second axis")
	time.sleep(3)
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

	#closed loop control for both axis
	odrv0.save_configuration()
	odrv0.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
	odrv0.axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL

	return odrv0


def set_rps(axis, rps):
	axis.controller.config.control_mode = CTRL_MODE_VELOCITY_CONTROL
	axis.controller.vel_setpoint = rps * 8192

def set_velocity(axis, v):
	rps = v * config["drive_gearing"] / (2 * 3.1415 * config["wheel_radius"])
	print(rps * 8192)
	set_rps(axis, rps)
	axis.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL


args = parse_args()
#check if the setting parsed is correct
if args.mode != 'calibration':
	print("Will begin process without calibration")
else:
	print("Execute full reset and calibrate sequence")

print("\nDo you want to continue? Press enter if so")
input()


app = Tk()
app.title("Velocity control")
app.geometry("500x200")
Direction = Label(app, text = "Direction = Stopped")
Direction.pack(side = TOP)
CurrentSpeed = Label(app, text = "Current speed = left: 0  right: 0")
CurrentSpeed.pack(side = TOP)
Speed = Label(app, text = "Speed setting = left: 0  right: 0")
Speed.pack(side = TOP)

if args.mode != 'control_test':
	# Find a connected ODrive (this will block until you connect one)
	print("Finding an odrive...")
	my_drive = odrive.find_any()

	if args.mode == 'calibration':
		my_drive = full_reset_and_calibrate(my_drive)
	# To read a value, simply read the property
	# print("Bus voltage is " + str(my_drive.vbus_voltage) + "V")

	my_drive.axis0.controller.config.vel_limit = 500000.0
	my_drive.axis0.motor.config.current_lim = 50

	my_drive.axis1.controller.config.vel_limit = 500000.0
	my_drive.axis1.motor.config.current_lim = 50

else: # if mode == control test
	testing = True

listener = keyboard.Listener(
    on_press=on_press,
    on_release=on_release)
listener.start()



# control sequence
while 1:
	speed = determineSpeed()
	Direction.config(text = "Direction = " + command[0])
	CurrentSpeed.config(text = "Current speed = left: " + str(speed[1][0]) + "right: " + str(speed[1][1]))
	Speed.config(text = "Spped setting = left: " + str(speed[0][0]) + "right: " + str(speed[0][1]))
	app.update()

	maxrps = 0.5
	if not testing:
		
		set_velocity(my_drive.axis0, speed[1][0]*maxrps*0.01)
		set_velocity(my_drive.axis1, -1*speed[1][1]*maxrps*0.01)