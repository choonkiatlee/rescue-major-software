#!/usr/bin/env python

import os
import subprocess
import threading
import time
import math


def run_update():
    logfile = open("log/" + str(log_id) + "update.txt", 'a+')
    global update
    catkin_ws_path = os.path.expanduser("~/rescue-major-software/catkin_ws")

    update = subprocess.Popen("catkin_make", cwd=catkin_ws_path, shell=True, stdout=logfile)
    update.wait()


def run_roscore():
    logfile = open("log/" + str(log_id) + "roscore.txt", 'a+')
    global roscore

    roscore = subprocess.Popen("killall -9 roscore", shell=True, stdout=logfile)
    roscore.wait()

    roscore = subprocess.Popen("killall -9 rosmaster", shell=True, stdout=logfile)
    roscore.wait()

    roscore = subprocess.Popen("roscore", shell=True, stdout=logfile)


def run_nuc_main():
    logfile = open("log/" + str(log_id) + "nuc_main.txt", 'a+')
    global nuc_main
    catkin_ws_path = os.path.expanduser("~/rescue-major-software/catkin_ws")

    # nuc_main = subprocess.Popen(". " + os.path.expanduser("~/rescue-major-software/catkin_ws/devel/setup.bash"),
    #                             cwd=catkin_ws_path, shell=True, stdout=logfile)
    # nuc_main.wait()

    nuc_main = subprocess.Popen("rosrun nuc_main nuc_main.py", cwd=catkin_ws_path, shell=True, stdout=logfile)


def run_xbox():
    logfile = open("log/" + str(log_id) + "xbox.txt", 'a+')
    global xbox
    catkin_ws_path = os.path.expanduser("~/rescue-major-software/catkin_ws")

    # nuc_main = subprocess.Popen(". " + os.path.expanduser("~/rescue-major-software/catkin_ws/devel/setup.bash"),
    #                             cwd=catkin_ws_path, shell=True, stdout=logfile)
    # nuc_main.wait()

    xbox = subprocess.Popen("rosrun xbox_controller xbox_controller.py",
                                cwd=catkin_ws_path, shell=True, stdout=logfile)


if not os.path.exists("log/"):
    os.mkdir("log/")

log_id = int(math.ceil((len(os.listdir("log/")) - 1) / 4.0))

run_roscore()
run_nuc_main()
run_xbox()

print("init done")

while True:
    time.sleep(0.1)
