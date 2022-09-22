#!/usr/bin/env python

"""
altitude hold mode only
"""

import time
import sys
import math
import threading

import control

from pymavlink import mavutil

### Start program ###

# Create the connectionc
master = mavutil.mavlink_connection("/dev/ttyACM0", baud=115200)
boot_time = time.time()
# Wait a heartbeat before sending commands
master.wait_heartbeat()

control = control.Control(master, boot_time)

# Arm
master.arducopter_arm()
print("Waiting for the vehicle to arm")
master.motors_armed_wait()
print('Armed!')

# Choose a mode
mode = 'MANUAL'
mode_id = master.mode_mapping()[mode]
master.set_mode(mode_id)

try:

    for i in range(5):
        control.send_manual_control(200,0,500,0)

    # Disarm
    time.sleep(3)   # Wait 3 sec to disarm
    master.arducopter_disarm()
    print("Waiting for the vehicle to disarm")
    # Wait for disarm
    master.motors_disarmed_wait()
    print('Disarmed!')

except KeyboardInterrupt:
    # Disarm
    time.sleep(3)
    master.arducopter_disarm()
    print("Waiting for the vehicle to disarm")
    # Wait for disarm
    master.motors_disarmed_wait()
    print('Disarmed!')
