#!/usr/bin/env python

"""
stablize mode only
"""

import sys
from pymavlink import mavutil
import time

def send_manual_control(x,y,z,r):
    master.mav.manual_control_send(
        master.target_system,
        x,y,z,r,0
    )


### Start program ###

# Create the connection
master = mavutil.mavlink_connection("/dev/ttyACM0", baud=115200)    
# Wait a heartbeat before sending commands
master.wait_heartbeat()

# Arm
master.arducopter_arm()
print("Waiting for the vehicle to arm")
# Wait to arm
master.motors_armed_wait()
print('Armed!')

# Choose a mode
mode = 'STABILIZE'
mode_id = master.mode_mapping()[mode]
master.set_mode(mode_id)

try:
    # stablize
    time.sleep(1)
    print('1')
    send_manual_control(0,0,400,0)
    time.sleep(0.6)
    print('2')
    send_manual_control(0,0,500,0)
    time.sleep(30)

    # Disarm
    master.arducopter_disarm()
    print("Waiting for the vehicle to disarm")
    # Wait for disarm
    master.motors_disarmed_wait()
    print('Disarmed!')

except KeyboardInterrupt:
    # Disarm
    master.arducopter_disarm()
    print("Waiting for the vehicle to disarm")
    # Wait for disarm
    master.motors_disarmed_wait()
    print('Disarmed!')
