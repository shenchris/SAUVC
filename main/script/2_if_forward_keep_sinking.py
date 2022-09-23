#!/usr/bin/env python3

# Description: program for qualification
# Using SetDepth and P control z_axis when moving
# Don't include CV

import sys
from pymavlink import mavutil
import time
import threading
import math

### hard code variable
Forward_second = 30
Sink_second = 1 # time let auv sink at begining


def send_manual_control(x, y, z, r):
    master.mav.manual_control_send(
        master.target_system,
        x,  # -1000 to 1000, static=0, backward<0, forward>0
        y,  # -1000 to 1000, static=0, left<0, right>0
        z,  # 0 to 1000, static=500, downward<500, upward>500
        r,  # -1000 to 1000, static=0, anti-clockwise<0, clockwise>0
        0  # useless (for other purpose)
    )


def set_target_depth(depth):
    """ Sets the target depth while in depth-hold mode.

    Uses https://mavlink.io/en/messages/common.html#SET_POSITION_TARGET_GLOBAL_INT

    'depth' is technically an altitude, so set as negative meters below the surface
        -> set_target_depth(-1.5) # sets target to 1.5m below the water surface.

    Comment out X_IGNORE and Y_IGNORE for working on ardusub
    """
    master.mav.set_position_target_global_int_send(
        int(1e3 * (time.time() - boot_time)),  # ms since boot
        master.target_system, master.target_component,
        coordinate_frame=mavutil.mavlink.MAV_FRAME_GLOBAL_INT,
        type_mask=(  # ignore everything except z position
            #    mavutil.mavlink.POSITION_TARGET_TYPEMASK_X_IGNORE |
            #    mavutil.mavlink.POSITION_TARGET_TYPEMASK_Y_IGNORE |
            # DON'T mavutil.mavlink.POSITION_TARGET_TYPEMASK_Z_IGNORE |
                mavutil.mavlink.POSITION_TARGET_TYPEMASK_VX_IGNORE |
                mavutil.mavlink.POSITION_TARGET_TYPEMASK_VY_IGNORE |
                mavutil.mavlink.POSITION_TARGET_TYPEMASK_VZ_IGNORE |
                mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE |
                mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE |
                mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE |
                # DON'T mavutil.mavlink.POSITION_TARGET_TYPEMASK_FORCE_SET |
                mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_IGNORE |
                mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE
        ), lat_int=0, lon_int=0, alt=depth,  # (x, y WGS84 frame pos - not used), z [m]
        vx=0, vy=0, vz=0,  # velocities in NED frame [m/s] (not used)
        afx=0, afy=0, afz=0, yaw=0, yaw_rate=0
        # accelerations in NED frame [N], yaw, yaw_rate
        #  (all not supported yet, ignored in GCS Mavlink)
    )


def thread_function():
    global z
    while True:
        try:
            z = master.recv_match(type='SCALED_PRESSURE2', blocking=True).to_dict()["press_abs"]
        except KeyboardInterrupt:
            master.arducopter_disarm()
            master.motors_disarmed_wait()
            print('Disarmed!')


master = mavutil.mavlink_connection("/dev/ttyACM0", baud=115200)
boot_time = time.time()
master.wait_heartbeat()

print("Arm Status: " + str(bool(master.motors_armed())))

master.arducopter_arm()
print("Waiting for the vehicle to arm")
master.motors_armed_wait()
print('Armed!')

mode = 'ALT_HOLD'
mode_id = master.mode_mapping()[mode]
master.set_mode(mode_id)

global z
z = 0
print("Starting Thread")
x = threading.Thread(target=thread_function)
x.start()
time.sleep(0.5)

try:
    print("Sinking into water")
    # set depth
    set_target_depth(-1)
    print("Arm Status: " + str(bool(master.motors_armed())))

    ### Can shorter?
    time.sleep(Sink_second)
    ###

    # recode pressure after sink
    target_height = z

    # forward until pass gate by hard code
    print("Forawrd")
    ### Modify Forward_second by hard code
    for i in range(Forward_second * 10):
        send_manual_control(1000, 0, int((math.floor(z) - target_height) * 3), 0)
        time.sleep(0.1)

    # Dont fking know why but just simply arm again and it will work
    master.arducopter_arm()
    master.motors_armed_wait()
    print("Returning to surface");

    # Goes up
    set_target_depth(0)
    time.sleep(5)

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
