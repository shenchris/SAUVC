#!/usr/bin/env python
import rospy
from std_msgs.msg import String
import time
import math
import numpy as np
from pymavlink import mavutil
from pymavlink.quaternion import QuaternionBase  # Imports for attitude
import threading

# Need 9/24 pool test
Time_to_left_corner = 5
Time_to_Right_corner = 5
Time_to_top_corner = 5
# pool test
Speed_translation = 400
Speed_move_forward = 400

# Set
Task2_Camera_topic = ".."
moving_right = True
task2_finish = False


def task1():
    return 1, 2


# based on task1 return
def move_to_top_left_corner(data):
    while not data.detect:
        if time.time() - boot_time < (Time_to_left_corner - seconds_from_task1):
            send_manual_control(0, -Speed_translation, 0, 0)
        elif (Time_to_left_corner - seconds_from_task1) < time.time() - boot_time < (
                Time_to_left_corner - seconds_from_task1 + Time_to_top_corner):
            send_manual_control(Speed_move_forward, 0, 0, 0)


def wait_and_check_detect_flag(data, second):
    for i in range(second*1000):
        if data.detect:
            break
        else:
            time.sleep(0.001)


def meandeling(data):
    while not data.detect:
        send_manual_control(0,400,0,0)
        wait_and_check_detect_flag()
        if data.detect:
            break
        send_manual_control(400,0,0,0)
        wait_and_check_detect_flag()
        if data.detect:
            break
        send_manual_control(0,-400,0,0)
        wait_and_check_detect_flag()
        if data.detect:
            break
        send_manual_control(400,0,0,0)
        wait_and_check_detect_flag()
        if data.detect:
            break



# data.detected
# data.detecting

def motion(data):
    move_to_top_left_corner(data)
    meandeling(data)
    while not task2_finish:
        while data.detected or data.detecting:
            if data.front_detect:
                send_manual_control(Speed_move_forward, data.force, 0, 0)
                data.detected = True
            elif data.bottom_detect:
                data.detected = True
        else:



def task2(x_seconds_from_task1, y_second_from_task1):
    # Create the connectionc
    task2_start_time = time.time()
    global master, boot_time
    master = mavutil.mavlink_connection("/dev/ttyACM0", baud=115200)
    boot_time = time.time()
    # Wait a heartbeat before sending commands
    master.wait_heartbeat()

    # Arm
    master.arducopter_arm()
    print("Waiting for the vehicle to arm")
    master.motors_armed_wait()
    print('Armed!')

    # Choose a mode
    mode = 'ALT_HOLD'
    mode_id = master.mode_mapping()[mode]
    master.set_mode(mode_id)

    try:
        # hold altitude(depth)
        set_target_depth(-0.5)
        # may need to dive by our self if set_target_depth not working

        msg = master.recv_match()
        if msg.get_type() == 'ATTITUDE':
            # hold angle base on startup
            set_target_attitude(msg.roll, msg.pitch, msg.yaw)
            print(f'set_target_attitude({msg.roll}, {msg.pitch}, {msg.yaw})')


    except KeyboardInterrupt:
        # Disarm
        time.sleep(3)
        master.arducopter_disarm()
        print("Waiting for the vehicle to disarm")
        # Wait for disarm
        master.motors_disarmed_wait()
        print('Disarmed!')

    listener_to_task2_camara()


def listener_to_task2_camara():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber(Task2_Camera_topic, String, motion)
    rospy.spin()


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
    master.mav.set_position_target_global_int_send(
        int(1e3 * (time.time() - boot_time)),  # ms since boot
        master.target_system, master.target_component,
        coordinate_frame=mavutil.mavlink.MAV_FRAME_GLOBAL_INT,
        type_mask=(  # ignore everything except z position
                mavutil.mavlink.POSITION_TARGET_TYPEMASK_X_IGNORE |
                mavutil.mavlink.POSITION_TARGET_TYPEMASK_Y_IGNORE |
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


def set_target_attitude(roll, pitch, yaw):
    master.mav.set_attitude_target_send(
        int(1e3 * (time.time() - boot_time)),  # ms since boot
        master.target_system, master.target_component,
        # allow throttle to be controlled by depth_hold mode
        mavutil.mavlink.ATTITUDE_TARGET_TYPEMASK_THROTTLE_IGNORE,
        # -> attitude quaternion (w, x, y, z | zero-rotation is 1, 0, 0, 0)
        QuaternionBase([math.radians(angle) for angle in (roll, pitch, yaw)]),
        0, 0, 0, 0  # roll rate, pitch rate, yaw rate, thrust
    )


if __name__ == '__main__':
    seconds_from_task1 = task1()
    task2(seconds_from_task1)  # seconds > 0 : move right, seconds < 0 : move left
