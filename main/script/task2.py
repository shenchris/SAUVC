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
Speed_translation_bottom_CV = 150
Speed_move_forward = 400

# Set
Task2_Camera_topic = ".."
task2_finish = 0


def task1():
    return 1, 2


# based on task1 return, move auv from gate to top_left_corner which should around left and before drum area
def move_to_top_left_corner(data):
    # break when ever detected drum
    while not data.front_detect and not data.bottom_detect:
        # move left base on task1 return
        if (time.time() - task2_start_time) < (Time_to_left_corner - x_second_from_task1):
            send_manual_control(0, -Speed_translation, 500, 0)
        # move forward base on task1 return
        elif (Time_to_left_corner - x_second_from_task1) < time.time() - task2_start_time < (
                (Time_to_left_corner - x_second_from_task1) + (Time_to_top_corner - y_second_from_task1)):
            send_manual_control(Speed_move_forward, 0, 500, 0)


# check detect_flag while moving
def wait_and_check_detect_flag(data, second):
    for i in range(second * 100):
        if data.front_detect or data.bottom_detect:
            break
        else:
            time.sleep(0.01)


# after arrive top left corner, snake_motion to find drum, loop: right -> forward -> left -> forward
def snake_motion(data):
    while not data.front_detect or not data.bottom_detect:
        send_manual_control(0, Speed_translation, 500, 0)
        wait_and_check_detect_flag(data, Time_to_left_corner + Time_to_Right_corner)
        if data.front_detect or data.bottom_detect:
            break
        send_manual_control(Speed_move_forward, 0, 500, 0)
        wait_and_check_detect_flag(data, 3)
        if data.front_detect or data.bottom_detect:
            break
        send_manual_control(0, -Speed_translation, 500, 0)
        wait_and_check_detect_flag(data, Time_to_left_corner + Time_to_Right_corner)
        if data.front_detect or data.bottom_detect:
            break
        send_manual_control(Speed_move_forward, 0, 0, 0)
        wait_and_check_detect_flag(data, 3)
        if data.front_detect or data.bottom_detect:
            break


# after snake_motion terminate by front camera detected
# use front camera to locate and move forward until bottom camera detected
# if both front and bottom detect don't detect, just move forward
def front_to_bottom(data):
    while True:
        if data.bottom_detect:
            break
        elif data.front_detect:
            send_manual_control(0, data.front_y_force, 500, 0)
        else:
            send_manual_control(Speed_move_forward, 0, 500, 0)

def release():

def motion(data):
    move_to_top_left_corner(data)
    snake_motion(data)
    front_to_bottom(data)

    # start when bottom camera detected
    while not task2_finish:
        # first move : right
        losing_detect = False
        while losing_detect < 120:
            if data.bottom_detect:
                losing_detect = 0
                if data.type_of_drum == 1:
                    if data.can_sink:
                        if data.can_release:
                            release()
                        else:
                            send_manual_control(data.bottom_x_force, data.bottom_y_force, 400, 0)
                    else:
                        send_manual_control(data.bottom_x_force, data.bottom_y_force, 500, 0)
                elif data.type_of_drum == 2:
                    send_manual_control(data.bottom_x_force, Speed_translation_bottom_CV, 500, 0)
            elif data.front_detect:  # should not happen
                send_manual_control(Speed_move_forward, data.front_y_force, 500, 0)
            else:
                send_manual_control(Speed_move_forward, Speed_translation_bottom_CV, 500, 0)
                losing_detect += 1

    print("Task2 finished, floating up")
    send_manual_control(0, 0, 1000, 0)
    time.sleep(8)
    master.arducopter_disarm()
    master.motors_disarmed_wait()
    print('Disarmed!')


def task2(x_seconds_from_task1, y_second_from_task1):
    # Create the connection
    global master, task2_start_time, boot_time
    master = mavutil.mavlink_connection("/dev/ttyACM0", baud=115200)
    task2_start_time = time.time()
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
    # x_second_from_task1 > 0 : move left, x_second_from_task1 < 0 : move right
    x_second_from_task1, y_second_from_task1 = task1()
    task2(x_second_from_task1, y_second_from_task1)
