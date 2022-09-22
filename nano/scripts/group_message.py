#!/usr/bin/env python

import sys
import time
import math
import threading

import rospy
from geometry_msgs.msg import Vector3, Quaternion
from std_msgs.msg import Int16

from pymavlink import mavutil
from pymavlink.quaternion import QuaternionBase # Imports for attitude



def request_message_interval(message_id: int, frequency_hz: float):
    """
    Request MAVLink message in a desired frequency,
    documentation for SET_MESSAGE_INTERVAL:
        https://mavlink.io/en/messages/common.html#MAV_CMD_SET_MESSAGE_INTERVAL

    Args:
        message_id (int): MAVLink message ID
        frequency_hz (float): Desired frequency in Hz
    """
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
        message_id, # The MAVLink message ID
        1e6 / frequency_hz, # The interval between two messages in microseconds. Set to -1 to disable and 0 to request default rate.
        0, 0, 0, 0, # Unused parameters
        0, # Target address of message stream (if message has target address fields). 0: Flight-stack default (recommended), 1: address of requestor, 2: broadcast.
    )

 
def talker():
    pub_acc = rospy.Publisher('/imu/acceleration', Vector3, queue_size=10)
    pub_gyro = rospy.Publisher('/imu/angular_acceleration', Vector3, queue_size=10)
    pub_prs = rospy.Publisher('/pressure', Int16, queue_size=10)
    pub_att = rospy.Publisher('/attitude/angle', Vector3, queue_size=10)
    pub_att_spd = rospy.Publisher('/attitude/angular_speed', Vector3, queue_size=10)
    pub_att_qua = rospy.Publisher('/attitude/quaternion', Quaternion, queue_size=10)
    rospy.init_node('talker_data', anonymous=True, disable_signals=True)
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        msg = master.recv_match()
        if not msg:
            continue
        if msg.get_type() == 'RAW_IMU':
            imu_acc = Vector3()    # mG
            imu_acc.x = msg.xacc
            imu_acc.y = msg.yacc
            imu_acc.z = msg.zacc
            pub_acc.publish(imu_acc)
            imu_gyro = Vector3()    #mrad/s
            imu_gyro.x = msg.xgyro
            imu_gyro.y = msg.ygyro
            imu_gyro.z = msg.zgyro
            pub_gyro.publish(imu_gyro)
        if msg.get_type() == 'SCALED_PRESSURE2':
            prs = msg.press_abs    #hPa
            pub_prs.publish(prs)
        if msg.get_type() == 'ATTITUDE':
            att = Vector3()    #rad
            att.x = msg.roll
            att.y = msg.pitch
            att.z = msg.yaw
            pub_att.publish(att)
            att_spd = Vector3()    #rad/s
            att_spd.x = msg.rollspeed
            att_spd.y = msg.pitchspeed
            att_spd.z = msg.yawspeed
            pub_att_spd.publish(att_spd)
        if msg.get_type() == 'ATTITUDE_QUATERNION':
            q = Quaternion()
            q.x = msg.q1
            q.y = msg.q2
            q.z = msg.q3
            q.w = msg.q4
            pub_att_qua.publish(q)


def pub():
    talker()

def sub():
    while True:
        print('Hi')
        time.sleep(0.5)

# Create the connection
master = mavutil.mavlink_connection("/dev/ttyACM0", baud=115200)
boot_time = time.time()
# Wait a heartbeat before sending commands
master.wait_heartbeat()

# Change message interval
request_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_RAW_IMU, 5)
request_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE_QUATERNION, 5)

t1 = threading.Thread(target=pub, name='t1')
#t2 = threading.Thread(target=sub, name='t2')
t1.start()
#t2.start()
