import queue
import time
import math

import rospy
from geometry_msgs.msg import Vector3, Quaternion
from std_msgs.msg import Int16

from pymavlink import mavutil

class Messages():
    def __init__(self, master):
        self.master = master

    def request_message_interval(self, message_id: int, frequency_hz: float):
        """
        Request MAVLink message in a desired frequency,
        documentation for SET_MESSAGE_INTERVAL:
            https://mavlink.io/en/messages/common.html#MAV_CMD_SET_MESSAGE_INTERVAL

        Args:
            message_id (int): MAVLink message ID
            frequency_hz (float): Desired frequency in Hz
        """
        self.master.mav.command_long_send(
            self.master.target_system, self.master.target_component,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
            message_id, # The MAVLink message ID
            1e6 / frequency_hz, # The interval between two messages in microseconds. Set to -1 to disable and 0 to request default rate.
            0, 0, 0, 0, # Unused parameters
            0, # Target address of message stream (if message has target address fields). 0: Flight-stack default (recommended), 1: address of requestor, 2: broadcast.
        )

    def publish(self):
        def talker(master_):
            pub_acc = rospy.Publisher('/imu/acceleration', Vector3, queue_size=10)
            pub_gyro = rospy.Publisher('/imu/angular_acceleration', Vector3, queue_size=10)
            pub_prs = rospy.Publisher('/pressure', Int16, queue_size=10)
            pub_att = rospy.Publisher('/attitude/angle', Vector3, queue_size=10)
            pub_att_spd = rospy.Publisher('/attitude/angular_speed', Vector3, queue_size=10)
            pub_att_qua = rospy.Publisher('/attitude/quaternion', Quaternion, queue_size=10)
            rospy.init_node('talker_data', anonymous=True, disable_signals=True)
            rate = rospy.Rate(10)

            while not rospy.is_shutdown():
                msg = master_.recv_match()
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
                if msg.get_type() == 'RAW_PRESSURE':
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
        
        try:
            talker(self.master)
        except rospy.ROSInterruptException:
            pass
