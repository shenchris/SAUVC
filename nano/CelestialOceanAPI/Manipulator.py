import time

from pymavlink import mavutil

#500 - 2500
class Manipulator:

    def __init__(self, master):
        self.master = master
        self.__set_servo_pwm(2, 1500)
        self.pwmStatus = 1500

    def __set_servo_pwm(self, servo_n, microseconds):
        self.master.mav.command_long_send(
            self.master.target_system, self.master.target_component,
            mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
            0,            # first transmission of this command
            servo_n + 8,  # servo instance, offset by 8 MAIN outputs
            microseconds, # PWM pulse-width
            0,0,0,0,0     # unused parameters
        )

    def grab(self):
        if self.pwmStatus > 1200:
            for pulseDuration in range(self.pwmStatus, 1200, -10):
                self.__set_servo_pwm(2, pulseDuration)
                time.sleep(0.125)
            self.pwmStatus = 1200
        else:
            raise Exception("Already in Grab state")

    def release(self):
        if self.pwmStatus < 1800:
            for pulseDuration in range(self.pwmStatus, 1800, 10):
                self.__set_servo_pwm(2, pulseDuration)
                time.sleep(0.125)
            self.pwmStatus = 1800
        else:
            raise Exception("Already in Release state")
