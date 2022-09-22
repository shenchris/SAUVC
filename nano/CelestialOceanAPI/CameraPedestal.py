from pymavlink import mavutil

class CameraPedestal:

    def __init__(self, master):
        self.master = master
        self.__set_servo_pwm(1, 1500)

    def __set_servo_pwm(self, servo_n, microseconds):
        self.master.mav.command_long_send(
            self.master.target_system, self.master.target_component,
            mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
            0,            # first transmission of this command
            servo_n + 8,  # servo instance, offset by 8 MAIN outputs
            microseconds, # PWM pulse-width
            0,0,0,0,0     # unused parameters
        )


    def setAngle(self, angle):
        """
        Tune camera pedestal servo setAngl
        Range: -90 - 45, 0 means middle/horizontal
        """
        pulseDuration = 500 + 2000/180*(angle+90)
        #print(pulseDuration)
        if pulseDuration >= 1000 and pulseDuration <= 2000:
            self.__set_servo_pwm(1, pulseDuration)
        else:
            raise Exception("Out of Range, Angle: " + str(angle))
