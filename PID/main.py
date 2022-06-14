import numpy as np


class PID:

    def __init__(self):
        # Todo: testing & change to suitable value
        self.Kp = 100
        self.Ki = 30
        self.Kd = 30
        self.max_output = 1.0
        # self.ITerm_max = 10
        self.delta_time = 0.01  # 100Hz

        # init
        self.target_height = 0.0
        self.PTerm = 0.0
        self.ITerm = 0.0
        self.DTerm = 0.0
        self.last_error = 0.0
        self.int_error = 0.0
        self.output = 0.0

    def clear(self):
        self.target_height = 0.0
        self.PTerm = 0.0
        self.ITerm = 0.0
        self.DTerm = 0.0
        self.last_error = 0.0
        self.int_error = 0.0
        self.output = 0.0

    # Calculate P, I ,D , and output thruster command
    def update(self, feedback_value):
        error = self.target_height - feedback_value
        delta_time = self.delta_time
        delta_error = error - self.last_error

        self.PTerm = self.Kp * error
        self.ITerm += error * delta_time

        """
        if (self.ITerm < -self.ITerm_max):
            self.ITerm = -self.ITerm_max
        elif (self.ITerm > self.ITerm_max):
            self.ITerm = self.ITerm_max
        """

        self.DTerm = delta_error / delta_time

        # Remember last time and last error for next calculation
        self.last_error = error
        self.output = np.clip(self.PTerm + (self.Ki * self.ITerm) + (self.Kd * self.DTerm), -self.max_output,
                              self.max_output)

    def set_target_height(self, goal):
        self.target_height = goal


# Todo: finish loading function for target height & sensor input from kalman filter
def load_z_position():
    return 0


def load_target_height():
    return 100


if __name__ == '__main__':
    z_controller = PID()
    while True:
        z = load_z_position()
        target_height = load_target_height()
        z_controller.set_target_height(target_height)
        z_controller.update(z)
        print(z_controller.output)
