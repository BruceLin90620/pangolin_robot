import numpy as np

# main control command
DEVICE_NAME = "/dev/ttyUSB0"
B_RATE      = 57600
LED_ADDR_LEN = (65,1)
LED_ON = 1
LED_OFF = 0

leg_forward = 200
leg_backward = -300 # test 200
turn_forward = 250
turn_backward = -250


class MotorConfig:
    def __init__(self):
        self.head_motor = []

class PangolinConfiguration:
     def __init__(self):
        self.pangolin_height = 95

        self.L = 205
        self.H = self.pangolin_height
        self.B = 90
        self.W = 92

        self.dt = 0.01

        self.leg_center_position = np.array([1535, 2561, 2567, 1525])

class PangolinState:
     def __init__(self):
        self.pangolin_height = 95

        self.L = 205
        self.H = self.pangolin_height
        self.B = 90
        self.W = 92

        self.dt = 0.01

class PangolinDynamixel:
    def __init__(self):
        self.DEVICE_NAME = "/dev/ttyUSB0"
        self.B_RATE      = 57600
        self.LED_ADDR_LEN = (65,1)
        self.LED_ON = 1
        self.LED_OFF = 0