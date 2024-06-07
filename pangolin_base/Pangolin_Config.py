import numpy as np

leg_forward = 200
leg_backward = -300 # test 200
turn_forward = 250
turn_backward = -250


class PangolinConfiguration:
     def __init__(self):
        self.pangolin_height = 95

        self.L = 205
        self.H = self.pangolin_height
        self.B = 90
        self.W = 92

        self.dt = 0.01

        self.leg_center_position = np.array([1535, 2561, 2567, 1525, 2000, 2000, 2000, 2000])

        self.move_forward = 30
        self.move_backward = -30
        self.turn_forward = 30
        self.turn_backward = -30


class PangolinDynamixel:
    def __init__(self):
        self.DEVICE_NAME = "/dev/ttyUSB0"
        self.B_RATE      = 57600
        self.LED_ADDR_LEN = (65,1)
        self.LED_ON = 1
        self.LED_OFF = 0

# class PangolinState:
#     def __init__(self):
#         self.pangolin_height = 95

#         self.L = 205
#         self.H = self.pangolin_height
#         self.B = 90
#         self.W = 92

#         self.dt = 0.01