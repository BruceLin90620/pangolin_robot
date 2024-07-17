import numpy as np
from dataclasses import dataclass

class PangolinConfiguration:
     def __init__(self):
        self.pangolin_height = 95

        self.leg_length = 95
        self.L = 205
        self.H = self.pangolin_height
        self.B = 90
        self.W = 92

        self.LF = 150
        self.LR = 70

        self.dt = 0.01

        self.leg_motor_direction = np.array([1, -1, 1, -1]) # 1-2
                                                            # 3-4

        self.leg_center_position = np.array([1400, 2675, 2567, 1517, 2000, 2000, 2074, 2040])


        self.move_forward = 20
        self.move_backward = -20
        self.turn_forward = 20
        self.turn_backward = -20

        self.max_linear_vel = self.leg_length * 0.001 *0.001 * np.sin(np.deg2rad(self.move_forward)) * 2/3 / self.dt #0.00216 m/s
        
class PangolinDynamixel:
    def __init__(self):
        self.DEVICE_NAME = "/dev/ttyUSB0"
        self.B_RATE      = 57600
        self.LED_ADDR_LEN = (65,1)
        self.LED_ON = 1
        self.LED_OFF = 0

@dataclass
class Vector3:
    x: float
    y: float
    z: float

@dataclass
class Vel:
    linear: Vector3
    angular: Vector3

# class PangolinState:
#     def __init__(self):
#         self.pangolin_height = 95

#         self.L = 205
#         self.H = self.pangolin_height
#         self.B = 90
#         self.W = 92

#         self.dt = 0.01

if __name__ == "__main__":
    import traceback
    pangolin_config = PangolinConfiguration()

    command_dict = {
        "spine":pangolin_config.max_linear_vel,
    }


    while True:
        try:
            cmd = input("CMD : ")
            if cmd in command_dict:
                command_dict[cmd]()
            elif cmd == "exit":
                break
        except Exception as e:
            traceback.print_exc()
            break