from Pangolin_ControlCmd import ControlCmd, PangolinControl

#TODo: input 
class PangolinInverseKinematics:
    def __init__(self):
        self.control_cmd = ControlCmd()
        self.pangolin_control = PangolinControl()

    def inverse_kinematic(self, leg_pos, motor_name):
        if motor_name == 'motor1':
            leg_side = 0
            direction = 1
        elif motor_name == 'motor2':
            leg_side = 1
            direction = -1
        elif motor_name == 'motor4':
            leg_side = 0
            direction = 1
        elif motor_name == 'motor5':
            leg_side = 1
            direction = -1

        return int(direction*leg_pos*self.pangolin_control.servo_rate[leg_side]+self.pangolin_control.motor_center_position[motor_name])


if __name__ == '__main__':
    pass