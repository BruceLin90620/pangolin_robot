from Pangolin_Config import Vel, Vector3, PangolinConfiguration
import numpy as np



class PangolinKinematic:

    def __init__(self):
        self.pangolin_config = PangolinConfiguration()
        self.req_vel = Vel(linear= Vector3(x=0.0, y=0.0, z=0.0), angular=Vector3(x=0.0, y=0.0, z=0.0))

        self.head_position  = np.zeros(2)
        self.spine_position = np.zeros(2)
        self.leg_position   = np.zeros(4)


    def calculate_joint(self, gait_name, leg_position, req_vel):
        """ Calculates joint angles for legs, head, and spine based on gait type and desired velocity. """

        self.leg_position = leg_position
        self.req_vel = req_vel
        self.gait_name = gait_name

        self.leg_angle = self.leg_controller()
        self.head_angle = self.head_controller()
        self.spine_angle = self.spine_controller()

        return self.leg_angle, self.head_angle, self.spine_angle


    def leg_controller(self):
        """ Calculates leg angles based on the current gait and desired velocity. """

        if self.gait_name == 'move_linear':
            leg_angle = np.array(self.leg_position) * self.req_vel.linear.x / self.pangolin_config.max_linear_vel

        elif self.gait_name == 'turn_left':
            leg_angle = np.array(self.leg_position) * -self.req_vel.angular.z

        elif self.gait_name == 'turn_right':
            leg_angle = np.array(self.leg_position) * self.req_vel.angular.z

        return leg_angle


    def head_controller(self):
        """ Calculates head angles (currently not implemented). """

        head_angle = np.zeros(2)

        return head_angle


    def spine_controller(self):
        """ Calculates spine angles based on the current gait and desired velocity. """

        # self.gait_name = 'move_linear'
        # self.req_vel.linear.x = 10.0
        # self.req_vel.angular.z = 5.0

        if self.gait_name == 'move_linear':
            spine_angle = np.array([41.0, 0]) * -self.req_vel.angular.z

            # r = self.req_vel.linear.x / self.req_vel.angular.z

            # spine_angle = np.arctan2(self.pangolin_config.LR, r) + np.arctan2(self.pangolin_config.LF, r)
        
        else:
            spine_angle = np.zeros(2)

        # print(spine_angle)
        return spine_angle
    

if __name__ == "__main__":
    import traceback
    pangolin_kinematic = PangolinKinematic()

    command_dict = {
        "spine":pangolin_kinematic.spine_controller,
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
