from Pangolin_Config import Vel, Vector3, PangolinConfiguration
import numpy as np

class PangolinKinematic:

    def __init__(self):
        self.pangolin_config = PangolinConfiguration()
        self.req_vel = Vel(linear= Vector3(x=0.0, y=0.0, z=0.0), angular=Vector3(x=0.0, y=0.0, z=0.0))

        self.head_position  = np.zeros(2)
        self.spine_position = np.zeros(2)
        self.leg_position   = np.zeros(4)

        self.spine_speed = 1
        self.spine_speedMax = 0
        self.spine_value = 0
        


    def calculate_joint(self, gait_name, leg_position, req_vel, is_walking):
        """ Calculates joint angles for legs, head, and spine based on gait type and desired velocity. """

        self.leg_position = leg_position
        self.req_vel = req_vel
        self.gait_name = gait_name
        self.is_walking = is_walking

        self.leg_angle   = np.zeros(4)
        self.head_angle   = np.zeros(2)
        self.spine_angle   = np.zeros(2)

        self.leg_angle = self.leg_controller()
        self.head_angle = self.head_controller()
        self.spine_angle = self.spine_controller()

        return self.leg_angle, self.head_angle, self.spine_angle


    def leg_controller(self):
        """ Calculates leg angles based on the current gait and desired velocity. """
        if self.req_vel.linear.x > 0.1: self.req_vel.linear.x = 1.0
        elif self.req_vel.linear.x < -0.1: self.req_vel.linear.x = -1.0
        
        # print("gait_name",self.gait_name)
        if self.gait_name == 'move_linear':
            leg_angle = np.array(self.leg_position) * self.req_vel.linear.x
            
        elif self.gait_name == 'turn_left':
            if self.req_vel.angular.z > 0.0: self.req_vel.angular.z = 1.0 
            elif self.req_vel.angular.z < -0.0: self.req_vel.angular.z = -1.0 

            leg_angle = np.array(self.leg_position) * -self.req_vel.angular.z

        elif self.gait_name == 'turn_right':
            if self.req_vel.angular.z > 0.0: self.req_vel.angular.z = 1.0
            elif self.req_vel.angular.z < -0.0: self.req_vel.angular.z = -1.0
        
            leg_angle = np.array(self.leg_position) * self.req_vel.angular.z
            # print("leg_angle",leg_angle)
        elif self.gait_name == 'IDLE':
            leg_angle = np.array(self.leg_position)
            
        return leg_angle


    def head_controller(self):
        """ Calculates head angles (currently not implemented). """

        head_angle = np.zeros(2)
        
        return head_angle


    def spine_controller(self):
        """ Calculates spine angles based on the current gait and desired velocity. """

        if self.gait_name == 'move_linear' and self.is_walking == True:            
            if round(self.req_vel.angular.z, 1) == 0:              
                # Update spine_value using spine_speed incrementally
                self.spine_speedMax += self.spine_speed
                
                # Change direction at boundaries 
                if self.spine_speedMax >= 5.0:
                    self.spine_speed = -abs(self.spine_speed)  # Set speed to negative
                    self.spine_speedMax = 0
                    self.spine_value = 16
                elif self.spine_speedMax <= -5.0:
                    self.spine_speed = abs(self.spine_speed)  # Set speed to positive
                    self.spine_speedMax = 0
                    self.spine_value = -16.0
                
                spine_angle = np.array([self.spine_value, 0]) # for cute
                # spine_angle = np.zeros(2)
                # print("spine angle (smooth):", spine_angle)
            else:
                if round(self.req_vel.linear.x, 1) > 0:  # fix the problem that when using keyboard control, key 'M', and key '>' have opposite direction
                    self.req_vel.angular.z = self.req_vel.angular.z
                else:
                    self.req_vel.angular.z = - self.req_vel.angular.z

                # Apply multiplier to angular.z for spine angle
                # Increment or decrement based on angular.z direction, up to max of 28 or -28
                
                if round(self.req_vel.angular.z, 1) < 0:
                    self.spine_value = 28 #30
                    # self.spine_value = min(self.spine_value + 1, 20) #30
                else:
                    self.spine_value = -28 #30
                    # self.spine_value = max(self.spine_value - 1, -20) #30
                
                spine_angle = np.array([self.spine_value, 0])
                # print("spine angle (incremental):", spine_angle)
        else:
            spine_angle = np.zeros(2)

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
