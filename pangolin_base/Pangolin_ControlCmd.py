from DXL_motor_control import DXL_Communication
from Pangolin_Config import PangolinConfiguration, PangolinDynamixel, Vel, Vector3
from Pangolin_Gait import PangolinGait
from Pangolin_Kinematic import PangolinKinematic

import numpy as np
import time
import traceback
import atexit
import threading

DEGREE_TO_SERVO = 4095/360

class PangolinControl:
    """High-level control of the Pangolin robot."""
    def __init__(self):

        self.req_vel = Vel(linear= Vector3(x=0.0, y=0.0, z=0.0), angular=Vector3(x=0.0, y=0.0, z=0.0))
        self.control_cmd = ControlCmd()
        self.pangolin_config = PangolinConfiguration()
        self.pangolin_gait = PangolinGait()
        self.pangolin_kinematic = PangolinKinematic()

        self.is_walking = False # Flag to indicate if the robot is walking

        self.leg_center_position = self.pangolin_config.leg_center_position

        self.motor_position = np.zeros(8) 

        self.gait_name = 'move_linear'


    def __del__(self): # Called when object is deleted
        self.stop_gait()

    
    def cleanup(self): # Ensures both high and low-level objects are cleaned up
        self.__del__()
        self.control_cmd.cleanup() 


    def reset_to_orginal(self): 
        """Resets the robot to its default pose."""

        self.angle_to_servo(np.zeros(4), np.zeros(2), np.zeros(2))
        self.control_cmd.motor_position_control(self.motor_position)

    def process_gait(self):
        """Executes the selected gait pattern."""

        gait_num = 0
        while self.is_walking == True:
            # print("walking")
            gait = self.pangolin_gait.gait_dic[self.gait_name]

            if gait_num >= len(gait): gait_num = 0
            leg_gait_position = gait[gait_num]
            gait_num += 1
            leg_angle, head_angle, spine_angle = self.pangolin_kinematic.calculate_joint(self.gait_name, leg_gait_position, self.req_vel, self.is_walking)
            
            motor_position = self.angle_to_servo(leg_angle, head_angle, spine_angle)
            #print(motor_position)
            self.control_cmd.motor_position_control(motor_position)

    def angle_to_servo(self, leg_motor_angle: np.array, head_motor_angle: np.array, spine_motor_angle: np.array)-> np.array: 
        """Converts desired joint angles into raw motor positions."""

        leg_motor_pos = leg_motor_angle*DEGREE_TO_SERVO*np.transpose(self.pangolin_config.leg_motor_direction) + self.leg_center_position[:4] #TODO　4095/360 to constant

        self.motor_position[:4] = leg_motor_pos

        head_motor_pos = head_motor_angle*DEGREE_TO_SERVO + self.leg_center_position[4:6]
        self.motor_position[4:6] = head_motor_pos

        spine_motor_pos = spine_motor_angle*DEGREE_TO_SERVO + self.leg_center_position[6:8]
        self.motor_position[6:8] = spine_motor_pos

        return self.motor_position

    def start_walking(self, gait_name='move_linear'):
        self.stop_gait()
        self.set_gait_name(gait_name)
        self.start_gait()

    def start_gait(self):
        """Starts the gait in a thread."""

        self.is_walking = True
        self.is_turning = True

        self.walking_thread = threading.Thread(target=self.process_gait, args=(), daemon=True)
        self.walking_thread.start()

    # Stop moving 
    def stop_gait(self):
        """Stops the gait and resets to the original position."""

        self.is_walking = False
        self.is_turning = False

        self.reset_to_orginal()

    def set_gait_name(self, gait_name='move_linear'):
        """Selects the gait pattern to use."""

        self.gait_name = gait_name
        
    def set_velocity(self, cmd_vel):
        self.req_vel.linear.x  = cmd_vel.linear.x
        self.req_vel.angular.z = cmd_vel.angular.z

    def set_head_position(self):
        pass

    def set_spine_position(self):
        pass

class ControlCmd:
    """Low-level control of Dynamixel motors."""
    def __init__(self): 

        pangolin_config = PangolinConfiguration()
        pangolin_dynamixel = PangolinDynamixel()

        #Coummunicate the dynamixel motors
        self.dynamixel = DXL_Communication(pangolin_dynamixel.DEVICE_NAME, pangolin_dynamixel.B_RATE)
        self.dynamixel.activateDXLConnection()

        #Create the dynamixel motors
        leg_motor_FL  = self.dynamixel.createMotor('motor1', motor_number=1)
        leg_motor_FR  = self.dynamixel.createMotor('motor2', motor_number=2)
        leg_motor_HL  = self.dynamixel.createMotor('motor3', motor_number=3)
        leg_motor_HR  = self.dynamixel.createMotor('motor4', motor_number=4)
        head_motor_Y  = self.dynamixel.createMotor('motor5', motor_number=5)
        head_motor_P  = self.dynamixel.createMotor('motor6', motor_number=6)
        spine_motor_Y = self.dynamixel.createMotor('motor7', motor_number=7)
 
        #Create the motor list
        self.leg_motor_list     = [leg_motor_FL, leg_motor_FR, leg_motor_HL, leg_motor_HR]
        self.spine_motor_list   = [spine_motor_Y]
        self.head_motor_list    = [head_motor_Y, head_motor_P]

        #Reboot and update the state
        self.dynamixel.rebootAllMotor()
        self.dynamixel.updateMotorData()

        #Enalbe the motor torque
        self.enable_all_motor()

        #define the walk frequency
        self.walking_freq = pangolin_config.dt*1000

        #check list
        self.joint_position = np.zeros(8)

    def __del__(self): # Disable motors on object deletion
        self.disable_all_motor()

    # Extends the cleanup process to close the communication handler
    def cleanup(self): 
        self.__del__()
        
    def enable_all_motor(self):
        for motor in self.leg_motor_list:
            motor.enableMotor()
        for motor in self.head_motor_list:
            motor.enableMotor()
            motor.directWriteData(50, 112, 4)
        for motor in self.spine_motor_list:
            motor.enableMotor()
            motor.directWriteData(50, 112, 4) # set motor speed
            
    def disable_all_motor(self):
        for motor in self.leg_motor_list:
            motor.disableMotor()
        for motor in self.head_motor_list:
            motor.disableMotor()
        for motor in self.spine_motor_list:
            motor.disableMotor()

        self.dynamixel.closeHandler()
    
    # Read all the motors data
    def read_all_motor_data(self):
        self.update_joint_state()
        return print(self.joint_position)

    # Update all the motors data
    def update_joint_state(self):
        motor_id = 0
        self.dynamixel.updateMotorData()
        for motor in self.leg_motor_list:
            self.joint_position[motor_id] = motor.PRESENT_POSITION_value
            motor_id += 1
        for motor in self.head_motor_list:
            self.joint_position[motor_id] = motor.PRESENT_POSITION_value
            motor_id += 1
        for motor in self.spine_motor_list:
            self.joint_position[motor_id] = motor.PRESENT_POSITION_value
            motor_id += 1

    # Control all motors position
    def motor_position_control(self, position: np.array):
        motor_id = 0
        for motor in self.leg_motor_list:
            motor.writePosition(int(position[motor_id]))
            motor_id += 1
        for motor in self.head_motor_list:
            motor.writePosition(int(position[motor_id]))
            motor_id += 1
        for motor in self.spine_motor_list:
            motor.writePosition(int(position[motor_id]))
            motor_id += 1

        self.dynamixel.sentAllCmd()
        time.sleep(1 / self.walking_freq)





if __name__ == "__main__":
    pangolin_control = PangolinControl()
    # command_dict = {
    #     "enable":pangolin_control.control_cmd.enable_all_motor,
    #     "record":pangolin_control.start_record_action_points,
    #     "stop":pangolin_control.stop_record_action_points,
    #     "replay":pangolin_control.replay_recorded_data,
    #     "disable":pangolin_control.control_cmd.disable_all_motor,
    #     "read":pangolin_control.control_cmd.read_all_motor_data,
    #     "pos":pangolin_control.control_cmd.leg_motor_position_control,
    #     # "led":pangolin_control.start_led_blink,
    #     "curl":pangolin_control.run_action_curl,
    #     "getdown_right":pangolin_control.run_action_get_down_right,
    #     "getdown_left":pangolin_control.run_action_get_down_left,
    #     "standup_right":pangolin_control.run_action_stand_up_from_right,
    #     "standup_left":pangolin_control.run_action_stand_up_from_left,
    #     "reset":pangolin_control.reset_to_orginal,
    #     "stance":pangolin_control.stance_control,
    # }
    command_dict = {
        "stop":pangolin_control.stop_gait,
        "move":pangolin_control.start_gait,
        "read":pangolin_control.control_cmd.read_all_motor_data,
        "pos":pangolin_control.control_cmd.motor_position_control,
        "reset":pangolin_control.reset_to_orginal,
    }

    atexit.register(pangolin_control.cleanup)

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

        # finally:
        #     atexit._run_exitfuncs() 