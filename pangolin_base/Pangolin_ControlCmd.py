from DXL_motor_control import DXL_Conmunication
import time
import traceback
from time import sleep
import numpy as np

from Pangolin_Config import PangolinConfiguration, PangolinDynamixel
from Pangolin_Gait import gait_dic

class PangolinControl:
    def __init__(self):

        self.control_cmd = ControlCmd()
        self.is_walking = False

        self.motor_origin_pos = np.array[0, 0, 0, 0, 0, 0, 0, 0]

        self.gait_name = 'move_linear'


    def reset_to_orginal(self): 
        pass

    def process_gait(self):
        if self.gait_name == 'move_linear':
            # while True:
            for motor_position in gait_dic[self.gait_name]:
            # if self.is_walking == False: break
                print(motor_position)
                time.sleep(1)
            # self.control_cmd.leg_motor_position_control()

    def angle_to_servo(self, motor_angle: np.array)-> np.array: 
        
        motor_angle * 4095 / 360 * 
        

















class ControlCmd:
    def __init__(self): 

        pangolin_config = PangolinConfiguration()
        pangolin_dynamixel = PangolinDynamixel()

        #Coummunicate the dynamixel motors
        self.dynamixel = DXL_Conmunication(pangolin_dynamixel.DEVICE_NAME, pangolin_dynamixel.B_RATE)
        self.dynamixel.activateDXLConnection()

        #Create the dynamixel motors
        leg_motor_FL = self.dynamixel.createMotor('motor1', motor_number=1)
        leg_motor_FR = self.dynamixel.createMotor('motor2', motor_number=2)
        leg_motor_HL = self.dynamixel.createMotor('motor3', motor_number=3)
        leg_motor_HR = self.dynamixel.createMotor('motor4', motor_number=4)
        head_motor_Y = self.dynamixel.createMotor('motor5', motor_number=5)
        head_motor_P = self.dynamixel.createMotor('motor6', motor_number=6)
        spine_motor_Y = self.dynamixel.createMotor('motor7', motor_number=7)
        spine_motor_P = self.dynamixel.createMotor('motor8', motor_number=8)
 
        #Create the motor list
        self.leg_motor_list     = [leg_motor_FL, leg_motor_FR, leg_motor_HL, leg_motor_HR]
        self.head_motor_list    = [head_motor_Y, head_motor_P]
        self.spine_motor_list   = [spine_motor_Y, spine_motor_P]

        #Reboot and update the state
        self.dynamixel.rebootAllMotor()
        self.dynamixel.updateMotorData()

        #Enalbe the motor torque
        self.enable_all_motor()

        #define the walk frequency
        self.walking_freq = pangolin_config.dt*1000

        #check list
        self.joint_position = []

    def __del__(self):
        self.disable_all_motor()
        
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

        # self.dynamixel.closeHandler()
    
    # Read all the motors data
    def read_all_motor_data(self):
        self.update_joint_state()
        return print(self.joint_position)

    # Update all the motors data
    def update_joint_state(self):
        self.dynamixel.updateMotorData()
        self.joint_position = []
        for motor in self.leg_motor_list:
            self.joint_position.append(motor.PRESENT_POSITION_value)
        for motor in self.head_motor_list:
            self.joint_position.append(motor.PRESENT_POSITION_value)
        for motor in self.spine_motor_list:
            self.joint_position.append(motor.PRESENT_POSITION_value)

    # Control all motors position
    def motor_position_control(self, position = np.array[935, 2560, 2371, 1638, 1958, 2049, 2047, 2008]):
        motor_id = 0
        for motor in self.leg_motor_list:
            motor.writePosition(position[motor_id])
            motor_id += 1
        for motor in self.head_motor_list:
            motor.writePosition(position[motor_id])
            motor_id += 1
        for motor in self.spine_motor_list:
            motor.writePosition(position[motor_id])
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
        "move":pangolin_control.process_gait,
        "read":pangolin_control.control_cmd.read_all_motor_data,
        "pos":pangolin_control.control_cmd.motor_position_control,
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