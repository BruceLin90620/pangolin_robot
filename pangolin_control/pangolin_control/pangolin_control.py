#！usr/bin python3

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from sensor_msgs.msg import Imu

import time
import os, sys, math
import numpy as np

sys.path.append('/home/pangolin/pangolin_ws/src/pangolin_robot/pangolin_base')

from Pangolin_ControlCmd import PangolinControl
from Pangolin_ControlCmd import ControlCmd
from Pangolin_Config import *

class Pangolin(Node):
    def __init__(self):
        super().__init__('pangolin_control')
        self.control_cmd = PangolinControl()

        self.joy_subscriber_ = self.create_subscription(Joy, 'joy', self.joy_callback, 0)
        self.cmd_vel_subscriber_ = self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 1)
        

        # self.timer = self.create_timer(6.0, self.timer_callback)
        self.current_motion_index = 0
        self.motions = [
            ('turn_left', 1.0, 0.0),
            ('move_linear', 0.0, 1.0),
            ('turn_right', -1.0, 0.0),
            ('move_linear', 0.0, 1.0)
            
        ]
        
        self.is_first_time = True
        self.is_random_walk_mode = False
        self.last_joy_msgs_buttons = []



    # destroy ros    
    def destroy(self):
        self.cmd_vel_subscriber_.destroy()
        self.imu_subscriber_.destroy()
        super().destroy_node()

    def joy_callback(self, msg):
        # X:0, []:3, <|:4, O:1
        # L1:6, R1:7
        # L2:5, R2:4 (axes)
        # Select:10, START:11, 
        # arrow x 7, arrow y 6（axes）

        if self.is_first_time == True:
            self.last_joy_msgs_buttons = msg.buttons
            self.is_first_time = False

        if msg.buttons[10] != self.last_joy_msgs_buttons[10]:
            self.is_random_walk_mode = not self.is_random_walk_mode

        self.last_joy_msgs_buttons = msg.buttons
        

    # def cmd_vel_callback(self, msg):
    #     if not self.is_random_walk_mode:
    #         if msg.linear.x <= 1.0 and msg.linear.x > 0.05:
    #             msg.linear.x = 1.0

    #         self.control_cmd.set_velocity(msg)
    #         self.get_logger().info(f'{self.control_cmd.motor_position}')
    #         self.get_logger().info(f'{msg.linear.x}')
    #         self.get_logger().info(f'{msg.angular.z}')

    #         if (round(msg.linear.x, 1) != 0 or round(msg.angular.z, 1) != 0 or round(msg.linear.y, 1)!=0) and msg != None:
    #             if self.control_cmd.is_walking == False :
    #                 self.get_logger().info(f'start_gait')

    #             if round(msg.linear.x, 1) != 0:
    #                 self.get_logger().info(f'cmd_vel linear')
    #                 self.control_cmd.stop_gait()
    #                 self.control_cmd.set_gait_name('move_linear')
    #                 self.control_cmd.start_gait()
                    
    #             elif round(msg.angular.z, 1) < 0:
    #                 self.get_logger().info(f'cmd_vel turn_right')
    #                 self.control_cmd.stop_gait()
    #                 self.control_cmd.set_gait_name('turn_right')
    #                 self.control_cmd.start_gait()

    #             elif round(msg.angular.z, 1) > 0:
    #                 self.get_logger().info(f'cmd_vel turn_left')
    #                 self.get_logger().info(f'{msg}')
    #                 self.control_cmd.stop_gait()
    #                 self.control_cmd.set_gait_name('turn_left')
    #                 self.control_cmd.start_gait()

    #         else:
    #             if self.control_cmd.is_walking == True:
    #                 self.get_logger().info(f'stop')
    #                 self.control_cmd.stop_gait()

    def timer_callback(self):
        
        if not self.is_random_walk_mode:
            # self.get_logger().info(f'self.is_random_walk_mode : {self.is_random_walk_mode}')

            self.control_cmd.stop_gait()

            motion_type, angular_z, linear_x = self.motions[self.current_motion_index]
            msg = Twist()
            msg.angular.z = angular_z
            msg.linear.x = linear_x
            msg.linear.y = 0.0

            self.control_cmd.set_velocity(msg)
            self.control_cmd.set_gait_name(motion_type)
            self.control_cmd.start_gait()


            self.current_motion_index = (self.current_motion_index + 1) % len(self.motions)





    def cmd_vel_callback(self, msg):
        if msg.linear.x <= 1.0 and msg.linear.x > 0.05:
            msg.linear.x = 1.0

        self.control_cmd.set_velocity(msg)
        # self.get_logger().info(f'self.control_cmd.is_walking: {self.control_cmd.is_walking}')
        # self.get_logger().info(f'self.is_random_walk_mode : {self.is_random_walk_mode}')

        if self.is_random_walk_mode:
            if (round(msg.linear.x, 1) != 0 or round(msg.angular.z, 1) != 0) and msg != None:
                if self.control_cmd.is_walking == False:
                    self.control_cmd.start_gait()
                    self.get_logger().info(f'start_gait')

                if round(msg.linear.x, 1) != 0:
                    self.get_logger().info(f'cmd_vel linear')
                    self.control_cmd.set_gait_name('move_linear')
                    
                elif round(msg.angular.z, 1) < 0:
                    self.get_logger().info(f'cmd_vel turn_right')
                    self.control_cmd.set_gait_name('turn_right')

                elif round(msg.angular.z, 1) > 0:
                    self.get_logger().info(f'cmd_vel turn_left')
                    self.control_cmd.set_gait_name('turn_left')

            else:
                if self.control_cmd.is_walking == True:
                    self.get_logger().info(f'stop')
                    self.control_cmd.stop_gait()
            
        


            

def main(args=None):
    rclpy.init(args=args)
    PangolinControl = Pangolin()

    rclpy.spin(PangolinControl)
    
    PangolinControl.destroy()
    rclpy.shutdown()


if __name__ == '__main__':
    main()