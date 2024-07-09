#！usr/bin python3

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from sensor_msgs.msg import Imu

import time
import os, sys, math
import numpy as np

sys.path.append('/home/pangolin/pangolin_ws/pangolin_robot/pangolin_base')

from Pangolin_ControlCmd import PangolinControl
from Pangolin_ControlCmd import ControlCmd
from Pangolin_Config import *

class Pangolin(Node):
    def __init__(self):
        super().__init__('pangolin_control')
        self.control_cmd = PangolinControl()

        self.joy_subscriber_ = self.create_subscription(Joy, 'joy', self.joy_callback, 0)
        self.cmd_vel_subscriber_ = self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 1)
        
        self.is_first_time = True
        self.is_disalbe_motor = False
        self.is_freedom_mode = False
        self.is_stance_mode = False
        self.is_record_mode = False
        self.is_sit_mode = False
        self.is_curl = False
        self.last_joy_msgs_buttons = []
        self.time_1 = 0

        #imu
        self.pitch = None
        self.roll = None
        self.yaw = None

    # destroy ros    
    def destroy(self):
        self.cmd_vel_subscriber_.destroy()
        self.imu_subscriber_.destroy()
        super().destroy_node()

    
    def joy_callback(self, msg):
        # X:0, A:1, B:2, Y:3
        # LB:4, RB:5, LT:6, RT:7
        # BACK:8, START:9, 
        # L3:10, R3:11  

        if self.is_first_time == True:
            self.last_joy_msgs_buttons = msg.buttons
            self.is_first_time = False

        self.last_joy_msgs_buttons = msg.buttons
        # self.get_logger().info(f'pitch: {self.pitch}')
        # self.get_logger().info(f'roll: {self.roll}')
        # self.get_logger().info(f'yaw: {self.yaw}')




# Pangolin cmd_vel callback
    def cmd_vel_callback(self, msg):
        # if msg.linear.x <= 1.0 and msg.linear.x > 0.05:
        #     msg.linear.x = 1.0


        self.control_cmd.set_velocity(msg)

        if (round(msg.linear.x, 1) != 0 or round(msg.angular.z, 1) != 0) and msg != None:
            if self.control_cmd.is_walking == False:
                self.control_cmd.start_gait()

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
