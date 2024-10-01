#!/usr/bin/python3
# coding=utf8
import os, sys, math
import numpy as np
import subprocess
# import cv2
# import ServoCmd 
import signal
import threading
import pygame
import time
import RPi.GPIO as GPIO

from time import sleep
from queue import Queue
from os import geteuid

button_map = {"PSB_CROSS":2, "PSB_CIRCLE":1, "PSB_SQUARE":3, "PSB_TRIANGLE":0,
    "PSB_L1": 4, "PSB_R1":5, "PSB_L2":6, "PSB_R2":7,
    "PSB_SELECT":8, "PSB_START":9, "PSB_L3":10, "PSB_R3":11}
axis_map = {"PSB_Left_Horizontal_Axis":0, "PSB_Left_Vertical_Axis":1, "PSB_Right_Horizontal_Axis":2, "PSB_Right_Vertical_Axis":3}

servo = []
left_rate = 0.5
right_rate = 1.3
pSB_R2 = 0

GPIO.setmode(GPIO.BCM)
GPIO.setup(6, GPIO.OUT)
p = GPIO.PWM(6, 50)

def control_thread(q):
    start_time = None
    elapsed_time = 0

    while True:
        if q != None:
            joystick_queue = q.get()
            q.queue.clear()

        if joystick_queue["PSB_R1"] == 1:
            if start_time is None:
                start_time = time.time()
            else:
                elapsed_time = time.time() - start_time
                
            if elapsed_time >= 3:
                #sound
                print("shutdown")
                p.start(50)
                p.ChangeFrequency(523)
                time.sleep(0.5)
                p.stop()
                GPIO.cleanup()

                # Execute shutdown command with sudo, reading password from stdin
                sudo_password = "csl92021164"
                command = "echo {} | sudo -S shutdown -h now".format(sudo_password) #TODO -h ??
                subprocess.Popen(command, shell=True)
                start_time = None
                elapsed_time = 0

        else:
            start_time = None
            elapsed_time = 0

        time.sleep(0.1)
            

def joystick_thread(q):
    connected = False
    lastTime = time.time()
    try:
        while True:
            if os.path.exists("/dev/input/js0") is True:
                if connected is False:
                    joystick_init()
                    jscount =  pygame.joystick.get_count()
                    if jscount > 0:
                        try:
                            js=pygame.joystick.Joystick(0)
                            js.init()
                            connected = True
                        except Exception as e:
                            print(e)
                    else:
                        pygame.joystick.quit()
            else:
                if connected is True:
                    connected = False
                    js.quit()
                    pygame.joystick.quit()
                    
            if connected is True:
                pygame.event.pump()     
                try:
                    motor_move = {}
                    # pSB_Left_Vertical_Axis = js.get_axis(axis_map["PSB_Left_Vertical_Axis"])
                    # pSB_Right_Horizontal_Axis = -js.get_axis(axis_map["PSB_Right_Horizontal_Axis"])
                    # pSB_Right_Vertical_Axis = -js.get_axis(axis_map["PSB_Right_Vertical_Axis"])
                    pSB_R1 = js.get_button(button_map["PSB_R1"])
                    # pSB_L1 = js.get_button(button_map["PSB_L1"])
                    # pSB_CIRCLE = js.get_button(button_map["PSB_CIRCLE"])
                    # pSB_TRIANGLE = js.get_button(button_map["PSB_TRIANGLE"])

                    motor_move = {  
                        # "PSB_Left_Vertical_Axis":pSB_Left_Vertical_Axis, 
                        # "PSB_Right_Vertical_Axis":pSB_Right_Vertical_Axis, 
                        # "PSB_Right_Horizontal_Axis":pSB_Right_Horizontal_Axis,
                        "PSB_R1":pSB_R1, 
                        # "PSB_L1":pSB_L1, 
                        # "PSB_CIRCLE":pSB_CIRCLE, 
                        # "PSB_TRIANGLE":pSB_TRIANGLE
                    }
                    # print(motor_move)
                    q.put(motor_move)

                    if time.time() - lastTime > 1:
                        lastTime = time.time()

                except Exception as e:
                    print(e)
                    connected = False

            time.sleep(0.01)

    except KeyboardInterrupt:
        time.sleep(0.2)
        pass

def joystick_init():
    os.environ["SDL_VIDEODRIVER"] = "dummy"
    pygame.display.init()
    pygame.joystick.init()
    if pygame.joystick.get_count() > 0:
        js=pygame.joystick.Joystick(0)
        js.init()
        jsName = js.get_name()
        print("Name of the joystick:", jsName)
        jsAxes=js.get_numaxes()
        print("Number of axif:",jsAxes)
        jsButtons=js.get_numbuttons()
        print("Number of buttons:", jsButtons)
        jsBall=js.get_numballs()
        print("Numbe of balls:", jsBall)
        jsHat= js.get_numhats()
        print("Number of hats:", jsHat)

def main():
    q = Queue()
    threads = []
    threads.append(threading.Thread(target = control_thread, args = (q,)))
    threads.append(threading.Thread(target = joystick_thread, args = (q,)))
    threads[0].start()
    threads[1].start()

if __name__ == "__main__":
    main()