import time
import RPi.GPIO as GPIO

def DoReMi():
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(6, GPIO.OUT)
    p = GPIO.PWM(6, 50)
    p.start(50)

    print("Do")
    p.ChangeFrequency(523)
    time.sleep(0.5)

    # print("Re")
    # p.ChangeFrequency(587)
    # time.sleep(1)
   
    # print("Mi")
    # p.ChangeFrequency(659)
    # time.sleep(1)

    p.stop()
    GPIO.cleanup()

DoReMi()