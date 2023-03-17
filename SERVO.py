# -*- coding: utf-8 -*-
"""
Created on Fri Mar 17 08:31:29 2023

@author: admin
"""

import Rpi.GPIO as GPIO
from time import sleep

GPIO.setmode(GPIO.BOARD)

GPIO.setup(11,GPIO.OUT)

def SetAngle(angle,pin):
    pwm = GPIO.PWM(pin,50)
    pwm.start(0)
    duty = angle / 18 + 2
    pwm.ChangeDutyCycle(duty)
    sleep(0.5)
    pwm.ChangeDutyCycle(0)

pin=11 #GPIO pin (SERVO MOTOR)
angle=90 #Angle of the servo motor

SetAngle(angle, pin)   #call the function to set angle to the servo motor 
