# -*- coding: utf-8 -*-
"""
Created on Thu Mar  9 16:36:43 2023

@author: admin
"""

import pygame

# initialize pygame
pygame.init()

# initialize joystick
joystick = pygame.joystick.Joystick(1)
joystick.init()

# map button names to indices
button_names = {
    0: "1",
    1: "2",
    2: "3",
    3: "4",
    4: "L1",
    5: "R1",
    6: "L2",
    7: "R2",
    8: "Left Stick",
    9: "START",
    10: "Home",
    11: "DPad"
}


def joystick_info():
    # print some joystick info
    print(f"Joystick Name: {joystick.get_name()}")
    print(f"Number of Buttons: {joystick.get_numbuttons()}")
    print(f"Number of Axes: {joystick.get_numaxes()}")
    print(f"Number of HAT: {joystick.get_numhats()}")
# Define the minimum and maximum duty cycle for the servo
min_duty_cycle = 5
max_duty_cycle = 10

# Define a function to convert the signal to a PWM signal
def signal_to_pwm(signal):
    pwm_signal = (signal * (max_duty_cycle - min_duty_cycle) / 2) + min_duty_cycle
    return pwm_signal

def get_signal():
    button_name = ""
    stick = ""
    HAT =""
    for event in pygame.event.get():
        if event.type == pygame.JOYBUTTONDOWN:
            #button = joystick.get_button(event.button)
            button_name = "b," + button_names.get(event.button, f"Button {event.button}") +"; "
          
        if event.type == pygame.JOYAXISMOTION:
            axis = joystick.get_axis(event.axis)
            PWM = signal_to_pwm(axis + 1)
            print(axis)
            stick = "s," + str(event.axis) + ", " + str(PWM) 
          
        if event.type == pygame.JOYHATMOTION:
            hat = joystick.get_hat(event.hat)
            HAT =  "h,"+ str(hat[0]) + ", " + str(hat[1]) 
            
        signal =  button_name  + stick + HAT   
        return signal

#use a while loop to like this to acces the controller signal
while True:
    signal = get_signal()
    if signal == None:
        continue
    else:
        print(signal)
