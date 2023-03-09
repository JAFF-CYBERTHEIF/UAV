# -*- coding: utf-8 -*-
"""
Created on Thu Mar  9 17:13:18 2023

@author: admin
"""

import cv2
import socket
import struct
import pickle
import RPi.GPIO as GPIO
import time

# Set up the network socket
HOST = '169.254.229.73'  # Replace with your Raspberry Pi IP address
PORT = 8080

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.bind((HOST, PORT))
s.listen(1)

# Set up the GPIO pins for the servo motor
GPIO.setmode(GPIO.BCM)
GPIO.setup(18, GPIO.OUT)


# Create a PWM object with a frequency of 50Hz
servo1 = GPIO.PWM(18, 50) 
pwm_signal = 5.0

conn, addr = s.accept()

# Create a VideoCapture object for the Raspberry Pi camera
cap = cv2.VideoCapture(0)

# Set the resolution of the capture to 640x480
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

while True:
    # Capture a frame from the video stream
    ret, frame = cap.read()
    
    # Serialize the frame using pickle
    data = pickle.dumps(frame)
    
    # Pack the serialized data into a struct
    message = struct.pack("Q", len(data)) + data
    
    # Send the message over the network
    conn.sendall(message)

    # Receive data from the client
    data = conn.recv(1024)
    if not data:
        break

    # decode the received data
    signal = data.decode("utf-8")
        # Set the duty cycle of the PWM signal
    signal = signal.split(",")
    if signal[0] == "s1":
        pwm_signal = float(signal[1])
        servo1.ChangeDutyCycle(pwm_signal)

    # Wait for a moment to let the servo move to the desired position
    time.sleep(0.5)
    
    # Check for user input to exit the loop
    if cv2.waitKey(1) == ord('q'):
        break

# Release the VideoCapture object and close the network connection
cap.release()
servo1.stop()
GPIO.cleanup()
conn.close()
