import cv2
import numpy as np
import socket
import sys
import pickle
import struct

clientsocket=socket.socket(socket.AF_INET,socket.SOCK_STREAM)
clientsocket.connect(('localhost',8089))

cap=cv2.VideoCapture(0)
while True:
    ret,frame=cap.read()
    data = pickle.dumps(frame)
    message_size = struct.pack("L", len(data))
    clientsocket.sendall(message_size + data)
