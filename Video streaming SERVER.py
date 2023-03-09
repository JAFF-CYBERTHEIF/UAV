import cv2
import socket
import struct
import pickle

# Set up the network socket
HOST = '192.168.0.100'  # Replace with your Raspberry Pi IP address
PORT = 8080

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.bind((HOST, PORT))
s.listen(1)

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
    
    # Check for user input to exit the loop
    if cv2.waitKey(1) == ord('q'):
        break

# Release the VideoCapture object and close the network connection
cap.release()
conn.close()
