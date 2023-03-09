import cv2
import socket
import struct
import pickle

# Set up the network socket
HOST = '169.254.229.73'  # Replace with your Raspberry Pi IP address
PORT = 8080

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect((HOST, PORT))

# Create a window to display the video stream
cv2.namedWindow('Video Stream', cv2.WINDOW_NORMAL)

while True:
    # Receive the message from the network
    data = b""
    payload_size = struct.calcsize("Q")
    while len(data) < payload_size:
        packet = s.recv(payload_size - len(data))
        if not packet:
            break
        data += packet
    if not data:
        break
        
    # Unpack the message and deserialize the frame using pickle
    packed_msg_size = data[:payload_size]
    data = data[payload_size:]
    msg_size = struct.unpack("Q", packed_msg_size)[0]
    while len(data) < msg_size:
        data += s.recv(msg_size - len(data))
    frame = pickle.loads(data)
    
    # Display the frame in the window
    cv2.imshow('Video Stream', frame)
    
    # Check for user input to exit the loop
    if cv2.waitKey(1) == ord('q'):
        break

# Close the network connection and destroy the window
s.close()
