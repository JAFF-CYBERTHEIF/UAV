import socket

HOST = 'localhost'
PORT = 5555

client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
client_socket.connect((HOST, PORT))

while True:
    message = input("Enter message to send: ")
    client_socket.sendall(message.encode())
    data = client_socket.recv(1024).decode()
    print(f"Received message: {data}")
    
client_socket.close()
