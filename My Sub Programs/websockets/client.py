#!/usr/bin/env python

import socket

client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

server_ip = "10.9.117.11"
port = 9999
client_socket.connect((server_ip, port))
print(f"Connected to Server")

message = "Hello, Server!"
client_socket.send(message.encode())

reply = client_socket.recv(1024).decode()

print(f"Serever: {reply}")

client_socket.close()
