#!/usr/bin/env python

import socket

server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

host ="10.9.117.11"
port = 9999
server_socket.bind((host, port))

server_socket.listen(3)
print(f"Server is listening on {host}:{port}...")
print(f"Waiting for connections...")

while True:
    client_socket, addr = server_socket.accept()
    print(f"Connected to {addr}")

    message = client_socket.recv(1024).decode()
    print(f"Client says: {message}")

    client_socket.send("Message received!".encode())

    client_socket.close()