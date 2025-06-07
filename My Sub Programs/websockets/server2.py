from socket import *
HOST = "10.9.117.11"
PORT = 9999
BUFSIZ = 1024
ADDR = (HOST, PORT)

tcpSerSock = socket(AF_INET, SOCK_STREAM)
tcpSerSock.bind(ADDR)
tcpSerSock.listen(5)

# Just setting up the server upto now

print("Starting the server")
while True:
	tcpCliSock, addr = tcpSerSock.accept()      # passing on the client handler to a new socket
	print("Connected to ", addr)
	while True:
		data = tcpCliSock.recv(BUFSIZ)
		if not data:                        # Go to next client if this one send empty 
			break
		while data.decode().strip().lower() != 'continue':    # Receiving data loop
			print(data.decode().strip())                
			data = tcpCliSock.recv(BUFSIZ)
        #client sends continue once its sending data loop is done    
		message = input(">")
		while message.lower() != 'go' and message != '':       # Sending data loop
			message += '\r\n'
			tcpCliSock.send(message.encode())
			message = input(">")
        # go is the word to stop the sending loop
		else:
			if message != '':
				tcpCliSock.send('continue\r\n'.encode())
				break
			else:
				break
	tcpCliSock.close()