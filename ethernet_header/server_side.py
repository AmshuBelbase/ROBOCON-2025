import socket
import struct
import select

counter = 1
pack_format = '<4i6h'


def start_server():
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
    sock.bind(('0.0.0.0', 10069))
    sock.listen()
    sock.setblocking(False)

    inputs = [sock]
    clients = {}

    while True:
        readable, _, _ = select.select(inputs, [], [], 0.001)
        for s in readable:
            if s is sock:
                conn, addr = s.accept()
                conn.setblocking(False)
                inputs.append(conn)
                clients[conn] = {'buffer': b''}
            else:
                try:
                    data = s.recv(1)
                    if data == b'\x01':
                        global counter
                        response = struct.pack(pack_format,
                                               1, 2, 3, 4,
                                               counter, 1, 0, 1, 150, 200)
                        counter = (counter + 1) % 65535
                        s.sendall(response)
                except:
                    inputs.remove(s)
                    s.close()


if __name__ == '__main__':
    start_server()
