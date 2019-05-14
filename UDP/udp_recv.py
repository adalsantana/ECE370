import socket
import udp_struct

UDP_IP = "127.0.0.1" #placeholder
UDP_PORT = 4242

recv = udp_struct.receive()

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))

while True:
    data, addr = sock.recvfrom(1024) #buffer size 1024 bytes
    print "received message: "
