import socket
import udp_struct

#UDP_IP = "192.168.4.26"
UDP_IP = "127.0.0.1" #placeholder
#UDP_IP = "192.168.1.100"
UDP_PORT = 4242

recv = udp_struct.receive()

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))

def udpToStruct(data):
    #empty byte array
    ba = bytearray()
    #make it the size of received data
    ba.extend(data)
    #put it in receive udp structure
    ret = udp_struct.receive().from_buffer(ba)

while True:
    data, addr = sock.recvfrom(1024) #buffer size 1024 bytes
    print "received message\n"
    recv = udpToStruct(data)
    
    print "imu ax, ay, az, mx, my, mz"
    for i in range(len(recv.ImuData)):
        print recv.ImuData[i]
    print "IMU header"
    print recv.heading
    
    print "odometry x, y, z"
    for i in range(len(recv.OdometryData)):
        print recv.OdometryData[i]
    
