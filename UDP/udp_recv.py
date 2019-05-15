import socket
import udp_struct

#UDP_IP = "192.168.4.26"
#UDP_IP = "127.0.0.1" #placeholder
UDP_IP = "192.168.1.100"
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
    udpStr = udp_struct.receive.from_buffer(ba)
    return udpStr

while True:
    data, addr = sock.recvfrom(1024) #buffer size 1024 bytes
    print "received message\n"
    recv = udpToStruct(data)
    print "imu ax " + str(recv.ImuData.ax)
    print "imu ay " + str(recv.ImuData.ay)
    print "imu az " + str(recv.ImuData.az)
    print "imu mx " + str(recv.ImuData.mx)
    print "imu my " + str(recv.ImuData.my)
    print "imu mz " + str(recv.ImuData.mz)
    print "Odometry x location " + str(recv.OdometryData.x)
    print "Odometry y location " + str(recv.OdometryData.y)
    print "Odometry z location " + str(recv.OdometryData.z)
    print "IMU Heading " + str(recv.heading)
