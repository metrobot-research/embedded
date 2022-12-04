#!/usr/bin/env python
# The line above tells Linux that this file is a Python script, and that the OS
# should use the Python interpreter in /usr/bin/env to run it. Don't forget to
# use "chmod +x [filename]" to make this script executable.

# Use pyserial to read info from esp32 to the nano (interface node)
# Input character array <- test with 1 character
# both read and write capabilities
# https://github.com/JetsonHacksNano/UARTDemo/blob/master/uart_example.py 
# this stuff ^ (read input) and send to ROS topic

import time
import serial
# import rospy
from struct import pack, unpack
# from std_msgs.msg import SensorData, ControllerCommands #TODO: create SensorData and ControllerCommands message types

def callback(command):
    print("command received: ", command)
    # send commands to esp32
    # velocity, theta_dot are shorts. hip_angle, lower_neck_angle, upper_neck_angle are unsigned shorts. grasper is an unsigned short. state is a char.
    commands_to_sensor = pack('h', command.velocity) + pack('h', command.theta_dot) + pack('H', command.hip_angle) + pack('H', command.lower_neck_angle) + pack('H', command.upper_neck_angle) + pack('B', command.grasper) + pack('c', command.state)
    serial_port.write(commands_to_sensor)

# def nano_interface():
#     r = rospy.Rate(10) # loop runs at 10hz (10 loops / sec)
#     while not rospy.is_shutdown():
#         if serial_port.in_waiting() > 0:
#             data = serial_port.read().decode() # reads bytes from serial port and decodes into unicode (default for python)
#             print("I received ", data)
#             # put data into SensorData msg
#             sensorData = SensorData()
#             sensorData.acceleration = [unpack('h', data[:2]), unpack('h', data[2:4]), unpack('h', data[4:6])]
#             sensorData.orientation = [unpack('h', data[6:8]), unpack('h', data[8:10]), unpack('h', data[10:12])]
#             sensorData.wheel_speeds = [unpack('h', data[12:14]), unpack('h', data[14:16])]
#             sensorData.hip_angles = [unpack('h', data[16:18]), unpack('h', data[18:20])]
#             sensorData.neck_angle = unpack('h', data[20:22])
#             sensorData.head_angle = unpack('h', data[22:24])
#         # Send message: short integer for now:
#         message = pack('>H',42)
#         serial_port.write(message)
#         print("Sent: ",message,end='\n')
#         r.sleep()

def testing_serial():
    while True:
        # message = pack('<HH',42, 32) + b'\n'
        message = pack('<f', 3.1415926) + b'\n'
        serial_port.write(message)
        print("Sent: ", message, end='\n')
        
        time.sleep(2)

if __name__ == '__main__':
    serial_port = serial.Serial( # opens serial port
        port="/dev/ttyTHS1",
        baudrate=115200,
        bytesize=serial.EIGHTBITS,
        parity=serial.PARITY_NONE,                  
        stopbits=serial.STOPBITS_ONE,
        timeout = 10
    )

    # rospy.init_node('nano_interface')
    # rospy.Subscriber("controller_commands", ControllerCommands, callback)
    # pub = rospy.Publisher('sensor_data', SensorData, queue_size=10)
    # rospy.on_shutdown(serial_port.close()) # closes the serial port when ROS gets the shutdown signal
    # Check if the node has received a signal to shut down. If not, run the method.
    # try:
    #     nano_interface()
    # except rospy.ROSInterruptException: pass
    testing_serial()
