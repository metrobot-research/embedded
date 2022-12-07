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
import rospy
from struct import pack, unpack
from jetson_nano.msg import SensorData, ControllerCommands #TODO: create SensorData and ControllerCommands message types

def callback(command):
    print("command received: ", command)
    # send commands to esp32
    # velocity, theta_dot are shorts. hip_angle, lower_neck_angle, upper_neck_angle are unsigned shorts. grasper is an unsigned short. state is a char.
    commands_to_sensor = pack('h', command.velocity) + pack('h', command.theta_dot) + pack('H', command.hip_angle) + pack('H', command.lower_neck_angle) + pack('H', command.upper_neck_angle) + pack('B', command.grasper) + pack('c', command.state)
    serial_port.write(commands_to_sensor)

def nano_interface(serial_port):
    buffer = b''
    r = rospy.Rate(10)
    while not rospy.is_shutdown():
        # receive
        if serial_port.in_waiting > 0:
            abyte = serial_port.read()
            # print("I received ", Abyte)

            if abyte == b'\n':
                interpret_msg(buffer)
                buffer = b''
            else:
                buffer += abyte
        r.sleep()
        
        # send
        message = pack('<f', 3.1415926) + b'\n'
        serial_port.write(message)

def interpret_msg(buffer): 
    if(len(buffer) != 56): 
        print("wrong length: ", len(buffer))
        return
    print("buffer", buffer)
    val_tuples = unpack('<ffffffffffffHHc', buffer)
    sensor_msg = SensorData()
    sensor_msg.acceleration = list(val_tuples[:3])
    sensor_msg.orientation = list(val_tuples[3:6]) #[unpack('h', data[6:8]), unpack('h', data[8:10]), unpack('h', data[10:12])]
    sensor_msg.phi = val_tuples[6]
    sensor_msg.wheel_speeds = list(val_tuples[7:9]) 
    sensor_msg.hip_angles = list(val_tuples[9:11]) 
    sensor_msg.neck_angle = val_tuples[11] 
    sensor_msg.head_angle = val_tuples[12] 
    rospy.loginfo(sensor_msg)

if __name__ == '__main__':
    serial_port = serial.Serial( # opens serial port
        port="/dev/cu.SLAB_USBtoUART",
        baudrate=115200,
        bytesize=serial.EIGHTBITS,
        parity=serial.PARITY_NONE,                  
        stopbits=serial.STOPBITS_ONE,
        timeout = 10
    )

    rospy.init_node('nano_interface')
    rospy.Subscriber("controller_commands", ControllerCommands, callback)
    pub = rospy.Publisher('sensor_data', SensorData, queue_size=10)
    rospy.on_shutdown(serial_port.close()) # closes the serial port when ROS gets the shutdown signal
    # Check if the node has received a signal to shut down. If not, run the method.
    try:
        nano_interface(serial_port)
    except rospy.ROSInterruptException: pass
