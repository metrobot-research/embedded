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
from std_msgs.msg import SensorData, ControllerCommands #TODO: create SensorData and ControllerCommands message types

# def floatToString(value: float):
#     val = ""
#     if value < 10:
#         val = "00"
#     elif value < 100:
#         val = "0"
#     return val + (str (round(value, 2)))

def callback(command):
    print("command received: ", command)
    # send commands to esp32
    commands_to_sensor = command.state + pack('e', command.velocity) + pack('e', command.theta_dot) + pack('e', command.hip_angle) + pack('e', command.lower_neck_angle) + pack('e', command.upper_neck_angle) + pack('B', command.grasper)
    # commands_to_sensor = command.state + floatToString(command.velocity) + floatToString(theta_dot[0]) + floatToString(theta_dot[1]) + floatToString(command.hip_angle) + floatToString(command.lower_neck_angle) + floatToString(command.upper_neck_angle) + (str (int (round(command.grapser, 0)))) + "\n"
    serial_port.write(commands_to_sensor)

def nano_interface():
    r = rospy.Rate(10) # loop runs at 10hz (10 loops / sec)
    while not rospy.is_shutdown():
        if serial_port.in_waiting() > 0:
            data = serial_port.read().decode() # reads bytes from serial port and decodes into unicode (default for python)
            print("I received ", data)
            accel_x, accel_y, accel_z = unpack('e', data[:2]), unpack('e', data[2:4]), unpack('e', data[4:6])
            # put data into SensorData msg
            sensorData = SensorData()
            sensorData.acceleration = [unpack('e', data[:2]), unpack('e', data[2:4]), unpack('e', data[4:6])]
            sensorData.orientation = [unpack('e', data[6:8]), unpack('e', data[8:10]), unpack('e', data[10:12])]
            sensorData.wheel_speeds = [unpack('e', data[12:14]), unpack('e', data[14:16])]
            sensorData.hip_angles = [unpack('e', data[16:18]), unpack('e', data[18:20])]
            sensorData.neck_angle = unpack('e', data[20:22])
            sensorData.head_angle = unpack('e', data[20:22])
        r.sleep()

if __name__ == '__main__':
    serial_port = serial.Serial( # opens serial port
        port="/dev/ttyTHS1",
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
        nano_interface()
    except rospy.ROSInterruptException: pass