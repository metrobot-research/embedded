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
from std_msgs.msg import SensorData, ControllerCommands #TODO: create SensorData and ControllerCommands message types

def floatToString(value: float):
    val = ""
    if value < 10:
        val = "00"
    elif value < 100:
        val = "0"
    return val + (str (round(value, 2)))

def callback(command):
    print("command received: ", command)
    velocity = command.velocity
    theta_dot = command.theta_dot
    # send commands to esp32
    commands_to_sensor = command.state + floatToString(velocity[0]) + floatToString(velocity[1]) + floatToString(theta_dot[0]) + floatToString(theta_dot[1]) + floatToString(command.hip_angle) + floatToString(command.lower_neck_angle) + floatToString(command.upper_neck_angle) + (str (int (round(command.grapser, 0)))) + "\n"
    serial_port.write(commands_to_sensor)

def nano_interface():
    r = rospy.Rate(10) # loop runs at 10hz (10 loops / sec)
    while not rospy.is_shutdown():
        if serial_port.in_waiting() > 0:
            data = serial_port.read().decode() # reads bytes from serial port and decodes into unicode (default for python)
            print("I received ", data)
            # TODO: put data into SensorData msg
            sensorData = SensorData()
            pub.publish(sensorData)
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