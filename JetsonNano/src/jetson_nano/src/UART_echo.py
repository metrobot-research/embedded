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
from customized_msgs.msg import cmd

def callback(command):
    # print("command received: ", command)
    # send commands to esp32 
    # velocity, theta_dot are shorts. hip_angle, lower_neck_angle, upper_neck_angle are unsigned shorts. grasper is an unsigned short. state is a char.
    command.neckPosition = 0
    commands_to_sensor = pack('<ccHfffHH', command.state, command.furtherState, command.x_dot, command.theta_dot, command.hipAngle, command.neckPosition, command.headVelocity, command.grasperVelocity)
    serial_port.write(commands_to_sensor)

def nano_interface(serial_port):
    pub = rospy.Publisher('sensor_data', SensorData, queue_size=10)
    buffer = b''
    r = rospy.Rate(20)
    while not rospy.is_shutdown():
        # receive
        if serial_port.in_waiting > 0:
            abyte = serial_port.read()
            # print("I received ", Abyte)
            if abyte == b'\n':
                sensor_msg = interpret_msg(buffer)
                if sensor_msg:
                    pub.publish(sensor_msg)
                buffer = b''
            else:
                buffer += abyte
        r.sleep()

def interpret_msg(buffer): 
    if(len(buffer) != 56): 
        print("wrong length: ", len(buffer))
        return
    print("buffer", buffer)
    val_tuples = unpack('<ffffffffffffHHc', buffer)
    sensor_msg = SensorData()
    sensor_msg.acceleration = list(val_tuples[:3])
    sensor_msg.orientation = list(val_tuples[3:6]) 
    sensor_msg.phi = val_tuples[6]
    sensor_msg.wheel_speeds = list(val_tuples[7:9]) 
    sensor_msg.hip_angles = list(val_tuples[9:11]) 
    sensor_msg.neck_angle = val_tuples[11]
    sensor_msg.head_angle = val_tuples[12]
    rospy.loginfo(sensor_msg)
    return sensor_msg
    

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
    rospy.Subscriber("controller_commands", cmd, callback)
    rospy.on_shutdown(serial_port.close()) # closes the serial port when ROS gets the shutdown signal
    # Check if the node has received a signal to shut down. If not, run the method.
    try:
        nano_interface(serial_port)
    except rospy.ROSInterruptException: pass
