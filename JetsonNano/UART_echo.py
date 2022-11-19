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
# from std_msgs.msg import String

serial_port = serial.Serial( # opens serial port
    port="/dev/ttyTHS1",
    baudrate=115200,
    bytesize=serial.EIGHTBITS,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    timeout = 10
)

try:
    # pub = rospy.Publisher('chatter_talk', String, queue_size=10)
    while True:
        if serial_port.in_waiting() > 0:
            data = serial_port.read().decode() # reads bytes from serial port and decodes into unicode (defaultf for python)
            # if data == "\r".encode(): # i still dont entirely get this
            #     serial_port.write("\n".encode())
            serial_port.write("Sent by Nano.\n".encode())
            # pub.publish(data)
            print("I received ", data)

except KeyboardInterrupt:
    print("Exiting Program")

except Exception as exception_error:
    print("Error occurred. Exiting Program")
    print("Error: " + str(exception_error))

finally:
    serial_port.close()
    pass