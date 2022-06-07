#!/usr/bin/env python3

import pygame, time, sys
from pygame.locals import *

import subprocess
import asyncio
import serial_asyncio
import os

DEVICE_NAME = 'ESP32'
DEVICE_MAC_ADDRESS = '40-f5-20-45-22-d2'
CONNECTION_TIMEOUT = 10

FORWARD_SPEED = 0.5
TURN_SPEED = 0.5

async def receive(reader):
    print("receiving...")
    while True:
        data = await reader.readuntil(b'\n')
        print(f'(recv): {data.strip().decode()}')

async def run_controller(writer, input_type):
    global angle1, angle2
    writer.write(b"Controller connected.\n")

    pygame.init()
    screen = pygame.display.set_mode((640, 480))
    pygame.display.set_caption('ChickenBot Controller')
    pygame.mouse.set_visible(0)

    pygame.joystick.init()
    joysticks = [pygame.joystick.Joystick(x) for x in range(pygame.joystick.get_count())]
    
    keyboard = input_type == '0'
    if not keyboard and len(joysticks) == 0:
        print("Error: No joysticks detected, exiting...")
        sys.exit()

    while True:
        if not keyboard:
            writer.write(b'{"command":0, "args":[%6f]}\n' % (-joysticks[0].get_axis(1)))
            writer.write(b'{"command":1, "args":[%6f]}\n' % joysticks[0].get_axis(2))
            print("Setting speed: ", -joysticks[0].get_axis(1), ", turn speed: ", joysticks[0].get_axis(2))
        else:
            keys = pygame.key.get_pressed()
            if keys[pygame.K_UP]:
                writer.write(b'{"command":0,"args":[%6f]}\n' % FORWARD_SPEED)
                print("Setting speed: ", FORWARD_SPEED)
            elif keys[pygame.K_DOWN]:
                writer.write(b'{"command":0,"args":[%6f]}\n' % (-FORWARD_SPEED))
                print("Setting speed: ", -FORWARD_SPEED)
            else:
                writer.write(b'{"command":0,"args":[0]}\n')
                print("Setting speed: ", 0)
            if keys[pygame.K_LEFT]:
                writer.write(b'{"command":1,"args":[%6f]}\n' % (-TURN_SPEED))
                print("Setting turn speed", -TURN_SPEED)
            elif keys[pygame.K_RIGHT]:
                writer.write(b'{"command":1,"args":[%6f]}\n' % TURN_SPEED)
                print("Setting turn speed: ", TURN_SPEED)
            else:
                writer.write(b'{"command":1,"args":[0]}\n')
                print("Setting turn speed: ", 0)
        for event in pygame.event.get():
            if event.type == QUIT:
                sys.exit()

        await asyncio.sleep(0.1)

async def open_bluetooth_terminal(port, baudrate, input_type):
    reader, writer = await serial_asyncio.open_serial_connection(url=port, baudrate=baudrate)
    receiver = receive(reader)
    controller = run_controller(writer, input_type)
    await asyncio.wait([receiver, controller])

if len(sys.argv) != 3:
    print("Invalid arguments. Correct usage: python3 controller.py <port name> <input type> (0 for keyboard, 1 for joystick)")
    sys.exit()
loop = asyncio.get_event_loop()
task = open_bluetooth_terminal(sys.argv[1], 115200, sys.argv[2])
try:
    loop.run_until_complete(task)
    loop.close()
except KeyboardInterrupt:
    print("Exiting...")
