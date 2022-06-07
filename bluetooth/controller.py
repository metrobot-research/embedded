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

async def run_controller(writer):
    global angle1, angle2
    writer.write(b"Controller connected.\n")

    pygame.init()
    screen = pygame.display.set_mode((640, 480))
    pygame.display.set_caption('ChickenBot Controller')
    pygame.mouse.set_visible(0)

    pygame.joystick.init()
    joysticks = [pygame.joystick.Joystick(x) for x in range(pygame.joystick.get_count())]
    
    keyboard = len(joysticks) == 0
    if keyboard:
        print("Warning: No joysticks detected, enabling keyboard control")

    while True:
        if not keyboard:
            writer.write(b'{"command":0, "args":[%6f]}\n' % (-joysticks[0].get_axis(1)))
            writer.write(b'{"command":1, "args":[%6f]}\n' % joysticks[0].get_axis(2))
            print("Setting speed: ", -joysticks[0].get_axis(1), ", turn speed: ", joysticks[0].get_axis(2))
        forward_control = False
        turn_control = False
        for event in pygame.event.get():
            if keyboard:
                if event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_UP:
                        writer.write(b'{"command":0,"args":[%6f]}\n' % FORWARD_SPEED)
                        print("Setting speed: ", FORWARD_SPEED)
                        forward_control = True
                    if event.key == pygame.K_DOWN:
                        writer.write(b'{"command":0,"args":[%6f]}\n' % (-FORWARD_SPEED))
                        print("Setting speed: ", -FORWARD_SPEED)
                        forward_control = True
                    if event.key == pygame.K_LEFT:
                        writer.write(b'{"command":1,"args":[%6f]}\n' % (-TURN_SPEED))
                        print("Setting turn speed", -TURN_SPEED)
                        turn_control = True
                    if event.key == pygame.K_RIGHT:
                        writer.write(b'{"command":1,"args":[%6f]}\n' % TURN_SPEED)
                        print("Setting turn speed: ", TURN_SPEED)
                        turn_control = True
            if event.type == QUIT:
                pygame.quit()
                sys.exit()
        if keyboard:
            if not forward_control:
                writer.write(b'{"command":0,"args":[0]}\n')
            if not turn_control:
                writer.write(b'{"command":1,"args":[0]}\n')

        await asyncio.sleep(0.1)

async def open_bluetooth_terminal(port, baudrate):
    # print("Checking if device at %s is paired..." % DEVICE_MAC_ADDRESS)
    # if not os.path.exists(port):
    #     print("Pairing with device at %s..." % DEVICE_MAC_ADDRESS)
    #     rc = run_terminal_command(["blueutil", "--pair", DEVICE_MAC_ADDRESS])
    #     if rc != 0:
    #         print("Failed to pair with device. Exiting...")
    #         return
    # print("Connecting to device at %s..." % DEVICE_MAC_ADDRESS)
    # rc = run_terminal_command(["blueutil", "--connect", DEVICE_MAC_ADDRESS])
    # if rc != 0:
    #     print("Failed to connect to device. Exiting...")
    #     return
    # print("Connected successfully.")
    reader, writer = await serial_asyncio.open_serial_connection(url=port, baudrate=baudrate)
    receiver = receive(reader)
    controller = run_controller(writer)
    await asyncio.wait([receiver, controller])

loop = asyncio.get_event_loop()
task = open_bluetooth_terminal('/dev/cu.%s' % DEVICE_NAME, 115200)
try:
    loop.run_until_complete(task)
    loop.close()
except KeyboardInterrupt:
    print("Exiting...")
