#!/usr/bin/env python3

import pygame, pygame_gui, time, sys
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

CONTROL_DELAY = 100


KP_W_DEFAULT = 0.030
KI_W_DEFAULT = 0.3
KD_W_DEFAULT = 0.002

kp_w = KP_W_DEFAULT
ki_w = KI_W_DEFAULT
kd_w = KD_W_DEFAULT


KP_PHI_DEFAULT = 12500
KI_PHI_DEFAULT = 145000
KD_PHI_DEFAULT = 2000

kp_phi = KP_PHI_DEFAULT
ki_phi = KI_PHI_DEFAULT
kd_phi = KD_PHI_DEFAULT


KP_THETA_DOT_DEFAULT = 200
KI_THETA_DOT_DEFAULT = 150
KD_THETA_DOT_DEFAULT = 5

kp_theta_dot = KP_THETA_DOT_DEFAULT
ki_theta_dot = KI_THETA_DOT_DEFAULT
kd_theta_dot = KD_THETA_DOT_DEFAULT

async def receive(reader):
    print("receiving...")
    while True:
        data = await reader.readuntil(b'\n')
        print(f'(recv): {data.strip().decode()}')

async def run_controller(writer, input_type):
    global kp_w, ki_w, kd_w, kp_phi, ki_phi, kd_phi, kp_theta_dot, ki_theta_dot, kd_theta_dot, angle1, angle2
    writer.write(b"Controller connected.\n")

    pygame.init()
    screen = pygame.display.set_mode((800, 600))
    background = pygame.Surface((800, 600))
    background.fill(pygame.Color('#ffffff'))

    manager = pygame_gui.UIManager((800, 600))

    grid = [[(100 + 120*i, 75 + 150*j) for j in range(3)] for i in range(5)]

    vel_kp = pygame_gui.elements.UITextEntryLine(relative_rect=pygame.Rect(grid[0][0], (100, 50)),
            manager=manager)
    vel_kp.set_text(str(kp_w))

    vel_ki = pygame_gui.elements.UITextEntryLine(relative_rect=pygame.Rect(grid[1][0], (100, 50)),
            manager=manager)
    vel_ki.set_text(str(ki_w))

    vel_kd = pygame_gui.elements.UITextEntryLine(relative_rect=pygame.Rect(grid[2][0], (100, 50)),
            manager=manager)
    vel_kd.set_text(str(kd_w))

    vel_save = pygame_gui.elements.UIButton(relative_rect=pygame.Rect(grid[3][0], (100, 50)),
            text='Save',
            manager=manager)

    vel_reset = pygame_gui.elements.UIButton(relative_rect=pygame.Rect(grid[4][0], (100, 50)),
            text='Reset',
            manager=manager)

    phi_kp = pygame_gui.elements.UITextEntryLine(relative_rect=pygame.Rect(grid[0][1], (100, 50)),
            manager=manager)
    phi_kp.set_text(str(kp_phi))

    phi_ki = pygame_gui.elements.UITextEntryLine(relative_rect=pygame.Rect(grid[1][1], (100, 50)),
            manager=manager)
    phi_ki.set_text(str(ki_phi))

    phi_kd = pygame_gui.elements.UITextEntryLine(relative_rect=pygame.Rect(grid[2][1], (100, 50)),
            manager=manager)
    phi_kd.set_text(str(kd_phi))

    phi_save = pygame_gui.elements.UIButton(relative_rect=pygame.Rect(grid[3][1], (100, 50)),
            text='Save',
            manager=manager)

    phi_reset = pygame_gui.elements.UIButton(relative_rect=pygame.Rect(grid[4][1], (100, 50)),
            text='Reset',
            manager=manager)

    theta_dot_kp = pygame_gui.elements.UITextEntryLine(relative_rect=pygame.Rect(grid[0][2], (100, 50)),
            manager=manager)
    theta_dot_kp.set_text(str(kp_theta_dot))

    theta_dot_ki = pygame_gui.elements.UITextEntryLine(relative_rect=pygame.Rect(grid[1][2], (100, 50)),
            manager=manager)
    theta_dot_ki.set_text(str(ki_theta_dot))

    theta_dot_kd = pygame_gui.elements.UITextEntryLine(relative_rect=pygame.Rect(grid[2][2], (100, 50)),
            manager=manager)
    theta_dot_kd.set_text(str(kd_theta_dot))

    theta_dot_save = pygame_gui.elements.UIButton(relative_rect=pygame.Rect(grid[3][2], (100, 50)),
            text='Save',
            manager=manager)

    theta_dot_reset = pygame_gui.elements.UIButton(relative_rect=pygame.Rect(grid[4][2], (100, 50)),
            text='Reset',
            manager=manager)

    pygame.display.set_caption('ChickenBot Controller')

    pygame.joystick.init()
    joysticks = [pygame.joystick.Joystick(x) for x in range(pygame.joystick.get_count())]

    keyboard = input_type == '0'
    if not keyboard and len(joysticks) == 0:
        print("Error: No joysticks detected, exiting...")
        sys.exit()

    clock = pygame.time.Clock()

    while True:
        time_delta = clock.tick(60)
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
            try:
                if event.type == pygame_gui.UI_BUTTON_PRESSED:
                    if event.ui_element == vel_save:
                        kp_w = float(vel_kp.get_text())
                        ki_w = float(vel_ki.get_text())
                        kd_w = float(vel_kd.get_text())
                        writer.write(b'{"command":2,"args":[%6f, %6f, %6f]}\n' % (kp_w, ki_w, kd_w))
                    if event.ui_element == vel_reset:
                        kp_w = KP_W_DEFAULT
                        ki_w = KI_W_DEFAULT
                        kd_w = KD_W_DEFAULT
                        vel_kp.set_text(str(kp_w))
                        vel_ki.set_text(str(ki_w))
                        vel_kd.set_text(str(kd_w))
                        writer.write(b'{"command":2,"args":[%6f, %6f, %6f]}\n' % (kp_w, ki_w, kd_w))
                    if event.ui_element == phi_save:
                        kp_phi = float(phi_kp.get_text())
                        ki_phi = float(phi_ki.get_text())
                        kd_phi = float(phi_kd.get_text())
                        writer.write(b'{"command":3,"args":[%6f, %6f, %6f]}\n' % (kp_phi, ki_phi, kd_phi))
                    if event.ui_element == phi_reset:
                        kp_phi = KP_PHI_DEFAULT
                        ki_phi = KI_PHI_DEFAULT
                        kd_phi = KD_PHI_DEFAULT
                        phi_kp.set_text(str(kp_phi))
                        phi_ki.set_text(str(ki_phi))
                        phi_kd.set_text(str(kd_phi))
                        writer.write(b'{"command":3,"args":[%6f, %6f, %6f]}\n' % (kp_phi, ki_phi, kd_phi))
                    if event.ui_element == theta_dot_save:
                        kp_theta_dot = float(theta_dot_kp.get_text())
                        ki_theta_dot = float(theta_dot_ki.get_text())
                        kd_theta_dot = float(theta_dot_kd.get_text())
                        writer.write(b'{"command":4,"args":[%6f, %6f, %6f]}\n' % (kp_theta_dot, ki_theta_dot, kd_theta_dot))
                    if event.ui_element == theta_dot_reset:
                        kp_theta_dot = KP_THETA_DOT_DEFAULT
                        ki_theta_dot = KI_THETA_DOT_DEFAULT
                        kd_theta_dot = KD_THETA_DOT_DEFAULT
                        theta_dot_kp.set_text(str(kp_theta_dot))
                        theta_dot_ki.set_text(str(ki_theta_dot))
                        theta_dot_kd.set_text(str(kd_theta_dot))
                        writer.write(b'{"command":4,"args":[%6f, %6f, %6f]}\n' % (kp_theta_dot, ki_theta_dot, kd_theta_dot))
            except Exception as e:
                print(f"UI Error: {e}")

            manager.process_events(event)

        manager.update(time_delta/1000.0)
        screen.blit(background, (0, 0))
        manager.draw_ui(screen)

        pygame.display.update()
        await asyncio.sleep(0.02)

async def open_bluetooth_terminal(port, baudrate, input_type):
    print(port)
    reader, writer = await serial_asyncio.open_serial_connection(url=port, baudrate=baudrate)
    writer.write(b"connected")
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
