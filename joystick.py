#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import pygame
import paho.mqtt.client as mqtt
import cv2
import pickle
import time

client = mqtt.Client()

client.connect("192.168.42.12", 1883, 60)

play = 1

axs_down = 0
axs_up = 0
axsSpeed = 0

auto_switch = 0
stop_switch = 0
speed = 100
step = 10
leftSpeed = 0
rightSpeed = 0
speedValue = [0,0,0,0]
#leftSpeed 4wd, rightSpeed 4wd, leftSpeed 2wd, rightSpeed 2wd

wdType = 0

pygame.init()  # Инициализирует библиотеку пугейма
try:
    joystick = pygame.joystick.Joystick(0)  # Присваиваем первому подключенному джойстику(0) имя "joystick"
    joystick.init()  # Инициализируем джойстик "joystick"
    name = joystick.get_name()
    print("Joystick initialized")
    print(name)
except:
    print("Problem with joystick")
while True:
    # EVENT PROCESSING STEP
    for event in pygame.event.get():  # User did something
        # Possible joystick actions: JOYAXISMOTION JOYBALLMOTION JOYBUTTONDOWN JOYBUTTONUP JOYHATMOTION
        if auto_switch == 0:
            if play == 0:
                if event.type == pygame.JOYAXISMOTION:
                    leftSpeed = -int(100*joystick.get_axis(1))
                    rightSpeed = -int(100*joystick.get_axis(4))
                    # speedValue = [leftSpeed, rightSpeed]
                    # motorsSpeed = pickle.dumps(speedValue, protocol=4)
                    # client.publish('pi/motorValue/motorSpeed', motorsSpeed, 0)

            else:
                if event.type == pygame.JOYHATMOTION:
                    hat = joystick.get_hat(0)
                    horizHat = hat[0]
                    vericalHat = hat[1]

                    if (horizHat == 1 and vericalHat == 1):
                        rightSpeed = 0
                        leftSpeed = speed
                    elif (horizHat == 1 and vericalHat == -1):
                        rightSpeed = 0
                        leftSpeed = -speed
                    elif(horizHat == -1 and vericalHat == 1):
                        rightSpeed = speed
                        leftSpeed = 0
                    elif (horizHat == -1 and vericalHat == -1):
                        rightSpeed = -speed
                        leftSpeed = 0
                    elif(horizHat == 1 and vericalHat == 0):
                        rightSpeed = 0
                        leftSpeed = speed
                    elif (horizHat == -1 and vericalHat == 0):
                        rightSpeed = speed
                        leftSpeed = 0
                    elif(vericalHat == 1 and horizHat !=1 and horizHat !=-1):
                        rightSpeed = speed
                        leftSpeed = speed
                    elif(vericalHat == -1 and horizHat !=1 and horizHat !=-1):
                        rightSpeed = -speed
                        leftSpeed = -speed
                    else:
                        rightSpeed = 0
                        leftSpeed = 0

            if wdType == 0:
                speedValue = [leftSpeed,rightSpeed,leftSpeed,rightSpeed]
            elif wdType == 1:
                speedValue = [leftSpeed,rightSpeed,0,0]
            else:
                speedValue = [0,0,leftSpeed,rightSpeed]

            if event.type == pygame.JOYAXISMOTION:
                if(joystick.get_axis(2) != -1):
                    axs_down = -(int(((100 * joystick.get_axis(2))+100)/20))*3
                if (joystick.get_axis(5) != -1):
                    axs_up = (int(((100 * joystick.get_axis(5))+100)/20))*3
                axsSpeed = axs_up + axs_down
                axs_down, axs_up = 0,0

            motorsSpeed = pickle.dumps(speedValue)
            client.publish('pi/motorValue/motorSpeed',motorsSpeed, 0)
            client.publish('pi/motorValue/axsSpeed', axsSpeed, 0)


        if event.type == pygame.JOYBUTTONDOWN:
            # print(event)
            if joystick.get_button(0) == 1:
                if auto_switch == 0:
                    print("start")
                    auto_switch = 1
                    client.publish('pi/auto/auto', auto_switch, 0)

                else:
                    auto_switch = 0
                    print("stop")
                    client.publish('pi/auto/auto', auto_switch, 0)

            if joystick.get_button(1) == 1:
                if stop_switch == 0:
                    stop_switch = 1
                    client.publish('pi/stop/stop', stop_switch, 0)

                else:
                    stop_switch = 0
                    client.publish('pi/stop/stop', stop_switch, 0)

            if joystick.get_button(8) == 1:
                print("Speed: %d" % (speed))

            if joystick.get_button(2) == 1:
                if wdType == 0:
                    wdType = 1
                    print("4wd is active")
                elif wdType == 1:
                    wdType = 2
                    print("2wd is active")
                else:
                    wdType = 0
                    print("6wd is active")
            if joystick.get_button(4) == 1:
                if wdType == 0:
                    print("6wd is active")
                if wdType == 1:
                    print("4wd is active")
                if wdType == 2:
                    print("2wd is active")

            if joystick.get_button(6) == 1 and speed !=0:
                speed -= step
            if joystick.get_button(7) == 1 and speed !=100:
                speed += step
