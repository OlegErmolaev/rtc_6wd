#!/usr/bin/env python3
# -*- coding: utf-8 -*-
from RoboMind import *
import paho.mqtt.client as mqtt
import numpy as np
import cv2
import time
import pickle
from threading import Thread

from can.interfaces.usb2canInterface import message_convert_rx

cap = cv2.VideoCapture(0)
isStopped = False

class autonomka(Thread):

    def __init__(self):
        global cap
        ''' Constructor. '''
        Thread.__init__(self)
        self.leftMotor = 0
        self.rightMotor = 0

        self.imageWidth = 176
        self.imageHeight = 144
        self.frameCol = 3
        self.frameRow = 3

        self.stopping = 0

        self.frameWidth = int(self.imageWidth / self.frameCol)
        self.frameHeight = int(self.imageHeight / self.frameRow)

        print("OpenCV version: %s" % cv2.__version__)



        assert cap.isOpened, "Ошибка веб-камеры!"

        cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.imageWidth)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.imageHeight)

        #print("Image size: %dx%d" % (self.cap.get(cv2.CAP_PROP_FRAME_WIDTH), self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT)))
        #print("Frame size: %dx%d" % (self.frameWidth, self.frameHeight))

    # def run(self):
    #     self.running()

    def run(self):

        global cap
        global isStopped

        while (cap.isOpened()):
            while(isStopped):
                print(isStopped)

                # Capture frame-by-frame
                ret, frame = cap.read()

                assert ret, "Ошибка при получении картинки с камеры!"

                # преобразуем полученное цветное изображение в ч/б
                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

                intensityList = []
                for frameRowCount in range(self.frameRow):  # цикл по всем строкам

                    colIntensity = []  # создаем список для строк
                    for frameColCount in range(self.frameCol):  # цикл по всем столбцам

                        # задаем облась фрейма для вычисления средней интенсивности gray[y:y+h, x:x+w]
                        frame = gray[self.frameHeight * frameRowCount: self.frameHeight * (frameRowCount + 1),
                                self.frameWidth * frameColCount: self.frameWidth * (frameColCount + 1)]

                        intensity = int(frame.mean())  # вычисляем среднюю интенсивность области
                        colIntensity.append(intensity)  # добавляем интенсивность в список
                        font = cv2.FONT_HERSHEY_SIMPLEX
                        cv2.putText(frame, str(intensity), (0, 30), font, 1, (255, 255, 255),
                                    1)  # выводим значение интенсивности во фрейм
                        cv2.rectangle(frame, (0, 0), (self.frameWidth - 1, self.frameHeight - 1), (255, 255, 255), 1)
                    intensityList.append(
                        colIntensity)  # добавляем в список интенсивностей всего кадра, список интенсивностей строки
                # package = pickle.dumps(gray, protocol=4)

                # client.publish('pi/cam/frame', package, qos=0)

                leftSensor = intensityList[2][0]  # получаем значение левого сенсора
                rightSensor = intensityList[2][2]  # получаем значения правого сенсора

                diffSensors = int(
                    (rightSensor - leftSensor) * 0.1)  # находим разницу --> используем, как управляющее воздействие
                speed = 10  # задаём среднюю скорость
                self.leftMotor = speed - diffSensors  # получаем скорость левого мотора
                self.rightMotor = speed + diffSensors  # получаем скорость правого мотора
                if (self.leftMotor < 0):
                    self.leftMotor = 1
                elif (self.rightMotor < 0):
                    self.rightMotor = 1

                # print("Left sensor value: " + str(leftSensor) + ' Right sensor value ' + str(rightSensor))
                # print("Left speed: " + str(self.leftMotor) + " Right speed: " + str(self.rightMotor))

                can_bus.Set_motor_speed(0, self.leftMotor)  # запускаем левый мотор
                can_bus.Set_motor_speed(1, -self.rightMotor)  # запускаем правый мотор
                self.stopping = 1
            if(self.stopping == 1):
                self.stopping = 0
                can_bus.Set_motor_speed(0, 0)
                can_bus.Set_motor_speed(1, 0)

aut = autonomka()
aut.start()

O = Onliner()
O.start()
O.online_flag = True
can_bus_0 = Motor_Controller(0)
can_bus_1 = Motor_Controller(1)
can_bus_2 = Motor_Controller(2)
can_bus_3 = Motor_Controller(3)

can_bus_0.Mode(1)
can_bus_1.Mode(1)
can_bus_2.Mode(1)
can_bus_3.Mode(1)

brokerAdress = "192.168.42.12"

client = mqtt.Client()

def on_connect(client, userdata, flags, rc):  # что делаем при коннекте
    print("Connected with result code: " + str(rc))
    client.subscribe("pi/#", qos=0)  # подписываемся на топик
    client.message_callback_add("pi/motorValue/motorSpeed", Motor_callback)
    client.message_callback_add("pi/auto/auto", auto_callback)
    client.message_callback_add("pi/stop/stop", break_callback)
    client.message_callback_add("pi/motorValue/axsSpeed", axs_callback)

def on_message(client, userdata, message):
    print("Received message '" + str(message.payload) + "' on topic '"
            + message.topic + "' with QoS " + str(message.qos))

def Motor_callback(client, userdata, message):

    leftSpeed_4wd, rightSpeed_4wd, leftSpeed_2wd, rightSpeed_2wd = pickle.loads(message.payload)
    # print("Left speed: %d, Right speed: %d" % (leftSpeed, rightSpeed))
    can_bus_0.Set_motor_speed(0, leftSpeed_4wd)  # запускаем первый левый мотор
    can_bus_0.Set_motor_speed(1, -rightSpeed_4wd)  # запускаем первый правый мотор
    can_bus_1.Set_motor_speed(0, leftSpeed_4wd)  # запускаем второй левый мотор
    can_bus_1.Set_motor_speed(1, -rightSpeed_4wd) #запускаемвторой правый мотор
    can_bus_2.Set_motor_speed(0, leftSpeed_2wd)  # запускаем левый мотор двухколёски
    can_bus_2.Set_motor_speed(1, -rightSpeed_2wd) # запускаем правый мотор двухколёски
def axs_callback(client, userdata, message):
    axsSpeed = int(message.payload) #
    can_bus_3.Set_motor_speed(0, axsSpeed) #

def break_callback(client, userdata, message):
    brak = int(message.payload)
    if(brak == 1):
        can_bus_0.Mode(0)
        can_bus_1.Mode(0)
        can_bus_2.Mode(0)
        can_bus_3.Mode(0)
        print("motors disabled")
    else:
        can_bus_0.Mode(1)
        can_bus_1.Mode(1)
        can_bus_2.Mode(1)
        can_bus_3.Mode(1)
        print("Motors turned on!")
        print("Be carefull")

def auto_callback(client, userdata, message):
    global isStopped
    auto = int(message.payload)
    if(auto == 1):
        isStopped = True
        print("Auto enabled")
    else:
        print("Auto disabled")
        isStopped = False

client.connect(brokerAdress, 1883, 60)
client.on_connect = on_connect
client.on_message = on_message

client.loop_forever()
can_bus.Mode(0)