#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import numpy as np
import cv2
import time
import paho.mqtt.client as mqtt
import pickle
from RoboMind import *
'''
    motor - 1| right speed" with" "-" !!///motor-0| left
'''

# client = mqtt.Client()
# client.connect("192.168.42.12", 1883, 60)

O = Onliner()
O.start()
O.online_flag = True

can_bus = Motor_Controller(0)
can_bus.Mode(1)

imageWidth = 176
imageHeight = 144
frameCol = 3
frameRow = 3

frameWidth = int(imageWidth / frameCol)
frameHeight = int(imageHeight / frameRow)

print ("OpenCV version: %s" % cv2.__version__)

cap = cv2.VideoCapture(0)

assert cap.isOpened, "Ошибка веб-камеры!"

cap.set(cv2.CAP_PROP_FRAME_WIDTH, imageWidth)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, imageHeight)

print ("Image size: %dx%d" % (cap.get(cv2.CAP_PROP_FRAME_WIDTH), cap.get(cv2.CAP_PROP_FRAME_HEIGHT)))
print ("Frame size: %dx%d" % (frameWidth, frameHeight))

while (cap.isOpened()):

    # Capture frame-by-frame
    ret, frame = cap.read()

    assert ret, "Ошибка при получении картинки с камеры!"

    # преобразуем полученное цветное изображение в ч/б
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    intensityList = []
    for frameRowCount in range(frameRow):  # цикл по всем строкам

        colIntensity = []  # создаем список для строк
        for frameColCount in range(frameCol):  # цикл по всем столбцам

            # задаем облась фрейма для вычисления средней интенсивности gray[y:y+h, x:x+w]
            frame = gray[frameHeight * frameRowCount: frameHeight * (frameRowCount + 1),frameWidth * frameColCount: frameWidth * (frameColCount + 1)]

            intensity = int(frame.mean())  # вычисляем среднюю интенсивность области
            colIntensity.append(intensity)  # добавляем интенсивность в список
            font = cv2.FONT_HERSHEY_SIMPLEX
            cv2.putText(frame, str(intensity), (0, 30), font, 1, (255, 255, 255),1)  # выводим значение интенсивности во фрейм
            cv2.rectangle(frame, (0, 0), (frameWidth - 1, frameHeight - 1), (255, 255, 255), 1)
        intensityList.append(colIntensity)  # добавляем в список интенсивностей всего кадра, список интенсивностей строки
    package = pickle.dumps(gray, protocol=4)

    # client.publish('pi/cam/frame', package, qos=0)


    leftSensor = intensityList[2][0]#получаем значение левого сенсора
    rightSensor = intensityList[2][2]#получаем значения правого сенсора

    diffSensors = int((rightSensor - leftSensor)*0.1)#находим разницу --> используем, как управляющее воздействие
    speed = 10#задаём среднюю скорость
    leftMotor = speed - diffSensors#получаем скорость левого мотора
    rightMotor = speed + diffSensors#получаем скорость правого мотора
    if(leftMotor < 0):
        leftMotor = 1
    elif(rightMotor < 0):
        rightMotor = 1

    print("Left sensor value: " + str(leftSensor) + ' Right sensor value ' + str(rightSensor))
    print("Left speed: " + str(leftMotor) + " Right speed: " + str(rightMotor))

    can_bus.Set_motor_speed(0, leftMotor)#запускаем левый мотор
    can_bus.Set_motor_speed(1, -rightMotor)#запускаем правый мотор

    if cv2.waitKey(1) & 0xFF == ord('q'):
        can_bus.Mode(0)
        cv2.destroyAllWindows()
        break
can_bus.Mode(0)
# client.disconnect()