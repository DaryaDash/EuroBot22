#!/usr/bin/env python
import rospy
import serial
try:
    ser = serial.Serial('/dev/ttyACM0')
except:
    try:
        ser = serial.Serial('/dev/ttyACM1')
    except:
        ser = serial.Serial('/dev/ttyACM2')
while not rospy.is_shutdown():
    yaw = ser.readline()[2:9]
    print(yaw)
