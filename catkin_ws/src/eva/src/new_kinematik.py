#!/usr/bin/env python
from math import sin, cos, sqrt
import rospy
import roslib
from std_msgs.msg import Float64, Float64MultiArray, Bool
import time


l = 0.1          #длинна луча
velocity = 80    #скорость



pub_lmotor = rospy.Publisher('v_left', Float64, queue_size=10)
pub_rmotor = rospy.Publisher('v_right', Float64, queue_size=10)
pub_bmotor = rospy.Publisher('v_back', Float64, queue_size=10)
pub = rospy.Publisher('moveing', Bool, queue_size=1)

def x_y_local(x, y, corner_absolute=0):
    x_local = cos(corner_absolute)*x + sin(corner_absolute)*y
    y_local = cos(corner_absolute)*y - sin(corner_absolute)*x
    return x_local, y_local

def x_y_world(x, y, corner_absolute=0):
    x_world = cos(corner_absolute)*x - sin(corner_absolute)*y
    y_world = cos(corner_absolute)*y + sin(corner_absolute)*x
    return x_world, y_world

def v1v2v3(x_local, y_local, corner_change=0):             
    v1 = -x_local/2 - sqrt(3)*y_local/2 + l*corner_change
    v2 = x_local + l*corner_change
    v3 = -x_local/2 + sqrt(3)*y_local/2 + l*corner_change
    return v1, v2, v3

def v1v2v3_to_xylocal(v1, v2, v3):
    x_local = (2*v2 - v1 - v3)/3
    y_local = ((sqrt(3))*v3 - sqrt(3)*v1)/3
    corner_change = (v1, v2, v3)/3
    return x_local, y_local, corner_change

def kinematik_local(xv, yv, corner_to_change=0):
    global moveing
    moveing = True
    v1, v2, v3 = v1v2v3(xv, yv, corner_to_change)
    pub_lmotor.publish(v1)
    pub_rmotor.publish(v2)
    pub_bmotor.publish(v3)
    print(v1, v2, v3)

def kinematik_world(data):
    pub.publish(Bool(False))
    x_plus_y = data.data[0] + data.data[1]
    xv = velocity * (data.data[0] / x_plus_y)
    yv = velocity * (data.data[1] / x_plus_y)
    time_move = x_plus_y / velocity
    if len(data.data) < 3:
        corner_change = 0
    else:
        corner_change = data.data[2]
    xv, yv = x_y_local(xv, yv)
    v1, v2, v3 = v1v2v3(xv, yv, corner_change)
    print(v1, v2, v3)
    pub_lmotor.publish(v1)
    pub_rmotor.publish(v2)
    pub_bmotor.publish(v3)
    time.sleep(time_move)
    print('stop')
    pub.publish(Bool(True))


def stop():
    pub_lmotor.publish(0)
    pub_rmotor.publish(0)
    pub_bmotor.publish(0)
    print('stop')
    pub.publish(Bool(True))     #говорит, что остановился
    moveing = False

def move_local(x, y):
    distance = sqrt(x**2+y**2)
    time_move = (distance / velocity ) * 100
    xv = (x/distance)*velocity
    yv = (y/distance)*velocity
    kinematik_local(xv, yv)
    time.sleep(time_move)
    stop()


def main():
    global moveing
    rospy.init_node('kinematik')
    move_local(-2,0)
    time.sleep(2)
    move_local(0,-3)
    time.sleep(2)
    move_local(-1,1)
    time.sleep(2)
    move_local(1,0)
    time.sleep(2)
    move_local(2,-1)
    time.sleep(2)
    move_local(-1,-1)
    stop()
    # distance = sqrt(x**2+y**2)
    # time_move = distance / velocity 
    # time.sleep(time_move*2)          #едет по времени




if __name__ == '__main__':	
    try:
        main()
    except rospy.ROSInterruptException: pass
