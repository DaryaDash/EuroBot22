#!/usr/bin/env python
from math import sin, cos, sqrt
import rospy
import roslib
from std_msgs.msg import Float64, Float64MultiArray, Bool
import time


l = 0.1          #длинна луча
velocity = 255    #скорость
cornerMotor_to_distance = 0.0007412223

x_pos, y_pos, theta_pos, l_enc, r_enc = 0,0,0,0,0


pub_lmotor = rospy.Publisher('v_left', Float64, queue_size=10)
pub_bmotor = rospy.Publisher('v_back', Float64, queue_size=10)
pub_rmotor = rospy.Publisher('v_right', Float64, queue_size=10)
pub = rospy.Publisher('moveing', Bool, queue_size=1)

def x_y_local(x, y, corner_absolute=0):                       #перевод из мировых координат в локальные(относительно позиции робота)
    x_local = cos(corner_absolute)*x + sin(corner_absolute)*y
    y_local = cos(corner_absolute)*y - sin(corner_absolute)*x
    return x_local, y_local

def x_y_world(x, y, corner_absolute=0):                      #перевод из локальных координат в мировые
    x_world = cos(corner_absolute)*x - sin(corner_absolute)*y
    y_world = cos(corner_absolute)*y + sin(corner_absolute)*x
    return x_world, y_world

def v1v2v3(x_local, y_local, corner_change=0):             #формулы для расчета скоростей, относительно скорости Х и У
    v1 = -x_local/2 - sqrt(3)*y_local/2 + l*corner_change
    v2 = x_local + l*corner_change
    v3 = -x_local/2 + sqrt(3)*y_local/2 + l*corner_change
    return v1, -v3, v2

def v1v2v3_to_xylocal(v1, v2, v3):                       #скорости на моторы перевести в координаты Х У (не используется)
    x_local = (2*v2 - v1 - v3)/3
    y_local = ((sqrt(3))*v3 - sqrt(3)*v1)/3
    corner_change = (v1, v2, v3)/3
    return x_local, y_local, corner_change

def kinematik_local(xv, yv, corner_to_change=0):                   #публикует команды на моторы в топики
    global moveing
    moveing = True
    v1, v2, v3 = v1v2v3(xv, yv, corner_to_change)
    pub_lmotor.publish(v1)
    pub_rmotor.publish(v3)
    pub_bmotor.publish(v2)
    pub_lmotor.publish(v1)
    pub_rmotor.publish(v2)
    pub_bmotor.publish(v3)
    pub_lmotor.publish(v1)
    pub_rmotor.publish(v2)
    pub_bmotor.publish(v3)
    pub_lmotor.publish(v1)
    pub_rmotor.publish(v2)
    pub_bmotor.publish(v3)
    pub_lmotor.publish(v1)
    pub_rmotor.publish(v2)
    pub_bmotor.publish(v3)
    pub_lmotor.publish(v1)
    pub_rmotor.publish(v2)
    pub_bmotor.publish(v3)
    pub_lmotor.publish(v1)
    pub_rmotor.publish(v2)
    pub_bmotor.publish(v3)
    pub_lmotor.publish(v1)
    pub_rmotor.publish(v2)
    pub_bmotor.publish(v3)
    pub_lmotor.publish(v1)
    pub_rmotor.publish(v2)
    pub_bmotor.publish(v3)
    pub_lmotor.publish(v1)
    pub_rmotor.publish(v2)
    pub_bmotor.publish(v3)
    pub_lmotor.publish(v1)
    pub_rmotor.publish(v2)
    pub_bmotor.publish(v3)
    pub_lmotor.publish(v1)
    pub_rmotor.publish(v2)
    pub_bmotor.publish(v3)
    print(v1, v2, v3)

def kinematik_world(data):               #не готово
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


def stop():                                    #функция для остановки робота, публикует нули в топики
    pub_lmotor.publish(0)
    pub_rmotor.publish(0)
    pub_bmotor.publish(0)
    pub_lmotor.publish(0)
    pub_rmotor.publish(0)
    pub_bmotor.publish(0)
    pub_lmotor.publish(0)
    pub_rmotor.publish(0)
    pub_bmotor.publish(0)
    pub_lmotor.publish(0)
    pub_rmotor.publish(0)
    pub_bmotor.publish(0)
    pub_lmotor.publish(0)
    pub_rmotor.publish(0)
    pub_bmotor.publish(0)
    pub_lmotor.publish(0)
    pub_rmotor.publish(0)
    pub_bmotor.publish(0)
    pub_lmotor.publish(0)
    pub_rmotor.publish(0)
    pub_bmotor.publish(0)
    pub_lmotor.publish(0)
    pub_rmotor.publish(0)
    pub_bmotor.publish(0)
    pub_lmotor.publish(0)
    pub_rmotor.publish(0)
    pub_bmotor.publish(0)
    pub_lmotor.publish(0)
    pub_rmotor.publish(0)
    pub_bmotor.publish(0)
    pub_lmotor.publish(0)
    pub_rmotor.publish(0)
    pub_bmotor.publish(0)
    pub_lmotor.publish(0)
    pub_rmotor.publish(0)
    pub_bmotor.publish(0)
    print('stop')
    pub.publish(Bool(True))     #говорит, что остановился
    moveing = False

def move_local_time(x, y):                            #принимает Х, У; считает время, которое ему нужно двигаться; движется и останавливается 
    distance = sqrt(x**2+y**2)
    time_move = (distance / velocity ) * 100
    xv = (x/distance)*velocity
    yv = (y/distance)*velocity
    kinematik_local(xv, yv)
    time.sleep(time_move)
    stop()




def get_position_odom(odom_l, odom_r, odom_b):
    global x_pos, y_pos, theta_pos
    x_pos = (2*odom_b - odom_l - odom_r)/3 * cornerMotor_to_distance
    y_pos = (sqrt(3)*odom_r - sqrt(3)*odom_l)/3 * cornerMotor_to_distance
    theta_pos = (odom_l + odom_r + odom_b)/3*l



def left_enc(data):
    global l_enc
    l_enc = data.data
def right_enc(data):
    global r_enc
    r_enc = data.data    


def move_local_odom(x, y):                            #принимает Х, У; движется по координатам и останавливается 
    distance = sqrt(x**2+y**2)
    xv = (x/distance)*velocity
    yv = (y/distance)*velocity
    kinematik_local(xv, yv)
    delta_x = 10
    delta_y = 10
    while (abs(delta_x) > 1 or abs(delta_y) > 1) and not rospy.is_shutdown():
        rospy.Subscriber('ENCL_POS', Float64, left_enc)
        rospy.Subscriber('ENCR_POS', Float64, right_enc)
        get_position_odom(l_enc, r_enc, 0)
        delta_x = x - x_pos
        delta_y = y - y_pos
        print('x: ', x_pos, end="      ")
        print('y: ', y_pos, end="      ")
        print("theta ", theta_pos)
    stop()



dt = 0.3

def vel_odom():
    global vel_left_odom, vel_right_odom
    rospy.Subscriber('ENCL_POS', Float64, left_enc)
    rospy.Subscriber('ENCR_POS', Float64, right_enc)
    time.sleep(0.1)
    left_last = l_enc
    right_last = r_enc
    time.sleep(1)
    rospy.Subscriber('ENCL_POS', Float64, left_enc)
    rospy.Subscriber('ENCR_POS', Float64, right_enc)
    time.sleep(0.1)
    vel_left_odom = (l_enc - left_last)
    vel_right_odom = (r_enc - right_last)
    print('L: ', vel_left_odom, 'R: ', vel_right_odom, 'L_enc: ', l_enc, "R_enc: ", r_enc)
    return vel_left_odom, vel_right_odom




kp= 0.7
kd = 0.0
ki = 0
def corect_left_motor(prevErr):
    vel_left_odom, vel_right_odom = vel_odom()
    err = vel_left_odom - vel_right_odom
    integ = err * dt
    d = (err - prevErr) / dt
    prevErr = err
    output =  err * kp + ki*integ + kd * 3
    pub_lmotor.publish(output)
    print(output)
    return prevErr







def move_forward(distance):
    rospy.Subscriber('ENCL_POS', Float64, left_enc)
    rospy.Subscriber('ENCR_POS', Float64, right_enc)
    v1, v2, v3 = v1v2v3(0, velocity*0.7)
    err = (l_enc + r_enc)*0.1
    v1 = v1 - err
    v2 = v2 + err
    if v1 > 255: v1 = 255
    elif v1 < -255: v1 = -255
    if v2 > 255: v2 = 255
    elif v2 < -255: v2 = -255
    pub_lmotor.publish(v1)
    pub_rmotor.publish(v2)
    print(v1, v2, l_enc, r_enc, err)
    



def main():                              #главный код
    global moveing
    rospy.init_node('kinematik')
    # print('x: ', x_pos)
    # print('y: ', y_pos)
    # print('theta: ', theta_pos)
    # print()
   # kinematik_local(100, 250)
   # time.sleep(2)
   # stop()
   # time.sleep(1)
   # kinematik_local(100, -250)
   # pub_bmotor.publish(200)
   # time.sleep(2)
    # move_local_time(-0.7,3)
    # time.sleep(2)
    # move_local_time(0.7,-3)
    #pub_rmotor.publish(150)
    prevErr = 0
    while not rospy.is_shutdown():
        move_forward(10)
        time.sleep(0.3)
    stop()










if __name__ == '__main__':	
    try:
        main()
    except rospy.ROSInterruptException: pass
