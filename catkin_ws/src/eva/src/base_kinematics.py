#!/usr/bin/env python3
# -*- coding: utf-8 -*-
from math import sin, cos, sqrt
import rospy
import roslib
from std_msgs.msg import Float64, Float64MultiArray, Bool, Int8, Int64, Float32
from sensor_msgs.msg import Range
import time
import serial



right_move = 1 # 1 если робот у правого края, -1 если у левого

 
send_topics = 3  #сколько топиков отправляет за раз
debag = False
min_distance = -3   #минимальное растояние при котором он останавливается


l = 0.1          #длинна луча
velocity = 200    #скорость
cornerMotor_to_distance = 2105

x_pos, y_pos, theta_pos, l_enc, r_enc, yaw = 0,0,0,0,0,0
r_ping, l_ping, f_ping = 0,0,0
integral, prevErr = 0,0
start_button = False

timer_val = True #таймер разрешает работать
now_yaw = 0
yaw_topic = 0
correct = 0

pub_front_servo = rospy.Publisher('front_servo', Int64, queue_size=10)
pub_front_manipul = rospy.Publisher('front_grab', Bool, queue_size=10)
pub_back_servo = rospy.Publisher('back_servo', Bool, queue_size=10)
pub_back_manipul = rospy.Publisher('back_grab', Bool, queue_size=10)
pub_enc_zero = rospy.Publisher('ENC_zero', Bool, queue_size=10)
pub_yaw = rospy.Publisher('yaw', Float64, queue_size=10)
pub_linear_y = rospy.Publisher('linear_y', Float32, queue_size=10)
pub_lmotor = rospy.Publisher('v_left', Float32, queue_size=10)
pub_bmotor = rospy.Publisher('v_back', Float32, queue_size=10)
pub_rmotor = rospy.Publisher('v_right', Float32, queue_size=10)
pub = rospy.Publisher('moveing', Bool, queue_size=10)

'''

try:
    ser_navx = serial.Serial('/dev/ttyACM3')
except:
    try:
        ser_navx = serial.Serial('/dev/ttyACM2')
    except:
        try:
            ser_navx = serial.Serial('/dev/ttyACM1')
        except:
            if debag:
                pass
            else:
                ser_navx = serial.Serial('/dev/ttyACM0')


'''


line_val = 0


#движение используя pid; возвращает текущий угол и целевой
def move_navx(target_x, target_y, target_yaw=0, now_yaw=False):
    rospy.Subscriber('yaw', Float64, get_navx_ros)
    distance = sqrt(target_x**2+target_y**2)
    if distance > 0:
        xv = (target_x/distance)*velocity
        yv = (target_y/distance)*velocity
        v_left, v_right, v_back = v1v2v3(xv, yv)
    else:
        v_left, v_right, v_back = 0,0,0
    if not now_yaw:
        now_yaw = float(get_yaw_navx())
    if target_yaw == None:
        target_yaw = now_yaw
    print('input data', now_yaw, target_yaw)
    if (target_yaw - now_yaw) > 180:
        now_yaw += 360
    if (target_yaw - now_yaw) < -180:
        now_yaw -= 360
    print('2input data', now_yaw, target_yaw)
#    if target_yaw > 180:
 #       target_yaw -= 180
  #  elif target_yaw < -180:
   #     target_yaw += 180
    err = pid(now_yaw, target_yaw) #, get_yaw_navx(original=True))
    v_left = v_left-err
    v_right = v_right-err
    v_back = v_back-err
    if v_left > 255: v_left = 255
    elif v_left < -255: v_left = -255
    if v_right > 255: v_right = 255
    elif v_right < -255: v_right = -255
    if v_back > 255: v_back = 255
    elif v_back < -255: v_back = -255
    v_left *= 0.6
    v_right = -v_right
    v_back *= 0.7
    pub_lmotor.publish(v_left)
    pub_rmotor.publish(v_right)
    pub_bmotor.publish(v_back)
    #print('Yaw='+str(now_yaw)[:6], str(target_yaw), 'Motors: Left=' + str(v_left)[:6], 'Right=' + str(v_right)[:6], 'Back=' + str(v_back)[:6])
    return now_yaw, target_yaw


#переместиться на угол (простая логика)
def move_yaw(target_yaw):
    rospy.Subscriber('yaw', Float64, get_navx_ros)
    for i in range(40):
        try:
            now_yaw = get_yaw_navx()
        except:
            pass
    #err = pid(now_yaw, target_yaw, kp=1, ki=10, kd=0, dt=0.03)
    if now_yaw > target_yaw:
        right = -1
    else:
        right = 1
    v_left = -130*right
    v_right = -200*right
    v_back = -130*right
    if v_left > 255: v_left = 255
    elif v_left < -255: v_left = -255
    if v_right > 255: v_right = 255
    elif v_right < -255: v_right = -255
    if v_back > 255: v_back = 255
    elif v_back < -255: v_back = -255
    pub_lmotor.publish(v_left*0.6)
    pub_rmotor.publish(-v_right)
    pub_bmotor.publish(v_back*0.6)
    print(v_left, v_right, v_back)
    print(now_yaw)
    time.sleep(0.3)
    stop()
    stop()
    stop()
    stop()
    print('stop')



def check_distance(target_f=0, target_l=0, target_r=0, move_forward=True):
    rospy.Subscriber('range_front_ping', Range, front_ping)
    rospy.Subscriber('range_left_ping', Range, left_ping)
    rospy.Subscriber('range_right_ping', Range, right_ping)
    rospy.Subscriber('line', Int64, detection_line)
    rate.sleep()
    print('Distance left='+str(l_ping)[:6], 'front='+str(f_ping)[:6], 'right='+str(r_ping)[:6])
    if f_ping < min_distance:
        return 'shot_f'
    if l_ping < min_distance:
        return 'shot_l'
    if r_ping < min_distance:
        return 'shot_r'
    if move_forward:
        if target_f and (f_ping < target_f):
            return True
        elif target_l and (l_ping < target_l):
            return True
        elif target_r and (r_ping < target_r):
            return True
    else:
        if target_f and (f_ping > target_f):
            return True
        elif target_l and (l_ping > target_l):
            return True
        elif target_r and (r_ping > target_r):
            return True
    return False






def move_forward_navx(target_distance, target_yaw=0):
    rospy.Subscriber('yaw', Float64, get_navx_ros)
    v_left, v_right, v_back = v1v2v3(0, velocity)
    _, target_r,_ = v1v2v3(0, target_distance)
    target_r = target_r / cornerMotor_to_distance
    while r_enc < target_r*0.95 and not rospy.is_shutdown():
        now_yaw = get_yaw_navx()
        err = (now_yaw - target_yaw)*0.005
        v_left = velocity*(1-err)
        v_right = velocity*(-1-err)
        if v_left > 255: v_left = 255
        elif v_left < -255: v_left = -255
        if v_right > 255: v_right = 255
        elif v_right < -255: v_right = -255
        pub_lmotor.publish(v_left)
        pub_rmotor.publish(-v_right)
        pub_bmotor.publish(0)
        rospy.Subscriber('ENCR_POS', Float64, right_enc)
        print(v_left, v_right, now_yaw, err)
    stop()



def x_y_local(x, y, corner_absolute=0):                       #перевод из мировых координат в локальные(относительно позиции робота)
    x_local = cos(corner_absolute)*x + sin(corner_absolute)*y
    y_local = cos(corner_absolute)*y - sin(corner_absolute)*x
    return x_local, y_local

def x_y_world(x, y, corner_absolute=0):                      #перевод из локальных координат в мировые
    x_world = cos(corner_absolute)*x - sin(corner_absolute)*y
    y_world = cos(corner_absolute)*y + sin(corner_absolute)*x
    return x_world, y_world

def v1v2v3(x_local, y_local, corner_change=0):             #формулы для расчета скоростей, относительно скорости Х и У
    v_l = -x_local/2 - sqrt(3)*y_local/2 + l*corner_change
    v_b = x_local + l*corner_change
    v_r = -x_local/2 + sqrt(3)*y_local/2 + l*corner_change
    return v_l, v_r, v_b

def v1v2v3_to_xylocal(v1, v2, v3):                       #скорости на моторы перевести в координаты Х У (не используется)
    x_local = (2*v2 - v1 - v3)/3
    y_local = ((sqrt(3))*v3 - sqrt(3)*v1)/3
    corner_change = (v1, v2, v3)/3
    return x_local, y_local, corner_change

def kinematik_local(xv, yv, corner_to_change=0):                   #публикует команды на моторы в топики
    global moveing
    moveing = True
    v1, v2, v3 = v1v2v3(xv, yv, corner_to_change)
    v2 = -v2
    for i in range():
        pub_lmotor.publish(v1)
        pub_rmotor.publish(v3)
        pub_bmotor.publish(v2)
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
    for i in range(10):
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



def get_yaw_navx(original=False):

    try:
       # yaw = ser_navx.readline()[2:9]
        yaw = yaw_topic
        if original:
            return float(yaw)
        out = float(yaw) - correct
        if out > 180:
            out -= 360
        elif out < -180:
            out += 360
        pub_yaw.publish(out)
        return out
    except:
        return now_yaw

def get_navx_ros(data):
    global yaw_topic
    yaw_topic = data.data


def set_null_navx():
    global correct
    correct = get_yaw_navx(original=True)


def get_start_button(data):
    global start_button
    start_button = data.data

last_line_val = 0
detect_line = False
def detection_line(data):
    global line_val#, last_line_val
    line_val = data.data
    

def left_enc(data):
    global l_enc
    l_enc = data.data
def right_enc(data):
    global r_enc
    r_enc = data.data
def front_ping(data):
    global f_ping
    f_ping = data.range
def left_ping(data):
    global l_ping
    l_ping = data.range
def right_ping(data):
    global r_ping
    r_ping = data.range
fail = True
def fail_button(data):
    global fail
    fail = data.data


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
        # print('x: ', x_pos, end="    ")
        # print('y: ', y_pos, end="    ")
        # print("theta ", theta_pos)
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


def timer():
    global min_start, sec_start
    min_now = time.gmtime().tm_min
    sec_now = time.gmtime().tm_sec
    time_ = (min_now - min_start)* 60 + (sec_now - sec_start)
    if time_ > 100:
        while True:
            stop()
            if rospy.is_shutdown():
                break
   # elif time_ > 110:
    #    return False
    return True

def timer2():
    global min_start2, sec_start2
    min_now = time.gmtime().tm_min
    sec_now = time.gmtime().tm_sec
    time_ = (min_now - min_start)* 60 + (sec_now - sec_start)
    if time_ < 10:
        return False
    return True




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


def pid(inp, setpoint, kp=15, ki=0.01, kd=4, dt=0.03):
    global integral, prevErr
    err = setpoint - inp
    integral = integral + err * dt * ki
    if integral > 255: integral = 255
    elif integral < -255: integral = -255
    D = (err - prevErr) / dt
    res = err * kp + integral + D * kd
   # if res > 255: res = 255
   # elif res < -255: res = -255
    prevErr = err
    return res
 



def move_forward_odom(distance):
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
    




def move_forward_navx(target_distance, target_yaw=0):
    v_left, v_right, v_back = v1v2v3(0, velocity)
    _, target_r,_ = v1v2v3(0, target_distance)
    target_r = target_r / cornerMotor_to_distance
    while r_enc < target_r*0.95 and not rospy.is_shutdown():
        now_yaw = get_yaw_navx()
        err = (now_yaw - target_yaw)*0.005
        v_left = velocity*(1-err)
        v_right = velocity*(-1-err)
        if v_left > 255: v_left = 255
        elif v_left < -255: v_left = -255
        if v_right > 255: v_right = 255
        elif v_right < -255: v_right = -255
        pub_lmotor.publish(v_left)
        pub_rmotor.publish(-v_right)
        pub_bmotor.publish(0)
        rospy.Subscriber('ENCR_POS', Float64, right_enc)
        print(v_left, v_right, now_yaw, err)
    stop()






def move_navx_odom(target_x, target_y, target_yaw=0):
    distance = sqrt(target_x**2+target_y**2)
    xv = (target_x/distance)*velocity
    yv = (target_y/distance)*velocity
    v_l, v_r, v_b = v1v2v3(xv, yv)
    rospy.Subscriber('ENCR_POS', Float64, left_enc)
    time.sleep(0.1)
    first_l_enc = l_enc
    target_l, target_r,_ = v1v2v3(target_x, target_y)
    print(target_r)
    target_l = target_l * cornerMotor_to_distance
    while abs(-l_enc+first_l_enc) < abs(target_l) and not rospy.is_shutdown():
        try:
            now_yaw = get_yaw_navx()
        except:
            pass
        err = (now_yaw - target_yaw)*0.02
        v_left = v_l+err*velocity
        v_right = v_r+err*velocity
        v_back = v_b+err*velocity
        if v_left > 255: v_left = 255
        elif v_left < -255: v_left = -255
        if v_right > 255: v_right = 255
        elif v_right < -255: v_right = -255
        if v_back > 255: v_back = 255
        elif v_back < -255: v_back = -255
        pub_lmotor.publish(v_left*0.6)
        pub_rmotor.publish(-v_right)
        pub_bmotor.publish(v_back*0.6)
        rospy.Subscriber('ENCR_POS', Float64, left_enc)
        print(now_yaw, v_left*0.6, v_right, v_back*0.6)
        print(l_enc,first_l_enc,  target_l)
        #time.sleep(0.1)
    stop()













if __name__ == '__main__':	
    try:
        pass
    except rospy.ROSInterruptException: pass
