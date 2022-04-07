#!/usr/bin/env python
# -*- coding: utf-8 -*-
import base_kinematics as b_kine
import rospy, time

b_kine.velocity = 200

aruco_data = 100
b_kine.rate = rospy.Rate(50) # hz

def move_dist_f(target_x, target_y, target_yaw=0, target_f=0, target_l=0, target_r=0, move_forward=True):
    start_yaw = b_kine.get_yaw_navx()-b_kine.correct
    while(not b_kine.check_distance(target_f=target_f, target_l=target_l, target_r=target_r, move_forward=move_forward)
    and not rospy.is_shutdown() and check_time()):
        b_kine.move_navx(target_x, target_y, target_yaw)
        if target_x == 0 and target_y == 0 and abs(b_kine.get_yaw_navx()-b_kine.correct - target_yaw) < 15:
            b_kine.stop()
            break
    b_kine.stop()

def move_time(target_x, target_y, target_time, target_yaw=0, move_forward=True):
    hour_before = time.gmtime().tm_hour
    min_before = time.gmtime().tm_min
    sec_before = time.gmtime().tm_sec
    while(check_time_action(hour_before, min_before, sec_before, target_time) and not rospy.is_shutdown() and check_time()):
        b_kine.move_navx(target_x, target_y, target_yaw)
    for i in range(b_kine.send_topics):
        b_kine.stop()

def move_yaw_aruco():
    rospy.Subscriber('aruco_stat', b_kine.Float32, get_aruco_data)
    while(abs(aruco_data) > 10 and not rospy.is_shutdown() and check_time()):
        b_kine.move_navx(0,0,0, now_yaw=-aruco_data/5)
        rospy.Subscriber('aruco_stat', b_kine.Float32, get_aruco_data)
        b_kine.rate.sleep()
    for i in range(b_kine.send_topics):
        b_kine.stop()


def move_aruco(target_x, target_y, target_yaw=0, target_f=0, target_l=0, target_r=0, move_forward=True):
    rospy.Subscriber('aruco_stat', b_kine.Float32, get_aruco_data)
    while(not b_kine.check_distance(target_f=target_f, target_l=target_l, target_r=target_r, move_forward=move_forward)
    and not rospy.is_shutdown() and check_time()):
        b_kine.move_navx(target_x, target_y, target_yaw=0, now_yaw=-aruco_data/20)
        rospy.Subscriber('aruco_stat', b_kine.Float32, get_aruco_data)
        b_kine.rate.sleep()
        if target_x == 0 and target_y == 0 < 10:
            b_kine.stop()
            break

def prepare_to_start():
    for i in range(b_kine.send_topics):
        b_kine.pub_front_servo.publish(1)
        b_kine.pub_front_manipul.publish(False)

def wait_start():
    while  not b_kine.start_button and not rospy.is_shutdown():
        rospy.Subscriber('range_front_ping', b_kine.Range, b_kine.front_ping)
        rospy.Subscriber('range_left_ping', b_kine.Range, b_kine.left_ping)
        rospy.Subscriber('range_right_ping', b_kine.Range, b_kine.right_ping)
        print(b_kine.get_yaw_navx(), b_kine.l_ping, b_kine.f_ping, b_kine.r_ping)
        rospy.Subscriber('start', b_kine.Bool, b_kine.get_start_button)
        b_kine.correct = b_kine.get_yaw_navx()
        b_kine.rate.sleep()



def check_time():
    hour_now = time.gmtime().tm_hour
    min_now = time.gmtime().tm_min
    sec_now = time.gmtime().tm_sec
    time_ = (hour_now - hour_start)* 3600 + (min_now - min_start)* 60 + (sec_now - sec_start)
    if time_ > 100:
        while True and not rospy.is_shutdown():
            b_kine.stop()
            b_kine.rate.sleep()
    return True

def check_time_action(hour_start, min_start, sec_start, target_time):
    hour_now = time.gmtime().tm_hour
    min_now = time.gmtime().tm_min
    sec_now = time.gmtime().tm_sec
    time_ = (hour_now - hour_start)* 3600 + (min_now - min_start)* 60 + (sec_now - sec_start)
    if time_ >= target_time:
        return False
    return True


def get_aruco_data(data):
    global aruco_data
    aruco_data = data.data
