#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy, time
rospy.init_node('kinematik')
import kinematics as kine
import base_kinematics as b_kine
from base_kinematics import stop


# 1 - правоя сторона, -1 - левая сторона
right = 1

def main():
    kine.wait_start()

    kine.prepare_manipulators()
    go_to_statuette()
    time.sleep(5)
    replace_statuette()
    time.sleep(5)
    go_to_home_and_push_stat()

    for i in range(b_kine.send_topics):
        stop()
        b_kine.rate.sleep()


def go_to_statuette():
    b_kine.set_null_navx()
    if right:
        kine.move_aruco(-1,0,target_r=25, move_forward=False, target_yaw=0)
    else:
        kine.move_aruco(1,0,target_l=25, move_forward=False, target_yaw=0)
    kine.move_aruco(0,1,target_f=25, move_forward=True)
    kine.move_time(0,1,1, target_yaw=None)

def replace_statuette():
    b_kine.set_null_navx()
    for i in range(b_kine.send_topics):
        b_kine.pub_front_manipul.publish(False)
    time.sleep(1)
    for i in range(b_kine.send_topics):
        b_kine.pub_front_servo.publish(0)
    #statuette_pose = b_kine.get_yaw_navx()-b_kine.correct
    kine.move_time(0,-1,1, target_yaw=0)
    #time.sleep(1)
    #print(100000.00001, b_kine.correct, b_kine.get_yaw_navx(original=True), b_kine.get_yaw_navx())
    #kine.move_dist(0,0, target_yaw=90)
    kine.move_dist(0,0, target_yaw=180)
    time.sleep(1)
    #kine.move_yaw(180)
    kine.move_time(0,-1,1, target_yaw=180)
    time.sleep(1)
    for i in range(b_kine.send_topics):
        b_kine.pub_back_manipul.publish(True)


def go_to_home_and_push_stat():
    b_kine.set_null_navx()
    kine.move_dist(0,1,target_f=15, target_yaw=45*right, move_forward=True)
    time.sleep(1)
    for i in range(b_kine.send_topics):
        b_kine.pub_front_manipul.publish(True)
        b_kine.pub_front_servo.publish(1)
    kine.move_dist(0,1,target_f=30, target_yaw=45*right, move_forward=False)


if __name__ == '__main__':	
    try:
        main()
    except rospy.ROSInterruptException: pass
