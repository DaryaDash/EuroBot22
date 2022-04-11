#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy, time
rospy.init_node('kinematik')
import kinematics as kine
import base_kinematics as b_kine
from base_kinematics import stop


def main():
    kine.wait_start()
    kine.prepare_manipulators()
    #kine.move_yaw_aruco()
    b_kine.set_null_navx()
    kine.move_aruco(1,0,target_l=25, move_forward=False, target_yaw=0)
    kine.move_aruco(0,1,target_f=25, move_forward=True, target_yaw=0)
    time.sleep(1)
    kine.move_aruco(0,1,target_f=10, target_yaw=None)
    stop()
    time.sleep(1)
    #kine.move_time(0,1,1, target_yaw=None)
    #kine.move_yaw_aruco()
    for i in range(10):
        b_kine.pub_front_manipul.publish(False)
    stop()
    time.sleep(2)
    b_kine.pub_front_servo.publish(0)
    statuette_pose = b_kine.get_yaw_navx()-b_kine.correct
    kine.move_time(0,1,2, target_yaw=0)
    time.sleep(1)
    b_kine.set_null_navx()
    print(100000.00001, b_kine.correct, b_kine.get_yaw_navx(original=True), b_kine.get_yaw_navx())

    kine.move_dist_f(0,0, target_yaw=90)
    kine.move_dist_f(0,0, target_yaw=180)
    time.sleep(1)
    #kine.move_yaw(180)
    kine.move_time(0,-1,1, target_yaw=None)
    time.sleep(1)
    for i in range(10):
        b_kine.pub_back_manipul.publish(True)
    time.sleep(1)
    b_kine.set_null_navx()
    kine.move_dist_f(0,1,target_f=30, target_yaw=45, move_forward=True)
    time.sleep(1)
    time.sleep(1)
    for i in range(5):
        b_kine.pub_front_manipul.publish(True)
        b_kine.pub_front_servo.publish(1)
    for i in range(10):
        stop()
        b_kine.rate.sleep()
    #return






if __name__ == '__main__':	
    try:
        main()
    except rospy.ROSInterruptException: pass
