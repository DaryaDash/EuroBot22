#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy, time
rospy.init_node('kinematik')
import kinematics as kine
import base_kinematics as b_kine
from base_kinematics import stop


def main():
    kine.hour_start = time.gmtime().tm_hour
    kine.min_start = time.gmtime().tm_min
    kine.sec_start = time.gmtime().tm_sec
    kine.wait_start()
    kine.hour_start = time.gmtime().tm_hour
    kine.min_start = time.gmtime().tm_min
    kine.sec_start = time.gmtime().tm_sec
    b_kine.pub_front_servo.publish(1)
    for i in range(b_kine.send_topics):
        b_kine.pub_front_manipul.publish(True)
    for i in range(1):
        kine.move_yaw_aruco()
        time.sleep(1)
    kine.move_aruco(0,1,target_f=10, target_yaw=None)
    stop()
    time.sleep(1)
    #kine.move_yaw_aruco()
    for i in range(10):
        b_kine.pub_front_manipul.publish(False)
    stop()
    time.sleep(2)
    kine.move_time(0,-1,1, target_yaw=None)
    kine.move_dist_f(0,-1,target_f=10, target_yaw=None, move_forward=False)
    time.sleep(3)
    for i in range(1):
        b_kine.pub_front_manipul.publish(True)
    for i in range(10):
        stop()
        b_kine.rate.sleep()
    return
    kine.move_dist_f(-1,0, target_yaw=0, target_r = 25, move_forward=False)
    kine.move_dist_f(0,1, target_yaw=0, target_f = 20)
    kine.move_dist_f(0,-1, target_yaw=45, target_f = 65, move_forward=False)
    kine.move_dist_f(0,1, target_yaw=0, target_f = 25)
    kine.move_time(0,0,2, target_yaw=-90)
    kine.move_dist_f(-1,0, target_yaw=-90, target_r = 60, move_forward=False)
    kine.move_dist_f(0,1, target_yaw=-145, target_f = 30)
    stop()
    time.sleep(1)
    for i in range(10):
        b_kine.pub_front_manipul.publish(False)
    #kine.move_time(1,-1,2, target_yaw=-45)





if __name__ == '__main__':	
    try:
        main()
    except rospy.ROSInterruptException: pass
