#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import kinematics as kine
import base_kinematics as b_kine
import rospy, time


def main():
    rospy.init_node('kinematik')
    kine.hour_start = time.gmtime().tm_hour
    kine.min_start = time.gmtime().tm_min
    kine.sec_start = time.gmtime().tm_sec
    kine.wait_start()
    kine.hour_start = time.gmtime().tm_hour
    kine.min_start = time.gmtime().tm_min
    kine.sec_start = time.gmtime().tm_sec
    kine.move_dist_f(0,0, target_yaw=10)
    kine.move_time(1,0,12)





if __name__ == '__main__':	
    try:
        main()
    except rospy.ROSInterruptException: pass
