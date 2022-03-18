#!/usr/bin/env python
import rospy
import roslib
from std_msgs.msg import Float64, Float64MultiArray, Bool
from geometry_msgs.msg import Twist 
from math import sin, cos, sqrt
import numpy as np
import math
from time import sleep

#############################################################
#############################################################
#############################################################
class TwistToMotors():
#############################################################
#############################################################

    #############################################################
    def __init__(self):
    #############################################################
        rospy.init_node("twist_to_motors")
        nodename = rospy.get_name()
        rospy.loginfo("%s started" % nodename)
    
        self.w = rospy.get_param("~base_width", 0.2)
    
        self.pub_lmotor = rospy.Publisher('/open_base/left_joint_velocity_controller/command', Float64, queue_size=10)
        self.pub_rmotor = rospy.Publisher('/open_base/right_joint_velocity_controller/command', Float64, queue_size=10)
        self.pub_bmotor = rospy.Publisher('/open_base/back_joint_velocity_controller/command', Float64, queue_size=10)
        self.pub = rospy.Publisher('move_stop', Bool, queue_size=1)
        rospy.Subscriber('cmd_vel', Twist, self.twistCallback)
    
    
        self.rate = rospy.get_param("~rate", 50)
        self.timeout_ticks = rospy.get_param("~timeout_ticks", 2)
        self.left = 0
        self.right = 0
        
    #############################################################
    def spin(self):
    #############################################################
    
        r = rospy.Rate(self.rate)
        idle = rospy.Rate(10)
        then = rospy.Time.now()
        self.ticks_since_target = self.timeout_ticks
    
        ###### main loop  ######
        while not rospy.is_shutdown():
        
            while not rospy.is_shutdown() and self.ticks_since_target < self.timeout_ticks:
                self.spinOnce()
                r.sleep()
            idle.sleep()
                
    #############################################################
    def spinOnce(self):
    #############################################################
        r = 0.025 
        d = 0.05 
        meh=math.pi/3
        A= np.array([[-d/r, 1/r, 0], [-d/r, -1/(2*r), (-math.sin(meh))/r], [-d/r, -1/(2*r), (math.sin(meh))/r]])
        B= np.array([self.dr,self.dx, self.dy])
        X= A @ B
        print(A)

        self.pub_lmotor.publish(X[0])
        self.pub_rmotor.publish(X[1])
        self.pub_bmotor.publish(X[2])
        self.ticks_since_target += 1

    #############################################################
    def twistCallback(self,msg):
    #############################################################
        # rospy.loginfo("-D- twistCallback: %s" % str(msg))
        self.ticks_since_target = 0
        self.dx = msg.linear.y
        self.dr = msg.angular.z
        self.dy = msg.linear.x
    
#############################################################


l = 0.1          #длинна луча
velocity = 5    #скорость


class MyKinematik():
    def __init__(self):
        self.move_stop = True
        while not rospy.is_shutdown():
            sleep(0.1)
            if self.move_stop:
                # print(1)
                rospy.init_node("twist_to_motors")
                rospy.Subscriber('must_move_local', Float64MultiArray, self.move_local)
                rospy.Subscriber('must_move_world', Float64MultiArray, self.move_world)

                self.pub_lmotor = rospy.Publisher('/open_base/left_joint_velocity_controller/command', Float64, queue_size=10)
                self.pub_rmotor = rospy.Publisher('/open_base/right_joint_velocity_controller/command', Float64, queue_size=10)
                self.pub_bmotor = rospy.Publisher('/open_base/back_joint_velocity_controller/command', Float64, queue_size=10)
                self.pub = rospy.Publisher('move_stop', Bool, queue_size=1)
                
            
    def x_y_local(self, x, y, corner_absolute=0):
        x_local = cos(corner_absolute)*x + sin(corner_absolute)*y
        y_local = cos(corner_absolute)*y - sin(corner_absolute)*x
        return x_local, y_local

    def x_y_world(self, x, y, corner_absolute=0):
        x_world = cos(corner_absolute)*x - sin(corner_absolute)*y
        y_world = cos(corner_absolute)*y + sin(corner_absolute)*x
        return x_world, y_world

    def v1v2v3(self, x_local, y_local, corner_change=0):             
        v1 = -x_local/2 - sqrt(3)*y_local/2 + l*corner_change
        v2 = x_local + l*corner_change
        v3 = -x_local/2 + sqrt(3)*y_local/2 + l*corner_change
        return v1, v2, v3

    def v1v2v3_to_xylocal(self, v1, v2, v3):
        x_local = (2*v2 - v1 - v3)/3
        y_local = ((sqrt(3))*v3 - sqrt(3)*v1)/3
        corner_change = (v1, v2, v3)/3
        return x_local, y_local, corner_change

    def move_local(self, data):
        self.pub.publish(Bool(False))
        self.move_stop = False
        x_plus_y = data.data[0] + data.data[1]
        xv = velocity * (data.data[0] / x_plus_y)
        yv = velocity * (data.data[1] / x_plus_y)
        time = x_plus_y / velocity
        if len(data.data) < 3:
            corner_change = 0
        else:
            corner_change = data.data[2]
        v1, v2, v3 = self.v1v2v3(xv, yv, corner_change)
        self.pub_lmotor.publish(v1)
        self.pub_rmotor.publish(v2)
        self.pub_bmotor.publish(v3)
        print(v1, v2, v3)
        sleep(time*5)
        self.pub_lmotor.publish(0)
        self.pub_rmotor.publish(0)
        self.pub_bmotor.publish(0)
        print('stop')
        self.pub.publish(Bool(True))     #говорит локал_планеру, что остановился
        self.move_stop = True
        sleep(0.1)

    def move_world(self, data):
        self.pub.publish(Bool(False))
        x_plus_y = data.data[0] + data.data[1]
        xv = velocity * (data.data[0] / x_plus_y)
        yv = velocity * (data.data[1] / x_plus_y)
        time = x_plus_y / velocity
        if len(data.data) < 3:
            corner_change = 0
        else:
            corner_change = data.data[2]
        xv, yv = self.x_y_local(xv, yv)
        v1, v2, v3 = self.v1v2v3(xv, yv, corner_change)
        print(v1, v2, v3)
        self.pub_lmotor.publish(v1)
        self.pub_rmotor.publish(v2)
        self.pub_bmotor.publish(v3)
        sleep(time)
        print('stop')
        self.pub.publish(Bool(True))     #говорит локал_планеру, что остановился


















#############################################################
if __name__ == '__main__':
    """ main """
    try:
        
        twistToMotors = TwistToMotors()
        MyKinematik()
        twistToMotors.spin()
    except rospy.ROSInterruptException:
        pass
