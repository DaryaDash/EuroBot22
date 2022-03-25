#!/usr/bin/env python
from std_msgs.msg import Float64MultiArray, Bool
import rospy
from time import sleep

move_stop = True

def move(a):
    if a.data:
        pass
        #print(a.data)

def check_move_stop(data):
    global move_stop 
    move_stop = data.data
    #print(move_stop)



def main():
    rospy.init_node('local_planer')
    pub = rospy.Publisher('must_move_local', Float64MultiArray, queue_size=1)
    demo = [[1, 0.5], [0.5, -0.5], [-1, 2], [1, 1], [0, -0.7], [0, 0]]
    while not rospy.is_shutdown():
        
        if move_stop:
            if len(demo):
                flarray = Float64MultiArray()
                flarray.data = demo[0]
                pub.publish(flarray)
                print(demo[0])
                demo.pop(0)
            sleep(0.1)
        rospy.Subscriber('move_stop', Bool, check_move_stop)
        sleep(0.1)
        
        # pub.publish(flarray)
        # print(1)
        
        
if __name__ == '__main__':	
    try:
        main()
    except rospy.ROSInterruptException: pass
