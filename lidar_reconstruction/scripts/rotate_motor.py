#!/usr/bin/env python

from __future__ import print_function

import sys
import rospy
from dynamixel_workbench_msgs.srv import *

#h={'',1,'Goal_Position',2048}
r=474
t=0
#print('a')
#def rotate_motor(a,b,c,d):
#print('b')
rospy.wait_for_service('lidar_reconstruction/dynamixel_command')
try:
    #print('c')
    rotate = rospy.ServiceProxy('lidar_reconstruction/dynamixel_command', DynamixelCommand)#dynamixel_workbench
    #print('d')
except rospy.ServiceException as e:
    print("Service call failed: %s"%e)


if __name__ == "__main__":
    while(1):
        if r<=1124 and t==0:
            rotate2 = rotate('',5,'Goal_Position',r)
            r=r+3
            #print(r)
            if r>=1124:
                t = 1
            #else:
            #    print('error 1')
        elif r>=474 and t==1:
            r=r-3
            rotate2 = rotate('',5,'Goal_Position',r)
            if r<=474:
                t = 0
            #else:
            #    print('error 2')
        #print("Good Stuff")
