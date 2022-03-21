#!/usr/bin/env python

import sys
import rospy
from dynamixel_workbench_msgs.srv import *

def setSpeed_client(motorID, value):
    rospy.wait_for_service('lidar_reconstruction/dynamixel_command')
    try:
        setSpeed = rospy.ServiceProxy('lidar_reconstruction/dynamixel_command', DynamixelCommand)
        resp1 = setSpeed('', motorID, 'Goal_Velocity', value)
        return resp1.comm_result
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

def usage():
    return "%s [motorID value]"%sys.argv[0]

if __name__ == "__main__":
    if len(sys.argv) >= 3:
        motorID = int(sys.argv[1])
        value = int(sys.argv[2])
    else:
        print usage()
        sys.exit(1)
    print "Requesting motor %s set to %s"%(motorID, value)
    print "%s and %s returns %s"%(motorID, value, setSpeed_client(motorID, value))
