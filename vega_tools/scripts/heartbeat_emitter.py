#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool,Header

rospy.init_node('heartbeat_emitter', anonymous=True)

r = rospy.Rate(1)

heartbeat_pub = rospy.Publisher("heartbeat",Header,queue_size=1)
h =  Header()
count = 0

while not rospy.is_shutdown():
        r.sleep()
        h.seq = count
        h.stamp = rospy.get_rostime()
        heartbeat_pub.publish(h)
        count +=1
        print "ping"

