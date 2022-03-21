#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool,Header

rospy.init_node('heartbeat_reciever', anonymous=True)


class heartbeat(object):
    def __init__(self):
        self.prev_stamp = rospy.get_rostime()
        self.heartbeat_sub = rospy.Subscriber("heartbeat",Header,self.heartbeat_cb)
        self.rtb_pub = rospy.Publisher("vega_rtb",Bool,queue_size=1)
        self.r = rospy.Rate(1)

    def heartbeat_cb(self,msg):
        #self.prev_stamp = msg.stamp
        self.prev_stamp = rospy.get_rostime() 
        print "pong"




    def run(self):
        while not rospy.is_shutdown():
                self.r.sleep()
                if (rospy.get_rostime() -self.prev_stamp).to_sec() > 10.0:
                    print rospy.get_rostime().to_sec(), self.prev_stamp.to_sec(), rospy.get_rostime().to_sec()- self.prev_stamp.to_sec()
                    x = Bool()
                    x.data = True
                    self.rtb_pub.publish(x)
                    
                
            
hb = heartbeat()
hb.run()
