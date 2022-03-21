#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool,Header
from geometry_msgs.msg import PointStamped,PoseStamped
from vega_tools.msg import SignalStrength


rospy.init_node('heartbeat_reciever', anonymous=True)


class RTBControl(object):
    def __init__(self):

        self.clicked_point = rospy.Subscriber("clicked_point",PointStamped,self.clickedpoint_cb)
        self.rtb_sub = rospy.Subscriber("vega_rtb",Bool,self.rtb_cb)
        self.signal_sub = rospy.Subscriber("signal_strength",SignalStrength,self.signal_cb)
        
        self.goal_pub = rospy.Publisher("/move_base_simple/goal",PoseStamped,queue_size=1)
        self.home_pub = rospy.Publisher("home_pose",PoseStamped,queue_size=1)
        self.rtb_pub = rospy.Publisher("vega_rtb",Bool,queue_size=1)
        

        self.r = rospy.Rate(1)
        
        self.rtb_pose = PoseStamped()
        self.rtb_pose.header.frame_id = "map"
        self.rtb_pose.header.stamp = rospy.get_rostime()
        self.rtb_pose.pose.orientation.w = 1.0
        self.home_pub.publish(self.rtb_pose)
        self.external_rtb = False
        self.signal_strength = SignalStrength()
        self.signal_rtb = False

        self.signal_strength_threshold = rospy.get_param("signal_strenth_threshold",10)
        self.signal_quality_threshold = rospy.get_param("signal_quality_threshold",10)


    def clickedpoint_cb(self,msg):
        self.rtb_pose.pose.position.x = msg.point.x
        self.rtb_pose.pose.position.y = msg.point.y
        self.rtb_pose.pose.position.z = msg.point.z
        self.rtb_pose.header.frame_id = msg.header.frame_id
        self.rtb_pose.header.stamp = rospy.get_rostime()
        self.home_pub.publish(self.rtb_pose)


    def rtb_cb(self,msg):
        self.external_rtb = msg.data

    def signal_cb(self,msg):
        self.signal_strength = msg
        print self.signal_strength.signal , self.signal_strength_threshold ,self.signal_strength.quality , self.signal_quality_threshold
        if ((self.signal_strength.signal < self.signal_strength_threshold) | (self.signal_strength.quality < self.signal_quality_threshold)):
            self.signal_rtb = True
            print "low signal"
        else:
            self.signal_rtb = False



    def run(self):
        while not rospy.is_shutdown():
            self.r.sleep()
            if self.signal_rtb | self.external_rtb:
                self.rtb_pose.header.stamp = rospy.get_rostime()
                self.goal_pub.publish(self.rtb_pose)
                x = Bool()
                x.data = True
                self.rtb_pub.publish(x)
                print "rtb"
            else:
               print "fine"



                    
                
            
rtb = RTBControl()
rtb.run()
