#!/usr/bin/env python

import rospy
from geometry_msgs.msg import TwistStamped,Twist
from std_msgs.msg import Bool

class Mux():
	def __init__(self):


		self.pub = rospy.Publisher("cmd_vel", Twist, queue_size=10)

		self.teleop_sub = rospy.Subscriber("cmd_vel_teleop", TwistStamped, self.teleop_callback)
		self.move_base_sub = rospy.Subscriber("cmd_vel_movebase", Twist, self.movebase_callback)
		self.pubsoftware_killswitch_sub = rospy.Subscriber("software_killswitch", Bool, self.killswitch_callback)
		self.rtb_sub = rospy.Subscriber("vega_rtb", Bool, self.rtb_callback)
		self.killswitch_var = False
		self.timeout_var = False
		self.rtb_var = False
		self.last_teleop_time = rospy.Time.now()
		self.last_movebase_time = rospy.Time.now()
		self.teleop_msg = Twist()
		self.movebase_msg = Twist()

	def teleop_callback(self,msg):
		self.teleop_msg = msg.twist
		#self.last_teleop_time = msg.header.stamp
		self.last_teleop_time = rospy.Time.now()

	def movebase_callback(self,msg):
		self.movebase_msg = msg
		self.last_movebase_time = rospy.get_rostime()

	def killswitch_callback(self,msg):
		self.killswitch_var = msg.data
		
	def rtb_callback(self,msg):
		self.rtb_var = msg.data

	def run(self):
		#should probably be called timeout

		r = rospy.Rate(10)
		while not rospy.is_shutdown():
			r.sleep()
			timenow = rospy.Time.now()
			print (timenow - self.last_teleop_time).to_sec()
			if  (((timenow - self.last_teleop_time).to_sec() > 1.0) & (self.rtb_var == False)):
				x = Twist()
				self.pub.publish(x)
				print "case1"
			elif (((timenow - self.last_movebase_time).to_sec() <= 1.0) & (self.rtb_var == True)):
				self.pub.publish(self.movebase_msg)
				print "case2"
			elif ((timenow - self.last_teleop_time).to_sec() <= 1.0):
				self.pub.publish(self.teleop_msg)
				print "case3"
			elif ((timenow - self.last_movebase_time).to_sec() <= 1.0):
				self.pub.publish(self.movebase_msg)
				print "case4"
			else: 
				x = Twist()
				self.pub.publish(x)
				print "case5"				


rospy.init_node('cmd_vel_mux', anonymous=True)

mux = Mux()
mux.run()