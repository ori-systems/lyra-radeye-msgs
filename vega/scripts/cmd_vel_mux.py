#!/usr/bin/env python

import rospy
from geometry_msgs.msg import TwistStamped,Twist
from std_msgs.msg import Bool

class Mux():
	def __init__(self):


		self.pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)
		self.teleop_sub = rospy.Subscriber("cmd_vel_teleop", TwistStamped, self.teleop_callback)
		self.pubsoftware_killswitch_sub = rospy.Subscriber("software_killswitch", Bool, self.killswitch_callback)
		self.killswitch_var = False
		self.timeout_var = False
		self.lasttime = rospy.Time.now()

	def teleop_callback(self,msg):


		if (self.killswitch_var == False ):
			
			try:
				x = Twist()
				if float(msg.twist.linear.x) > 0.22:
					x.linear.x = 0.21
				elif  float(msg.twist.linear.x) < -0.22:
					x.linear.x = -0.21
				else:
					x.linear.x = float(msg.twist.linear.x)
				x.linear.y = 0.0
				x.linear.z = 0.0

				x.angular.x = 0.0
				x.angular.y = 0.0
				if float(msg.twist.angular.z) > 0.5:
					x.angular.z = 0.5
				elif float(msg.twist.angular.z) < -0.5:
					x.angular.z = -0.5
				else:
					x.angular.z = float(msg.twist.angular.z)
			except:
				x = Twist()
				x.linear.x = 0.0
				x.linear.y = 0.0
				x.linear.z = 0.0

				x.angular.x = 0.0
				x.angular.y = 0.0
				x.angular.z = 0.0				

			self.pub.publish(x)
			#self.lasttime = msg.header.stamp
			self.lasttime = rospy.Time.now()
		else:
			x = Twist()
			x.linear.x = 0.0
			x.linear.y = 0.0
			x.linear.z = 0.0

			x.angular.x = 0.0
			x.angular.y = 0.0
			x.angular.z = 0.0

			self.pub.publish(x)


	def killswitch_callback(self,msg):
		self.killswitch_var = msg.data
		

	def run(self):
		#should probably be called timeout

		r = rospy.Rate(10) # 10hz
		while not rospy.is_shutdown():

			timenow = rospy.Time.now()
			if  (timenow - self.lasttime).to_sec() > 1.0:
				x = Twist()
				self.pub.publish(x)
			r.sleep()

		#check if time now - last time is greater than 0.5 seconds 
		# if yes publish blank cmd_vel 
		




rospy.init_node('cmd_vel_mux', anonymous=True)

mux = Mux()
mux.run()
