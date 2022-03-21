#!/usr/bin/env python

from comms_lib import PD_comms_lib as comms_lib
import rospy
from std_msgs.msg import UInt8


rospy.init_node('lights_control' , anonymous=True)

class Lights(object):
    def __init__(self):


        
        self.front_sub = rospy.Subscriber("vega_front_lights", UInt8,self.front_lights_cb)

        self.arm_sub = rospy.Subscriber("vega_arm_lights", UInt8,self.arm_lights_cb)
        # self.comms = comms_lib.PD_comms_protocol('/dev/ttyACM3',False)
        self.comms = comms_lib.PD_comms_protocol('/dev/PD',False)

        if (not(self.comms.test_comms())):
            print "The board was not found, double check the connection and the usb port"
            exit()

    def front_lights_cb(self,msg):
        self.comms.set_pwm_value(comms_lib.Front_Light_PWM_addrs,msg.data)

    def arm_lights_cb(self,msg):
        self.comms.set_pwm_value(comms_lib.CAM1_Light_PWM_addrs,msg.data)


l = Lights()
rospy.spin()

