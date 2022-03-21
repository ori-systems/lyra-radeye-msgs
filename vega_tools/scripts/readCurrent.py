#!/usr/bin/env python
import rospy
from std_msgs.msg import String
import PD_comms_lib as comms_lib
from sensor_msgs.msg import Joy

def joy_callback(joyData):
    global joy_button
    joy_button = joyData.buttons[0]  # X

def currentReader():
    global joy_button
    joy_button = 0
    sub = rospy.Subscriber("joy", Joy, joy_callback)

    pub = rospy.Publisher('current', String, queue_size=10)
    rospy.init_node('currentReader', anonymous=True)
    rate = rospy.Rate(4) # hz
    while not rospy.is_shutdown():
        if joy_button:
            print "Successful"
            current = comms.read_register(comms_lib.current_12V_instant_addrs) + " mA"
            rospy.loginfo(current)
            pub.publish(current)
            rate.sleep()

if __name__ == '__main__':
    # comms = comms_lib.PD_comms_protocol('/dev/ttyACM3',False)
    comms = comms_lib.PD_comms_protocol('/dev/PD',False)
    if (not(comms.test_comms())):
        print "The board was not found, double check the connection and the usb port"
        exit()

    try:
        currentReader()
    except rospy.ROSInterruptException:
        pass
