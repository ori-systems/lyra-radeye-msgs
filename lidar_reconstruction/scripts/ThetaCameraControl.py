#!/usr/bin/env python

import rospy
import subprocess
import sensor_msgs.msg as senmsg
from lidar_reconstruction.srv import *

import cv2

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class thetaCamControl(object):
    """docstring for ."""
    def __init__(self):
        self.initCam()
        rospy.init_node('theta_camera_control')
        srv = rospy.Service('cameraControl', TakeImage, self.srvCallback)

        self._image_topic = "theta/image_raw"
        self._frame_id = "theta_camera"
        self._image_publisher = rospy.Publisher(self._image_topic, Image, queue_size=3)

        self.recentFileHex = ""
        self.recentFileName = ""

        self._cv_bridge = CvBridge()
        self._image_folder = "/media/rosbag/"



    def initCam(self):
        initString = ''
        timeout = 0
        #Test connection to camera
        while len(initString) == 0:
            initString = subprocess.Popen("ptpcam -i| grep RICOH", shell=True, stdout=subprocess.PIPE).stdout.read()
            rospy.sleep(0.1)
            timeout += 1
            if timeout > 10:
                break
                rospy.shutdown()

    def takeImage(self):
        returnString = subprocess.call(["ptpcam","-c"])
        listFiles = subprocess.Popen("ptpcam -L", shell=True, stdout=subprocess.PIPE).stdout.read()
        self.recentFileHex = listFiles.split()[-5].strip(":")
        self.recentFileName = listFiles.split()[-1]

    def imgGrabRecent(self):
        subprocess.call(["ptpcam","--get-file="+self.recentFileHex], cwd=self._image_folder) #cwd changes the working directory for the subprocess

    def imgPubRecent(self):
        print("Reading Image")
        cv_image = cv2.imread((self._image_folder + self.recentFileName))
        print("Publishing Image")
        if cv_image is not None:
            ros_msg = self._cv_bridge.cv2_to_imgmsg(cv_image, "bgr8")
            ros_msg.header.frame_id = self._frame_id
            ros_msg.header.stamp = rospy.Time.now()
            self._image_publisher.publish(ros_msg)
            rospy.sleep(1.0)

    def srvCallback(self, request):
        self.takeImage()
        self.imgGrabRecent()
        self.imgPubRecent()
        return TakeImageResponse()




if __name__ == "__main__":
    try:
        print("Starting Ricoh Theta V Camera Service")
        s = thetaCamControl()
        rospy.spin()
    except rospy.ROSInterruptException: pass
