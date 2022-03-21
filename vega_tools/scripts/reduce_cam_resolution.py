#!/usr/bin/env python


import rospy

import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image,CameraInfo
from std_msgs.msg import Bool
import rospkg
from datetime import datetime

#class to intercept the images from the camera
# the node then makes a copy of th eimag reduces the resolution using opencv and 
#publishes both the original and reduced resolution image with their respective camere models
# this allow the reduced resolution image to be used by orbslaw whilst the full resolution image can be 
#passed to node which saves the images for MVS

class resolutionReducer():
    def __init__(self,w,h,d,ct,ci):
        self._width = w
        self._height = h
        self._working_dir = d
        self.camera_info_topic = ci
        self.camera_topic = ct
        self._camera_info_sub = rospy.Subscriber(self.camera_info_topic, CameraInfo,self.cam_callback)
        self._camera_callback = rospy.Subscriber(self.camera_topic,Image,self.callback)
        self._picture_callback = rospy.Subscriber('/capture_image',Bool,self.picture_callback)
        self._camInfo = CameraInfo()
        self._reducedResCamInfo = CameraInfo()
        self._bridge = CvBridge()
        self._img = None
        self._img_pub = rospy.Publisher("/camera/rgb/image_raw",Image,queue_size=1)
        self._rimg_pub = rospy.Publisher("/reduced/rgb/image_raw",Image,queue_size=1)
        self._camInfo_pub = rospy.Publisher("/camera/rgb/camera_info",CameraInfo,queue_size=1)
        self._reduced_camInfo_pub = rospy.Publisher("/reduced/rgb/cam_info",CameraInfo,queue_size=1)

        self.populateRCI()



    def populateRCI(self):
        # populate reduced resolution camera model. This should really be pulled in from a config file and not hard coded.
        if self._img is not None:
            self._reducedResCamInfo.header.stamp = rospy.get_rostime()
            self._reducedResCamInfo.height = self._height
            self._reducedResCamInfo.width = self._width
            self._reducedResCamInfo.distortion_model = 'plumb_bob'
            #This might need changing if the camera intrinsics change dramically when reducing resolution
            self._reducedResCamInfo.D = self._camInfo.D #[ 1.8376550328000904e-01,-8.8572323677855724e-01,0.,0.,8.1508364492711838e-01]
            self._reducedResCamInfo.K = self._camInfo.K #[3.0111534027864121e+02,0.,1.5950000000000000e+02,0.,3.0111534027864121e+02,1.1950000000000000e+02,0.,0.,1]
            self._reducedResCamInfo.R = self._camInfo.R #[1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
            self._reducedResCamInfo.P = self._camInfo.P #[3.0111534027864121e+02,0.,1.5950000000000000e+02,0,0.,3.0111534027864121e+02,1.1950000000000000e+02,0,0.,0.,1,0]

    def cam_callback(self,msg):
        #intercept camera info
        self._camInfo = msg

    def picture_callback(self,msg):
        
        if msg.data == True:
            if self._img is not None:
                fname = self._working_dir + datetime.now().strftime("%Y-%b-%d-%H:%M:%S.%f") + '.png'
                cv2.imwrite(fname, self._img) 
                print "image saved to: ", fname
            else:
                print "No Video stream detected!"

    def callback(self,msg):
        #intercept camera image and process the image
        self._img = self._bridge.imgmsg_to_cv2(msg, "bgr8")
        reducedResImage = cv2.resize(self._img, (self._width,self._height), interpolation = cv2.INTER_AREA)  
        self._img_pub.publish(self._bridge.cv2_to_imgmsg(self._img, "bgr8"))
        self._rimg_pub.publish(self._bridge.cv2_to_imgmsg(reducedResImage, "bgr8"))
        self._camInfo_pub.publish(self._camInfo)
        self._reduced_camInfo_pub.publish(self._reducedResCamInfo)
        



def main():
    rospy.init_node('res_reducer', anonymous=True)
    rospack = rospkg.RosPack()
    path = rospack.get_path('vega_tools')    

    w = rospy.get_param('reduced_width', 320)
    h = rospy.get_param('reduced_height', 240)
    wd = rospy.get_param('working_directory', 'images/')
    ct = rospy.get_param('camera_topic','/camera/color/image_raw')
    ci = rospy.get_param('camera_info_topic','/camera/color/camera_info')


    print ct
    print ci

    d = path + "/" +wd

    print "Images will be saved here: ",d

    rR = resolutionReducer(w,h,d,ct,ci)

    r = rospy.Rate(50) # 1hz 
    while not rospy.is_shutdown():
        r.sleep()

        
if __name__ == "__main__":
    main()
