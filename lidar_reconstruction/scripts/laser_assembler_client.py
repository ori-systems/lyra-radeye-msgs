#!/usr/bin/env python
import roslib
#roslib.load_manifest('laser_assembler')
import rospy
from laser_assembler.srv import *
from lidar_reconstruction.srv import *
import sensor_msgs.msg as sensmsg






#try:
#    theta_camera_control = rospy.ServiceProxy(theta_camera_service, TakeImage)
#    resp = theta_camera_control()
#    print "Camera Image Captured"
#except rospy.ServiceException, e:
#    print "Service call failed: %s"%e

class lidar_assembly_service(object):
    """docstring for lidar_assmebly_service."""
    def __init__(self):
	print("a")
        laser_assembler_service_name = 'assemble_scans2'
 #       theta_camera_service = 'theta_camera_control'
 #       self.cameraFlag = bool(0)

        rospy.init_node("laser_assembler_caller", anonymous=True)
        rospy.wait_for_service(laser_assembler_service_name)

 #       try:
 #           rospy.wait_for_service(theta_camera_service, timeout=10.0) # Wait for camera service, if not that after 10 seconds, forget about it
 #           self.take_theta_image = rospy.ServiceProxy(theta_camera_service, TakeImage)
 #           self.cameraFlag = bool(1)
 #       except:
 #           rospy.loginfo("Theta camera service not available")
 #           self.cameraFlag = bool(0)
        self.assemble_scans = rospy.ServiceProxy(laser_assembler_service_name, AssembleScans2)  # Make service caller for the laser assembler service

        self.assembledPointcloudPublisher = rospy.Publisher("assembled_pcl", sensmsg.PointCloud2, queue_size=2)
        #rospy.sleep(1.0)  # Give ROS master time to sort things out
	print("b")
        self.rqst_srv_obj = rospy.Service('request_assembled_pcl', TakeImage, self.callForScans)

    def callForScans(self, serviceConfig):
        try:
 #           if self.cameraFlag:
 #               cam_resp = self.take_theta_image()
            print("c")
            rospy.sleep(5) # Wait for 30 seconds
            print("d")
            resp = self.assemble_scans(rospy.Time(0,0), rospy.get_rostime())  # Request assembly of all scans between now since forever (total is limited by buffer)
            self.assembledPointcloudPublisher.publish(resp.cloud)  # Publish the assembled cloud
            print "Published cloud with %u points" % (resp.cloud.height * resp.cloud.width)
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
        return TakeImageResponse()



if __name__ == "__main__":
    try:
        print("Starting Lidar Assembly Service")
        s = lidar_assembly_service()
        rospy.spin()
    except rospy.ROSInterruptException: pass
