#!/usr/bin/env python

#Alex Brown
#2019

import roslib
roslib.load_manifest('arucomeasure')
import sys
import rospy
#from cv2 import cv
from std_msgs.msg import *
from geometry_msgs.msg import *
#from preview_filter.msg import * #this is very important! we have custom message types defined in this package!!
from sensor_msgs.msg import Image,CompressedImage,CameraInfo
from visualization_msgs.msg import Marker #we will use this message for the perceived fish. then pop it into Rviz
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from numpy import pi
import math
import cv2
import tf
from numpy import *
import time

class measure_marker:

    def __init__(self):
        #dictionary of aruco markers
        self.dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        # self.dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_ARUCO_ORIGINAL)

        self.CI = CameraInfo()

        self.D = np.array([-0.40541413163196455, 0.09621547958919903, 0.029070017586547533, 0.005280797822816339, 0.0])
        self.K = np.array([[529.8714858851022, 0.0, 836.4563887311622], [0.0, 1547.2605077363528, 83.19276259345895], [0.0, 0.0, 1.0]])
        self.frame = 'camera'

        #this is how we get our image in to use openCV
        self.bridge = CvBridge()
        # self.namespace = rospy.get_param('namespace','arucomeasure')
        # self.cam_topic = rospy.get_param('imagetopic','/camera1/usb_cam1/image_raw/compressed')
        # self.caminfo_topic = rospy.get_param('caminfotopic','/camera1/usb_cam1/camera_info')
        # rospy.logwarn(str(rospy.get_param_names()))
        # rospy.logwarn(str(rospy.has_param('~namespace')))
        # time.sleep(2)
        self.namespace = rospy.get_param("~namespace")
        self.cam_topic = rospy.get_param("~imagetopic")
        self.caminfo_topic = rospy.get_param("~caminfotopic")
        self.image_sub = rospy.Subscriber(self.cam_topic,CompressedImage,self.callback,queue_size=1)#change this to proper name!
        self.info_sub = rospy.Subscriber(self.caminfo_topic,CameraInfo,self.infocallback,queue_size=1)
        self.markerpub = rospy.Publisher('/measuredmarker',Marker,queue_size=1)
        self.imagepub = rospy.Publisher(self.namespace+'/overlay_image',Image,queue_size=1)
        self.timenow = rospy.Time.now()

        self.cam_pos = (0,0,18*.0254)
        self.cam_quat = tf.transformations.quaternion_from_euler(0,0,0)

    def infocallback(self,data):
        self.CI = data
        self.K = array([self.CI.K]).reshape(3,3)
        self.D = array([self.CI.D])
        self.frame = data.header.frame_id
        #print self.CI1.P

    #this function fires whenever a new image_raw is available. it is our "main loop"
    def callback(self,data):
        try:
            #use the np_arr thing if subscribing to compressed image
            #frame = self.bridge.imgmsg_to_cv2(data, "bgr8")
            np_arr = np.fromstring(data.data, np.uint8)
            # Decode to cv2 image and store
            frame= cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        except CvBridgeError, e:
          print e
        self.timenow = rospy.Time.now()
        rows,cols,depth = frame.shape
        frame_out = frame.copy()
        if rows>0:
            corns,ids,rejected = cv2.aruco.detectMarkers(frame,self.dict)
            
            if ids is not None:
                rvecs,tvecs = cv2.aruco.estimatePoseSingleMarkers(corns[0],.025,self.K,self.D)
                
                frame_out =cv2.aruco.drawDetectedMarkers(frame.copy(),corns,ids)


                #print rvecs
                quat = tf.transformations.quaternion_from_euler(rvecs[0][0][0],rvecs[0][0][1],rvecs[0][0][2])
                br = tf.TransformBroadcaster()
                br.sendTransform((tvecs[0][0][0],tvecs[0][0][1],tvecs[0][0][2]),quat,self.timenow,self.namespace+'/measuredmarker',self.frame)
                # br.sendTransform(self.cam_pos,self.cam_quat,self.timenow,self.namespace+'/camera',self.frame)
                #publish a marker representing the fish body position
                marker = Marker()
                marker.header.frame_id=self.namespace+'/measuredmarker'
                marker.header.stamp = self.timenow
                marker.type = marker.CUBE
                # marker.mesh_resource = 'package://servo_ros/meshes/body.dae'
                # marker.mesh_use_embedded_materials = True
                marker.action = marker.MODIFY
                marker.scale.x = .2
                marker.scale.y = .2
                marker.scale.z = .01
                tempquat = tf.transformations.quaternion_from_euler(0,0,0)#this is RELATIVE TO  ORIENTATION IN TF (does the mesh have a rotation?)
                marker.pose.orientation.w = tempquat[3]
                marker.pose.orientation.x = tempquat[0]
                marker.pose.orientation.y = tempquat[1]
                marker.pose.orientation.z = tempquat[2]
                marker.pose.position.x = 0
                marker.pose.position.y = 0
                marker.pose.position.z = 0
                marker.color.r = .8
                marker.color.g = .5
                marker.color.b = .5
                marker.color.a = 1.0#transparency

                self.markerpub.publish(marker)
            img_out = self.bridge.cv2_to_imgmsg(frame_out, "8UC3")
            img_out.header.stamp = rospy.Time.now()
            self.imagepub.publish(img_out)

def main(args):
  
  rospy.init_node('measure_marker', anonymous=True)
  ic = measure_marker()
  
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print "Shutting down"
  cv2.DestroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
