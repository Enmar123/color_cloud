#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image, PointCloud2
import sensor_msgs.point_cloud2
import tf
from tf.transformations import euler_from_quaternion 

from cv_bridge import CvBridge, CvBridgeError

import numpy as np
import copy

def pc2_callback(msg):
    for point in sensor_msgs.point_cloud2.read_points(msg, skip_nans=True):
        x = point[0]
        y = point[1]
        z = point[2]
        print([x,y,z])
            
def img_callback(msg):
    bridge = CvBridge()
    try:
        cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
    except CvBridgeError as e:
        print(e)
    print cv_image

if __name__=="__main__":
    #data = [60,221,25,64,160,147,170,191,0,0,128,63,
    #        30,100,10,32,160,147,170,191,0,0,128,63]
    
    fov_width = 140 * np.pi/180
    
    rospy.init_node("color_cloud")
    rospy.loginfo("Starting Node: /color_cloud")
    #rospy.Subscriber("pointcloud", PointCloud2, pc2_callback)
    rospy.Subscriber("/usb_cam/image_raw", Image, img_callback)
    listener = tf.TransformListener()
    
    rospy.spin()
#    rate = rospy.Rate(10)
#    while not rospy.is_shutdown():