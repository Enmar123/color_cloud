#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image, PointCloud2
import sensor_msgs.point_cloud2
import tf
from tf.transformations import euler_from_quaternion 

from cv_bridge import CvBridge, CvBridgeError

import numpy as np
import copy

def uint8arr_to_float32(values):
    raw_unit8_data = np.array(values, dtype='uint8')
    num = np.fromstring(raw_unit8_data.tostring(), dtype='<f4')
    return num[0]

def break_list(my_list, n):
    final = [my_list[i * n:(i + 1) * n] for i in range((len(my_list) + n - 1) // n )]  
    return final

def pc2msg_to_points(msg):
    points = []
    for point in sensor_msgs.point_cloud2.read_points(msg, skip_nans=True):
            x = point[0]
            y = point[1]
            z = point[2]
            points.append([x,y,z])
    return points

def x_rotation(vector,theta):
    """Rotates 3-D vector around x-axis"""
    R = np.array([[1,0,0],[0,np.cos(theta),-np.sin(theta)],[0, np.sin(theta), np.cos(theta)]])
    return np.dot(R,vector)

def y_rotation(vector,theta):
    """Rotates 3-D vector around y-axis"""
    R = np.array([[np.cos(theta),0,np.sin(theta)],[0,1,0],[-np.sin(theta), 0, np.cos(theta)]])
    return np.dot(R,vector)

def z_rotation(vector,theta):
    """Rotates 3-D vector around z-axis"""
    R = np.array([[np.cos(theta), -np.sin(theta),0],[np.sin(theta), np.cos(theta),0],[0,0,1]])
    return np.dot(R,vector)

def translation(vector, xyz):
    return vector + np.array(xyz)

def transform_points(trans, euler, points):
    points = [translation(point, trans) for point in points]
    points = [z_rotation(point, euler[2])  for point in points]
    points = [y_rotation(point, euler[1])  for point in points]
    points = [x_rotation(point, euler[0])  for point in points]
    return points

def imgdata_to_matrix(data, width):
    mat = break_list(data, int(width*3))
    mat = [break_list(row, 3) for row in mat]
    return mat

def pc2_callback(msg):
    global pc2_msg
    pc2_msg = msg
   
def img_callback(msg):
    global img_msg, img_width, img_height
    img_msg = msg
    img_width = msg.width
    img_height = msg.height
    pass

#def img_callback(msg):
#    bridge = CvBridge()
#    try:
#        cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
#    except CvBridgeError as e:
#        print(e)
#    print cv_image

        

if __name__=="__main__":
    #data = [60,221,25,64,160,147,170,191,0,0,128,63,
    #        30,100,10,32,160,147,170,191,0,0,128,63]
    
    fov_width = 140 * np.pi/180
    
    rospy.init_node("color_cloud")
    rospy.loginfo("Starting Node: /color_cloud")
    rospy.Subscriber("pointcloud", PointCloud2, pc2_callback)
    rospy.Subscriber("/usb_cam/image_raw", Image, img_callback)
    
    listener = tf.TransformListener()
    bridge = CvBridge()
    
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        # Get current transform
        try:
            (trans,rot) = listener.lookupTransform('/usb_cam', '/laser', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
        
        # Get current image and pointcloud
        try:
            img_msg_now = copy.copy(img_msg)
            pc2_msg_now = copy.copy(pc2_msg)
        except NameError:
            continue
        
        # transform img data into matrix
        try:
            img = bridge.imgmsg_to_cv2(img_msg_now, "bgr8")
        except CvBridgeError as e:
            print(e)
        # Transform point data to new ref frame
        euler = euler_from_quaternion (rot)
        points = pc2msg_to_points(pc2_msg_now)
        points_new = transform_points(trans, euler, points)
        # filter out any unusable points
        points_filtered = []
        for point in points_new:
            if point[0] > 0:
                points_filtered.append(point)
        #print(points_filtered)
        # get pix size for x distance
        for point in points_filtered:
            print(point)
            x = point[0]
            y = point[1]
            z = point[2]
            pix_size = (2 * point[0] * np.tan(fov_width/2))/img_width
            # Check if point is outside image bounds
            if abs(y) > pix_size*img_width or abs(z) > pix_size*img_height:
                pass
            else:
                # Get row and column coordinates
                y_mod = img_width/2  + y/pix_size 
                z_mod = img_height/2 - z/pix_size 
                row = int(z_mod)
                col = int(y_mod)
                print((row, col))
                # Get color of that row and column
                rgb = img[row][col]
                print(rgb)
                

        #
        
        
        
        #print("points =")
        #print(np.array(points))
        #print("points_new =")
        #print(np.array(points_new))
        
        #rospy.loginfo("quat  = %s"%(str(rot)))
        #rospy.loginfo("euler = %s"%(str(euler)))
        
        rate.sleep()
        
    
