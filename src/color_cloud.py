#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image, PointCloud2, PointField
import sensor_msgs.point_cloud2
import tf
from tf.transformations import euler_from_quaternion 

from cv_bridge import CvBridge, CvBridgeError
import struct

import numpy as np


# ----------------------------------------------------------------------------

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

def translation2(vector, xyz):
    return vector - np.array(xyz)

def transform_points(trans, euler, points):
    """This transormer doesnt work"""
    threshold = 0.001
    #print(euler)
    points = [translation(point, trans) for point in points]
    if euler[2] > threshold:
        points = [z_rotation(point, euler[2])  for point in points] 
    if euler[1] > threshold:
        points = [y_rotation(point, euler[1])  for point in points]
    if euler[0] > threshold:
        points = [x_rotation(point, euler[0])  for point in points]
    return points

def transform_points2(trans, quat, points):
    #points = [translation(point, trans) for point in points]
    #trans_new = qv_mult(quat, trans)
    points = [qv_mult(quat, point) for point in points]
    points = [translation(point, trans) for point in points]
    #print(trans)
    #print(trans_new)
    return points
    
def qv_mult(q1, v1):
    v1_new = tf.transformations.unit_vector(v1)
    q2 = list(v1_new)
    q2.append(0.0)
    unit_vector = tf.transformations.quaternion_multiply(
                    tf.transformations.quaternion_multiply(q1, q2), 
                    tf.transformations.quaternion_conjugate(q1)
                    )[:3]
    
    vector_len = np.sqrt(v1[0]**2 + v1[1]**2 + v1[2]**2)
    vector = unit_vector * vector_len
    return vector

def pc2_callback(msg):
    global pc2_msg
    pc2_msg = msg
   
def img_callback(msg):
    global img_msg, img_width, img_height
    img_msg = msg
    img_width = msg.width
    img_height = msg.height
    pass


def setup_pc2msg():
    msg = PointCloud2()
    
    f1 = PointField()
    f2 = PointField()
    f3 = PointField()
    f4 = PointField()
    
    #msg.header.frame_id = "usb_cam"
    
    msg.height = 1
    #msg.width = 3
    msg.point_step = 20
    #msg.row_step = 30
    
    f1.name = "x"
    f1.offset = 0
    f1.datatype = 7
    f1.count = 1
    
    f2.name = "y"
    f2.offset = 4
    f2.datatype = 7
    f2.count = 1
    
    f3.name = "z"
    f3.offset = 8
    f3.datatype = 7
    f3.count = 1
     
    f4.name = "rgb"
    f4.offset = 16
    f4.datatype = 7
    f4.count = 1
    
    msg.fields = [f1, f2, f3, f4]
    msg.is_dense = False
    
    return msg
    
def binary(num):
    return ''.join(bin(ord(c)).replace('0b', '').rjust(8, '0') for c in struct.pack('!f', num))


def point_to_data(point, rgb):
    data_segment = []
    for value in point:
        binTest = binary(value)
        bin1 = binTest[ 0: 8]
        bin2 = binTest[ 8:16]
        bin3 = binTest[16:24]
        bin4 = binTest[24:32]
        converted_value = [int(bin4,2),int(bin3,2),int(bin2,2),int(bin1,2)]
        data_segment = data_segment + converted_value
    data_segment = data_segment + [0, 0, 0, 0]   #paddig
    data_segment = data_segment + [rgb[0], rgb[1], rgb[2]]
    data_segment = data_segment + [0]       # Padding
    return data_segment
    
    

if __name__=="__main__":
    
    rospy.init_node("color_cloud")
    
    # load params from launch file
    fov_width = rospy.get_param('~fov_width', 140)
    fov_width = fov_width * np.pi/180
    pointcloud_topic = rospy.get_param('~pc2_topic_in', 'pointcloud')
    image_topic = rospy.get_param('~image_topic_in', 'usb_cam/image_raw')
    pc2_topic_out = rospy.get_param('~pc2_topic_out', 'rgb_cloud')
    
    rospy.loginfo("Starting Node: color_cloud")
    rospy.Subscriber(pointcloud_topic, PointCloud2, pc2_callback)
    rospy.Subscriber(image_topic, Image, img_callback)
    pub = rospy.Publisher(pc2_topic_out, PointCloud2, queue_size=10)
    
    listener = tf.TransformListener()
    bridge = CvBridge()
    
    msg = setup_pc2msg()
    
    # wait for message data to load
    while not rospy.is_shutdown():
        try:
            img_msg_now = img_msg
            pc2_msg_now = pc2_msg
        except NameError:
            continue
        break
    
    img_frame_id = img_msg_now.header.frame_id
    pc2_frame_id = pc2_msg_now.header.frame_id
    
    
    rate = rospy.Rate(50)
    while not rospy.is_shutdown():

        # Get current transform
        try:
            (trans, quat) = listener.lookupTransform(img_frame_id, pc2_frame_id, rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
        
        # Get current image and pointcloud
        try:
            img_msg_now = img_msg
            pc2_msg_now = pc2_msg
        except NameError:
            continue
        
        # transform img data into matrix
        try:
            img = bridge.imgmsg_to_cv2(img_msg_now, "bgr8")
                          # Flip the image to correct view
            #img = np.array(img, dtype="float32")
        except CvBridgeError as e:
            print(e)
        img = np.flip(img, 1)        
            
        # Transform point data to new ref frame
        euler = euler_from_quaternion(quat)
        points = pc2msg_to_points(pc2_msg_now)
        points_new = transform_points2(trans, quat, points)
        
        # filter out any unusable points
        points_filtered = []
        for point in points_new:
            if point[0] > 0:              # Get rid of all points behind camera
                points_filtered.append(point)
                
        # get pix size for x distance
        data = []
        for point in points_filtered:    
            x = point[0]
            y = point[1]
            z = point[2]
            pix_size = (2 * x * np.tan(fov_width/2))/img_width
            # Get row and column coordinates
            y_mod = img_width/2  + y/pix_size 
            z_mod = img_height/2 - z/pix_size 
            row = int(z_mod)
            col = int(y_mod)
            # Get color of that row and column
            # Check if point is inside image bounds
            if 0 <= col < img_msg_now.width and 0 <= row < img_msg_now.height:    
                rgb = img[row][col]
                # convert the data to pc2data
                data_segment = point_to_data(point, rgb)
                # add the data
                data = data + data_segment
        
        rospy.loginfo("publishing point")
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = img_msg_now.header.frame_id
        msg.width = int(len(data)/msg.point_step)
        msg.data = data
        msg.row_step = msg.width * msg.point_step
        pub.publish(msg)

        rate.sleep()
        
    
