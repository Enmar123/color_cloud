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

def translation(vector, xyz):
    return vector + np.array(xyz)

def transform_points(trans, quat, points):
    points = [qv_mult(quat, point) for point in points]
    points = [translation(point, trans) for point in points]

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
    
    msg = setup_pc2msg()
    
    pc2_frame_id = pc2_msg.header.frame_id
    
#    p = PointCloud2()
#    p.header.stamp.
    
    while not rospy.is_shutdown():

        # Get current transform
        #tf_time = rospy.Time(pc2_msg.header.stamp.secs, pc2_msg.header.stamp.nsecs )
        try:
            (trans, quat) = listener.lookupTransform(img_frame_id,
                                                     pc2_frame_id,
                                                     pc2_msg.header.stamp)
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
        break
        
    try:
	img = np.flip(img, 1) 
    except:
        img = np.fliplr(img)
    
        
    # Transform point data to new ref frame
    points = pc2msg_to_points(pc2_msg_now)
    points_new = transform_points(trans, quat, points)
    

    pts = np.array(points_new).tolist()
    # Get rid of all points behind camera
    pts_fil = []
    for pt in pts:
        if pt[2] > 0:              
            pts_fil.append(pt)
            
    # filter based on FOV and then color points
    pts_color = []
    for point in pts_fil:    
        x = point[0]
        y = point[1]
        z = point[2]
        pix_width = (2 * z * np.tan(fov_width/2))/img_width
        pix_height = (2 * z * np.tan(fov_height/2))/img_height
        # Get row and column coordinates
        y_mod = img_width/2  + y/pix_height + height_offset
        x_mod = img_height/2 - x/pix_width  + width_offset
        row = int(y_mod)
        col = int(x_mod)
        # Check if point is inside image bounds
        if 0 <= col < img_msg_now.width and 0 <= row < img_msg_now.height:    
            rgb = img[row][col]              # Get color of that row and column
            pts_color.append([point[0],point[1],point[2]] + list(rgb))    
            
    # Convert points with rgb to data
    data = []      
    for point in pts_color:
        # convert the data to p/rs435_camera/color/image_rawc2dat
        data_segment = point_to_data(point[0:3], point[3:])
        # add the data
        data = data + data_segment
    
    rospy.loginfo("publishing points")
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = img_msg_now.header.frame_id
    msg.width = int(len(data)/msg.point_step)
    msg.data = data
    msg.row_step = msg.width * msg.point_step
    pub.publish(msg)
    #rospy.loginfo(pts_color[0])
    
def pts_filter_color(points):
    """ Filter out any unusable points and color the remaining """
    pts = np.array(points).tolist()
    # Get rid of all points behind camera
    pts_fil = []
    for pt in pts:
        if pt[2] > 0:              
            pts_fil.append(pt)
            
    # get pix size for x distance
    pts_col = []
    for point in pts_fil:    
        x = point[0]
        y = point[1]
        z = point[2]
        pix_width = (2 * z * np.tan(fov_width/2))/img_width
        pix_height = (2 * z * np.tan(fov_height/2))/img_height
        # Get row and column coordinates
        y_mod = img_width/2  + y/pix_height + height_offset
        x_mod = img_height/2 - x/pix_width  + width_offset
        row = int(y_mod)
        col = int(x_mod)
        # Check if point is inside image bounds
        if 0 <= col < img_msg_now.width and 0 <= row < img_msg_now.height:    
            rgb = img[row][col]              # Get color of that row and column
            pts_col.append(point + rgb)

   
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
    #print rgb
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
    fov_width = rospy.get_param('~fov_width', 54)
    fov_width = fov_width * np.pi/180
    fov_height = rospy.get_param('~fov_height', 41)
    fov_height = fov_height * np.pi/180
    width_offset = rospy.get_param('~width_offset', 90)
    height_offset = rospy.get_param('~height_offset', -100)
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
            #pc2_msg_now = pc2_msg
        except NameError:
            continue
        break
    
    img_frame_id = img_msg_now.header.frame_id
    #pc2_frame_id = pc2_msg_now.header.frame_id
    
    rospy.spin()
        
    
