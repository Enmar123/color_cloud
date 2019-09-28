#!/usr/bin/env python


import rospy
from sensor_msgs.msg import PointCloud2, PointField
import numpy as np
import struct

class RosNode:
    def __init__(self):
        rospy.init_node("fake_cloud")
        self.pub = rospy.Publisher("pointcloud", PointCloud2, queue_size=10)
        
        msg = PointCloud2()
        f1 = PointField()
        f2 = PointField()
        f3 = PointField()
        
        
        msg.header.frame_id = "laser"
        
        msg.height = 1
        msg.width = 10
        msg.point_step = 12
        msg.row_step = 12
        
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
        
        msg.fields = [f1, f2, f3]
        msg.is_dense = True
        
        #data = [1,1,1]
        #data = [60,221,25,64,160,147,170,191,0,0,128,63,0,0,128,63]
        #data = [60,221,25,64,160,147,170,191,0,0,128,63]
        #data = [60,221,25,64,160,147,170,191,0,0,128,63,
        #        30,100,10,32,160,147,170,191,0,0,128,63]
        #data = [np.uint8(item) for item in data]
        #print(data)
        
        points = make_circle_points(0,1,10)
        data = points_to_data(points)
        
        msg.data = data
        
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            rospy.loginfo("publishing point")
            msg.header.stamp = rospy.Time.now()
            self.pub.publish(msg)
            rate.sleep()
        
def binary(num):
    return ''.join(bin(ord(c)).replace('0b', '').rjust(8, '0') for c in struct.pack('!f', num))
    
def points_to_data(points):
    data=[]
    for point in points:
        for value in point:
            binTest = binary(value)
            bin1 = binTest[ 0: 8]
            bin2 = binTest[ 8:16]
            bin3 = binTest[16:24]
            bin4 = binTest[24:32]
            converted_value = [int(bin4,2),int(bin3,2),int(bin2,2),int(bin1,2)]
            data = data + converted_value
    return data
        
def make_circle_points(height, radius, count):
    points = []
    ang = np.pi * 2/count
    for i in range(count):
        y = np.sin(ang * i, dtype="float32")
        x = np.cos(ang * i, dtype="float32")
        z = np.float32(height)
        points.append((x,y,z))
    return points           



if __name__=="__main__":
    try:
        node = RosNode()
    except rospy.ROSInterruptException:
        pass