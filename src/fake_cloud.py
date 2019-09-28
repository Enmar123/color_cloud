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
        msg.width = 2
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
        data = [60,221,25,64,160,147,170,191,0,0,128,63,
                30,100,10,32,160,147,170,191,0,0,128,63]
        #data = [np.uint8(item) for item in data]
        #print(data)    
        msg.data = data
        
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            rospy.loginfo("publishing point")
            msg.header.stamp = rospy.Time.now()
            self.pub.publish(msg)
            rate.sleep()
        
    def circle(self, height, radius, count):
        ang = np.pi * 2/count
        for i in range(count)
            y = np.sin(ang * i)
            x = np.cos(ang * i)
            z = height
            



if __name__=="__main__":
    try:
        node = RosNode()
    except rospy.ROSInterruptException:
        pass