#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Sat Sep 28 13:32:07 2019

@author: lattice
"""

#!/usr/bin/env python

import rospy

import tf
import tf2_ros
from geometry_msgs.msg import TransformStamped

import numpy as np

if __name__ == '__main__':
    rospy.init_node('tf_broadcaster')
    rospy.loginfo("starting tf_broadcaster")
    broadcaster = tf2_ros.StaticTransformBroadcaster()
    
    static_tf_0 = TransformStamped()
    static_tf_1 = TransformStamped()
    
    while not rospy.is_shutdown():
        try:
            
            static_tf_0.header.stamp = rospy.Time.now()
            static_tf_0.header.frame_id = "base_link"
            static_tf_0.child_frame_id = "laser"
        
            static_tf_0.transform.translation.x = float('0')
            static_tf_0.transform.translation.y = float('0')
            static_tf_0.transform.translation.z = float('0')
        
            quat = tf.transformations.quaternion_from_euler(float('0.00'),float('0.00'),float('0.00'))
            static_tf_0.transform.rotation.x = quat[0]
            static_tf_0.transform.rotation.y = quat[1]
            static_tf_0.transform.rotation.z = quat[2]
            static_tf_0.transform.rotation.w = quat[3]
            
            static_tf_1.header.stamp = rospy.Time.now()
            static_tf_1.header.frame_id = "base_link"
            static_tf_1.child_frame_id = "usb_cam"
        
            static_tf_1.transform.translation.x = float('1')
            static_tf_1.transform.translation.y = float('0')
            static_tf_1.transform.translation.z = float('0')
        
            quat = tf.transformations.quaternion_from_euler(float('0.00'),float('0.00'),float('0.00'))
            static_tf_1.transform.rotation.x = quat[0]
            static_tf_1.transform.rotation.y = quat[1]
            static_tf_1.transform.rotation.z = quat[2]
            static_tf_1.transform.rotation.w = quat[3]
            
            broadcaster.sendTransform([static_tf_0,
                                       static_tf_1])
        
        except rospy.ROSInterruptException:
            pass
        
        rospy.spin()