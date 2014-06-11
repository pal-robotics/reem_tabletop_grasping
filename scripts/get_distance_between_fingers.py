#!/usr/bin/env python
"""
Created on 11/06/14

@author: Sam Pfeiffer
"""

# system stuff

import numpy as np

# ROS stuff
import rospy
from geometry_msgs.msg import Point, PointStamped
import tf

if __name__ == '__main__':
    rospy.init_node("dist_bet_fingers")
    rospy.sleep(0.3)
    rospy.loginfo("Getting a TransformListener...")
    tf_listener = tf.TransformListener()
    
    # Get the point of the tip of the left hand index
    ps = PointStamped()
    ps.point = Point(0.0, 0.0, 0.0)
    ps.header.frame_id = 'hand_left_index_3_link'
    ps.header.stamp = rospy.Time(0) # For getting last transform
    
    # Transform this point to the frame reference of
    # right hand index
    got_transform = False
    while not got_transform:
        try:
            tps = tf_listener.transformPoint('hand_right_index_3_link', ps)
            got_transform = True
        except:
            print "Transformation failed, waiting 0.3 and retrying"
            rospy.sleep(0.3)
            
    # Calculate distances between points
    p1 = np.array(ps.point.__getstate__())
    print "ps looks like: " + str(ps)
    p2 = np.array(tps.point.__getstate__())
    print "pts looks like: " + str(tps)
    dist = np.linalg.norm(p1 - p2, ord=3)
    
    print "And dist is: " + str(dist)
    