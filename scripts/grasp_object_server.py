#!/usr/bin/python
# Software License Agreement (BSD License)
#
# Copyright (c) 2014, PAL Robotics SL
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of the PAL Robotics nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# author: Bence Magyar

import rospy
import tf
from object_recognition_msgs.msg import RecognizedObjectArray
from object_recognition_clusters import ClusterBoundingBoxFinder
from actionlib import ActionServer
from reem_tabletop_grasping.msg import GraspObjectAction 

class GraspObjectServer:

    def __init__(self, name):
        # stuff for grasp planning
        self.tf_listener = tf.TransformListener()
        self.tf_broadcaster = tf.TransformBroadcaster()
        self.cbbf = ClusterBoundingBoxFinder(self.tf_listener, self.tf_broadcaster)
#        self.cbbf3d = cluster_bounding_box_finder_3d.ClusterBoundingBoxFinder3D(self.tf_listener)
        self.last_objects = None
        rospy.Subscriber("object_array", RecognizedObjectArray, self.objects_callback)
        
        # blocking action server implementation
        self.grasp_obj_as = ActionServer(name, GraspObjectAction, self.goal_callback, self.cancel_callback)

    def objects_callback(self, data):
        rospy.loginfo(rospy.get_name() + ": This message contains %d objects." % len(data.objects))
        self.last_objects = data
        
    def goal_callback(self):
        None
        
    def cancel_callback(self):
        None
        # stop motions?

if __name__ == '__main__':
    name = 'grasp_object_server'
    rospy.init_node(name, anonymous=False)
    server = GraspObjectServer(name)
    rospy.loginfo(name + ": Ready to roll.")
    rospy.spin()

