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
from reem_tabletop_grasping.msg import GraspObjectFeedback
from actionlib_msgs.msg import GoalStatus

class GraspObjectServer:

    def __init__(self, name):
        # stuff for grasp planning
        self.tf_listener = tf.TransformListener()
        self.tf_broadcaster = tf.TransformBroadcaster()
        self.cbbf = ClusterBoundingBoxFinder(self.tf_listener, self.tf_broadcaster)
#        self.cbbf3d = cluster_bounding_box_finder_3d.ClusterBoundingBoxFinder3D(self.tf_listener)
        self.last_objects = None
        #rospy.Subscriber("object_array", RecognizedObjectArray, self.objects_callback)
        rospy.Subscriber("/recognized_object_array", RecognizedObjectArray, self.objects_callback)
        
        # blocking action server
        
        self.grasp_obj_as = ActionServer(name, GraspObjectAction, self.goal_callback, self.cancel_callback, False)
        self.feedback = GraspObjectFeedback()
        self.current_goal = None
        self.grasp_obj_as.start()

    def objects_callback(self, data):
        rospy.loginfo(rospy.get_name() + ": This message contains %d objects." % len(data.objects))
        self.last_objects = data
        
    def goal_callback(self, goal):
        if self.current_goal:
          goal.set_rejected("Server busy")
          return
        elif not self.last_objects:
          goal.set_rejected("No objects to grasp were received on the objects topic.")
          return
        else:
          #store and accept new goal
          self.current_goal = goal
          self.current_goal.set_accepted()
          #run grasping state machine
          self.grasping_sm()
          #finished, get rid of goal
          self.current_goal = None
        
    def cancel_callback(self, goal):
        #TODO stop motions?
        self.current_goal.set_canceled()

    def grasping_sm(self):
      if self.current_goal:
        self.update_feedback("Running clustering")
        #self.current_goal.set_aborted()
        rospy.sleep(1.0)
        self.update_feedback("check reachability")
        rospy.sleep(1.0)
        self.update_feedback("generate grasps")
        rospy.sleep(1.0)
        self.update_feedback("setup planning scene")
        rospy.sleep(1.0)
        self.update_feedback("execute grasps")
        rospy.sleep(1.0)
        self.current_goal.set_succeeded()
        
    def update_feedback(self, text):
        self.feedback.last_state = text
        self.current_goal.publish_feedback(self.feedback)

if __name__ == '__main__':
    name = 'grasp_object_server'
    rospy.init_node(name, anonymous=False)
    server = GraspObjectServer(name)
    rospy.loginfo(name + ": Ready to roll.")
    rospy.spin()

