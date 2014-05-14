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
# author: Sammy Pfeiffer

import rospy
import actionlib
from reem_tabletop_grasping.msg import ObjectManipulationAction, ObjectManipulationGoal, ObjectManipulationResult
from moveit_msgs.msg import MoveItErrorCodes
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from std_msgs.msg import Header

OBJECT_MANIP_AS = '/object_manipulation_server'

if __name__ == '__main__':
    rospy.init_node("send_pick_and_place_")
    rospy.loginfo("Connecting to reem_tabletop_grasping AS: '" + OBJECT_MANIP_AS + "'")
    manip_as = actionlib.SimpleActionClient(OBJECT_MANIP_AS, ObjectManipulationAction)
    manip_as.wait_for_server()
    rospy.loginfo("Succesfully connected.")

    goal = ObjectManipulationGoal()
    goal.group = "right_arm_torso"
    goal.operation = ObjectManipulationGoal.PICK
    p = Pose(position=Point(0.3, -0.3, 1.1), orientation=Quaternion(0.0, 0.0, 0.0, 1.0))
    target = PoseStamped(header=Header(frame_id="base_link"), pose=p)
    goal.target_pose = target

    rospy.loginfo("Sending pick goal:\n" + str(goal))
    manip_as.send_goal(goal)
    manip_as.wait_for_result()
    result = manip_as.get_result()
    rospy.loginfo("Got result:\n" + str(result))
    if not result.error_code == MoveItErrorCodes.SUCCESS:
        rospy.logerr("Not executing place as pick failed")
        exit()

    goal = ObjectManipulationGoal()
    goal.group = "right_arm_torso"
    goal.operation = ObjectManipulationGoal.PLACE
    p = Pose(position=Point(0.3, -0.3, 1.1), orientation=Quaternion(0.0, 0.0, 0.0, 1.0))
    target = PoseStamped(header=Header(frame_id="base_link"), pose=p)
    goal.target_pose = target

    rospy.loginfo("Sending place goal:\n" + str(goal))
    manip_as.send_goal(goal)
    manip_as.wait_for_result()
    result = manip_as.get_result()
    rospy.loginfo("Got result:\n" + str(result))
