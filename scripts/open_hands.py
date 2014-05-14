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

import actionlib
import rospy

from control_msgs.msg import FollowJointTrajectoryGoal, FollowJointTrajectoryAction, FollowJointTrajectoryResult, JointTolerance
from trajectory_msgs.msg import JointTrajectoryPoint

# Useful dictionary for reading in an human friendly way errors
traj_error_dict = {}
for name in FollowJointTrajectoryResult.__dict__.keys():
    if not name[:1] == '_':
        code = FollowJointTrajectoryResult.__dict__[name]
        traj_error_dict[code] = name


def createHandGoal(side, j1, j2, j3):
    """Creates a FollowJointTrajectoryGoal with the values specified in j1, j2 and j3 for the joint positions
    with the hand specified in side
    @arg side string 'right' or 'left'
    @arg j1 float value for joint 'hand_'+side+'_thumb_joint'
    @arg j2 float value for joint 'hand_'+side+'_middle_joint'
    @arg j3 float value for joint 'hand_'+side+'_index_joint'
    @return FollowJointTrajectoryGoal with the specified goal"""
    fjtg = FollowJointTrajectoryGoal()
    fjtg.trajectory.joint_names.append('hand_' + side + '_thumb_joint')
    fjtg.trajectory.joint_names.append('hand_' + side + '_middle_joint')
    fjtg.trajectory.joint_names.append('hand_' + side + '_index_joint')
    point = JointTrajectoryPoint()
    point.positions.append(j1)
    point.positions.append(j2)
    point.positions.append(j3)
    point.velocities.append(0.0)
    point.velocities.append(0.0)
    point.velocities.append(0.0)
    point.time_from_start = rospy.Duration(4.0)
    fjtg.trajectory.points.append(point)
    for joint in fjtg.trajectory.joint_names:  # Specifying high tolerances for the hand as they are slow compared to other hardware
        goal_tol = JointTolerance()
        goal_tol.name = joint
        goal_tol.position = 5.0
        goal_tol.velocity = 5.0
        goal_tol.acceleration = 5.0
        fjtg.goal_tolerance.append(goal_tol)
    fjtg.goal_time_tolerance = rospy.Duration(3)
    fjtg.trajectory.header.stamp = rospy.Time.now()
    return fjtg

if __name__ == '__main__':
    rospy.init_node('reem_opens_hands')
    rhand_as = actionlib.SimpleActionClient('/right_hand_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
    rospy.loginfo("Connecting to right hand AS...")
    rhand_as.wait_for_server()
    rospy.sleep(1.0)
    rospy.loginfo("Connected, sending goal.")
    goal = createHandGoal("right", 0.1, 0.1, 0.1)
    rhand_as.send_goal(goal)
    rospy.loginfo("Goal sent, waiting...")
    rhand_as.wait_for_result()
    hand_result = rhand_as.get_result()
    rospy.loginfo("Done with result: " + traj_error_dict[hand_result.error_code])

    lhand_as = actionlib.SimpleActionClient('/left_hand_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
    rospy.loginfo("Connecting to left hand AS...")
    lhand_as.wait_for_server()
    rospy.sleep(1.0)
    rospy.loginfo("Connected, sending goal.")
    goal = createHandGoal("left", 0.1, 0.1, 0.1)
    lhand_as.send_goal(goal)
    rospy.loginfo("Goal sent, waiting...")
    lhand_as.wait_for_result()
    hand_result = lhand_as.get_result()
    rospy.loginfo("Done with result: " + traj_error_dict[hand_result.error_code])
