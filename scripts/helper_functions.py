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
# author: Bence Magyar

# ROS stuff
from tf.transformations import quaternion_from_euler, euler_from_quaternion
import rospy
# Messages
from geometry_msgs.msg import Pose, PoseStamped, PoseArray, Vector3Stamped, Vector3, Quaternion, Point
from trajectory_msgs.msg import JointTrajectoryPoint, JointTrajectory
from moveit_msgs.msg import GripperTranslation, MoveItErrorCodes
from moveit_msgs.msg import PickupGoal, PlaceAction, PlaceGoal, PlaceResult, PlaceLocation
# System stuff
import numpy as np
from math import radians, pi, sqrt

# Useful dict for getting the string of an error code
moveit_error_dict = {}
for name in MoveItErrorCodes.__dict__.keys():
    if not name[:1] == '_':
        code = MoveItErrorCodes.__dict__[name]
        moveit_error_dict[code] = name


def createPlaceGoal(place_pose, group="right_arm_torso", target="part"):
    """ Create PlaceGoal with the provided data"""
    placeg = PlaceGoal()
    placeg.group_name = group
    placeg.attached_object_name = target
    placeg.place_locations = createPlaceLocations(place_pose)
    placeg.allowed_planning_time = 5.0
    placeg.planning_options.planning_scene_diff.is_diff = True
    placeg.planning_options.planning_scene_diff.robot_state.is_diff = True
    placeg.planning_options.plan_only = False
    placeg.planning_options.replan = True
    placeg.planning_options.replan_attempts = 10
    placeg.allow_gripper_support_collision = False
    placeg.allowed_touch_objects = ['table']  # TODO: Sometimes refuses to do the movement if this is not set, research about table names returned
    #placeg.planner_id
    return placeg


def createPlaceLocations(posestamped, deg_step=15):
    """Create a list of PlaceLocation of the object rotated every deg_step, defaults to 15 degrees"""
    place_locs = []
    for yaw_angle in np.arange(0, 2 * pi, radians(deg_step)):
        pl = PlaceLocation()
        pl.place_pose = posestamped
        newquat = quaternion_from_euler(0.0, 0.0, yaw_angle)
        pl.place_pose.pose.orientation = Quaternion(newquat[0], newquat[1], newquat[2], newquat[3])
        pl.pre_place_approach = createGripperTranslation(Vector3(0.0, 0.0, -1.0))
        pl.post_place_retreat = createGripperTranslation(Vector3(0.0, 0.0, 1.0))
        pl.post_place_posture = getPreGraspPosture()
        place_locs.append(pl)
    return place_locs


def createGripperTranslation(direction_vector, desired_distance=0.15, min_distance=0.01):
    """Returns a GripperTranslation message with the direction_vector and desired_distance and min_distance in it.
    Intended to be used to fill the pre_grasp_approach and post_grasp_retreat field in the Grasp message."""
    g_trans = GripperTranslation()
    g_trans.direction.header.frame_id = "base_link"
    g_trans.direction.header.stamp = rospy.Time.now()
    g_trans.direction.vector.x = direction_vector.x
    g_trans.direction.vector.y = direction_vector.y
    g_trans.direction.vector.z = direction_vector.z
    g_trans.desired_distance = desired_distance
    g_trans.min_distance = min_distance
    return g_trans


def getPreGraspPosture():
    """Returns our pregrasp posture JointTrajectory message to fill Grasp message"""
    pre_grasp_posture = JointTrajectory()
    pre_grasp_posture.header.frame_id = "base_link"
    pre_grasp_posture.header.stamp = rospy.Time.now()
    pre_grasp_posture.joint_names = ["hand_right_thumb_joint", "hand_right_index_joint", "hand_right_middle_joint"]
    pos = JointTrajectoryPoint()  # pre-grasp with thumb down and fingers open
    pos.positions.append(1.5)
    pos.positions.append(0.01)
    pos.positions.append(0.01)
    pos.time_from_start = rospy.Duration(3.0)
    pre_grasp_posture.points.append(pos)
    return pre_grasp_posture


def createPickupGoal(target, possible_grasps, group="right_arm_torso"):
    """Create a PickupGoal with the provided data."""
    pug = PickupGoal()
    pug.target_name = target
    pug.group_name = group
    pug.possible_grasps.extend(possible_grasps)
    pug.allowed_planning_time = 5.0
    pug.planning_options.planning_scene_diff.is_diff = True
    pug.planning_options.planning_scene_diff.robot_state.is_diff = True
    pug.planning_options.plan_only = False
    pug.planning_options.replan = True
    pug.planning_options.replan_attempts = 10
    #pug.attached_object_touch_links = ['arm_right_5_link', "hand_right_grasping_frame"]
    #pug.allowed_touch_objects.append(target)
    #pug.attached_object_touch_links.append('all')
    return pug  # Hehe, we return a dog


def dist_between_poses(pose1, pose2):
    """Calculates the distance between two points in euclidean space using numpy
    poses can be Pose or PoseStamped"""
    if type(pose1) == type(Pose()):
        p1 = np.array(pose1.position.__getstate__())
    elif type(pose1) == type(PoseStamped()):
        p1 = np.array(pose1.pose.position.__getstate__())

    if type(pose2) == type(Pose()):
        p2 = np.array(pose2.position.__getstate__())
    elif type(pose2) == type(PoseStamped()):
        p2 = np.array(pose2.pose.position.__getstate__())

    dist = np.linalg.norm(p1 - p2, ord=3)
    return dist
