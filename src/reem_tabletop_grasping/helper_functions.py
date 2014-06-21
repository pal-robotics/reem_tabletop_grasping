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
from std_msgs.msg import Header
from moveit_msgs.msg import GripperTranslation, MoveItErrorCodes, Constraints
from moveit_msgs.msg import PickupGoal, PlaceAction, PlaceGoal, PlaceResult, PlaceLocation
from moveit_msgs.srv import GetCartesianPath, GetCartesianPathRequest, GetCartesianPathResponse
from moveit_msgs.srv import ExecuteKnownTrajectory, ExecuteKnownTrajectoryRequest, ExecuteKnownTrajectoryResponse
from play_motion_msgs.msg import PlayMotionGoal
from control_msgs.msg import FollowJointTrajectoryGoal, FollowJointTrajectoryAction, FollowJointTrajectoryResult, JointTolerance
from trajectory_msgs.msg import JointTrajectoryPoint 
from moveit_msgs.msg import MoveGroupGoal, MoveGroupResult, MoveGroupAction, Constraints, PositionConstraint, OrientationConstraint, MoveItErrorCodes
from shape_msgs.msg import SolidPrimitive
# System stuff
import numpy as np
from math import radians, pi, sqrt
import copy

# Useful dict for getting the string of an error code
moveit_error_dict = {}
for name in MoveItErrorCodes.__dict__.keys():
    if not name[:1] == '_':
        code = MoveItErrorCodes.__dict__[name]
        moveit_error_dict[code] = name


def createPlaceGoal(place_pose, group="right_arm_torso", target="_undefined_target_"):
    """Create PlaceGoal with the provided data"""
    placeg = PlaceGoal()
    placeg.group_name = group
    placeg.attached_object_name = target
    if type(place_pose) == type(PoseStamped()):
        placeg.place_locations = createPlaceLocations(place_pose)
    elif type(place_pose) == type(Pose()):
        placeg.place_locations = createPlaceLocations(PoseStamped(header=Header(frame_id="base_link"), pose=place_pose))
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

def createCartesianPathRequest(frame_id, group_name, waypoints, max_step=0.01, jump_threshold=0.0, avoid_collisions=True, path_constraints=Constraints()):
    """Create a GetCartesianPathRequest with the specified data"""
    gcpr = GetCartesianPathRequest()
    gcpr.header.frame_id = frame_id
    gcpr.group_name = group_name
    gcpr.max_step = max_step
    gcpr.jump_threshold = jump_threshold
    gcpr.avoid_collisions = avoid_collisions
    gcpr.waypoints = copy.deepcopy(waypoints)
    gcpr.path_constraints = path_constraints
    return gcpr
    
def createExecuteKnownTrajectoryRequest(trajectory, wait_for_execution=True):
    """Create a ExecuteKnownTrajectoryRequest from the given data,
    trajectory must be a RobotTrajectory probably filled from a GetCartesianPath call"""
    ektr = ExecuteKnownTrajectoryRequest()
    ektr.trajectory = trajectory
    ektr.wait_for_execution = wait_for_execution
    return ektr
    
def createPlayMotionGoal(motion_name, priority=1, skip_planning=False):
    """Create a PlayMotionGoal with given data"""
    pmg = PlayMotionGoal()
    pmg.motion_name = motion_name
    pmg.priority = priority
    pmg.skip_planning = skip_planning
    return pmg

def createTorsoGoal(j1, j2, time=3.0):
    """Creates a FollowJointTrajectoryGoal with the values specified in j1 and j2 for the joint positions
    @arg j1 float value for head_1_joint
    @arg j2 float value for head_2_joint
    @returns FollowJointTrajectoryGoal with the specified goal"""
    fjtg = FollowJointTrajectoryGoal()
    fjtg.trajectory.joint_names.append('torso_1_joint')
    fjtg.trajectory.joint_names.append('torso_2_joint')
    point = JointTrajectoryPoint()
    point.positions.append(j1)
    point.positions.append(j2)
    point.velocities.append(0.0)
    point.velocities.append(0.0)
    point.time_from_start = rospy.Duration(time)
    for joint in fjtg.trajectory.joint_names:  # Specifying high tolerances
        goal_tol = JointTolerance()
        goal_tol.name = joint
        goal_tol.position = 5.0
        goal_tol.velocity = 5.0
        goal_tol.acceleration = 5.0
        fjtg.goal_tolerance.append(goal_tol)
    fjtg.goal_time_tolerance = rospy.Duration(3)

    fjtg.trajectory.points.append(point)
    fjtg.trajectory.header.stamp = rospy.Time.now()
    return fjtg

def createBendGoal(height):
    """Create a FollowJointTrajectoryGoal with """
    # calculate rads torso 2 by height
    # !!!!!!!!!!!!!! OLD DATA OLD DATA OLD DATA
    # there is a relation like 0.00 rad == 1.10m
    #                          0.23 rad == 0.99m
    #                          0.63 rad == 1.84m
    # We can figure this out as linear
    # TODO: Change this to be able to pick up things from higher than 1.10
    # for now this is just a test proof of concept
    if height > 1.10:
        print "HEIGHT HIGHER THAN 1.10, NOT POSSIBLE ATM!"
    height_diff = 1.10 - 0.84
    given_height = 1.10 - height
    max_torso_rads = 0.63
    torso_rads = 0.63 * given_height / float(height_diff)
    if torso_rads > 0.70: # upper joint limit, 45 deg = 0.78 rad... calibration sucks, so we set it even lower
        torso_rads = 0.70
    elif torso_rads < -0.18: # lower joint limit, -15 deg = 0.26 rad... calibration sucks, so we set it even lower
        torso_rads = -0.18
    goal = createTorsoGoal(0.0, torso_rads)
    return goal
   
def createHeadGoal(j1, j2, time=4.0):
    """Creates a FollowJointTrajectoryGoal with the values specified in j1 and j2 for the joint positions
    @arg j1 float value for head_1_joint
    @arg j2 float value for head_2_joint
    @returns FollowJointTrajectoryGoal with the specified goal"""
    fjtg = FollowJointTrajectoryGoal()
    fjtg.trajectory.joint_names.append('head_1_joint')
    fjtg.trajectory.joint_names.append('head_2_joint')
    point = JointTrajectoryPoint()
    point.positions.append(j1)
    point.positions.append(j2)
    point.velocities.append(0.0)
    point.velocities.append(0.0)
    point.time_from_start = rospy.Duration(time)
    for joint in fjtg.trajectory.joint_names: # Specifying high tolerances for the hand as they are slow compared to other hardware
        goal_tol = JointTolerance()
        goal_tol.name = joint
        goal_tol.position = 5.0
        goal_tol.velocity = 5.0
        goal_tol.acceleration = 5.0
        fjtg.goal_tolerance.append(goal_tol)
    fjtg.goal_time_tolerance = rospy.Duration(3)
    
    fjtg.trajectory.points.append(point)
    fjtg.trajectory.header.stamp = rospy.Time.now()
    return fjtg 
   
    
    
def create_move_group_pose_goal(goal_pose=Pose(), group="right_arm_torso", end_link_name=None, plan_only=True):
    """ Creates a move_group goal based on pose.
    @arg group string representing the move_group group to use
    @arg end_link_name string representing the ending link to use
    @arg goal_pose Pose() representing the goal pose
    @arg plan_only bool to for only planning or planning and executing
    @returns MoveGroupGoal with the data given on the arguments"""
    
    header = Header()
    header.frame_id = 'base_link'
    header.stamp = rospy.Time.now()
    # We are filling in the MoveGroupGoal a MotionPlanRequest and a PlanningOptions message
    # http://docs.ros.org/hydro/api/moveit_msgs/html/msg/MotionPlanRequest.html
    # http://docs.ros.org/hydro/api/moveit_msgs/html/msg/PlanningOptions.html
    moveit_goal = MoveGroupGoal()
    goal_c = Constraints()
    position_c = PositionConstraint()
    position_c.header = header
    if end_link_name != None: # For some groups the end_link_name can be deduced, but better add it manually
        position_c.link_name = end_link_name
    position_c.constraint_region.primitives.append(SolidPrimitive(type=SolidPrimitive.SPHERE, dimensions=[0.01])) # how big is the area where the end effector can be
    position_c.constraint_region.primitive_poses.append(goal_pose)
    position_c.weight = 1.0
    goal_c.position_constraints.append(position_c)
    orientation_c = OrientationConstraint()
    orientation_c.header = header
    if end_link_name != None:
        orientation_c.link_name = end_link_name
    orientation_c.orientation = goal_pose.orientation
    orientation_c.absolute_x_axis_tolerance = 0.01 # Tolerances, MoveIt! by default uses 0.001 which may be too low sometimes
    orientation_c.absolute_y_axis_tolerance = 0.01
    orientation_c.absolute_z_axis_tolerance = 0.01
    orientation_c.weight = 1.0
    goal_c.orientation_constraints.append(orientation_c)
    moveit_goal.request.goal_constraints.append(goal_c)
    moveit_goal.request.num_planning_attempts = 1 # The number of times this plan is to be computed. Shortest solution will be reported.
    moveit_goal.request.allowed_planning_time = 5.0
    moveit_goal.planning_options.plan_only = plan_only
    moveit_goal.planning_options.planning_scene_diff.is_diff = True # Necessary
    moveit_goal.request.group_name = group
    
    return moveit_goal


