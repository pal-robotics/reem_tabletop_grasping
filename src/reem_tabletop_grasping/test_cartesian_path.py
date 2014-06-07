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
# author: Sammy Pfeiffer

# ROS imports
import rospy
import tf
from actionlib import ActionServer, SimpleActionClient
from geometry_msgs.msg import PoseArray, Pose, Point, PoseStamped
from std_msgs.msg import Header
from std_srvs.srv import Empty, EmptyRequest
# own imports
from reem_tabletop_grasping.msg import ObjectManipulationAction, ObjectManipulationFeedback, ObjectManipulationActionResult, ObjectManipulationGoal, ObjectManipulationResult

# perception imports & grasp planning imports
from object_recognition_msgs.msg import RecognizedObjectArray, TableArray
from object_recognition_clusters import ClusterBoundingBoxFinder
from moveit_simple_grasps.msg import GenerateGraspsAction, GenerateGraspsGoal, GraspGeneratorOptions
# manipulation imports
from moveit_msgs.msg import PickupAction, PlaceAction
from moveit_commander import PlanningSceneInterface

from helper_functions import createPickupGoal, dist_between_poses, createPlaceGoal, moveit_error_dict, createCartesianPathRequest, createPlayMotionGoal, createExecuteKnownTrajectoryRequest
from moveit_msgs.msg._MoveItErrorCodes import MoveItErrorCodes
from play_motion_msgs.msg import PlayMotionAction
from moveit_msgs.srv import ExecuteKnownTrajectory, GetCartesianPath
from moveit_msgs.msg import Constraints, OrientationConstraint

import sys
import moveit_commander
import copy

CARTESIAN_PATH_SRV = '/compute_cartesian_path'
EXECUTE_KNOWN_TRAJ_SRV = '/execute_kinematic_path'

if __name__ == "__main__":
    rospy.init_node("testing_cart_path_")
    rospy.loginfo("Connecting to cartesian path server '" + CARTESIAN_PATH_SRV + "'...")
    cartesian_path_service = rospy.ServiceProxy(CARTESIAN_PATH_SRV, GetCartesianPath)
    cartesian_path_service.wait_for_service()
    
    rospy.loginfo("Connecting to known traj executor server '" + EXECUTE_KNOWN_TRAJ_SRV + "'...")
    execute_known_traj_service = rospy.ServiceProxy(EXECUTE_KNOWN_TRAJ_SRV, ExecuteKnownTrajectory)
    execute_known_traj_service.wait_for_service()
    
    moveit_commander.roscpp_initialize(sys.argv)
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    right_group = moveit_commander.MoveGroupCommander("right_arm")
    
    rospy.sleep(1)
    mywaypoints = []
    first_pose = right_group.get_current_pose().pose
    mywaypoints.append(first_pose)
    wp2 = copy.deepcopy(first_pose)
    wp2.position.z += 0.5
    mywaypoints.append(wp2)
    
    
    
    myconstraints = Constraints()
    oriconstr = OrientationConstraint()
    oriconstr.header.frame_id = "base_link"
    oriconstr.link_name = "hand_right_grasping_frame"
    oriconstr.orientation.w = 1.0
    oriconstr.absolute_x_axis_tolerance = 0.1
    oriconstr.absolute_y_axis_tolerance = 0.1
    oriconstr.absolute_z_axis_tolerance = 0.1
    oriconstr.weight = 1.0
    myconstraints.orientation_constraints.append(oriconstr)
    print "myconstraints:"
    print myconstraints
    
    cpr = createCartesianPathRequest(frame_id = "base_link",
                                      group_name="right_arm",
                                      waypoints=mywaypoints, 
                                      max_step=0.01, jump_threshold=0.0, avoid_collisions=True)#, 
                                      #path_constraints=myconstraints)
    
    print "Request cpr: " + str(cpr)
    
    answer = cartesian_path_service.call(cpr)
    print "answer: " + str(answer)
    
    print "Sending traj!"
    ektr = createExecuteKnownTrajectoryRequest(answer.solution)
    execute_known_traj_service.call(ektr)
     
    