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

import copy
import sys
import math
# ROS imports
import rospy
import tf
from actionlib import ActionServer, SimpleActionClient
from geometry_msgs.msg import PoseArray, Pose, Point, PoseStamped, Quaternion, PointStamped
from std_msgs.msg import Header
from std_srvs.srv import Empty, EmptyRequest
# own imports
from reem_tabletop_grasping.msg import ObjectManipulationAction, ObjectManipulationFeedback, \
                                        ObjectManipulationActionResult, ObjectManipulationGoal, \
                                        ObjectManipulationResult

# perception imports & grasp planning imports
from object_recognition_msgs.msg import RecognizedObjectArray, TableArray
from object_recognition_clusters import ClusterBoundingBoxFinder
from moveit_simple_grasps.msg import GenerateGraspsAction, GenerateGraspsGoal, GraspGeneratorOptions
# manipulation imports
from moveit_msgs.msg import PickupAction, PlaceAction
from moveit_commander import PlanningSceneInterface

from helper_functions import createPickupGoal, dist_between_poses, createPlaceGoal, \
                            moveit_error_dict, createCartesianPathRequest, createPlayMotionGoal, \
                            createExecuteKnownTrajectoryRequest, createBendGoal, create_move_group_pose_goal, \
                            createHeadGoal
from control_msgs.msg import FollowJointTrajectoryAction
from moveit_msgs.msg._MoveItErrorCodes import MoveItErrorCodes
from play_motion_msgs.msg import PlayMotionAction
from moveit_msgs.srv import ExecuteKnownTrajectory, GetCartesianPath
from moveit_msgs.msg._MoveGroupAction import MoveGroupAction
from odom_moving.srv import Turn, TurnRequest
from odom_moving.srv import Straight, StraightRequest
from pal_control_msgs.srv import CurrentLimit, CurrentLimitRequest

from speed_limit.msg import DisableGoal, DisableAction

from control_msgs.msg import JointTrajectoryControllerState

import moveit_commander
from cartesian_goals import trajectoryConstructor

# TODO: dynamic param to setup debug info
DEBUG_MODE = True
if DEBUG_MODE:
    from visualizing_functions import publish_grasps_as_poses

RH2 = True

RECOGNIZED_OBJECT_ARRAY_TOPIC = '/recognized_object_array'
TABLES_ARRAY_TOPIC = '/table_array'
TO_BE_GRASPED_OBJECT_POSE_TOPIC = '/to_grasp_object_pose'
PICKUP_AS = '/pickup'
PLACE_AS = '/place'
GRASP_GENERATOR_AS = '/moveit_simple_grasps_server/generate'
DEPTH_THROTLE_SRV = '/depth_throtle'
OBJECT_MANIPULATION_AS = 'object_manipulation_server'
PLAY_MOTION_AS = '/play_motion'
CARTESIAN_PATH_SRV = '/compute_cartesian_path'
EXECUTE_KNOWN_TRAJ_SRV = '/execute_kinematic_path'
TORSO_AS = '/torso_controller/follow_joint_trajectory'
HEAD_AS = '/head_controller/follow_joint_trajectory'
MOVE_GROUP_AS = '/move_group'
STRAIGHT_SRV = '/straight_nav'
TURN_SRV = '/turn_nav'
SPEED_LIMIT_AS = '/speed_limit/disable'
CURRENT_LIMIT_SRV = '/current_limit_controller/set_current_limit'

RIGHT_ARM_STATE_TOPIC = '/right_arm_controller/state'

OFFSET_OVER_TABLE_PLACING = 0.08


class ObjectManipulationAS:

    def __init__(self, name):
        # stuff for grasp planning
        rospy.loginfo("Getting a TransformListener...")
        self.tf_listener = tf.TransformListener()
        rospy.loginfo("Getting a TransformBroadcaster...")
        self.tf_broadcaster = tf.TransformBroadcaster()
        rospy.loginfo("Initializing a ClusterBoundingBoxFinder...")
        self.cbbf = ClusterBoundingBoxFinder(self.tf_listener, self.tf_broadcaster, "base_link")
        self.last_clusters = None
        rospy.loginfo("Subscribing to '" + RECOGNIZED_OBJECT_ARRAY_TOPIC + "'...")
        self.sub_objects = rospy.Subscriber(RECOGNIZED_OBJECT_ARRAY_TOPIC, RecognizedObjectArray, self.objects_callback)

        self.last_tables = None
        rospy.loginfo("Subscribing to '" + TABLES_ARRAY_TOPIC + "'...")
        self.sub_tables = rospy.Subscriber(TABLES_ARRAY_TOPIC, TableArray, self.tables_callback)

        if DEBUG_MODE:
            self.to_grasp_object_pose_pub = rospy.Publisher(TO_BE_GRASPED_OBJECT_POSE_TOPIC, PoseStamped)
            
        rospy.loginfo("Connecting to play_motion AS '" + PLAY_MOTION_AS + "'...")
        self.play_motion_ac = SimpleActionClient(PLAY_MOTION_AS, PlayMotionAction)
        self.play_motion_ac.wait_for_server()

        rospy.loginfo("Connecting to depth throttle server '" + DEPTH_THROTLE_SRV + "'...")
        self.depth_service = rospy.ServiceProxy(DEPTH_THROTLE_SRV, Empty)
        self.depth_service.wait_for_service()

#         rospy.loginfo("Connecting to cartesian path server '" + CARTESIAN_PATH_SRV + "'...")
#         self.cartesian_path_service = rospy.ServiceProxy(CARTESIAN_PATH_SRV, GetCartesianPath)
#         self.cartesian_path_service.wait_for_service()
        
#         rospy.loginfo("Connecting to known traj executor server '" + EXECUTE_KNOWN_TRAJ_SRV + "'...")
#         self.execute_known_traj_service = rospy.ServiceProxy(EXECUTE_KNOWN_TRAJ_SRV, ExecuteKnownTrajectory)
#         self.execute_known_traj_service.wait_for_service()

        rospy.loginfo("Getting a PlanningSceneInterface instance...")
        self.scene = PlanningSceneInterface()

        self.torso_as = SimpleActionClient(TORSO_AS, FollowJointTrajectoryAction)
        rospy.loginfo("Connecting to torso AS...")
        self.torso_as.wait_for_server()
        
        self.head_as = SimpleActionClient(HEAD_AS, FollowJointTrajectoryAction)
        rospy.loginfo("Connecting to head AS...")
        self.head_as.wait_for_server()
        
        self.move_group_as = SimpleActionClient(MOVE_GROUP_AS, MoveGroupAction)
        rospy.loginfo("Connecting to move_group AS...")
        self.move_group_as.wait_for_server()
        
        rospy.loginfo("Connecting to straight move server '" + STRAIGHT_SRV + "'...")
        self.straight_srv = rospy.ServiceProxy(STRAIGHT_SRV, Straight)
        self.straight_srv.wait_for_service()
        
        rospy.loginfo("Connecting to straight move server '" + TURN_SRV + "'...")
        self.turn_srv = rospy.ServiceProxy(TURN_SRV, Turn)
        self.turn_srv.wait_for_service()
        
        self.speedl_as = SimpleActionClient(SPEED_LIMIT_AS, DisableAction)
        rospy.loginfo("Connecting to speed limit disable AS...")
        self.speedl_as.wait_for_server()
        
        if RH2:
            print "We are on RH2 we are skipping current control limit (as for 3/7/14)"
        if not RH2:
            print "We are not in RH2 so we current control the right arm"
            rospy.loginfo("Trying to connect to Current Limit service'" + CURRENT_LIMIT_SRV + "'...")
            self.current_limit_srv = rospy.ServiceProxy(CURRENT_LIMIT_SRV, CurrentLimit)
            self.current_limit_srv.wait_for_service()
        
        
        self.grasped_object = False
        
        # the summation of the errors of the right arm
        # to know if we grasped should be bigger than this threshold
        self.right_arm_error_th = 0.020
        
        # Grasp motion names
        self.pre_grasp = 'pre_grasp'
        # Those grasps are all distances referenced with the robot with the torso
        # at 0.00 rad (top/up)
        # when using the torso down (0.61rad, bot/down) they are bigger, in between 0-6cm bigger
        self.top_bot_closing_variation = 0.04
        self.min_torso_rads = 0.61
        self.max_torso_rads = -0.18
        # X distance is 36cm at 0.00 rad and 43cm at 0.61 rad, so a variation of 7cm
        if RH2:
            self.x_min = 0.34 - 0.07  # Magic calibration distance
            self.x_max = 0.41 - 0.07
        else:
            self.x_min = 0.34 - 0.1  # Magic calibration distance
            self.x_max = 0.41 - 0.1
        self.x_dist_variation = self.x_max - self.x_min 
        # Z distance on 0.00 rad is 107cm and on 0.61 rad is 77cm 
        self.z_min = 0.77
        self.z_max = 1.07
        self.z_dist_variation = self.z_max - self.z_min
        self.distance_based_grasps = {
                                      0.24 : 'grasp_24cm',
                                      0.22 : 'grasp_22cm',
                                      0.19 : 'grasp_19cm',
                                      0.17 : 'grasp_17cm',
                                      0.14 : 'grasp_14cm',
                                      0.12 : 'grasp_12cm',
                                      0.10 : 'grasp_10cm',
                                      0.08 : 'grasp_8cm',
#                                       0.06 : 'grasp_6cm',
#                                       0.04 : 'grasp_4cm',
#                                       0.02 : 'grasp_2cm',
                                      0.00 : 'grasp_0cm'
                                      }
        
        # blocking action server
        rospy.loginfo("Creating Action Server '" + name + "'...")
        self.grasp_obj_as = ActionServer(name, ObjectManipulationAction, self.goal_callback, self.cancel_callback, False)
        self.as_feedback = ObjectManipulationFeedback()
        self.as_result = ObjectManipulationActionResult()
        self.current_goal = None

        rospy.loginfo("Starting '" + OBJECT_MANIPULATION_AS + "' Action Server!")
        print "Starting '" + OBJECT_MANIPULATION_AS + "' Action Server!"
        self.grasp_obj_as.start()

    def objects_callback(self, data):
        rospy.loginfo(rospy.get_name() + ": This message contains %d objects." % len(data.objects))
        self.last_clusters = data

    def tables_callback(self, data_tables):
        rospy.loginfo(rospy.get_name() + ": This message contains %d tables." % len(data_tables.tables))
        self.last_tables = data_tables

    def goal_callback(self, goal):
        if self.current_goal:
            goal.set_rejected()  # "Server busy"
            return
        else:
            #store and accept new goal
            self.current_goal = goal
            self.current_goal.set_accepted()
            #run the corresponding operation
            if self.current_goal.get_goal().operation == ObjectManipulationGoal.PICK:
                rospy.loginfo("ObjectManipulation: PICK!")
                self.pick_operation()
            elif self.current_goal.get_goal().operation == ObjectManipulationGoal.PLACE:
                rospy.loginfo("ObjectManipulation: PLACE!")
                self.place_operation()
            else:
                rospy.logerr("ObjectManipulation: Erroneous operation!")
            #finished, get rid of goal
            self.current_goal = None

    def cancel_callback(self, goal):
        #TODO stop motions?
        self.current_goal.set_canceled()

    def pick_operation(self):
        """Execute pick operation"""
        if self.message_fields_ok():
            self.as_result = ObjectManipulationResult()
            goal_message_field = self.current_goal.get_goal()
            # Get the objects on the table
            # MOVE HEAD DOWN!!
            print "Moving head down"
            look_down_goal = createHeadGoal(0.0, 0.5, time=2.0)
            self.head_as.send_goal_and_wait(look_down_goal)
            
            ## Publish pose of goal position
            if DEBUG_MODE:
                self.to_grasp_object_pose_pub.publish(goal_message_field.target_pose)
            self.update_feedback("Detecting clusters")

            object_pose, obj_bbox_dims = self.detect_object(goal_message_field.target_pose)
            
            
            print "object_pose is: " + str(object_pose) + "\n"
            print "objects dims are: " + str(obj_bbox_dims) + "\n"
            
            if object_pose.pose.position.z < 0.5:
                print "We can't grasp something so low, aborting"
                self.update_aborted("Object too low, aborting")
                return
            
            
            # Move to a correct orientation to just move straight
            print "\n====Moving to a position where we can just go straight forward to the distance we need"
            print "This position should be where we have a relative Y axis with the object to grasp ~= 0 "
            print "Which means we are looking straight to the object in the middle"
            nav_goal = PoseStamped()
            nav_goal.header.frame_id = "base_link"
            nav_goal.pose.position.y = object_pose.pose.position.y
            nav_goal.pose.orientation.w = 1.0
            print "So navigation goal would be:" + str(nav_goal) + "\n"
            
            if object_pose.pose.position.y > 0.0:
                print "\n\n~~~~~~Object to the LEFT~~~~~~~~~"
            elif object_pose.pose.position.y < 0.0:
                print "\n\n~~~~~~Object to the RIGHT~~~~~~~~~"

            if abs(object_pose.pose.position.y) > 0.02: # minimum threshold to rotate in cm
                # Calculate how much to turn to look in front
                angle_to_turn = self.get_angle_to_turn(object_pose.pose.position.x, object_pose.pose.position.y)
                print "We need to turn: " + str(math.degrees(angle_to_turn))
                if abs(math.degrees(angle_to_turn)) > 2.0: # minimum threshold to rotate... 
                    turn_req = TurnRequest()
                    turn_req.enable = True
                    if angle_to_turn > 0.0:
                        turn_req.degrees = math.degrees(angle_to_turn) - 1.5 # offset that the robot rotates too much 
                    else:
                        turn_req.degrees = math.degrees(angle_to_turn) + 1.5 # offset that the robot rotates too much
                    print "\nSending turn request: " + str(turn_req)
                    self.turn_srv.call(turn_req)
                
                    object_pose, obj_bbox_dims = self.detect_object(goal_message_field.target_pose)
                    print "\nAfter turning the object pose is:"
                    print object_pose
                    print "And now obj_bbox_dims are:"
                    print obj_bbox_dims
            
#             
#             if True:
#                 self.current_goal.set_succeeded(result=self.as_result)
#                 return
            
            
            # Get initial position, joint based, with arms open, torso still up
            print "\n====Moving to initial grasping pose" 
            initial_pose_g = createPlayMotionGoal(self.pre_grasp)
            self.play_motion_ac.send_goal_and_wait(initial_pose_g)
            
            # Adapt hands closing distance to object width + 5cm on each side (min dist between objects)
            print "\n=====Hands pose to object width"
            biggest_dim = 0.0
            for dim in obj_bbox_dims:
                if dim > biggest_dim:
                    biggest_dim = dim
            initial_pose_g = createPlayMotionGoal(self.get_motion_from_width_and_height(biggest_dim + 0.06, object_pose.pose.position.z), skip_planning=True)
            self.play_motion_ac.send_goal_and_wait(initial_pose_g)
            
            # Bend torso up as necessary
            print "\n===Bending torso up to the necessary height + an offset"
            torso_goal = createBendGoal(object_pose.pose.position.z + 0.2)
            self.torso_as.send_goal_and_wait(torso_goal)
            print self.torso_as.get_result()
            
            # Move in front to the required distance, 82cm from the object (thats the distance between finger and base_link when testing)
            print "\n====Moving magically some distance"
            print "The object is at: " + str(object_pose.pose.position.x),
            print " and for a height of " + str(object_pose.pose.position.z)
            # The more bent we are, the farther our grasp point is
            x_needed = self.x_min + (self.x_dist_variation * (1 - (object_pose.pose.position.z - self.z_min) / self.z_dist_variation ))
            print "We need to be at: " + str(x_needed) + "m"
            print "So we need to advance: " + str(object_pose.pose.position.x - x_needed) + "m"
            advance_dist = object_pose.pose.position.x - x_needed
            
            dg = DisableGoal(duration=15.0)
            self.speedl_as.send_goal(dg)
            self.speedl_as.wait_for_result(rospy.Duration(0.1))
            
            straight_req = StraightRequest()
            straight_req.enable = True
            straight_req.meters = advance_dist
            self.straight_srv.call(straight_req)


            expected_new_object_pose = copy.deepcopy(object_pose)
            expected_new_object_pose.pose.position.x -= advance_dist
            object_posecheck, obj_bbox_dimscheck = self.detect_object(expected_new_object_pose)
            if DEBUG_MODE:
                self.to_grasp_object_pose_pub.publish(expected_new_object_pose)
            print "Using as reference: " + str(expected_new_object_pose)
            print "\nAfter moving forward the object pose is:"
            print object_posecheck
            print "And now obj_bbox_dims are:"
            print obj_bbox_dimscheck


            # Add box representing the obstacle
            self.scene.add_box("object_bbx", object_pose,
                               (obj_bbox_dims[0], obj_bbox_dims[1], obj_bbox_dims[2]))
            
            # Bend torso down as necessary
            print "\n===Bending torso down to the necessary height"
            torso_goal = createBendGoal(object_pose.pose.position.z + 0.11)
            self.torso_as.send_goal_and_wait(torso_goal)
            print self.torso_as.get_result()
            
            # Soften the right arm so we dont crush objects
            print "\n===Softening right arm joints to not crush objects"
            self.soften_right_arm()
            
            # Close hands first to 8cm (so we dont close so fast)
            print "\n===closing hands first to 8cm"
            initial_pose_g = createPlayMotionGoal('grasp_8cm', skip_planning=True)
            self.play_motion_ac.send_goal_and_wait(initial_pose_g)
            
            # Close hands to object width - 3cm (empyrical)
            print "\n===Closing hands for grasp"
            initial_pose_g = createPlayMotionGoal('grasp_0cm', skip_planning=True)
            self.play_motion_ac.send_goal_and_wait(initial_pose_g)
            
            # Attaching object to hand
            self.scene.attach_box('hand_right_index_3_link', "object_bbx", object_pose, obj_bbox_dims)
            
            # Lift up torso
            print "\n===Hopefully picking up the object"
            torso_goal = createBendGoal(object_pose.pose.position.z + 0.2)
            self.torso_as.send_goal_and_wait(torso_goal)
            print self.torso_as.get_result()

            print "Moving head up"
            look_up_goal = createHeadGoal(0.0, -0.15, time=2.0)
            self.head_as.send_goal_and_wait(look_up_goal)


            # Let the robot move freely
            dg = DisableGoal(duration=15.0)
            self.speedl_as.send_goal(dg)
            self.speedl_as.wait_for_result(rospy.Duration(0.1))
            
            # Maybe some other safer position?
            print "\n===Moving to safer position"
            straight_req = StraightRequest()
            straight_req.enable = True
            straight_req.meters = -advance_dist
            self.straight_srv.call(straight_req)
        
            
            if self.grasped_from_joint_error():
                self.grasped_object = True    
                self.current_goal.set_succeeded(result=self.as_result)
            else:
                self.grasped_object = False
                print "\n\n===Returning right arm currents to normal"
                self.return_right_arm_to_normal()
                self.update_aborted("Failed grasping, object does not seem to be in between hands")
            

    def detect_object(self, target_pose):
        """Given a posestamped target_pose search the closer cluster to that pose"""
        if not self.wait_for_recognized_array(wait_time=5, timeout_time=10):  # wait until we get clusters published
            self.update_aborted("Failed detecting clusters")
        
        # Search closer cluster
        # transform pose to base_link if needed
        if target_pose.header.frame_id != "base_link":
            n_tries = 3
            while n_tries > 0:
                try:
                    print "Try " + str(n_tries) + " of transforming... (5s wait for transform)"
                    target_pose.header.stamp = rospy.Time.now()
                    self.tf_listener.waitForTransform("base_link", target_pose.header.frame_id, target_pose.header.stamp, rospy.Duration(5))
                    object_to_grasp_pose = self.tf_listener.transformPose("base_link", target_pose)
                    n_tries = 0
                except:
                    n_tries -= 1
        else:
            object_to_grasp_pose = target_pose.pose

        self.update_feedback("Searching closer cluster while clustering")
        (closest_cluster_id, (object_points, obj_bbox_dims, object_bounding_box, object_pose)) = self.get_id_of_closest_cluster_to_pose(object_to_grasp_pose)

        rospy.logdebug("in AS: Closest cluster id is: " + str(closest_cluster_id))
        #TODO visualize bbox
        #TODO publish filtered pointcloud?
        print "BBOX: " + str(obj_bbox_dims) 
        ########
        #self.update_feedback("Check reachability")
        # Given the bounding box... 
        
        # shift object pose up by halfway, clustering code gives obj frame on the bottom because of too much noise on the table cropping (2 1pixel lines behind the objects)
        # TODO remove this hack, fix it in table filtering
        object_pose.pose.position.z += obj_bbox_dims[2] / 2.0
        #object_pose.pose.position.y += 0.03 # RH3 needs an offset as of today to actually grasp the objects
        return object_pose, obj_bbox_dims


            
    def place_operation(self):
        """Execute the place operation"""
        if self.message_fields_ok():
            self.as_result = ObjectManipulationResult()
            goal_message_field = self.current_goal.get_goal()
            
            if self.grasped_object:
                print "Moving head down"
                look_down_goal = createHeadGoal(0.0, 0.5, time=2.0)
                self.head_as.send_goal_and_wait(look_down_goal)
                
                if not self.wait_for_tables_array(wait_time=5, timeout_time=10):  # wait until we get clusters published
                    self.update_aborted("Failed detecting tables")
                
                closest_tablemsg, closest_table_posestamped, closest_table_x_border = self.get_pose_of_closest_table(goal_message_field.target_pose.pose)
                print "closest_tablemsg: " + str(closest_tablemsg)
                print "closest_table_posestamped: " + str(closest_table_posestamped)
                print "closest convex hull of table x: " + str(closest_table_x_border)
                if DEBUG_MODE:
                    ps = PoseStamped()
                    ps.header.frame_id = "base_link"
                    ps.pose.position.x = closest_table_x_border
                    ps.pose.position.z = 1.0
                    ps.pose.orientation = Quaternion(0.0, 0.0, 0.0, 1.0)
                    self.to_grasp_object_pose_pub.publish(ps)
                
                dg = DisableGoal(duration=10.0)
                self.speedl_as.send_goal(dg)
                self.speedl_as.wait_for_result(rospy.Duration(0.1))
                print "\n\n===Move forward to get to the real POI"
                straight_req = StraightRequest()
                straight_req.enable = True
                straight_req.meters = closest_table_x_border - 0.1
                self.straight_srv.call(straight_req)
                
                print "====BEND TO THE DESIRED Z"
                print "goal message: " + str(goal_message_field)
                if RH2:
                    torso_goal = createBendGoal(closest_table_posestamped.pose.position.z + 0.14)
                else:
                    torso_goal = createBendGoal(closest_table_posestamped.pose.position.z + 0.05)
                self.torso_as.send_goal_and_wait(torso_goal)
                print self.torso_as.get_result()
                
                print "=====Hands pose to open a lot"
                initial_pose_g = createPlayMotionGoal("pre_grasp", skip_planning=True)
                self.play_motion_ac.send_goal_and_wait(initial_pose_g)
                
                print "====bend up to be safe"
                torso_goal = createBendGoal(closest_table_posestamped.pose.position.z + 0.2)
                self.torso_as.send_goal_and_wait(torso_goal)
                print self.torso_as.get_result()
                
                print "Moving head up"
                look_up_goal = createHeadGoal(0.0, -0.15, time=2.0)
                self.head_as.send_goal_and_wait(look_up_goal)
                
                dg = DisableGoal(duration=10.0)
                self.speedl_as.send_goal(dg)
                self.speedl_as.wait_for_result(rospy.Duration(0.1))
                
                print "\n\n===Move back to be safe"
                straight_req = StraightRequest()
                straight_req.enable = True
                straight_req.meters = - (closest_table_x_border - 0.1)
                self.straight_srv.call(straight_req)
                
                
                self.grasped_object = False
                self.current_goal.set_succeeded(result=self.as_result)
                self.scene.remove_attached_object("hand_right_index_3_link", "object_bbx")
                
                print "\n\n===Returning right arm currents to normal"
                self.return_right_arm_to_normal()
                
                            
                # MOVE ARMS TO HOME
                print "\n====Moving arms to home" 
                home_g = createPlayMotionGoal("home")
                self.play_motion_ac.send_goal_and_wait(home_g, skip_planning=True)
                
            else:
                rospy.logerr("No grasped object to place!!")
                self.update_aborted("No grasped object to place!")


    def grasped_from_joint_error(self):
        """From reading the current error in the state of the motors in the right arm
        we deduce if we have grasped. there should be some error bigger than a threshold
        True if grasped, False if not"""
        msg = rospy.wait_for_message(RIGHT_ARM_STATE_TOPIC, JointTrajectoryControllerState)
        sum_error = 0.0
        for joint_err_val in msg.error.positions:
            sum_error += abs(joint_err_val)
        print "\n///////threshold is: "
        print self.right_arm_error_th
        print "sum_error is:"
        print sum_error
        
        if sum_error > self.right_arm_error_th:
            print "We overpassed error... so we grasped!"
            return True

        print "We didnt overpass error... so we didnt grasp :("
        return False


    def soften_right_arm(self):
        """Lower the current of some right arm joints to not crush objects
        and be able to detect if we grasped them"""
        req = CurrentLimitRequest()
        req.actuator_name = 'arm_right_4_motor'
        req.current_limit = 0.5
        if not RH2:
            self.current_limit_srv.call(req)
        req = CurrentLimitRequest()
        req.actuator_name = 'arm_right_2_motor'
        req.current_limit = 0.5
        if not RH2:
            self.current_limit_srv.call(req)
        
    def return_right_arm_to_normal(self):
        """Give back the original current of joints"""
        req = CurrentLimitRequest()
        req.actuator_name = 'arm_right_4_motor'
        req.current_limit = 1.0
        if not RH2:
            self.current_limit_srv.call(req)
        req = CurrentLimitRequest()
        req.actuator_name = 'arm_right_2_motor'
        req.current_limit = 1.0
        if not RH2:
            self.current_limit_srv.call(req)
         

    def message_fields_ok(self):
        """Check if the minimal fields for the message are fulfilled"""
        if self.current_goal:
            goal_message_field = self.current_goal.get_goal()
            if len(goal_message_field.group) == 0:
                rospy.logwarn("Group field empty.")
                return False
            if goal_message_field.operation != ObjectManipulationGoal.PICK and goal_message_field.operation != ObjectManipulationGoal.PLACE:
                rospy.logwarn("Asked for an operation different from PICK or PLACE: " + str(goal_message_field.operation))
                return False
            if len(goal_message_field.target_pose.header.frame_id) == 0:
                rospy.logwarn("Target pose frame_id is empty")
                return False
            return True

    def update_feedback(self, text):
        """Publish feedback with given text"""
        self.as_feedback.last_state = text
        self.current_goal.publish_feedback(self.as_feedback)

    def update_aborted(self, text="", manipulation_result=None):
        """Publish feedback and abort with the text give and optionally an ObjectManipulationResult already fulfilled"""
        self.update_feedback("aborted." + text)
        if manipulation_result == None:
            aborted_result = ObjectManipulationResult()
            aborted_result.error_message = text
            self.current_goal.set_aborted(result=aborted_result)
        else:
            self.current_goal.set_aborted(result=manipulation_result)


    def get_id_of_closest_cluster_to_pose(self, input_pose):
        """Returns the id of the closest cluster to the pose provided (in Pose or PoseStamped)
        and all the associated clustering information"""
        current_id = 0
        closest_pose = None
        closest_object_points = None
        closest_id = 0
        closest_bbox = None
        closest_bbox_dims = None
        closest_distance = 99999.9
        for myobject in self.last_clusters.objects:
            (object_points, obj_bbox_dims, object_bounding_box, object_pose) = self.cbbf.find_object_frame_and_bounding_box(myobject.point_clouds[0])
            self.tf_listener.waitForTransform("base_link", object_pose.header.frame_id, object_pose.header.stamp, rospy.Duration(15))
            trans_pose = self.tf_listener.transformPose("base_link", object_pose)
            object_pose = trans_pose
            rospy.loginfo("id: " + str(current_id) + "\n pose:\n" + str(object_pose))
            if closest_pose == None:  # first loop iteration
                closest_object_points = object_points
                closest_pose = object_pose
                closest_bbox = object_bounding_box
                closest_bbox_dims = obj_bbox_dims
                closest_distance = dist_between_poses(closest_pose, input_pose)
            else:
                if dist_between_poses(object_pose, input_pose) < closest_distance:
                    closest_object_points = object_points
                    closest_distance = dist_between_poses(object_pose, input_pose)
                    closest_pose = object_pose
                    closest_bbox = object_bounding_box
                    closest_bbox_dims = obj_bbox_dims
                    closest_id = current_id
            current_id += 1
        rospy.loginfo("Closest id is: " + str(closest_id) + " at " + str(closest_pose))
        return closest_id, (closest_object_points, closest_bbox_dims, closest_bbox_dims, closest_pose)


    def wait_for_recognized_array(self, wait_time=6, timeout_time=10):
        """Ask for depth images until we get a recognized array
        wait for wait_time between depth throttle calls
        stop if timeout_time is achieved
        If we don't find it in the correspondent time return false, true otherwise"""
        initial_time = rospy.Time.now()
        self.last_clusters = None
        count = 0
        num_calls = 1
        self.depth_service.call(EmptyRequest())
        rospy.loginfo("Depth throttle server call #" + str(num_calls))
        rospy.loginfo("Waiting for a recognized array...")
        while rospy.Time.now() - initial_time < rospy.Duration(timeout_time) and self.last_clusters == None:
            rospy.sleep(0.1)
            count += 1

            if count >= wait_time / 10:
                self.depth_service.call(EmptyRequest())
                num_calls += 1
                rospy.loginfo("Depth throttle server call #" + str(num_calls))
        if self.last_clusters == None:
            return False
        else:
            return True

    def get_pose_of_closest_table(self, input_pose):
        """Get the PoseStamped and the Table msg of the closest table to adjust height of the placing
        also return closest_table_x_border to approach the table"""
        closest_table_posestamped = None
        closest_tablemsg = None
        closest_distance_z = 99999.9
        closest_table_x_border = 99999.9
        for mytable in self.last_tables.tables:
            table_posestamped = PoseStamped(header=mytable.header, pose=mytable.pose)
            if table_posestamped.header.frame_id != "base_link":
                self.tf_listener.waitForTransform("base_link", table_posestamped.header.frame_id, table_posestamped.header.stamp, rospy.Duration(5))
                table_pose = self.tf_listener.transformPose("base_link", table_posestamped)
                closer_convex_hull_in_x = 99999.9
                for convex_hull_point in mytable.convex_hull:
                    def sum_Points(a,b):
                        return Point(a.x + b.x, a.y + b.y, a.z  + b.z)
                    # Stupid convex hull is referenced around the table "centroid"... this has made me lose much time!
                    cv_p_pointstamped = PointStamped(header=mytable.header, point=sum_Points(convex_hull_point, mytable.pose.position))
                    cv_p_base_link = self.tf_listener.transformPoint("base_link", cv_p_pointstamped)
#                     def dist_xy(ax, ay, bx, by):
#                         return math.sqrt(abs(ax-bx)**2 + abs(ay-by)**2)
                    if cv_p_base_link.point.x < closer_convex_hull_in_x:
                        closer_convex_hull_in_x = cv_p_base_link.point.x
            else:
                table_pose = table_posestamped
                closer_convex_hull_in_x = 99999.9
                for convex_hull_point in mytable.convex_hull:
                    if convex_hull_point.x < closer_convex_hull_in_x:
                        closer_convex_hull_in_x = convex_hull_point.x
                
            if closest_table_posestamped == None:
                closest_table_posestamped = table_pose
                closest_tablemsg = mytable
                closest_distance_z = abs(table_pose.pose.position.z - input_pose.position.z)
                closest_table_x_border = closer_convex_hull_in_x
            else:
                if abs(table_pose.pose.position.z - input_pose.position.z) < closest_distance_z:
                    closest_distance_z = abs(table_pose.pose.position.z - input_pose.position.z)
                    closest_table_posestamped = table_pose
                    closest_tablemsg = mytable
                    closest_table_x_border = closer_convex_hull_in_x
        rospy.loginfo("Closest table is at pose: " + str(closest_table_posestamped))
        rospy.loginfo("With closest convex hull x point at: " + str(closest_table_x_border))
        return closest_tablemsg, closest_table_posestamped, closest_table_x_border

    def wait_for_tables_array(self, wait_time=6, timeout_time=10):
        """Ask for depth images until we get a table array
        wait for wait_time between depth throttle calls
        stop if timeout_time is achieved
        If we don't find it in the correspondent time return false, true otherwise"""
        initial_time = rospy.Time.now()
        self.last_tables = None
        count = 0
        num_calls = 1
        self.depth_service.call(EmptyRequest())
        rospy.loginfo("Depth throttle server call #" + str(num_calls))
        rospy.loginfo("Waiting for a table array...")
        while rospy.Time.now() - initial_time < rospy.Duration(timeout_time) and self.last_tables == None:
            rospy.sleep(0.1)
            count += 1

            if count >= wait_time / 10:
                self.depth_service.call(EmptyRequest())
                num_calls += 1
                rospy.loginfo("Depth throttle server call #" + str(num_calls))
        if self.last_tables == None:
            return False
        else:
            return True


    def get_motion_from_width_and_height(self, width, height):
        """Given a width of an object get the motion to attain that distance between
        the hands"""
        print "Got a width of " + str(width)
        print "And a height of " + str(height)
        # we adjust how much we want to close the hands
        # because the robot does not close the same on the same pose if he is bent over or not bend over           
        objective_width = width - self.top_bot_closing_variation * (height - self.z_min) / self.z_dist_variation
        print "objective_width: " + str(objective_width)
        best_dist = -1  # the best distance will be the first one that is bigger or equal than the width
        dist_list = []
        for dist in self.distance_based_grasps:
            dist_list.append(dist)
        dist_list.sort()
        
        for dist in dist_list:
            if dist >= objective_width:
                best_dist = dist
                break
        if best_dist == -1:
            rospy.logerr("Width too big, won't fit between any pose, returning biggest grasp")
            best_dist = dist
        print "Which got me a best_dist of " + str(best_dist)
        print "Which ends in the movement: " + self.distance_based_grasps.get(best_dist)
        return self.distance_based_grasps.get(best_dist)

    def hypotenuse(self, a, b):
        c = math.sqrt(a**2 + b**2)
        return c
    
    def angle(self, c, a, b):
        """C = hypotenuse"""
        return math.acos((c**2 - b**2 - a**2)/(-2.0 * a * b))
    
    def get_angle_to_turn(self, x, y):
        angle_to_turn = self.angle(y, x, self.hypotenuse(x, y))
        print "x, y, angle_to_turn: "
        print x
        print y
        print angle_to_turn
        if y > 0.0: # our turn node takes turns clockwise as positive, this calculation always gives positive angles
            angle_to_turn *= -1
        print "After *-1"
        print angle_to_turn
        return angle_to_turn
        