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
# ROS imports
import rospy
import tf
from actionlib import ActionServer, SimpleActionClient
from geometry_msgs.msg import PoseArray, Pose, Point, PoseStamped, Quaternion
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
                            createExecuteKnownTrajectoryRequest, createBendGoal, create_move_group_pose_goal
from control_msgs.msg import FollowJointTrajectoryAction
from moveit_msgs.msg._MoveItErrorCodes import MoveItErrorCodes
from play_motion_msgs.msg import PlayMotionAction
from moveit_msgs.srv import ExecuteKnownTrajectory, GetCartesianPath
from moveit_msgs.msg._MoveGroupAction import MoveGroupAction

import moveit_commander
from cartesian_goals import trajectoryConstructor

# TODO: dynamic param to setup debug info
DEBUG_MODE = True
if DEBUG_MODE:
    from visualizing_functions import publish_grasps_as_poses

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
MOVE_GROUP_AS = '/move_group'

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

        rospy.loginfo("Connecting to cartesian path server '" + CARTESIAN_PATH_SRV + "'...")
        self.cartesian_path_service = rospy.ServiceProxy(CARTESIAN_PATH_SRV, GetCartesianPath)
        self.cartesian_path_service.wait_for_service()
        
        rospy.loginfo("Connecting to known traj executor server '" + EXECUTE_KNOWN_TRAJ_SRV + "'...")
        self.execute_known_traj_service = rospy.ServiceProxy(EXECUTE_KNOWN_TRAJ_SRV, ExecuteKnownTrajectory)
        self.execute_known_traj_service.wait_for_service()

        rospy.loginfo("Getting a PlanningSceneInterface instance...")
        self.scene = PlanningSceneInterface()

        self.torso_as = SimpleActionClient(TORSO_AS, FollowJointTrajectoryAction)
        rospy.loginfo("Connecting to torso AS...")
        self.torso_as.wait_for_server()
        
        self.move_group_as = SimpleActionClient(MOVE_GROUP_AS, MoveGroupAction)
        rospy.loginfo("Connecting to move_group AS...")
        self.move_group_as.wait_for_server()
        
        rospy.loginfo("Creating trajectory constructor...")
        self.tC = trajectoryConstructor()
        
        moveit_commander.roscpp_initialize(sys.argv)
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.right_group = moveit_commander.MoveGroupCommander("right_arm")
        self.left_group = moveit_commander.MoveGroupCommander("left_arm")
        
        # blocking action server
        rospy.loginfo("Creating Action Server '" + name + "'...")
        self.grasp_obj_as = ActionServer(name, ObjectManipulationAction, self.goal_callback, self.cancel_callback, False)
        self.as_feedback = ObjectManipulationFeedback()
        self.as_result = ObjectManipulationActionResult()
        self.current_goal = None

        # Take care of left and right arm grasped stuff
        self.right_hand_object = None #"right_hand_object"
        self.left_hand_object = None
        self.current_side = 'right'

        rospy.loginfo("Starting '" + OBJECT_MANIPULATION_AS + "' Action Server!")
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
        # TODO: Check if pose is not empty, if it is, reject
#         elif len(self.last_clusters.objects) - 1 < goal.get_goal().target_id:
#             goal.set_rejected() # "No objects to grasp were received on the objects topic."
#             return
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
            # Bend the torso
            # As much as needed (we know we detected an object, so we adapt the bending to the height)
            # 0.63 deg torso 2 (max: 35deg)
           
            # Get the objects on the table
            ## Publish pose of goal position
            if DEBUG_MODE:
                self.to_grasp_object_pose_pub.publish(goal_message_field.target_pose)
            self.update_feedback("Detecting clusters")

            if not self.wait_for_recognized_array(wait_time=5, timeout_time=10):  # wait until we get clusters published
                self.update_aborted("Failed detecting clusters")
            
            # Search closer cluster
            # transform pose to base_link if needed
            if goal_message_field.target_pose.header.frame_id != "base_link":
                self.tf_listener.waitForTransform("base_link", goal_message_field.target_pose.header.frame_id, goal_message_field.target_pose.header.stamp, rospy.Duration(5))
                object_to_grasp_pose = self.tf_listener.transformPose("base_link", goal_message_field.target_pose)
            else:
                object_to_grasp_pose = goal_message_field.target_pose.pose

            self.update_feedback("Searching closer cluster while clustering")
            (closest_cluster_id, (object_points, obj_bbox_dims, object_bounding_box, object_pose)) = self.get_id_of_closest_cluster_to_pose(object_to_grasp_pose)

            rospy.logdebug("in AS: Closest cluster id is: " + str(closest_cluster_id))
            #TODO visualize bbox
            #TODO publish filtered pointcloud?
            rospy.loginfo("BBOX: " + str(obj_bbox_dims))
            ########
            self.update_feedback("Check reachability")
            # Given the bounding box... 
            # move the torso first
            
            # shift object pose up by halfway, clustering code gives obj frame on the bottom because of too much noise on the table cropping (2 1pixel lines behind the objects)
            # TODO remove this hack, fix it in table filtering
            object_pose.pose.position.z += obj_bbox_dims[2] / 2.0
            
            # TESTING HACK
            object_pose.pose.position.x = 0.29
            
            rospy.loginfo("object_pose is: " + str(object_pose))
            torso_goal = createBendGoal(object_pose.pose.position.z)
            self.torso_as.send_goal_and_wait(torso_goal)
            print self.torso_as.get_result()
            

            
            # Add box representing the obstacle
            self.scene.add_box("object_bbx", object_pose,
                               (obj_bbox_dims[0], obj_bbox_dims[1], obj_bbox_dims[2]))
                        
            # move arms safely to position around this object (note we need a perfect pose here)
            rospy.loginfo("Creating goal for left arm")
            goal_left_pose = copy.deepcopy(object_pose)
            goal_left_pose.pose.position.x -= obj_bbox_dims[0] / 2.0
            goal_left_pose.pose.position.y += obj_bbox_dims[1] * 2.5
            goal_left_pose.pose.position.z -= 0.0
            goal_left_pose.pose.orientation = Quaternion(0.0, 0.0, 0.0, 1.0)
            rospy.loginfo("Pose: " + str(goal_left_pose))
            if DEBUG_MODE:
                pub = rospy.Publisher("/AAA_left_arm_pose", PoseStamped, latch=True)
                pub.publish(goal_left_pose)
            goal_left = create_move_group_pose_goal(goal_left_pose.pose, "left_arm", "hand_left_grasping_frame", plan_only=False)
            self.move_group_as.send_goal_and_wait(goal_left)
            
            rospy.loginfo("Creating goal for right arm")
            goal_right_pose = copy.deepcopy(object_pose)
            goal_right_pose.pose.position.x -= obj_bbox_dims[0] / 2.0
            goal_right_pose.pose.position.y -= obj_bbox_dims[1] * 2.5
            goal_right_pose.pose.position.z -= 0.0
            goal_right_pose.pose.orientation = Quaternion(0.0, 0.0, 0.0, 1.0)
            rospy.loginfo("Pose: " + str(goal_right_pose))
            if DEBUG_MODE:
                pub = rospy.Publisher("/AAA_right_arm_pose", PoseStamped, latch=True)
                pub.publish(goal_right_pose)
            goal_right = create_move_group_pose_goal(goal_right_pose.pose, "right_arm", "hand_right_grasping_frame", plan_only=False)
            self.move_group_as.send_goal_and_wait(goal_right)
            
            # Do a cartesian path with left arm to the bounding box
            
            # Do a cartesian path with right arm to the bounding box... plus a little more to push
            
            # unbend torso
            
            # We are done, maybe put a safer position or something
            
            self.current_goal.set_succeeded(result=self.as_result)
            
    def place_operation(self):
        """Execute the place operation"""
        if self.message_fields_ok():
            self.as_result = ObjectManipulationResult()
            goal_message_field = self.current_goal.get_goal()
            
            # cartesian path down to the table height given


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

    def generate_grasps(self, pose, width):
        """Send request to block grasp generator service"""
        goal = GenerateGraspsGoal()
        goal.pose = pose.pose
        goal.width = width
        grasp_options = GraspGeneratorOptions()
        grasp_options.grasp_axis = GraspGeneratorOptions.GRASP_AXIS_Y
        grasp_options.grasp_direction = GraspGeneratorOptions.GRASP_DIRECTION_DOWN
        grasp_options.grasp_rotation = GraspGeneratorOptions.GRASP_ROTATION_HALF
        goal.options.append(grasp_options)
        self.grasps_ac.send_goal(goal)
        if DEBUG_MODE:
            rospy.loginfo("Sent goal, waiting:\n" + str(goal))
        self.grasps_ac.wait_for_result()
        grasp_list = self.grasps_ac.get_result().grasps
        return grasp_list

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

    def get_pose_of_closest_table(self, input_pose):
        """Get the PoseStamped and the Table msg of the closest table to adjust height of the placing"""
        closest_table_posestamped = None
        closest_tablemsg = None
        closest_distance_z = 99999.9
        for mytable in self.last_tables.tables:
            table_posestamped = PoseStamped(header=mytable.header, pose=mytable.pose)
            if table_posestamped.header.frame_id != "base_link":
                self.tf_listener.waitForTransform("base_link", table_posestamped.header.frame_id, table_posestamped.header.stamp, rospy.Duration(5))
                table_pose = self.tf_listener.transformPose("base_link", table_posestamped)
            else:
                table_pose = table_posestamped
            if closest_table_posestamped == None:
                closest_table_posestamped = table_pose
                closest_distance_z = abs(table_pose.pose.position.z - input_pose.position.z)
            else:
                if abs(table_pose.pose.position.z - input_pose.position.z) < closest_distance_z:
                    closest_distance_z = abs(table_pose.pose.position.z - input_pose.position.z)
                    closest_table_posestamped = table_pose
                    closest_tablemsg = mytable
        rospy.loginfo("Closest table is at pose: " + str(closest_table_posestamped))
        return closest_tablemsg, closest_table_posestamped

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

    def send_arms_to_initial_pose(self, object_pose, obj_bbox_dims):
        rospy.loginfo("Creating goal for left arm")
        goal_left_pose = copy.deepcopy(object_pose)
        goal_left_pose.pose.position.x -= obj_bbox_dims[0] / 2.0
        goal_left_pose.pose.position.y += obj_bbox_dims[1] * 2.5
        goal_left_pose.pose.position.z -= 0.0
        goal_left_pose.pose.orientation = Quaternion(0.0, 0.0, 0.0, 1.0)
        rospy.loginfo("Pose: " + str(goal_left_pose))
        if DEBUG_MODE:
            pub = rospy.Publisher("/AAA_left_arm_pose", PoseStamped, latch=True)
            pub.publish(goal_left_pose)
        goal_left = create_move_group_pose_goal(goal_left_pose.pose, "left_arm", "hand_left_grasping_frame", plan_only=False)
        self.move_group_as.send_goal_and_wait(goal_left)
        
        rospy.loginfo("Creating goal for right arm")
        goal_right_pose = copy.deepcopy(object_pose)
        goal_right_pose.pose.position.x -= obj_bbox_dims[0] / 2.0
        goal_right_pose.pose.position.y -= obj_bbox_dims[1] * 2.5
        goal_right_pose.pose.position.z -= 0.0
        goal_right_pose.pose.orientation = Quaternion(0.0, 0.0, 0.0, 1.0)
        rospy.loginfo("Pose: " + str(goal_right_pose))
        if DEBUG_MODE:
            pub = rospy.Publisher("/AAA_right_arm_pose", PoseStamped, latch=True)
            pub.publish(goal_right_pose)
        goal_right = create_move_group_pose_goal(goal_right_pose.pose, "right_arm", "hand_right_grasping_frame", plan_only=False)
        self.move_group_as.send_goal_and_wait(goal_right)
            
        
        return
    
    def move_arm_straight_line(self, arm):
        return
    