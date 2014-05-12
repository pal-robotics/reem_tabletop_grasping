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
from object_recognition_msgs.msg import RecognizedObjectArray
from object_recognition_clusters import ClusterBoundingBoxFinder
from block_grasp_generator.msg import GenerateBlockGraspsAction, GenerateBlockGraspsGoal
# manipulation imports
from moveit_msgs.msg import PickupAction, PlaceAction
from moveit_commander import PlanningSceneInterface

from helper_functions import createPickupGoal, dist_between_poses, createPlaceGoal, moveit_error_dict
import moveit_msgs
from moveit_msgs.msg._MoveItErrorCodes import MoveItErrorCodes

# TODO: dynamic param to setup debug info
DEBUG_MODE = True
if DEBUG_MODE:
    from visualizing_functions import publish_grasps_as_poses

RECOGNIZED_OBJECT_ARRAY_TOPIC = '/recognized_object_array'
TO_BE_GRASPED_OBJECT_POSE_TOPIC = '/to_grasp_object_pose'
PICKUP_AS = '/pickup'
PLACE_AS = '/place'
GRASP_GENERATOR_AS = '/grasp_generator_server/generate'
DEPTH_THROTLE_SRV = '/depth_throtle'

OBJECT_MANIPULATION_AS = 'object_manipulation_server'


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
        self.sub = rospy.Subscriber(RECOGNIZED_OBJECT_ARRAY_TOPIC, RecognizedObjectArray, self.objects_callback)

        if DEBUG_MODE:
            self.to_grasp_object_pose_pub = rospy.Publisher(TO_BE_GRASPED_OBJECT_POSE_TOPIC, PoseStamped)

        rospy.loginfo("Connecting to pickup AS '" + PICKUP_AS + "'...")
        self.pickup_ac = SimpleActionClient(PICKUP_AS, PickupAction)
        self.pickup_ac.wait_for_server()

        rospy.loginfo("Connecting to place AS '" + PLACE_AS + "'...")
        self.place_ac = SimpleActionClient(PLACE_AS, PlaceAction)
        self.place_ac.wait_for_server()

        rospy.loginfo("Connecting to grasp generator AS '" + GRASP_GENERATOR_AS + "'...")
        self.grasps_ac = SimpleActionClient(GRASP_GENERATOR_AS, GenerateBlockGraspsAction)
        self.grasps_ac.wait_for_server()

        rospy.loginfo("Connecting to depth throttle server '" + DEPTH_THROTLE_SRV + "'...")
        self.depth_service = rospy.ServiceProxy(DEPTH_THROTLE_SRV, Empty)
        self.depth_service.wait_for_service()

        rospy.loginfo("Getting a PlanningSceneInterface instance...")
        self.scene = PlanningSceneInterface()

        # blocking action server
        rospy.loginfo("Creating Action Server '" + name + "'...")
        self.grasp_obj_as = ActionServer(name, ObjectManipulationAction, self.goal_callback, self.cancel_callback, False)
        self.as_feedback = ObjectManipulationFeedback()
        self.as_result = ObjectManipulationActionResult()
        self.current_goal = None

        # Take care of left and right arm grasped stuff
        self.right_hand_object = None
        self.left_hand_object = None
        self.current_side = 'right'

        rospy.loginfo("Starting '" + OBJECT_MANIPULATION_AS + "' Action Server!")
        self.grasp_obj_as.start()

    def objects_callback(self, data):
        rospy.loginfo(rospy.get_name() + ": This message contains %d objects." % len(data.objects))
        self.last_clusters = data

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
            self.update_feedback("Checking hand to use")
            # Check which arm group was requested and if it's currently free of objects
            if 'right' in goal_message_field.group:
                if self.right_hand_object:  # Something already in the hand
                    self.update_aborted("Right hand already contains an object.")
                    return  # necessary?
                self.current_side = 'right'
            elif 'left' in goal_message_field.group:
                if self.left_hand_object:
                    self.update_aborted("Left hand already contains an object.")
                    return
                self.current_side = 'left'

            # Publish pose of goal position
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
            rospy.logdebug("BBOX: " + str(obj_bbox_dims))
            ########
            self.update_feedback("Check reachability")
            ########
            self.update_feedback("Generating grasps")
            rospy.logdebug("Object pose before tf thing is: " + str(object_pose))
            #transform pose to base_link, IS THIS NECESSARY?? should be a function in any case
            self.tf_listener.waitForTransform("base_link", object_pose.header.frame_id, object_pose.header.stamp, rospy.Duration(15))
            trans_pose = self.tf_listener.transformPose("base_link", object_pose)
            object_pose = trans_pose
            #HACK remove orientation -> pose is aligned with parent(base_link)
            object_pose.pose.orientation.w = 1.0
            object_pose.pose.orientation.x = 0.0
            object_pose.pose.orientation.y = 0.0
            object_pose.pose.orientation.z = 0.0
            # shift object pose up by halfway, clustering code gives obj frame on the bottom because of too much noise on the table cropping (2 1pixel lines behind the objects)
            # TODO remove this hack, fix it in table filtering
            object_pose.pose.position.z += obj_bbox_dims[2] / 2.0
            grasp_list = self.generate_grasps(object_pose, obj_bbox_dims[0])  # width is the bbox size on x
            # check if there are grasps, if not, abort
            if len(grasp_list) == 0:
                self.update_aborted("No grasps received")
                return
            if DEBUG_MODE:  # TODO: change to dynamic param
                publish_grasps_as_poses(grasp_list)
            self.current_goal.publish_feedback(self.as_feedback)
            ########
            self.update_feedback("Setup planning scene")
            #remove old objects
            #self.scene.remove_world_object("object_to_grasp")

            # Add object to grasp to planning scene
            self.scene.add_box(self.current_side + "_hand_object", object_pose,
                               (obj_bbox_dims[0], obj_bbox_dims[1], obj_bbox_dims[2]))
            self.as_result.object_scene_name = self.current_side + "_hand_object"
            ########
            self.update_feedback("Execute grasps")
            pug = createPickupGoal(self.current_side + "_hand_object", grasp_list, group=goal_message_field.group)
            rospy.loginfo("Sending pickup goal")
            self.pickup_ac.send_goal(pug)
            rospy.loginfo("Waiting for result...")
            self.pickup_ac.wait_for_result()
            result = self.pickup_ac.get_result()
            rospy.loginfo("Human readable error: " + str(moveit_error_dict[result.error_code.val]))
            ########
            self.update_feedback("finished")
            self.as_result.object_pose = object_pose
            self.as_result.error_code = result.error_code
            self.as_result.error_message = str(moveit_error_dict[result.error_code.val])
            if result.error_code.val == MoveItErrorCodes.SUCCESS:
                if self.current_side == 'right':
                    self.right_hand_object = self.current_side + "_hand_object"
                elif self.current_side == 'left':
                    self.left_hand_object = self.current_side + "_hand_object"
                self.current_goal.set_succeeded(result=self.as_result)
            else:
                self.update_aborted(text="MoveIt pick failed", manipulation_result=self.as_result)
        else:
            self.update_aborted("Goal fields not correctly fulfilled")

    def place_operation(self):
        """Execute the place operation"""
        if self.message_fields_ok():
            self.as_result = ObjectManipulationResult()
            goal_message_field = self.current_goal.get_goal()
            self.update_feedback("Checking arm to use")
            # Check which arm group was requested and if it currently has an object
            if 'right' in goal_message_field.group:
                if not self.right_hand_object:  # Something already in the hand
                    self.update_aborted("Right hand does not contain an object.")
                    return  # necessary?
                self.current_side = 'right'
                current_target = self.right_hand_object
            elif 'left' in goal_message_field.group:
                if not self.left_hand_object:
                    self.update_aborted("Left hand does not contain an object.")
                    return
                self.current_side = 'left'
                current_target = self.left_hand_object

            # transform pose to base_link if needed
            if goal_message_field.target_pose.header.frame_id != "base_link":
                self.tf_listener.waitForTransform("base_link", goal_message_field.target_pose.header.frame_id, goal_message_field.target_pose.header.stamp, rospy.Duration(5))
                placing_pose = self.tf_listener.transformPose("base_link", goal_message_field.target_pose)
            else:
                placing_pose = goal_message_field.target_pose.pose
            ####  TODO: ADD HERE LOGIC ABOUT SEARCHING GOOD PLACE POSE ####
            self.update_feedback("Sending place order to MoveIt!")
            placing_pose = PoseStamped(header=Header(frame_id="base_link"), pose=placing_pose)
            goal = createPlaceGoal(placing_pose, group=goal_message_field.group, target=current_target)
            rospy.loginfo("Sending place goal")
            self.place_ac.send_goal(goal)
            rospy.loginfo("Waiting for result...")
            self.place_ac.wait_for_result()
            result = self.place_ac.get_result()
            rospy.loginfo("Human readable error: " + str(moveit_error_dict[result.error_code.val]))
            self.as_result.object_pose = placing_pose
            # Emptying hand
            self.update_feedback("Emptying hand")
            if self.current_side == 'right':
                self.as_result.object_scene_name = current_target
                self.right_hand_object = None
            elif self.current_side == 'left':
                self.as_result.object_scene_name = current_target
                self.left_hand_object = None

            self.as_result.error_code = result.error_code
            self.as_result.error_message = str(moveit_error_dict[result.error_code.val])
            if result.error_code.val == MoveItErrorCodes.SUCCESS:
                self.current_goal.set_succeeded(result=self.as_result)
            else:
                self.update_aborted(text="MoveIt place failed", manipulation_result=self.as_result)
        else:
            self.update_aborted("Goal fields not correctly fulfilled")

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
        if not self.grasps_ac.wait_for_server(rospy.Duration(5.0)):
            return []
        rospy.loginfo("Successfully connected.")
        goal = GenerateBlockGraspsGoal()
        goal.pose = pose.pose
        goal.width = width
        self.grasps_ac.send_goal(goal)
        rospy.loginfo("Sent goal, waiting:\n" + str(goal))
        t_start = rospy.Time.now()
        self.grasps_ac.wait_for_result()
        t_end = rospy.Time.now()
        t_total = t_end - t_start
        rospy.loginfo("Result received in " + str(t_total.to_sec()))
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

    def wait_for_recognized_array(self, wait_time=6, timeout_time=10):
        """Ask for depth images until we get a recognized array
        wait for wait_time between depth throtle calls
        stop if timeout_time is achieved
        If we dont find it in the correspondent time return false, true otherwise"""
        initial_time = rospy.Time.now()
        self.last_clusters = None
        count = 0
        num_calls = 1
        self.depth_service.call(EmptyRequest())
        rospy.loginfo("Depth throtle server call #" + str(num_calls))
        rospy.loginfo("Waiting for a recognized array...")
        while rospy.Time.now() - initial_time < rospy.Duration(timeout_time) and self.last_clusters == None:
            rospy.sleep(0.1)
            count += 1

            if count >= wait_time / 10:
                self.depth_service.call(EmptyRequest())
                num_calls += 1
                rospy.loginfo("Depth throtle server call #" + str(num_calls))
        if self.last_clusters == None:
            return False
        else:
            return True
