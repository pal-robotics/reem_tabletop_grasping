#!/usr/bin/env python
"""
Created on 6/06/14

@author: Sam Pfeiffer
"""

# system stuff
import sys
import copy

# ROS stuff
import rospy
import moveit_commander
import tf
from moveit_msgs.srv import ExecuteKnownTrajectory, ExecuteKnownTrajectoryRequest
# My stuff
from cartesian_goals import trajectoryConstructor


if __name__ == '__main__':
    rospy.init_node("Testing_generating_cartesian")
    rospy.sleep(0.3)
    rospy.loginfo("Getting a TransformListener...")
    tf_listener = tf.TransformListener()
    tC = trajectoryConstructor()
    
    moveit_commander.roscpp_initialize(sys.argv)
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    right_group = moveit_commander.MoveGroupCommander("right_arm")
    
    rospy.sleep(1)
    ps = right_group.get_current_pose()
    print ps
    
    print "Transforming"
    ps.header.stamp = rospy.Time(0) # Setting time to last one
    try:
        base_link_hand_pose = tf_listener.transformPose("base_link", ps)
    except:
        print "Retry transform"
        rospy.sleep(0.4)
        ps.header.stamp = rospy.Time(0) # Setting time to last one
        base_link_hand_pose = tf_listener.transformPose("base_link", ps)

    rospy.loginfo("Transformed pose:\n" + str(base_link_hand_pose))
    
    first_pose = base_link_hand_pose.pose
    print first_pose
    second_pose = copy.deepcopy(first_pose)
    #second_pose.position.z += 0.3
    second_pose.position.y += 0.3
    print second_pose
    # Poses in base_link
    poselist = tC.createCartesianPoseList(first_pose, second_pose, 0.001)
    print "Got this poselist:"
    print poselist
    robot_traj = tC.computeIKsPose(poselist, "right_arm", 7.0)
    print "Got this robot_traj:"
    print robot_traj
    
    print "lets execute it then"
    
    ekp = rospy.ServiceProxy('/execute_kinematic_path', ExecuteKnownTrajectory)
    ekp.wait_for_service()
     
    print "connected to execute_kinematic_path"
    #rospy.sleep(3)
    ektr = ExecuteKnownTrajectoryRequest()
    ektr.trajectory = robot_traj
    ektr.wait_for_execution = True
    print "Sending call "
    ekp.call(ektr)
    print "!!!! Call done"
    while not rospy.is_shutdown():
        rospy.sleep(0.5)