#! /usr/bin/env python
"""
Created on 12/06/14

@author: Sam Pfeiffer

"""
import sys

import rospy
from sensor_msgs.msg import JointState
from control_msgs.msg import FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint

groups = ["all_joints", "left_arm", "right_arm", "both_arms", "left_arm_torso", "right_arm_torso", "both_arms_torso", "torso", "head", "right_hand", "left_hand", "right_hand_all", "left_hand_all"]
shortnamegroups = ["a", "la", "ra", "ba", "lat", "rat", "bat", "t", "h", "rh", "lh", "rha", "lha"]


class jointStateGrabber():
    """This class subscribes to the /joint_states topic of REEM
    and you can ask it for the current status of a group and get it
    outputted in some useful format, i.e.: print on screen,
    play_motion format, old xml format... """

    def __init__(self):
        self.current_joint_states = None
        self.head = ['head_1_joint', 'head_2_joint']
        self.torso = ['torso_1_joint', 'torso_2_joint']
        self.left_arm = ['arm_left_1_joint', 'arm_left_2_joint', 'arm_left_3_joint',
                           'arm_left_4_joint', 'arm_left_5_joint', 'arm_left_6_joint',
                           'arm_left_7_joint']
        self.right_arm = ['arm_right_1_joint', 'arm_right_2_joint', 'arm_right_3_joint',
                           'arm_right_4_joint', 'arm_right_5_joint', 'arm_right_6_joint',
                           'arm_right_7_joint']
        self.left_hand = ['hand_left_index_joint', 'hand_left_middle_joint', 'hand_left_thumb_joint'] # Only the actuated
        self.left_hand_all = self.left_hand + ['hand_left_index_1_joint', 'hand_left_index_2_joint', 'hand_left_index_3_joint',
                          'hand_left_middle_1_joint', 'hand_left_middle_2_joint', 'hand_left_middle_3_joint']
        self.right_hand = ['hand_right_index_joint', 'hand_right_middle_joint', 'hand_right_thumb_joint'] # Only the actuated
        self.right_hand_all = self.right_hand + ['hand_right_index_1_joint', 'hand_right_index_2_joint', 'hand_right_index_3_joint',
                               'hand_right_middle_1_joint', 'hand_right_middle_2_joint', 'hand_right_middle_3_joint']
        self.all_joints = self.torso + self.head + self.left_arm + self.right_arm + self.left_hand_all + self.right_hand_all
        self.right_arm_torso = self.torso + self.right_arm
        self.left_arm_torso = self.torso + self.left_arm
        self.both_arms = self.left_arm + self.right_arm
        self.both_arms_torso = self.torso + self.both_arms
        self.ids_list = []

        self.subs = rospy.Subscriber('/joint_states', JointState, self.getJointStates)

        # getting first message to correctly find joints
        while self.current_joint_states == None:
            rospy.sleep(0.1)
        rospy.loginfo("Node initialized. Ready to grab joint states.")


    def getJointStates(self, data):
        #rospy.loginfo("Received from topic data!")
        self.current_joint_states = data

    def createGoalFromCurrentJointStateForArm(self, group='right_arm_torso'):
        """ Get the joints for the specified group and create a FollowJointTrajectoryGoal
        with these joints and values for the joints """
        names, values = self.getNamesAndMsgList(group=group)

        fjtg = FollowJointTrajectoryGoal()
        fjtg.trajectory.joint_names.extend(names)
        jtp = JointTrajectoryPoint(positions=values, velocities=len(values) * [0.0], time_from_start=0)
        fjtg.trajectory.points.append(jtp)

        rospy.loginfo("follow joint trajectory goal:\n" + str(fjtg))

        return fjtg


    def getNamesAndMsgList(self, group='right_arm_torso'):
        """ Get the joints for the specified group and return this name list and a list of it's values in joint_states
        Note: the names and values are correlated in position """

        list_to_iterate = getattr(self, group)
        curr_j_s = self.current_joint_states
        ids_list = []
        msg_list = []
        rospy.logdebug("Current message: " + str(curr_j_s))
        for joint in list_to_iterate:
            idx_in_message = curr_j_s.name.index(joint)
            ids_list.append(idx_in_message)
            msg_list.append(curr_j_s.position[idx_in_message])
        rospy.logdebug("Current position of joints in message: " + str(ids_list))
        rospy.logdebug("Current msg:" + str(msg_list))

        return list_to_iterate, msg_list

    def printNamesAndValues(self, group='right_arm_torso'):
        """Given a group, print in screen in a pretty way joint names and it's values"""
        names, values = self.getNamesAndMsgList(group=group)
        print "Name =  Joint Value"
        print "================="
        for nam, val in zip(names, values):
            print nam + " = " + str(val)

    def printOnlyValues(self, group):
        names, values = self.getNamesAndMsgList(group=group)
        print group + " = " + str(names)
        print "[ ",
        for i in range(0, len(values)):
            if i == (len(values) - 1): # if it's the last one don't put comma and add an enter
                print str(values[i]) + " ]"
            else:
                print str(values[i])+",",

    def getNamesAndValues(self, group):
        names, values = self.getNamesAndMsgList(group=group)
        return names, values
        

def usage(program_name):
    """ print usage """
    print "Usage:"
    print program_name
    print "Print all joint names and it's values.\n"
    print program_name + " -i"
    print "Interactive mode, write a group name and it will be printed.\n"
    print program_name + " <group>"
    print "Will print the group joint names and its joint values."
    print "Available groups are: " + str(groups)
    print "Or their shortname version: " + str(shortnamegroups)

def shortToLargeName(shortname):
    """ Translate shortname of group to long name of group"""
    try:
        longname = groups[shortnamegroups.index(shortname)]
    except ValueError:
        print "\"" + shortname + "\" not a valid short name."
        print "Valid short names:"
        print shortnamegroups
        exit(0)
    return longname

def isGroupName(supposed_group_name):
    """ Check if a given group string is a group or shortname for a group"""
    if isShortGroupName(supposed_group_name):
        return True
    elif not isLongGroupName(supposed_group_name):
        return False
    return True

def isShortGroupName(group_name):
    """Check if a given group is a shortname for a group"""
    try:
        idx = shortnamegroups.index(group_name)
    except ValueError:
        return False
    return True

def isLongGroupName(group_name):
    """Check if a given group is a name for a group"""
    try:
        idx = groups.index(group_name)
    except ValueError:
        return False
    return True

def getGroupNameIfExists(group_name):
    """Returns the group name if it exists None otherwise"""
    if isGroupName(group_name): # if a valid name is given for a group, print that group
        if isShortGroupName(group_name):
            return shortToLargeName(group_name)
        else:
            return group_name
    else:
        return None

def create_motion(name, joints, positions, time_from_start, usage, description):
    """Given the input data create a string which represents a play_motion motion, should give something like:
    
    pre_grasp:
      joints: 
      ['arm_right_1_joint', 'arm_right_2_joint', 'arm_right_3_joint',
              'arm_right_4_joint', 'arm_right_5_joint', 'arm_right_6_joint',
              'arm_right_7_joint',
              'arm_left_1_joint', 'arm_left_2_joint', 'arm_left_3_joint',
              'arm_left_4_joint', 'arm_left_5_joint', 'arm_left_6_joint',
              'arm_left_7_joint']
      points:
      - positions: 
      [0.934186234366, 0.0384229614136, -1.28266429768,
              0.804876373305, 0.132163508053, 0.886241414735,
              0.687621553401,
              0.934186234366, 0.0384229614136, -1.28266429768,
              0.804876373305, 0.132163508053, 0.886241414735,
              0.687621553401 ]
        time_from_start: 0.0
      meta:
        name: pre_grasp
        usage: posture
        description: Initial pose for dual arms grasping. Arms up and parallel. 34cm between fingers in RH2.
        """
    
    motion = ""
    motion += "    " # Initial tabulation under "play_motion: \n motions:"
    motion += name + ":\n"
    motion += "      joints:\n"
    motion += "              " + str(joints) + "\n"
    motion += "      points:\n"
    motion += "      - positions:\n"
    motion += "              " + str(positions) + "\n"
    motion += "        time_from_start: " + str(time_from_start) + "\n"
    motion += "      meta:\n"
    motion += "        name: " + name + "\n"
    motion += "        usage: " + usage + "\n"
    motion += "        description: " + description + "\n\n"
    return motion

def create_file_header():
    """Return header of a play motion yaml file (in a string)"""
    header = "play_motion:\n  motions:\n"
    return header

if __name__ == '__main__':
    rospy.init_node('joint_state_grabber_for_play_motion')
    node = jointStateGrabber()
    
    print "Write 'g' (from Grab) to see the actual position of both arms"
    print "(Write exit to exit)."
    while True:
        input = raw_input("> ")
        if input == "exit":
            exit(0)
        elif input == "g":
            node.printNamesAndValues(getGroupNameIfExists("both_arms"))
            print "\n"
            node.printOnlyValues(getGroupNameIfExists("both_arms"))
            print "\nIf you are happy with these values"
            print "press 'k' (from oK) to create a motion, 'o' to set dummy values to most data"
            print "If not, press whatever else."
            confirm = raw_input("> ")
            if confirm == "k" or confirm == "o":
                #name, joints, positions, time_from_start, usage, description
                name = raw_input("name: ")
                if confirm == "o":
                    time_from_start = 0.0
                    usage = "posture"
                    description = "Posture generated with poses_grabber"
                else:
                    time_from_start = float(raw_input("time_from_start: "))
                    usage = raw_input("usage: ")
                    description = raw_input("description: ")

                names, values = node.getNamesAndValues(getGroupNameIfExists("both_arms"))
                motion = create_motion(name, names, values, time_from_start, usage, description)
                print "\nFinal motion:\n ###################"
                print motion
                print " ########################"
                
                
                
                

