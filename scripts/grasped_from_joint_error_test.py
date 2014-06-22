#! /usr/bin/env python
import rospy
from control_msgs.msg import JointTrajectoryControllerState
RIGHT_ARM_STATE_TOPIC = '/right_arm_controller/state'
ENDC = '\033[0m'
FAIL = '\033[91m'
OKGREEN = '\033[92m'

def grasped_from_joint_error():
    right_arm_error_th = 0.009 #0.015
    msg = rospy.wait_for_message(RIGHT_ARM_STATE_TOPIC, JointTrajectoryControllerState)
    sum_error = 0.0
    for joint_err_val in msg.error.positions:
        sum_error += abs(joint_err_val)
    print "threshold is: "
    print right_arm_error_th
    print "sum_error is:"
    print sum_error
    if sum_error > right_arm_error_th:
        return True
    return False

rospy.init_node('aaaa')

while True:
    if grasped_from_joint_error():
        print OKGREEN + "Grasped!" + ENDC
    else:
        print FAIL + "Not grasped :(" + ENDC

    rospy.sleep(0.5)