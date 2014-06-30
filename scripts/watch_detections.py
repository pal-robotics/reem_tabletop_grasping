#! /usr/bin/env python
'''
@author: Roger Boldu
@author: Sammy Pfeiffer

This class enables the robot to move straight
(forward or backwards) using odometry.
'''

import rospy


from blort_msgs.msg import RecognizeActionResult
from geometry_msgs.msg import PoseArray, Pose, Quaternion
from object_recognition_msgs.msg import RecognizedObjectArray, RecognizedObject
import copy

RESULT_TOPIC = '/blort_tracker/recognize_object/result'
POSE_OBJ_ARR_TOPIC = '/detected_objects'

class detectionShower():
    def __init__(self):
        self.subs = rospy.Subscriber(RESULT_TOPIC, RecognizeActionResult, self.results_cb)
        self.pub = rospy.Publisher(POSE_OBJ_ARR_TOPIC, PoseArray)
        self.last_detections = None
        
    def results_cb(self, data):
        print "Hey we got data!:" + str(data)
        self.last_detections = data

    def run(self):
        while not rospy.is_shutdown():
            if self.last_detections != None:
                if len(self.last_detections.result.recognized_objects.objects) > 0:         
                    pa = PoseArray()
                    pa.header.stamp = self.last_detections.result.recognized_objects.objects[0].pose.header.stamp
                    pa.header.frame_id = self.last_detections.result.recognized_objects.objects[0].pose.header.frame_id
                    for obj in self.last_detections.result.recognized_objects.objects:
                        topub_pose = copy.deepcopy(obj.pose.pose.pose)
                        topub_pose.orientation = Quaternion(0.0, 0.0, 0.0, 1.0)
                        pa.poses.append(topub_pose)
                    self.pub.publish(pa)
            rospy.sleep(0.2)

        
if __name__ == '__main__':
    rospy.init_node('show_results_detections')
    rospy.sleep(1)
    ds = detectionShower()
    ds.run()