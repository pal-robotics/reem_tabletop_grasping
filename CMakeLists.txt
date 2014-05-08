cmake_minimum_required(VERSION 2.8.3)
project(reem_tabletop_grasping)

find_package(catkin REQUIRED COMPONENTS
  actionlib actionlib_msgs message_generation block_grasp_generator geometry_msgs
  moveit_commander moveit_msgs object_recognition_clusters
  object_recognition_msgs rospy std_msgs tf trajectory_msgs
)

catkin_python_setup()

add_action_files(
  DIRECTORY action
  FILES GraspObject.action
        ObjectManipulation.action
)

generate_messages(
 DEPENDENCIES
 actionlib_msgs std_msgs object_recognition_msgs moveit_msgs  geometry_msgs
)

catkin_package(
 CATKIN_DEPENDS actionlib_msgs message_runtime actionlib block_grasp_generator 
                geometry_msgs moveit_commander moveit_msgs 
                object_recognition_clusters object_recognition_msgs 
                rospy std_msgs tf trajectory_msgs
)

install (PROGRAMS scripts/reem_object_manipulation_as.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})