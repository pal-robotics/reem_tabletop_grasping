cmake_minimum_required(VERSION 2.8.3)
project(reem_tabletop_grasping)

find_package(catkin REQUIRED COMPONENTS
  actionlib genmsg block_grasp_generator geometry_msgs
  moveit_commander moveit_msgs object_recognition_clusters
  object_recognition_msgs rospy std_msgs tf trajectory_msgs
)

add_action_files(
  FILES
  GraspObject.action
  ObjectManipulation.action
)

generate_messages(
 DEPENDENCIES
 std_msgs object_recognition_msgs moveit_msgs  geometry_msgs
)

catkin_package(
 CATKIN_DEPENDS actionlib block_grasp_generator 
                geometry_msgs moveit_commander moveit_msgs 
                object_recognition_clusters object_recognition_msgs 
                rospy std_msgs tf trajectory_msgs
)
