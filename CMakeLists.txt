cmake_minimum_required(VERSION 2.8.3)
project(reem_tabletop_grasping)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  genmsg
  block_grasp_generator
  geometry_msgs
  moveit_commander
  moveit_msgs
  object_recognition_clusters
  object_recognition_msgs
  rospy
  std_msgs
  tf
  trajectory_msgs
)

#catkin_python_setup()

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
add_action_files(
  FILES
  GraspObject.action
)

## Generate added messages and services with any dependencies listed here
generate_messages(
 DEPENDENCIES
 std_msgs object_recognition_msgs moveit_msgs  geometry_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES reem_tabletop_grasping
 CATKIN_DEPENDS actionlib block_grasp_generator geometry_msgs moveit_commander moveit_msgs object_recognition_clusters object_recognition_msgs rospy std_msgs tf trajectory_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)
