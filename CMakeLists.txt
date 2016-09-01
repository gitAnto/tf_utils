cmake_minimum_required(VERSION 2.8.3)
project(tf_utils)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  tf
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES tf_utils
#  CATKIN_DEPENDS rospy
#  DEPENDS system_lib
)

###########
## Build ##
###########


include_directories(
  ${catkin_INCLUDE_DIRS}
)


