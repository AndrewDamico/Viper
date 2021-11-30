cmake_minimum_required(VERSION 3.0.2)
project(model_server)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_generation
  roscpp
  rospy
  sensor_msgs
  std_msgs
)

catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
add_message_files(
    DIRECTORY msg
    FILES
    InferenceResults.msg
)

## Generate services in the 'srv' folder
add_service_files(
    FILES
    ImageRequest.srv
    ModelRequest.srv
)

generate_messages(
   DEPENDENCIES
   sensor_msgs
   std_msgs
   geometry_msgs
)


###################################
## catkin specific configuration ##
###################################

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES object_detection
   CATKIN_DEPENDS 
   roscpp rospy 
   sensor_msgs 
   std_msgs 
   geometry_msgs
   message_runtime
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

#############
## Install ##
#############

catkin_install_python(PROGRAMS
   scripts/modelserver.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
