cmake_minimum_required(VERSION 3.0.2)
project(pose_detection_module)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  std_msgs
)

catkin_python_setup()

catkin_package(
   CATKIN_DEPENDS 
   roscpp rospy 
   sensor_msgs 
   std_msgs 
   model_server
   image_server
)

 catkin_install_python(PROGRAMS
   scripts/pose_detection_module
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )

install(FILES
   model/*
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
