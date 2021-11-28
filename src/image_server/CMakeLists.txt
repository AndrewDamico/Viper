cmake_minimum_required(VERSION 3.0.2)
project(image_server)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  std_msgs
  message_generation
)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS 
  roscpp 
  rospy 
  sensor_msgs 
  std_msgs
  message_runtime
  model_server
)

catkin_install_python(PROGRAMS
   scripts/image_server
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
