cmake_minimum_required(VERSION 3.0.2)
project(augmented_vr_module)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  rospy
  sensor_msgs
  std_msgs
)

catkin_python_setup()

catkin_package()

catkin_install_python(PROGRAMS
   scripts/augmented_vr
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
