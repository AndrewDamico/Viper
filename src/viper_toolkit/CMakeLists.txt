cmake_minimum_required(VERSION 3.0.2)
project(viper_toolkit)

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
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
)
