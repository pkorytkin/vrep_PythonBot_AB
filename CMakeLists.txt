cmake_minimum_required(VERSION 3.0.2)
project(vrep_PythonBot_AB)
## Has dependency on message_generation, std_msgs, roscpp.
## An error occurs during the build if these packages do not exist.
find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    std_msgs
    message_generation
    )
catkin_package(
   CATKIN_DEPENDS
   message_generation
   roscpp
   rospy
   std_msgs
)