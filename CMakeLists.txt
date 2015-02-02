cmake_minimum_required(VERSION 2.8.3)
project(robotnik_trajectory_control)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  genmsg
  sensor_msgs
  robotnik_msgs
  actionlib_msgs
  actionlib
)


catkin_python_setup()

## Generate messages in the 'msg' folder
add_message_files(
   FILES
   Actions.msg
   State.msg
   SubcomponentState.msg
)

## Generate services in the 'srv' folder
add_service_files(
   FILES
   TrajExecActions.srv
   GetJointsLimits.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   std_msgs  # Or other packages containing msgs
   robotnik_msgs
)

catkin_package(

)

