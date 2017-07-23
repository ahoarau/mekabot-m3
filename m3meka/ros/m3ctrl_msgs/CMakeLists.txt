cmake_minimum_required(VERSION 2.8)
project(m3ctrl_msgs)

find_package(catkin REQUIRED COMPONENTS
   roscpp
   rospy
   std_msgs
   message_generation
)


add_message_files(
  FILES
  M3JointCmd.msg
)

generate_messages(   
  DEPENDENCIES  
  std_msgs 
)  

catkin_package(
  CATKIN_DEPENDS message_runtime )


