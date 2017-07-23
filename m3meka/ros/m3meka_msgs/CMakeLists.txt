cmake_minimum_required(VERSION 2.8)
project(m3meka_msgs)

find_package(catkin REQUIRED COMPONENTS
   roscpp
   rospy
   std_msgs
   message_generation
   m3_msgs
)


add_message_files(
  FILES
  M3JointStatus.msg
  M3OmnibaseJoy.msg
)

add_service_files(
  FILES
  M3HumanoidCmd.srv
  M3HumanoidParam.srv
  M3HumanoidStatus.srv
  M3JointArrayCmd.srv
  M3JointArrayParam.srv
  M3JointArrayStatus.srv
  M3LoadX6Cmd.srv
  M3LoadX6Param.srv
  M3LoadX6Status.srv

)

generate_messages(   
  DEPENDENCIES  
  std_msgs 
  m3_msgs
)  

catkin_package(
  CATKIN_DEPENDS message_runtime )


