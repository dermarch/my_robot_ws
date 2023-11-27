cmake_minimum_required(VERSION 3.0.2)
project(teleop_key)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
  robot_msgs
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES key_ctr
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
 include
  ${catkin_INCLUDE_DIRS}
)

add_executable(key_ctrl src/key_ctrl.cpp)
target_link_libraries(key_ctrl
 ${catkin_LIBRARIES}
)


