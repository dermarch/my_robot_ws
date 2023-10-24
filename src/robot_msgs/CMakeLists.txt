cmake_minimum_required(VERSION 3.0.2)
project(robot_msgs)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  sensor_msgs
  nav_msgs
  roscpp
  rospy
  std_msgs
  message_generation
)


add_message_files(FILES
  Pump_States.msg
  Pump_Cmd.msg
  Cylinder_Cmd.msg
  Cylinder_States.msg
  Joint_States.msg
  Joint_Cmd.msg
  Canopen.msg
)

add_service_files(
  FILES
  mcs_cmd.srv
  grab_put.srv
  mcs_function.srv
  tensor_camera.srv
)

generate_messages(DEPENDENCIES
  std_msgs geometry_msgs sensor_msgs
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES kzc_ros_msgs
   CATKIN_DEPENDS geometry_msgs roscpp rospy std_msgs message_runtime
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)


# install
## 添加头文件．Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

# 添加launch config
install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION} )
install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION} )
