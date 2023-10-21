cmake_minimum_required(VERSION 3.0.2)
project(can_drive)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  message_generation
  sensor_msgs
  tf 
  robot_msgs
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES can_chc
   CATKIN_DEPENDS message_runtime
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  usr/include/
)

link_directories(
 lib
  ${catkin_LIB_DIRS}
  usr/lib/
)

add_executable(can_main src/can_main.cpp src/canbus.cpp)
target_link_libraries(can_main
	${catkin_LIBRARIES}
	CanCmd
  pthread
)

# add_executable(can_main src/can_main.cpp)
# target_link_libraries(can_main
# 	${catkin_LIBRARIES}
# 	CanCmd
#   pthread
#   like
# )

# install

## 添加python程序．in contrast to setup.py, you can choose the destination
# install(PROGRAMS
# scripts/talker.py
# scripts/listener.py
# DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## 添加可执行文件或者库文件，Mark executables and/or libraries for installation
install(TARGETS can_main can_main
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# ## 添加头文件．Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

# # 添加launch config
install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION} )
install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION} )







