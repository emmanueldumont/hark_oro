# %Tag(FULLTEXT)%
cmake_minimum_required(VERSION 2.8.3)
project(hark_oro)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs message_generation) #genmsg)

add_message_files(FILES HarkSource.msg HarkSourceVal.msg)

## Generate added messages and services
generate_messages(DEPENDENCIES std_msgs)

## Declare a catkin package
catkin_package( CATKIN_DEPENDS message_runtime )

install(DIRECTORY include/${hark_msgs}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.hpp"
  PATTERN ".svn" EXCLUDE)

install(DIRECTORY include/${orolib}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.hpp"
  PATTERN ".svn" EXCLUDE)

## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS})

add_subdirectory(src)

# %EndTag(FULLTEXT)%

