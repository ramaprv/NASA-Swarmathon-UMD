cmake_minimum_required(VERSION 2.8.3)
project(ghost)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  ghost_srv
  
)

catkin_package(
CATKIN_DEPENDS roscpp rospy std_msgs ghost_srv
)

include_directories(
  include ${catkin_INCLUDE_DIRS}
)


add_executable(ghost
        src/Ghost.cpp
)

add_dependencies(ghost ghost_srv)

target_link_libraries(ghost
   ${catkin_LIBRARIES}
)

