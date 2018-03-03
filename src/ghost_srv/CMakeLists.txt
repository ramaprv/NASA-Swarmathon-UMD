cmake_minimum_required(VERSION 2.8.3)
project(ghost_srv)

SET(CMAKE_CXX_FLAGS "-std=c++11")

find_package(catkin REQUIRED COMPONENTS
  std_msgs 
  message_generation
)

# Service files will be added here
 add_service_files(
 FILES
 radius.srv
)

# Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   std_msgs
)

catkin_package(CATKIN_DEPENDS std_msgs message_runtime)
