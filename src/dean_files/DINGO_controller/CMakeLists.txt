cmake_minimum_required(VERSION 3.0.2)

project(DINGO_controller)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs geometry_msgs genmsg)


## Generate added messages and services
generate_messages(DEPENDENCIES std_msgs geometry_msgs)

## Declare a catkin package
catkin_package()

## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(DINGO_controller src/DINGO_controller.cpp)
target_link_libraries(DINGO_controller ${catkin_LIBRARIES})
add_dependencies(DINGO_controller DINGO_controller_generate_messages_cpp)




