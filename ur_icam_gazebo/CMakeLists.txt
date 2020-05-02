cmake_minimum_required(VERSION 2.8.3)

project(ur_icam_gazebo)

find_package(catkin REQUIRED COMPONENTS gazebo_msgs roscpp geometry_msgs std_srvs std_msgs message_generation rospy)

catkin_python_setup()

#generate_messages(DEPENDENCIES std_msgs)

#catkin_package(CATKIN_DEPENDS message_runtime)

# Specify locations of header files.
#include_directories(include ${catkin_INCLUDE_DIRS})


#install(DIRECTORY launch controller DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
