# What version of CMake is needed?
cmake_minimum_required(VERSION 2.8.3)

# The name of this package.
project(ur_icam)

# Find the catkin build system, and any other packages on which we depend.
find_package(catkin REQUIRED COMPONENTS gazebo_msgs roscpp geometry_msgs std_srvs std_msgs message_generation rospy)
# Declare our catkin package.

catkin_python_setup()

#add_message files
#add_message_files(FILES blocks_poses.msg)

generate_messages(DEPENDENCIES std_msgs)

catkin_package(CATKIN_DEPENDS message_runtime)

# Specify locations of header files.
include_directories(include ${catkin_INCLUDE_DIRS})
