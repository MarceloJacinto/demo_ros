cmake_minimum_required(VERSION 3.0.2)
project(demo_python)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  # Add other ROS packages here
  rospy
  std_msgs
  nav_msgs
  sensor_msgs

  # For generating the messages
  message_generation
)

catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
add_message_files(
  DIRECTORY msg
)

## Generate services in the 'srv' folder
# add_service_files(
#   DIRECTORY srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   DIRECTORY action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS 
  rospy 
  std_msgs
  nav_msgs
  sensor_msgs
  message_runtime
)

##########

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# catkin_install_python(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )
