################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 2.8.3)
project(vega_tools)

################################################################################
# Find catkin packages and libraries for catkin and system dependencies
################################################################################
find_package(catkin REQUIRED COMPONENTS
  rospy
  geometry_msgs
  std_msgs
  message_generation
)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Twist_timestamped.msg

#   Message2.msg
# )

################################################################################
# Setup for python modules and scripts
################################################################################
catkin_python_setup()


add_message_files(
  FILES
  SignalStrength.msg
#   Message2.msg
)


################################################################################
# Declare ROS messages, services and actions
################################################################################
generate_messages(
   DEPENDENCIES
   std_msgs 
#   geometry_msgs
 )

################################################################################
# Declare ROS dynamic reconfigure parameters
################################################################################

################################################################################
# Declare catkin specific configuration to be passed to dependent projects
################################################################################
catkin_package(
  CATKIN_DEPENDS rospy geometry_msgs std_msgs message_runtime
)

################################################################################
# Build
################################################################################

################################################################################
# Install
################################################################################
catkin_install_python(PROGRAMS
  nodes/vega_teleop_key
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

################################################################################
# Test
################################################################################
