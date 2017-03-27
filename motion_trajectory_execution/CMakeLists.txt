cmake_minimum_required(VERSION 2.8.3)
project(motion_trajectory_execution)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  kuka_motion_control
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  #vrep_common
  trajectory_planning
  planning_world_builder
  control_msgs
  tf
  #robot_interface_definition
  eband_local_planner
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependencies might have been
##     pulled in transitively but can be declared for certainty nonetheless:
##     * add a build_depend tag for "message_generation"
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
   INCLUDE_DIRS include
   LIBRARIES motion_trajectory_execution
   CATKIN_DEPENDS roscpp rospy std_msgs kuka_motion_control moveit_core moveit_ros_planning trajectory_planning planning_world_builder eband_local_planner
   DEPENDS system_lib
)


###########
## Build ##
###########

ADD_DEFINITIONS(
    -std=c++11
    -D__cplusplus=201103L #for eclipse
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS}
)


#Project Files
set(motion_trajectory_execution_files

#CPP Files
src/trajectory_execution_rviz.cpp
#src/trajectory_execution_vrep.cpp
src/trajectory_execution_robot.cpp

#Header files (added only for visualization in QtCreator)
include/motion_trajectory_execution/trajectory_execution_rviz.h
#include/motion_trajectory_execution/trajectory_execution_vrep.h
include/motion_trajectory_execution/trajectory_execution_robot.h

)

# Declare a cpp library
add_library(${PROJECT_NAME} ${motion_trajectory_execution_files})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

## Declare a cpp library
# add_library(motion_trajectory_execution
#   src/${PROJECT_NAME}/motion_trajectory_execution.cpp
# )


#Executable for running Trajectory Execution
add_executable(motion_execution_rviz src/run_motion_execution_rviz.cpp)
target_link_libraries(motion_execution_rviz ${PROJECT_NAME} ${catkin_LIBRARIES})

#add_executable(motion_execution_vrep src/run_motion_execution_vrep.cpp)
#target_link_libraries(motion_execution_vrep ${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(motion_execution_robot src/run_motion_execution_robot.cpp)
target_link_libraries(motion_execution_robot ${PROJECT_NAME} ${catkin_LIBRARIES})

 
#add_executable(vrep_ros_communication src/vrep_ros_communication.cpp)
#target_link_libraries(vrep_ros_communication ${PROJECT_NAME} ${catkin_LIBRARIES})
#add_dependencies(vrep_ros_communication vrep_common_generate_messages_cpp)

#Executable for running RRT-star Multi-Robot Algorithm
add_executable(motion_execution_rviz_multi_robot src/run_motion_execution_rviz_multi_robot.cpp)
target_link_libraries(motion_execution_rviz_multi_robot ${PROJECT_NAME} ${catkin_LIBRARIES})


#add_executable(vrep_ros_communication_lbr src/vrep_ros_communication_lbr.cpp)
#target_link_libraries(vrep_ros_communication_lbr ${PROJECT_NAME} ${catkin_LIBRARIES})
#add_dependencies(vrep_ros_communication_lbr vrep_common_generate_messages_cpp)




## Declare a cpp executable
# add_executable(motion_trajectory_execution_node src/motion_trajectory_execution_node.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(motion_trajectory_execution_node motion_trajectory_execution_generate_messages_cpp)


## Specify libraries to link a library or executable target against
# target_link_libraries(motion_trajectory_execution_node
#   ${catkin_LIBRARIES}
# )

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS motion_trajectory_execution motion_trajectory_execution_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_motion_trajectory_execution.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
