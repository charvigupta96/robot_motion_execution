# http://ros.org/doc/groovy/api/catkin/html/user_guide/supposed.html
cmake_minimum_required(VERSION 2.8.3)
project(eband_local_planner)

find_package(catkin REQUIRED 
  COMPONENTS
    base_local_planner
    cmake_modules
    control_toolbox
    costmap_2d
    geometry_msgs
    nav_core
    nav_msgs
    pluginlib
    roscpp
    tf
    tf_conversions
)

find_package(Eigen REQUIRED)
include_directories(
  include
  ${Eigen_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

catkin_package(
  CATKIN_DEPENDS base_local_planner control_toolbox costmap_2d geometry_msgs nav_core nav_msgs pluginlib roscpp tf tf_conversions
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
)


ADD_DEFINITIONS(
    -std=c++11
    -D__cplusplus=201103L #for eclipse
)

#Project Files
set(eband_planner_files

#CPP Files
src/eband_local_planner_ros.cpp
src/eband_local_planner.cpp
src/conversions_and_types.cpp
src/eband_visualization.cpp
src/eband_trajectory_controller.cpp

#Header files (added only for visualization in QtCreator)
include/eband_local_planner/eband_local_planner_ros.h
include/eband_local_planner/eband_local_planner.h
include/eband_local_planner/conversions_and_types.h
include/eband_local_planner/eband_visualization.h
include/eband_local_planner/eband_trajectory_controller.h
)


# build the library
add_library(${PROJECT_NAME} ${eband_planner_files})

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

install(
  DIRECTORY include/eband_local_planner/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(
  TARGETS eband_local_planner
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
install(
  FILES elp_plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
