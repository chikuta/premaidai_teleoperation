cmake_minimum_required(VERSION 2.8.3)
project(premaidai_teleoperation)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  interactive_markers
  roscpp
  std_srvs
  visualization_msgs
  tf2
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  # LIBRARIES premaidai_teleoperation
  CATKIN_DEPENDS interactive_markers roscpp visualization_msgs tf2 std_srvs
  # DEPENDS system_lib
)

###########
## Build ##
###########

## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(interactive_marker
  src/interactive_marker.cc
)

add_executable(interactive_marker_node src/interactive_marker_node.cc)
target_link_libraries(interactive_marker_node
  interactive_marker
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############

install(TARGETS interactive_marker_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark libraries for installation
install(TARGETS interactive_marker
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.hh"
  PATTERN ".svn" EXCLUDE
)

## Mark other files for installation (e.g. launch and bag files, etc.)
install(DIRECTORY
  launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)