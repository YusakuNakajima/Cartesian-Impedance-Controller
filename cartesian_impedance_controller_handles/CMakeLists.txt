cmake_minimum_required(VERSION 2.8.3)
project(cartesian_impedance_controller_handles)

## Compile as C++17, supported in ROS Noetic
add_compile_options(-std=c++17)

set(ADDITIONAL_COMPILE_OPTIONS -Wall -Wextra -Wpedantic -Wno-unused-parameter)
add_compile_options(${ADDITIONAL_COMPILE_OPTIONS})

# Sets build type to "Release" in case no build type has not been set before
if(NOT CMAKE_BUILD_TYPE) 
  set(CMAKE_BUILD_TYPE Release)
endif(NOT CMAKE_BUILD_TYPE)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  controller_interface
  kdl_parser
  interactive_markers
  geometry_msgs
  dynamic_reconfigure
  tf_conversions
)

## Find Eigen3
find_package(Eigen3 REQUIRED)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES cartesian_impedance_controller_handles
  CATKIN_DEPENDS
    roscpp
    geometry_msgs
    dynamic_reconfigure
    interactive_markers
  DEPENDS EIGEN3
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(${PROJECT_NAME}
  src/cartesian_impedance_controller_handles.cpp
  include/cartesian_impedance_controller_handles/MotionControlHandle.h
  include/cartesian_impedance_controller_handles/MotionControlHandle.hpp
)

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############

## Mark executable scripts (Python etc.) for installation
install(PROGRAMS
  nodes/moving_target.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING REGEX ".*\\.h(pp)?"
  PATTERN ".svn" EXCLUDE
)

## Mark other files for installation (e.g. launch and bag files, etc.)
install(FILES
  cartesian_impedance_controller_handles_plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)