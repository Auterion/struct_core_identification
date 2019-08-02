cmake_minimum_required(VERSION 2.8.3)
project(struct_core_identification)

add_definitions(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  dynamic_reconfigure
  tf
  mavlink
  rosbag
  cv_bridge
  struct_core_ros
  image_transport
  OpenCV
)


find_package(OpenCV REQUIRED)

get_filename_component(PARENT_DIR ${CMAKE_CURRENT_SOURCE_DIR} DIRECTORY)
set(SCSDK_ROOT ${PARENT_DIR}/structure_core_sdk_ros/)


if( ${CMAKE_HOST_SYSTEM_PROCESSOR} STREQUAL "x86_64" )
	set(SCSDK_TARGET_ARCH x86_64)
else()
	set(SCSDK_TARGET_ARCH arm64)
endif()

if(NOT SCSDK_TARGET_ARCH)
    message(WARNING "SCSDK_TARGET_ARCH is not set (build scripts will do this automatically). Assuming x86_64.")
    set(SCSDK_TARGET_ARCH x86_64)
endif()

set(SCSDK_STRUCTURE_HEADERS ${SCSDK_ROOT}/Libraries/Structure/Headers)
set(SCSDK_STRUCTURE_LIBDIR ${SCSDK_ROOT}Libraries/Structure/Linux/${SCSDK_TARGET_ARCH})
set(SCSDK_STRUCTURE_LIB ${SCSDK_STRUCTURE_LIBDIR}/libStructure.so)


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
  #INCLUDE_DIRS  ../structure_core_sdk_ros/Libraries/Structure/Headers
  CATKIN_DEPENDS roscpp rospy std_msgs geometry_msgs sensor_msgs message_runtime tf rosbag
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${SCSDK_STRUCTURE_HEADERS}
  ${OpenCV_INCLUDE_DIRS}
)

## Declare a C++ library
add_executable(struct_core_identification
  main.cpp
)

target_link_libraries(struct_core_identification ${SCSDK_STRUCTURE_LIB} ${OpenCV_LIBRARIES} ${ZBAR_LIBRARIES} zbar pthread)

