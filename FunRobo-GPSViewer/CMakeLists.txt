cmake_minimum_required(VERSION 2.8.3)
project(funrobo_gpsview)

add_definitions(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
	roscpp
	rospy
        roslib
	sensor_msgs
	std_msgs
	)

find_package(Qt5Widgets REQUIRED)
find_package(Qt5Core REQUIRED)

set (gpsview_lib_src src/mainwindow.cpp src/Worker.cpp)
set (gpsview_lib_ui assets/mainwindow.ui)
set (gpsview_bin_src src/main.cpp)
set (gpsview_lib_hdr include/funrobo_gpsview/mainwindow.h)
set (worker_hdr include/funrobo_gpsview/Worker.h)

set(CMAKE_INCLUDE_CURRENT_DIR ON)

qt5_wrap_ui(gpsview_lib_ui_moc ${gpsview_lib_ui})
qt5_wrap_cpp(gpsview_lib_hdr_moc ${gpsview_lib_hdr})
qt5_wrap_cpp(worker_moc ${worker_hdr})

MESSAGE(STATUS "MOCFILE : " ${worker_moc})

## hack to get headers to appear on file editor (really stupid...)
FILE(GLOB_RECURSE LibFiles "include/*.h")
add_custom_target(headers SOURCES ${LibFiles})

#file(COPY assets DESTINATION ${CMAKE_CURRENT_BINARY_DIR})

include_directories(
	include
	${PROJECT_SOURCE_DIR}
	${PROJECT_BINARY_DIR}
        ${CMAKE_BINARY_DIR}
	${catkin_INCLUDE_DIRS}
	${Qt5Widgets_INCLUDE_DIRS}
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
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
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
#   sensor_msgs#   std_msgs
# )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
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
	#  LIBRARIES funrobo_gpsview
	CATKIN_DEPENDS roscpp rospy sensor_msgs std_msgs
	DEPENDS system_lib
	)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations


## Declare a C++ library

add_library(gpsview_lib SHARED
        ${gpsview_lib_src}
        ${gpsview_lib_ui_moc}
	${gpsview_lib_hdr_moc}
        ${worker_moc}
)

target_link_libraries(gpsview_lib
    ${catkin_LIBRARIES}
    Qt5::Widgets
    )

add_executable(gpsview
	${gpsview_bin_src}
        )

target_link_libraries(gpsview
	${catkin_LIBRARIES}
	gpsview_lib
	)

#############
## Install ##
#############

install(TARGETS gpsview
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY assets
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  FILES_MATCHING PATTERN "*.png")

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS funrobo_gpsview funrobo_gpsview_node
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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_funrobo_gpsview.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
