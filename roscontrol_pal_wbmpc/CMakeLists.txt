cmake_minimum_required(VERSION 3.1)

set(PROJECT_NAMESPACE loco-3d)
set(PROJECT_NAME roscontrol_pal_wbmpc)
set(PROJECT_DESCRIPTION "WBMPC Controller using Crocoddyl")
set(PROJECT_URL https://github.com/${PROJECT_NAMESPACE}/ros-wbmpc)
set(CXX_DISABLE_WERROR False)
set(CMAKE_VERBOSE_MAKEFILE True)

# Check if the submodule cmake have been initialized
if(NOT EXISTS "${CMAKE_SOURCE_DIR}/cmake/base.cmake")
  message(
    FATAL_ERROR
      "\nPlease run the following command first:\ngit submodule update --init\n"
  )
endif()

# Include important cmake modules
include(cmake/base.cmake)

compute_project_args(PROJECT_ARGS LANGUAGES CXX)
project(${PROJECT_NAME} ${PROJECT_ARGS})

add_project_dependency(sobec REQUIRED)

set(CATKIN_COMPONENTS
    # pluginlib
    # controller_interface
    # rbdl
    # realtime_tools
    # sensor_msgs
    # std_msgs
    # rospy
    # message_generation
    # pal_statistics
    # ddynamic_reconfigure
    # nav_msgs
    # eigen_conversions
    # Ros general C++ API
    roscpp
    # Ros messages generation
    message_generation
    # Message dependencies
    std_msgs
    geometry_msgs
    # Converting Ros messages object into Eigen3 objects
    eigen_conversions
    # PAL Ros controller using their base state estimator.
    pal_base_ros_controller)
find_package(catkin REQUIRED COMPONENTS ${CATKIN_COMPONENTS})
# Flexibility estimation
find_package(flex-joints REQUIRED)

# Here we want catkin users to use our package natively.
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME} controller_wbmpc
  CATKIN_DEPENDS ${CATKIN_COMPONENTS}
  DEPENDS flex-joints)

option(BUILD_WITH_MULTITHREADS
       "Build the library with the Multithreading support (required OpenMP)" ON)
if(BUILD_WITH_MULTITHREADS)
  set(BUILD_WITH_NTHREADS
      "4"
      CACHE STRING "Number of threads")
  string(REGEX MATCH "^[0-9]+$" BUILD_WITH_NTHREADS ${BUILD_WITH_NTHREADS})
  if(NOT BUILD_WITH_NTHREADS MATCHES "^[0-9]+$")
    set(BUILD_WITH_NTHREADS 4)
    message(
      WARNING
        "the number of threads have to be an interger value, set to ${BUILD_WITH_NTHREADS}"
    )
  endif()
endif()

#
# Create main libs
#
set(${PROJECT_NAME}_SOURCES src/controller_wbmpc.cpp)
set(${PROJECT_NAME}_HEADERS
    include/${PROJECT_NAME}/memmo_trajectory_controller.h)
add_library(${PROJECT_NAME} ${PROJECT_NAME}_SOURCES ${PROJECT_NAME}_HEADERS)
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} pinocchio::pinocchio
                      flex-joints::flex-joints)

#
# Install
#

install(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(FILES controller_plugins.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)

install(DIRECTORY config/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config)

catkin_install_python(
  PROGRAMS scripts/server_asynchronous.py scripts/server_full_asynchronous.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
