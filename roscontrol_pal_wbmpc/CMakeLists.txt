cmake_minimum_required(VERSION 3.1)

SET(PROJECT_NAMESPACE loco-3d)
SET(PROJECT_NAME roscontrol_pal_wbmpc)
SET(PROJECT_DESCRIPTION "WBMPC Controller using Crocoddyl")
SET(PROJECT_URL https://github.com/${PROJECT_NAMESPACE}/ros-wbmpc)
SET(CXX_DISABLE_WERROR False)
SET(CMAKE_VERBOSE_MAKEFILE True)

# Check if the submodule cmake have been initialized
IF(NOT EXISTS "${CMAKE_SOURCE_DIR}/cmake/base.cmake")
  MESSAGE(FATAL_ERROR "\nPlease run the following command first:\ngit submodule update --init\n")
ENDIF()

# Include important cmake modules
INCLUDE(cmake/base.cmake)

COMPUTE_PROJECT_ARGS(PROJECT_ARGS LANGUAGES CXX)
PROJECT(${PROJECT_NAME} ${PROJECT_ARGS})

ADD_PROJECT_DEPENDENCY(sobec REQUIRED)

set(CATKIN_COMPONENTS
#   pluginlib
#   controller_interface
#   rbdl
#   realtime_tools
#   sensor_msgs
#   std_msgs
#   rospy
#   message_generation
#   pal_statistics
#   ddynamic_reconfigure
#   nav_msgs
#   eigen_conversions
  
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
  pal_base_ros_controller
)
find_package(catkin REQUIRED COMPONENTS ${CATKIN_COMPONENTS})

# Here we want catkin users to use our package natively.
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME} controller_wbmpc
  CATKIN_DEPENDS ${CATKIN_COMPONENTS}
  DEPENDS ""
)
# include_directories(SYSTEM
#   ${catkin_INCLUDE_DIRS}
#   ${OCTOMAP_INCLUDE_DIRS}
# )

# link_directories(
#   ${OCTOMAP_LIBRARY_DIRS}
# )
# get_property(dirs DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY INCLUDE_DIRECTORIES)


# ADD_SUBDIRECTORY(bindings)

SET(${PROJECT_NAME}_SOURCES
    src/controller_wbmpc.cpp
    src/flexibility_compensation.cpp
    src/foot_trajectory.cpp
)
SET(${PROJECT_NAME}_HEADERS
    include/${PROJECT_NAME}/memmo_trajectory_controller.h
    #include/${PROJECT_NAME}/hand_tracking_problem.h
    #include/${PROJECT_NAME}/hand_tracking_problem_full.h
    #include/${PROJECT_NAME}/hand_tracking_problem_collision_full.h
    # include/${PROJECT_NAME}/model_utils.h
    #include/locosensing/Markup.h
    #include/locosensing/MsgUtils.h
    #include/locosensing/Network.h
    #include/locosensing/RTPacket.h
    #include/locosensing/RTProtocol.h
    #include/locosensing/Utils.h
    include/${PROJECT_NAME}/flexibility_compensation.hpp
    include/${PROJECT_NAME}/foot_trajectory.h
)

SET(CMAKE_MODULE_PATH
  ${CMAKE_MODULE_PATH}
  ${CMAKE_SOURCE_DIR}/cmake/)

OPTION(BUILD_WITH_MULTITHREADS "Build the library with the Multithreading support (required OpenMP)" ON)
IF(BUILD_WITH_MULTITHREADS)
  SET(BUILD_WITH_NTHREADS "4" CACHE STRING "Number of threads")
  string(REGEX MATCH "^[0-9]+$" BUILD_WITH_NTHREADS  ${BUILD_WITH_NTHREADS})
  IF(NOT BUILD_WITH_NTHREADS MATCHES "^[0-9]+$")
    SET(BUILD_WITH_NTHREADS 4)
    MESSAGE(WARNING "the number of threads have to be an interger value, set to ${BUILD_WITH_NTHREADS}")
  ENDIF()
ENDIF()

add_library(memmo_subscriber_controller src/memmo_subscriber_controller.cpp)
add_dependencies(memmo_subscriber_controller ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(memmo_subscriber_controller ${catkin_LIBRARIES} pinocchio::pinocchio)

add_library(memmo_subscriber_controller_pointing src/memmo_subscriber_controller_pointing.cpp)
add_dependencies(memmo_subscriber_controller_pointing ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(memmo_subscriber_controller_pointing ${catkin_LIBRARIES} pinocchio::pinocchio)

add_library(memmo_subscriber_controller_squat src/memmo_subscriber_controller_squat.cpp)
add_dependencies(memmo_subscriber_controller_squat ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(memmo_subscriber_controller_squat ${catkin_LIBRARIES} pinocchio::pinocchio)

add_library(memmo_subscriber_controller_sanding src/memmo_subscriber_controller_sanding.cpp)
add_dependencies(memmo_subscriber_controller_sanding ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(memmo_subscriber_controller_sanding ${catkin_LIBRARIES} pinocchio::pinocchio)

add_library(memmo_subscriber_controller_sanding_lpf src/memmo_subscriber_controller_sanding_lpf.cpp)
add_dependencies(memmo_subscriber_controller_sanding_lpf ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(memmo_subscriber_controller_sanding_lpf ${catkin_LIBRARIES} pinocchio::pinocchio)


add_library(${PROJECT_NAME} ${${PROJECT_NAME}_SOURCES} ${${PROJECT_NAME}_HEADERS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} Boost::filesystem Boost::system Boost::serialization
   memmo_subscriber_controller memmo_subscriber_controller_squat memmo_subscriber_controller_pointing 
  memmo_subscriber_controller_sanding memmo_subscriber_controller_sanding_lpf)

add_library(memmo_trajectory_controller_plugins src/memmo_trajectory_controller_plugins.cpp)
target_link_libraries(memmo_trajectory_controller_plugins memmo_subscriber_controller memmo_subscriber_controller_squat memmo_subscriber_controller_pointing  
                      memmo_subscriber_controller_sanding memmo_subscriber_controller_sanding_lpf ${catkin_LIBRARIES})

#add_executable(trajectory_publisher_full src/trajectory_publisher_full.cpp)
#target_link_libraries(trajectory_publisher_full ${PROJECT_NAME} ${OCTOMAP_LIBRARIES})

#add_executable(trajectory_publisher_full_table src/trajectory_publisher_full_table.cpp)
#target_link_libraries(trajectory_publisher_full_table ${PROJECT_NAME} ${OCTOMAP_LIBRARIES})

#add_executable(trajectory_publisher_pointing src/trajectory_publisher_pointing.cpp)
#target_link_libraries(trajectory_publisher_pointing ${PROJECT_NAME} ${OCTOMAP_LIBRARIES})

#add_executable(trajectory_publisher_collision_full src/trajectory_publisher_collision_full.cpp)
#target_link_libraries(trajectory_publisher_collision_full ${PROJECT_NAME} ${OCTOMAP_LIBRARIES})

# add_executable(trajectory_publisher_walk src/trajectory_publisher_walk.cpp)
# target_link_libraries(trajectory_publisher_walk ${PROJECT_NAME} ${OCTOMAP_LIBRARIES} ndcurves::ndcurves sobec::sobec_py2cpp) 

add_executable(trajectory_publisher_sanding src/trajectory_publisher_sanding.cpp)
target_link_libraries(trajectory_publisher_sanding ${PROJECT_NAME} ${OCTOMAP_LIBRARIES} crocoddyl::crocoddyl sobec::sobec)

add_executable(trajectory_publisher_sanding_lpf src/trajectory_publisher_sanding_lpf.cpp)
target_link_libraries(trajectory_publisher_sanding_lpf ${PROJECT_NAME} ${OCTOMAP_LIBRARIES} crocoddyl::crocoddyl sobec::sobec)

#add_executable(trajectory_publisher_stairs src/trajectory_publisher_stairs.cpp)
#target_link_libraries(trajectory_publisher_stairs ${PROJECT_NAME} ${OCTOMAP_LIBRARIES} ndcurves::ndcurves) 

#add_executable(trajectory_publisher_walk_free src/trajectory_publisher_walk_free.cpp)
#target_link_libraries(trajectory_publisher_walk_free ${PROJECT_NAME} ${OCTOMAP_LIBRARIES})

#add_executable(Qualisys2Ros_LS src/qualisys/Qualisys2Ros_LS.cpp src/qualisys/Network.cpp 
#  src/qualisys/RTPacket.cpp src/qualisys/RTProtocol.cpp src/qualisys/Markup.cpp)
#target_link_libraries(Qualisys2Ros_LS ${PROJECT_NAME} )

ADD_UNIT_TEST(test_flex_estimator tests/test_flex_estimator.cpp)
target_link_libraries(test_flex_estimator PUBLIC ${PROJECT_NAME})
target_include_directories(test_flex_estimator PRIVATE Boost::unit_test_framework)

#############
## Install ##
#############

install(TARGETS ${PROJECT_NAME} memmo_subscriber_controller memmo_subscriber_controller_pointing
                                memmo_subscriber_controller_squat 
                                # trajectory_publisher_walk
                                memmo_subscriber_controller_sanding memmo_subscriber_controller_sanding_lpf
                                trajectory_publisher_sanding trajectory_publisher_sanding_lpf
                                memmo_trajectory_controller_plugins
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES controller_plugins.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY launch/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)

install(DIRECTORY config/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config)

catkin_install_python(PROGRAMS scripts/server_asynchronous.py scripts/server_full_asynchronous.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
