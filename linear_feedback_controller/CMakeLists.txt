cmake_minimum_required(VERSION 3.10)

#
# Choosing to build the unittest (Catkin vs Modern CMake vs Ament).
#
if(DEFINED CATKIN_ENABLE_TESTING)
  set(BUILD_TESTING ${CATKIN_ENABLE_TESTING})
endif()

#
# Project definition
#
project(linear_feedback_controller LANGUAGES CXX)

#
# Setup unit-tests
#
if(BUILD_TESTING)
  find_package(rostest REQUIRED)
  find_package(example-robot-data REQUIRED)
endif()

#
# Dependencies
#
# find_package(pinocchio REQUIRED)
find_package(catkin REQUIRED)
# find_package(roscpp REQUIRED)
# find_package(realtime_tools REQUIRED)
find_package(pal_base_ros_controller REQUIRED)
# find_package(linear_feedback_controller_msgs CONFIG REQUIRED)

#
# Export as catkin package (For ROS1 compatibility...).
#
# cmake-format: off
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp
                #  realtime_tools
                 linear_feedback_controller_msgs
                 pluginlib
                 pal_base_ros_controller 
  DEPENDS pinocchio)
# cmake-format: on

#
# Main Library
#
set(${PROJECT_NAME}_HEADERS
    include/${PROJECT_NAME}/linear_feedback_controller.hpp)
set(${PROJECT_NAME}_SOURCES
    src/linear_feedback_controller.cpp #
    src/averaging_filter.cpp #
    # src/model-utils.cpp
)
add_library(${PROJECT_NAME} SHARED ${${PROJECT_NAME}_SOURCES}
                                   ${${PROJECT_NAME}_HEADERS})
target_include_directories(
  ${PROJECT_NAME} SYSTEM
  PUBLIC ${roscpp_INCLUDE_DIRS} ${realtime_tools_INCLUDE_DIRS}
         ${pal_base_ros_controller_INCLUDE_DIRS}
         ${linear_feedback_controller_msgs_INCLUDE_DIRS})
target_include_directories(
  ${PROJECT_NAME} PUBLIC $<INSTALL_INTERFACE:include>
                         $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
target_link_libraries(
  ${PROJECT_NAME}
  PUBLIC # pinocchio::pinocchio
         ${roscpp_LIBRARIES} ${realtime_tools_LIBRARIES}
         ${pal_base_ros_controller_LIBRARIES}
         ${linear_feedback_controller_msgs_LIBRARIES})
set_target_properties(${PROJECT_NAME} PROPERTIES LINKER_LANGUAGE CXX
                                                 INSTALL_RPATH "\$ORIGIN")

#
# Plugins
#
add_library(${PROJECT_NAME}_plugins SHARED src/plugins.cpp)
target_include_directories(${PROJECT_NAME}_plugins SYSTEM
                           PUBLIC ${pluginlib_INCLUDE_DIRS})
target_include_directories(
  ${PROJECT_NAME} PUBLIC $<INSTALL_INTERFACE:include>
                         $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
target_link_libraries(${PROJECT_NAME}_plugins PUBLIC ${PROJECT_NAME}
                                                     ${pluginlib_LIBRARIES})
set_target_properties(${PROJECT_NAME}_plugins
                      PROPERTIES LINKER_LANGUAGE CXX INSTALL_RPATH "\$ORIGIN")

#
# Unit tests
#
if(BUILD_TESTING)
  add_rostest_gtest(
    test_linear_feedback_controller_basic
    tests/test_linear_feedback_controller_basic.test
    tests/test_linear_feedback_controller_basic.cpp
    tests/ros_gtest_main.cpp
    )
  target_link_libraries(test_linear_feedback_controller_basic
                        ${PROJECT_NAME} example-robot-data::example-robot-data)
  target_compile_definitions(
    test_linear_feedback_controller_basic
    PRIVATE TEST_ROS_NODE_NAME="test_linear_feedback_controller_basic")

  # Add the unit-test here in order for ctest to find it.
  catkin_add_gtest(test_averaging_filter tests/test_averaging_filter.cpp)
  target_link_libraries(test_averaging_filter gtest gtest_main ${PROJECT_NAME})
endif()

#
# Installation
#
install(FILES package.xml controller_plugins.xml
        DESTINATION share/${PACKAGE_NAME})
install(DIRECTORY config/ DESTINATION share/${PACKAGE_NAME}/config)
install(
  TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_plugins
  EXPORT ${TARGETS_EXPORT_NAME}
  DESTINATION lib)
install(FILES package.xml DESTINATION share/${PROJECT_NAME})
