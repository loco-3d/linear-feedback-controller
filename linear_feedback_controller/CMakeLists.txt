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
find_package(Eigen3 REQUIRED)
find_package(pinocchio REQUIRED)
set(MY_CATKIN_COMPONENTS
    roscpp realtime_tools pal_base_ros_controller
    linear_feedback_controller_msgs pluginlib controller_interface)
find_package(catkin REQUIRED COMPONENTS ${MY_CATKIN_COMPONENTS})

#
# Export as catkin package (For ROS1 compatibility...).
#
# cmake-format: off
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS ${MY_CATKIN_COMPONENTS}
  DEPENDS EIGEN3 pinocchio)
# cmake-format: on

#
# Main Library
#
set(${PROJECT_NAME}_HEADERS
    include/${PROJECT_NAME}/linear_feedback_controller.hpp
    include/${PROJECT_NAME}/averaging_filter.hpp
    include/${PROJECT_NAME}/averaging_filter.hxx
    include/${PROJECT_NAME}/min_jerk.hpp)
set(${PROJECT_NAME}_SOURCES
    src/linear_feedback_controller.cpp #
    src/min_jerk.cpp #
)
add_library(${PROJECT_NAME} SHARED ${${PROJECT_NAME}_SOURCES}
                                   ${${PROJECT_NAME}_HEADERS})
target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC ${catkin_INCLUDE_DIRS})
target_include_directories(
  ${PROJECT_NAME} PUBLIC $<INSTALL_INTERFACE:include>
                         $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
target_link_libraries(${PROJECT_NAME} PUBLIC pinocchio::pinocchio Eigen3::Eigen
                                             ${catkin_LIBRARIES})

#
# Plugins
#
add_library(${PROJECT_NAME}_plugins SHARED src/plugins.cpp)
target_link_libraries(${PROJECT_NAME}_plugins PUBLIC ${PROJECT_NAME})

#
# Unit tests
#
if(BUILD_TESTING)
  # Integration test of the roscontrol controller with simulation on Talos.
  add_rostest(tests/test_linear_feedback_controller.test)

  # Loading test, espaecially the parameters
  add_rostest_gtest(
    test_linear_feedback_controller_basic
    tests/test_linear_feedback_controller_basic.test
    tests/test_linear_feedback_controller_basic.cpp tests/ros_gtest_main.cpp)
  target_link_libraries(test_linear_feedback_controller_basic ${PROJECT_NAME}
                        example-robot-data::example-robot-data)
  target_compile_definitions(
    test_linear_feedback_controller_basic
    PRIVATE TEST_ROS_NODE_NAME="test_linear_feedback_controller_basic")

  # Test the Averaging filter class.
  catkin_add_gtest(test_averaging_filter tests/test_averaging_filter.cpp)
  target_link_libraries(test_averaging_filter gtest gtest_main ${PROJECT_NAME})
endif()

#
# Installation
#
install(PROGRAMS src/pd_controller.py
        DESTINATION lib/${PROJECT_NAME}
        RENAME pd_controller)
install(FILES package.xml controller_plugins.xml
        DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(
  TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_plugins
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})
