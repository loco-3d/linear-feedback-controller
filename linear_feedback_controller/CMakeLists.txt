cmake_minimum_required(VERSION 3.10)

#
# Project properties
#
set(PROJECT_NAMESPACE loco-3d)
set(PROJECT_NAME linear_feedback_controller)
set(PROJECT_DESCRIPTION "Network(ROS) interface to Whole Body MPC controllers.")
set(PROJECT_URL https://github.com/${PROJECT_NAMESPACE}/ros-wbmpc)

#
# Project options
#
option(SUFFIX_SO_VERSION "Suffix library name with its version" ON)

#
# Project configuration
#
set(PROJECT_USE_CMAKE_EXPORT TRUE)

#
# Check if the submodule cmake have been initialized
#
set(JRL_CMAKE_MODULES "${CMAKE_CURRENT_LIST_DIR}/../cmake")
if(NOT EXISTS "${JRL_CMAKE_MODULES}/base.cmake")
  if(${CMAKE_VERSION} VERSION_LESS "3.14.0")
    message(
      FATAL_ERROR
        "\nPlease run the following command first:\ngit submodule update --init\n"
    )
  else()
    include(FetchContent)
    FetchContent_Declare(
      "jrl-cmakemodules"
      GIT_REPOSITORY "https://github.com/jrl-umi3218/jrl-cmakemodules.git")
    FetchContent_MakeAvailable("jrl-cmakemodules")
    FetchContent_GetProperties("jrl-cmakemodules" SOURCE_DIR JRL_CMAKE_MODULES)
  endif()
endif()

#
# Choosing to build the unittest (Catkin vs Modern CMake vs Ament).
#
if(DEFINED CATKIN_ENABLE_TESTING)
  set(BUILD_TESTING ${CATKIN_ENABLE_TESTING})
endif()

#
# Project definition
#
include("${JRL_CMAKE_MODULES}/base.cmake")
compute_project_args(PROJECT_ARGS LANGUAGES CXX)
project(${PROJECT_NAME} ${PROJECT_ARGS})

#
# Setup unit-tests
#
if(BUILD_TESTING)
  # To be done before finding catkin because catkin looks for gtest.
  include("${JRL_CMAKE_MODULES}/test.cmake")
  include("${JRL_CMAKE_MODULES}/gtest.cmake")
  add_gtest_suite("main")
  find_package(rostest REQUIRED)
  find_package(example-robot-data REQUIRED)
endif()

#
# Dependencies
#
add_project_dependency(pinocchio REQUIRED)
find_package(catkin REQUIRED COMPONENTS roscpp realtime_tools linear_feedback_controller_msgs
                                        pal_base_ros_controller)

#
# Export as catkin package (For ROS1 compatibility...).
#
# cmake-format: off
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp realtime_tools linear_feedback_controller_msgs
  DEPENDS pinocchio)
# cmake-format: on

#
# Main Library
#
set(${PROJECT_NAME}_HEADERS
    include/${PROJECT_NAME}/linear_feedback_controller.hpp)
set(${PROJECT_NAME}_SOURCES src/linear_feedback_controller.cpp #
                            # src/model-utils.cpp
)
add_library(${PROJECT_NAME} SHARED ${${PROJECT_NAME}_SOURCES}
                                   ${${PROJECT_NAME}_HEADERS})
target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC ${catkin_INCLUDE_DIRS})
target_include_directories(
  ${PROJECT_NAME} PUBLIC $<INSTALL_INTERFACE:include>
                         $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
target_link_libraries(${PROJECT_NAME} PUBLIC pinocchio::pinocchio
                                             ${catkin_LIBRARIES})
set_target_properties(${PROJECT_NAME} PROPERTIES LINKER_LANGUAGE CXX
                                                 INSTALL_RPATH "\$ORIGIN")
if(SUFFIX_SO_VERSION)
  set_target_properties(${PROJECT_NAME} PROPERTIES SOVERSION ${PROJECT_VERSION})
endif()

#
# Unit tests
#
if(BUILD_TESTING)
  add_executable(
    test_linear_feedback_controller_basic
    tests/test_linear_feedback_controller_basic.cpp tests/ros_gtest_main.cpp)
  target_link_libraries(test_linear_feedback_controller_basic gtest
                        ${PROJECT_NAME} example-robot-data::example-robot-data)

  add_rostest(tests/test_linear_feedback_controller_basic.test)

endif()

#
# Installation
#
install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${TARGETS_EXPORT_NAME}
  DESTINATION lib)
install(FILES package.xml DESTINATION share/${PROJECT_NAME})
