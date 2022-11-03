cmake_minimum_required(VERSION 3.5)

message(WARNING "RUNTIME_OUTPUT_DIRECTORY = ${RUNTIME_OUTPUT_DIRECTORY}")

set(PROJECT_ORG loco-3d)
set(PROJECT_NAME linear_feedback_controller_msgs)
set(PROJECT_DESCRIPTION "Whole Body MPC ROS messages.")
set(PROJECT_URL "https://github.com/${PROJECT_ORG}/ros-wbmpc")

message(WARNING "RUNTIME_OUTPUT_DIRECTORY = ${RUNTIME_OUTPUT_DIRECTORY}")

#
# Project options
#
option(SUFFIX_SO_VERSION "Suffix library name with its version" ON)

#
# Project configuration
#
set(PROJECT_USE_CMAKE_EXPORT TRUE)

message(WARNING "RUNTIME_OUTPUT_DIRECTORY = ${RUNTIME_OUTPUT_DIRECTORY}")

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

message(WARNING "RUNTIME_OUTPUT_DIRECTORY = ${RUNTIME_OUTPUT_DIRECTORY}")

#
# Choosing to build the unittest (Catkin vs Modern CMake vs Ament).
#
if(DEFINED CATKIN_ENABLE_TESTING)
  set(BUILD_TESTING ${CATKIN_ENABLE_TESTING})
endif()

message(WARNING "RUNTIME_OUTPUT_DIRECTORY = ${RUNTIME_OUTPUT_DIRECTORY}")

#
# Project definition
#
include("${JRL_CMAKE_MODULES}/base.cmake")
compute_project_args(PROJECT_ARGS LANGUAGES CXX)
project(${PROJECT_NAME} ${PROJECT_ARGS})


message(WARNING "RUNTIME_OUTPUT_DIRECTORY = ${RUNTIME_OUTPUT_DIRECTORY}")
#
# Dependencies
#
if(BUILD_TESTING)
  # To be done before finding catkin because catkin looks for gtest.
  include("${JRL_CMAKE_MODULES}/test.cmake")
  include("${JRL_CMAKE_MODULES}/gtest.cmake")
  add_gtest_suite("main")
  add_unit_test(test_eigen_conversions tests/test_eigen_conversions.cpp)
endif()
add_project_dependency(Eigen3 REQUIRED)
add_project_dependency(std_msgs REQUIRED)
add_project_dependency(sensor_msgs REQUIRED)
add_project_dependency(geometry_msgs REQUIRED)
add_project_dependency(message_generation REQUIRED)
add_project_dependency(eigen_conversions REQUIRED)

message(WARNING "RUNTIME_OUTPUT_DIRECTORY = ${RUNTIME_OUTPUT_DIRECTORY}")

# Generate the messages
add_message_files(DIRECTORY msg FILES Contact.msg Control.msg Sensor.msg)
generate_messages(DEPENDENCIES std_msgs sensor_msgs geometry_msgs)

message(WARNING "RUNTIME_OUTPUT_DIRECTORY = ${RUNTIME_OUTPUT_DIRECTORY}")

# Main lib.
add_library(${PROJECT_NAME} INTERFACE)
target_link_libraries( ${PROJECT_NAME} INTERFACE Eigen3::Eigen)
target_include_directories(
  ${PROJECT_NAME} SYSTEM
  INTERFACE ${std_msgs_INCLUDE_DIRS} ${sensor_msgs_INCLUDE_DIRS} ${eigen_conversions_INCLUDE_DIRS}
            ${geometry_msgs_INCLUDE_DIRS} ${message_generation_INCLUDE_DIRS})
target_include_directories(
  ${PROJECT_NAME}
  INTERFACE
    $<INSTALL_INTERFACE:include/>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

message(WARNING "RUNTIME_OUTPUT_DIRECTORY = ${RUNTIME_OUTPUT_DIRECTORY}")

if(BUILD_TESTING)
  target_link_libraries(test_eigen_conversions gtest gtest_main ${PROJECT_NAME})
endif()

install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${TARGETS_EXPORT_NAME}
  DESTINATION lib)

install(FILES package.xml DESTINATION share/${PROJECT_NAME})
