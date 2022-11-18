cmake_minimum_required(VERSION 3.5)

#
# Choosing to build the unittest (Catkin vs Modern CMake vs Ament).
#
if(DEFINED CATKIN_ENABLE_TESTING)
  set(BUILD_TESTING ${CATKIN_ENABLE_TESTING})
endif()

#
# Project definition
#
project(linear_feedback_controller_msgs LANGUAGES CXX)

#
# Dependencies
#
find_package(Eigen3 REQUIRED)
find_package(catkin REQUIRED COMPONENTS std_msgs sensor_msgs geometry_msgs
                                        eigen_conversions message_generation)

#
# Generate the messages
#
add_message_files(DIRECTORY msg FILES Contact.msg Control.msg Sensor.msg)
generate_messages(DEPENDENCIES std_msgs sensor_msgs geometry_msgs)

#
# Export as catkin package (For ROS1 compatibility...).
#
# cmake-format: off
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS std_msgs sensor_msgs geometry_msgs message_runtime eigen_conversions
  DEPENDS EIGEN3)
# cmake-format: on

#
# Main lib.
#
set(${PROJECT_NAME}_HEADERS include/${PROJECT_NAME}/eigen_conversions.hpp)
set(${PROJECT_NAME}_SOURCES "")
add_library(${PROJECT_NAME} INTERFACE)
target_link_libraries(
  ${PROJECT_NAME}
  INTERFACE Eigen3::Eigen ${std_msgs_LIBRARIES} ${sensor_msgs_LIBRARIES}
            ${eigen_conversions_LIBRARIES} ${geometry_msgs_LIBRARIES}
            ${message_generation_LIBRARIES})
target_include_directories(
  ${PROJECT_NAME} SYSTEM
  INTERFACE ${std_msgs_INCLUDE_DIRS} ${sensor_msgs_INCLUDE_DIRS}
            ${eigen_conversions_INCLUDE_DIRS} ${geometry_msgs_INCLUDE_DIRS}
            ${message_generation_INCLUDE_DIRS})
target_include_directories(
  ${PROJECT_NAME}
  INTERFACE $<INSTALL_INTERFACE:include/>
            $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)

#
# Unit tests
#
if(BUILD_TESTING)
  catkin_add_gtest(test_eigen_conversions tests/test_eigen_conversions.cpp
                   tests/gtest_main.cpp)
  target_link_libraries(test_eigen_conversions ${PROJECT_NAME})
endif()

#
# Install
#
install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${TARGETS_EXPORT_NAME}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(FILES package.xml DESTINATION share/${PROJECT_NAME})
install(
  DIRECTORY include
  DESTINATION include
  PATTERN "third_party" EXCLUDE)
