cmake_minimum_required(VERSION 3.22.1)

#
# Project definition
#
project(linear_feedback_controller LANGUAGES CXX)

#
# Options
#
option(BUILD_SHARED_LIBS "Build using shared libraries" ON)

#
# Handle dependencies by reading the package.xml
#
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

#
# System dependencies
#
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3)
find_package(pinocchio REQUIRED)
# find_package(example-robot-data REQUIRED)

#
# Main Library
#
set(${PROJECT_NAME}_HEADERS
    # include/${PROJECT_NAME}/linear_feedback_controller.hpp
    include/${PROJECT_NAME}/averaging_filter.hpp
    include/${PROJECT_NAME}/averaging_filter.hxx
    include/${PROJECT_NAME}/contact_detector.hpp
    include/${PROJECT_NAME}/min_jerk.hpp)
set(${PROJECT_NAME}_SOURCES
    # src/linear_feedback_controller.cpp #
    src/min_jerk.cpp #
    src/contact_detector.cpp
)
ament_auto_add_library(${PROJECT_NAME}
  ${${PROJECT_NAME}_SOURCES} ${${PROJECT_NAME}_HEADERS})
ament_target_dependencies(${PROJECT_NAME} Eigen3)
ament_target_dependencies(${PROJECT_NAME} pinocchio)
# ament_target_dependencies(${PROJECT_NAME} example-robot-data)

#
# Plugins
#
# ament_auto_add_library(${PROJECT_NAME}_plugins src/plugins.cpp)
# target_link_libraries(${PROJECT_NAME}_plugins ${PROJECT_NAME})

#
# Unit tests
#
include(CTest)
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_auto_find_test_dependencies()
  # Integration test of the roscontrol controller with simulation on Talos.
  # add_rostest(tests/test_linear_feedback_controller.test)

  # Loading test, especially the parameters
  # add_rostest_gtest(
  #   test_linear_feedback_controller_basic
  #   tests/test_linear_feedback_controller_basic.test
  #   tests/test_linear_feedback_controller_basic.cpp tests/ros_gtest_main.cpp)
  # target_link_libraries(test_linear_feedback_controller_basic ${PROJECT_NAME}
  #                       example-robot-data::example-robot-data)
  # target_compile_definitions(
  #   test_linear_feedback_controller_basic
  #   PRIVATE TEST_ROS_NODE_NAME="test_linear_feedback_controller_basic")

  # Test the Averaging filter class.
  ament_auto_add_gtest(test_averaging_filter tests/test_averaging_filter.cpp)
  target_link_libraries(test_averaging_filter gtest gtest_main ${PROJECT_NAME})

  ament_auto_add_gtest(test_min_jerk tests/test_min_jerk.cpp)
  target_link_libraries(test_min_jerk gtest gtest_main ${PROJECT_NAME})
endif()

#
# Export plugins
#
pluginlib_export_plugin_description_file(
  controller_interface controller_plugins.xml)

#
# Installation
#
install(PROGRAMS src/pd_controller.py
        DESTINATION lib/${PROJECT_NAME}
        RENAME pd_controller)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_auto_package()
