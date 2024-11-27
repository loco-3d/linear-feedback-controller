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
# Generate the ROS2 parameters interface automatically.
#
include(cmake/sec_generate_parameter_library.cmake)
# Main node params
sec_generate_parameter_library_markdown(${PROJECT_NAME}_parameters_doc
                                        src/linear_feedback_controller.yaml)
sec_generate_parameter_library(
  generated_parameters # Lib name
  ${PROJECT_NAME}_parameters # CMake target name for the parameter library.
  src/linear_feedback_controller.yaml # Path to input yaml file
)
add_dependencies(${PROJECT_NAME}_parameters ${PROJECT_NAME}_parameters_doc)
# Joint estimator params
sec_generate_parameter_library_markdown(joint_state_estimator_parameters_doc
                                        src/linear_feedback_controller.yaml)
sec_generate_parameter_library(
  joint_state_estimator_generated_parameters # Lib name
  joint_state_estimator_parameters # CMake target name for the parameter
                                   # library.
  src/joint_state_estimator.yaml # Path to input yaml file
)
add_dependencies(joint_state_estimator_parameters
                 joint_state_estimator_parameters_doc)

#
# Main Library
#
set(${PROJECT_NAME}_headers
    include/${PROJECT_NAME}/averaging_filter.hpp
    include/${PROJECT_NAME}/averaging_filter.hxx
    include/${PROJECT_NAME}/joint_state_estimator.hpp
    include/${PROJECT_NAME}/lf_controller.hpp
    include/${PROJECT_NAME}/linear_feedback_controller.hpp
    include/${PROJECT_NAME}/linear_feedback_controller_ros.hpp
    include/${PROJECT_NAME}/min_jerk.hpp
    include/${PROJECT_NAME}/pd_controller.hpp
    include/${PROJECT_NAME}/robot_model_builder.hpp)
set(${PROJECT_NAME}_sources
    src/joint_state_estimator.cpp #
    src/lf_controller.cpp #
    src/linear_feedback_controller.cpp #
    src/linear_feedback_controller_ros.cpp #
    src/min_jerk.cpp #
    src/pd_controller.cpp #
    src/robot_model_builder.cpp)
ament_auto_add_library(${PROJECT_NAME} ${${PROJECT_NAME}_sources}
                       ${${PROJECT_NAME}_headers})
ament_target_dependencies(${PROJECT_NAME} Eigen3)
target_link_libraries(${PROJECT_NAME} pinocchio::pinocchio)
target_link_libraries(${PROJECT_NAME} ${PROJECT_NAME}_parameters)

#
# Unit tests
#
include(CTest)
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_auto_find_test_dependencies()
  # Integration test of the roscontrol controller with simulation on Talos.
  # add_rostest(tests/test_linear_feedback_controller.test)

  # Loading test, especially the parameters add_rostest_gtest(
  # test_linear_feedback_controller_basic
  # tests/test_linear_feedback_controller_basic.test
  # tests/test_linear_feedback_controller_basic.cpp tests/ros_gtest_main.cpp)
  # target_link_libraries(test_linear_feedback_controller_basic ${PROJECT_NAME}
  # example-robot-data::example-robot-data) target_compile_definitions(
  # test_linear_feedback_controller_basic PRIVATE
  # TEST_ROS_NODE_NAME="test_linear_feedback_controller_basic")

  # Test the Averaging filter class.
  ament_auto_add_gtest(test_averaging_filter tests/test_averaging_filter.cpp)
  target_link_libraries(test_averaging_filter ${PROJECT_NAME})

  ament_auto_add_gtest(test_min_jerk tests/test_min_jerk.cpp)
  target_link_libraries(test_min_jerk ${PROJECT_NAME})

  ament_auto_add_gtest(test_robot_model_builder
                       tests/test_robot_model_builder.cpp)
  target_link_libraries(test_robot_model_builder ${PROJECT_NAME}
                        example-robot-data::example-robot-data)
endif()

#
# Export plugins
#
pluginlib_export_plugin_description_file(controller_interface
                                         controller_plugins.xml)

#
# Installation
#
install(DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/doc
        DESTINATION share/${PROJECT_NAME})
install(
  PROGRAMS ${PROJECT_NAME}/pd_plus_demo.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME pd_plus_demo.py)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
ament_export_libraries(${PROJECT_NAME})
ament_auto_package()
