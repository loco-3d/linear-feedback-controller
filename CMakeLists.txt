cmake_minimum_required(VERSION 3.22.1)

set(PROJECT_NAME linear_feedback_controller)
set(PROJECT_DESCRIPTION "ROS2 control with Ricatti gains")
set(PROJECT_URL "http://github.com/loco-3d/linear-feedback-controller")

#
# Project definition
#
find_package(jrl-cmakemodules QUIET CONFIG)
if(jrl-cmakemodules_FOUND)
  message(STATUS "JRL cmakemodules found on system at ${JRL_CMAKE_MODULES}")
  get_property(
    JRL_CMAKE_MODULES
    TARGET jrl-cmakemodules::jrl-cmakemodules
    PROPERTY INTERFACE_INCLUDE_DIRECTORIES)
else()
  message(STATUS "JRL cmakemodules not found. Let's fetch it.")
  include(FetchContent)
  FetchContent_Declare(
    "jrl-cmakemodules"
    GIT_REPOSITORY "https://github.com/jrl-umi3218/jrl-cmakemodules.git")
  FetchContent_MakeAvailable("jrl-cmakemodules")
  FetchContent_GetProperties("jrl-cmakemodules" SOURCE_DIR JRL_CMAKE_MODULES)
endif()
include("${JRL_CMAKE_MODULES}/base.cmake")
compute_project_args(PROJECT_ARGS LANGUAGES CXX)
project(${PROJECT_NAME} ${PROJECT_ARGS})

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
                                        src/${PROJECT_NAME}.yaml)
sec_generate_parameter_library(
  generated_parameters # Lib name
  ${PROJECT_NAME}_parameters # CMake target name for the parameter library.
  src/${PROJECT_NAME}.yaml # Path to input yaml file
)
add_dependencies(${PROJECT_NAME}_parameters ${PROJECT_NAME}_parameters_doc)
# Joint estimator params
sec_generate_parameter_library_markdown(joint_state_estimator_parameters_doc
                                        src/joint_state_estimator.yaml)
sec_generate_parameter_library(
  joint_state_estimator_generated_parameters # Lib name
  joint_state_estimator_parameters # CMake target name for the parameter
                                   # library.
  src/joint_state_estimator.yaml # Path to input yaml file
)
add_dependencies(joint_state_estimator_parameters
                 joint_state_estimator_parameters_doc)
# Passthrough params
sec_generate_parameter_library_markdown(passthrough_controller_parameters_doc
                                        src/passthrough_controller.yaml)
sec_generate_parameter_library(
  passthrough_controller_generated_parameters # Lib name
  passthrough_controller_parameters # CMake target name for the parameter
  # library.
  src/passthrough_controller.yaml # Path to input yaml file
)
add_dependencies(passthrough_controller_parameters
                 passthrough_controller_parameters_doc)

#
# Main Library
#
set(${PROJECT_NAME}_headers
    include/${PROJECT_NAME}/visibility.hpp
    include/${PROJECT_NAME}/joint_state_estimator.hpp
    include/${PROJECT_NAME}/lf_controller.hpp
    include/${PROJECT_NAME}/linear_feedback_controller.hpp
    include/${PROJECT_NAME}/linear_feedback_controller_ros.hpp
    include/${PROJECT_NAME}/passthrough_controller.hpp
    include/${PROJECT_NAME}/pd_controller.hpp
    include/${PROJECT_NAME}/robot_model_builder.hpp)
set(${PROJECT_NAME}_sources
    src/joint_state_estimator.cpp #
    src/lf_controller.cpp #
    src/linear_feedback_controller.cpp #
    src/linear_feedback_controller_ros.cpp #
    src/passthrough_controller.cpp #
    src/pd_controller.cpp #
    src/robot_model_builder.cpp)
ament_auto_add_library(${PROJECT_NAME} ${${PROJECT_NAME}_sources}
                       ${${PROJECT_NAME}_headers})

set_target_properties(${PROJECT_NAME} PROPERTIES CXX_VISIBILITY_PRESET hidden)

target_compile_definitions(
  ${PROJECT_NAME}
  PUBLIC -DLINEAR_FEEDBACK_CONTROLLER_IS_SHARED
  PRIVATE -DLINEAR_FEEDBACK_CONTROLLER_DO_EXPORT)

ament_target_dependencies(${PROJECT_NAME} Eigen3)
# Extract the version components of the controller_interface package.
message(STATUS "controller_interface_VERSION: ${controller_interface_VERSION}")
string(REPLACE "." ";" version_list ${controller_interface_VERSION})
list(GET version_list 0 controller_interface_VERSION_MAJOR)
list(GET version_list 1 controller_interface_VERSION_MINOR)
list(GET version_list 2 controller_interface_VERSION_PATCH)
# Define macros for the version components.
target_compile_definitions(
  ${PROJECT_NAME}
  PRIVATE
    CONTROLLER_INTERFACE_MAJOR_VERSION=${controller_interface_VERSION_MAJOR}
    CONTROLLER_INTERFACE_MINOR_VERSION=${controller_interface_VERSION_MINOR}
    CONTROLLER_INTERFACE_PATCH_VERSION=${controller_interface_VERSION_PATCH})
target_link_libraries(${PROJECT_NAME} pinocchio::pinocchio)
target_link_libraries(
  ${PROJECT_NAME} ${PROJECT_NAME}_parameters joint_state_estimator_parameters
  passthrough_controller_parameters)

#
# Unit tests
#
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(GTest REQUIRED)
  ament_auto_find_test_dependencies()

  set(TESTS_LIST test_pd_controller test_lf_controller
                 test_linear_feedback_controller test_robot_model_builder)
  foreach(test_name ${TESTS_LIST})

    add_unit_test(${test_name} tests/${test_name}.cpp)
    target_link_libraries(${test_name} ${PROJECT_NAME} GTest::gtest_main)

  endforeach()

endif()

#
# Export plugins
#
pluginlib_export_plugin_description_file(controller_interface
                                         controller_plugins.xml)

#
# Installation
#
install(
  FILES ${CMAKE_CURRENT_BINARY_DIR}/linear_feedback_controller.md
        ${CMAKE_CURRENT_BINARY_DIR}/joint_state_estimator.md
        ${CMAKE_CURRENT_BINARY_DIR}/passthrough_controller.md
  DESTINATION share/${PROJECT_NAME}/doc)

install(
  PROGRAMS tests/pd_plus_controller.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME pd_plus_controller)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
ament_export_libraries(${PROJECT_NAME})
ament_auto_package()
