cmake_minimum_required(VERSION 3.5)

set(PROJECT_ORG loco-3d)
set(PROJECT_NAME ros_wbmpc_msgs)
set(PROJECT_DESCRIPTION "Dynamic graph bridge msgs ")
set(PROJECT_URL "https://github.com/${PROJECT_ORG}/ros-wbmpc")

if(DEFINED ENV{ROS_VERSION})
  set(ROS_VERSION $ENV{ROS_VERSION})
else()
  set(ROS_VERSION 1)
endif()

set(messages_to_build Contact.msg Control.msg Sensor.msg)
set(message_dependencies std_msgs nav_msgs sensor_msgs geometry_msgs)

if(ROS_VERSION EQUAL 1) # if ROS1
  # Here we use the jrl-cmakemodules to package things properly.
  set(CUSTOM_HEADER_DIR ros_wbmpc_msgs)
  set(PROJECT_USE_CMAKE_EXPORT TRUE)
  include(../cmake/base.cmake)

  compute_project_args(PROJECT_ARGS LANGUAGES CXX C)
  project(${PROJECT_NAME} ${PROJECT_ARGS})

  # Declare build dependencies
  find_package(catkin REQUIRED COMPONENTS message_generation
                                          ${message_dependencies})

  # Add message files
  add_message_files(DIRECTORY msg FILES ${messages_to_build})

  # And now generate the messages
  generate_messages(DEPENDENCIES ${message_dependencies})

  add_library(${PROJECT_NAME} INTERFACE)
  target_include_directories(${PROJECT_NAME}
                             INTERFACE $<INSTALL_INTERFACE:include>)

  catkin_package(
    LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS message_runtime std_msgs geometry_msgs)

  install(
    TARGETS ${PROJECT_NAME}
    EXPORT ${TARGETS_EXPORT_NAME}
    DESTINATION lib)

else() # if ROS2
  # Here ament is doing things ok.
  project(${PROJECT_NAME})

  find_package(ament_cmake REQUIRED)
  find_package(ament_cmake_python REQUIRED)
  find_package(rosidl_default_generators REQUIRED)

  set(messages_to_build_realtive "")
  foreach(msg ${messages_to_build})
    set(messages_to_build_realtive ${messages_to_build_realtive} msg/${msg})
  endforeach(msg ${messages_to_build})
  
  rosidl_generate_interfaces(
    ${PROJECT_NAME} "${messages_to_build_realtive}")
  ament_export_dependencies(rosidl_default_runtime)
  ament_package()
endif()

install(FILES package.xml DESTINATION share/${PROJECT_NAME})
