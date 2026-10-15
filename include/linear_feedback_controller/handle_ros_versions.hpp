#ifndef LINEAR_FEEDBACK_CONTROLLER__HANDLE_ROS_VERSIONS_HPP_
#define LINEAR_FEEDBACK_CONTROLLER__HANDLE_ROS_VERSIONS_HPP_

#define CONTROLLER_INTERFACE_VERSION_AT_LEAST(major, minor, patch) \
  ((CONTROLLER_INTERFACE_MAJOR_VERSION > (major)) ||               \
   (CONTROLLER_INTERFACE_MAJOR_VERSION == (major) &&               \
    CONTROLLER_INTERFACE_MINOR_VERSION > (minor)) ||               \
   (CONTROLLER_INTERFACE_MAJOR_VERSION == (major) &&               \
    CONTROLLER_INTERFACE_MINOR_VERSION == (minor) &&               \
    CONTROLLER_INTERFACE_PATCH_VERSION >= (patch)))

// realtime_tools renamed RealtimePublisher::tryPublish() to try_publish()
// somewhere between 2.x and 3.x (2.15.0: tryPublish only: 3.11.0: both,
// try_publish preferred, tryPublish a deprecated wrapper: 5.2.0: try_publish
// only).
#define REALTIME_TOOLS_VERSION_AT_LEAST(major, minor, patch) \
  ((REALTIME_TOOLS_MAJOR_VERSION > (major)) ||               \
   (REALTIME_TOOLS_MAJOR_VERSION == (major) &&               \
    REALTIME_TOOLS_MINOR_VERSION > (minor)) ||               \
   (REALTIME_TOOLS_MAJOR_VERSION == (major) &&               \
    REALTIME_TOOLS_MINOR_VERSION == (minor) &&               \
    REALTIME_TOOLS_PATCH_VERSION >= (patch)))

#endif  //  LINEAR_FEEDBACK_CONTROLLER__HANDLE_ROS_VERSIONS_HPP_
