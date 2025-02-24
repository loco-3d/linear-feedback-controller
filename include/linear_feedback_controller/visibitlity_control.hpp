#ifndef LINEAR_FEEDBACK_CONTROLLER__VISIBILITY_CONTROL_HPP_
#define LINEAR_FEEDBACK_CONTROLLER__VISIBILITY_CONTROL_HPP_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__

#ifdef __GNUC__
#define LINEAR_FEEDBACK_CONTROLLER_EXPORT __attribute__((dllexport))
#define LINEAR_FEEDBACK_CONTROLLER_IMPORT __attribute__((dllimport))
#else
#define LINEAR_FEEDBACK_CONTROLLER_EXPORT __declspec(dllexport)
#define LINEAR_FEEDBACK_CONTROLLER_IMPORT __declspec(dllimport)
#endif

#ifdef LINEAR_FEEDBACK_CONTROLLER_BUILDING_DLL
#define LINEAR_FEEDBACK_CONTROLLER_PUBLIC LINEAR_FEEDBACK_CONTROLLER_EXPORT
#else
#define LINEAR_FEEDBACK_CONTROLLER_PUBLIC LINEAR_FEEDBACK_CONTROLLER_IMPORT
#endif

#define LINEAR_FEEDBACK_CONTROLLER_PUBLIC_TYPE LINEAR_FEEDBACK_CONTROLLER_PUBLIC
#define LINEAR_FEEDBACK_CONTROLLER_LOCAL

#else  // defined _WIN32 || defined __CYGWIN__

#define LINEAR_FEEDBACK_CONTROLLER_EXPORT __attribute__((visibility("default")))
#define LINEAR_FEEDBACK_CONTROLLER_IMPORT

#if __GNUC__ >= 4
#define LINEAR_FEEDBACK_CONTROLLER_PUBLIC __attribute__((visibility("default")))
#define LINEAR_FEEDBACK_CONTROLLER_LOCAL __attribute__((visibility("hidden")))
#else
#define LINEAR_FEEDBACK_CONTROLLER_PUBLIC
#define LINEAR_FEEDBACK_CONTROLLER_LOCAL
#endif

#define LINEAR_FEEDBACK_CONTROLLER_PUBLIC_TYPE

#endif  // defined _WIN32 || defined __CYGWIN__

#endif  // LINEAR_FEEDBACK_CONTROLLER__VISIBILITY_CONTROL_HPP_
