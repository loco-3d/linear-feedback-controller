#ifndef LINEAR_FEEDBACK_CONTROLLER__VISIBILITY_CONTROL_HPP_
#define LINEAR_FEEDBACK_CONTROLLER__VISIBILITY_CONTROL_HPP_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

// Define LINEAR_FEEDBACK_CONTROLLER_[EXPORT, IMPORT, LOCAL]
#if defined _WIN32 || defined __CYGWIN__

#ifdef __GNUC__
#define LINEAR_FEEDBACK_CONTROLLER_HELPER_EXPORT __attribute__((dllexport))
#define LINEAR_FEEDBACK_CONTROLLER_HELPER_IMPORT __attribute__((dllimport))
#else
#define LINEAR_FEEDBACK_CONTROLLER_HELPER_EXPORT __declspec(dllexport)
#define LINEAR_FEEDBACK_CONTROLLER_HELPER_IMPORT __declspec(dllimport)
#endif

// All symbols are hidden by default in windows
#define LINEAR_FEEDBACK_CONTROLLER_HELPER_LOCAL

#else  // defined _WIN32 || defined __CYGWIN__

#if __GNUC__ >= 4
#define LINEAR_FEEDBACK_CONTROLLER_HELPER_EXPORT \
  __attribute__((visibility("default")))
#define LINEAR_FEEDBACK_CONTROLLER_HELPER_IMPORT \
  __attribute__((visibility("default")))
#define LINEAR_FEEDBACK_CONTROLLER_HELPER_LOCAL \
  __attribute__((visibility("hidden")))
#else
#define LINEAR_FEEDBACK_CONTROLLER_HELPER_EXPORT
#define LINEAR_FEEDBACK_CONTROLLER_HELPER_IMPORT
#define LINEAR_FEEDBACK_CONTROLLER_HELPER_LOCAL
#endif

#endif  // defined _WIN32 || defined __CYGWIN__

// Define LINEAR_FEEDBACK_CONTROLLER_[API, LOCAL] based on the
#ifdef LINEAR_FEEDBACK_CONTROLLER_IS_SHARED
// LFC lib is shared (.so)

#ifdef LINEAR_FEEDBACK_CONTROLLER_EXPORT  // Set only when compiling the lib
// We are building the shared lib -> EXPORT symbols
#define LINEAR_FEEDBACK_CONTROLLER_API LINEAR_FEEDBACK_CONTROLLER_HELPER_EXPORT
#else
// We are linking to the shared lib -> IMPORT symbols
#define LINEAR_FEEDBACK_CONTROLLER_API LINEAR_FEEDBACK_CONTROLLER_HELPER_IMPORT
#endif

#define LINEAR_FEEDBACK_CONTROLLER_LOCAL LINEAR_FEEDBACK_CONTROLLER_HELPER_LOCAL

#else  // LINEAR_FEEDBACK_CONTROLLER_IS_SHARED
// LFC lib is static (.a)
#define LINEAR_FEEDBACK_CONTROLLER_LOCAL
#define LINEAR_FEEDBACK_CONTROLLER_API
#endif

#endif  // LINEAR_FEEDBACK_CONTROLLER__VISIBILITY_CONTROL_HPP_
