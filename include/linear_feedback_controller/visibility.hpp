#ifndef LINEAR_FEEDBACK_CONTROLLER__VISIBILITY_HPP_
#define LINEAR_FEEDBACK_CONTROLLER__VISIBILITY_HPP_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

// Define LINEAR_FEEDBACK_CONTROLLER_[EXPORT, IMPORT, LOCAL]
// based on the OS
#if defined _WIN32 || defined __CYGWIN__

#ifdef __GNUC__
#define LINEAR_FEEDBACK_CONTROLLER_EXPORT __attribute__((dllexport))
#define LINEAR_FEEDBACK_CONTROLLER_IMPORT __attribute__((dllimport))
#else
#define LINEAR_FEEDBACK_CONTROLLER_EXPORT __declspec(dllexport)
#define LINEAR_FEEDBACK_CONTROLLER_IMPORT __declspec(dllimport)
#endif

// All symbols are hidden by default in windows
#define LINEAR_FEEDBACK_CONTROLLER_LOCAL

#else  // defined _WIN32 || defined __CYGWIN__

#if __GNUC__ >= 4
#define LINEAR_FEEDBACK_CONTROLLER_EXPORT __attribute__((visibility("default")))
#define LINEAR_FEEDBACK_CONTROLLER_IMPORT __attribute__((visibility("default")))
#define LINEAR_FEEDBACK_CONTROLLER_LOCAL __attribute__((visibility("hidden")))
#else
#define LINEAR_FEEDBACK_CONTROLLER_EXPORT
#define LINEAR_FEEDBACK_CONTROLLER_IMPORT
#define LINEAR_FEEDBACK_CONTROLLER_LOCAL
#endif

#endif  // defined _WIN32 || defined __CYGWIN__

// Define LINEAR_FEEDBACK_CONTROLLER_[PUBLIC, PRIVATE] based the following
// definitions forwarded by the build system:
// - LINEAR_FEEDBACK_CONTROLLER_IS_SHARED (If the project is a shared lib)
// - LINEAR_FEEDBACK_CONTROLLER_EXPORT (If we are building it directly)
#ifdef LINEAR_FEEDBACK_CONTROLLER_IS_SHARED

// LFC lib is shared (.so)
#ifdef LINEAR_FEEDBACK_CONTROLLER_DO_EXPORT
// We are building the shared lib -> EXPORT symbols
#define LINEAR_FEEDBACK_CONTROLLER_PUBLIC LINEAR_FEEDBACK_CONTROLLER_EXPORT
#else
// We are linking to the shared lib -> IMPORT symbols
#define LINEAR_FEEDBACK_CONTROLLER_PUBLIC LINEAR_FEEDBACK_CONTROLLER_IMPORT
#endif

#define LINEAR_FEEDBACK_CONTROLLER_PRIVATE LINEAR_FEEDBACK_CONTROLLER_LOCAL

#else  // LINEAR_FEEDBACK_CONTROLLER_IS_SHARED

// LFC lib is static (.a)
#define LINEAR_FEEDBACK_CONTROLLER_PRIVATE
#define LINEAR_FEEDBACK_CONTROLLER_PUBLIC

#endif

#endif  // LINEAR_FEEDBACK_CONTROLLER__VISIBILITY_HPP_
