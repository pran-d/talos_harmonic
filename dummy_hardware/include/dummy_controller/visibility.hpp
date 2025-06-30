#ifndef DUMMY_CONTROLLER__VISIBILITY_HPP_
#define DUMMY_CONTROLLER__VISIBILITY_HPP_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

// Define DUMMY_CONTROLLER_[EXPORT, IMPORT, LOCAL]
// based on the OS
#if defined _WIN32 || defined __CYGWIN__

#ifdef __GNUC__
#define DUMMY_CONTROLLER_EXPORT __attribute__((dllexport))
#define DUMMY_CONTROLLER_IMPORT __attribute__((dllimport))
#else
#define DUMMY_CONTROLLER_EXPORT __declspec(dllexport)
#define DUMMY_CONTROLLER_IMPORT __declspec(dllimport)
#endif

// All symbols are hidden by default in windows
#define DUMMY_CONTROLLER_LOCAL

#else  // defined _WIN32 || defined __CYGWIN__

#if __GNUC__ >= 4
#define DUMMY_CONTROLLER_EXPORT __attribute__((visibility("default")))
#define DUMMY_CONTROLLER_IMPORT __attribute__((visibility("default")))
#define DUMMY_CONTROLLER_LOCAL __attribute__((visibility("hidden")))
#else
#define DUMMY_CONTROLLER_EXPORT
#define DUMMY_CONTROLLER_IMPORT
#define DUMMY_CONTROLLER_LOCAL
#endif

#endif  // defined _WIN32 || defined __CYGWIN__

// Define DUMMY_CONTROLLER_[PUBLIC, PRIVATE] based the following
// definitions forwarded by the build system:
// - DUMMY_CONTROLLER_IS_SHARED (If the project is a shared lib)
// - DUMMY_CONTROLLER_EXPORT (If we are building it directly)
#ifdef DUMMY_CONTROLLER_IS_SHARED

// LFC lib is shared (.so)
#ifdef DUMMY_CONTROLLER_DO_EXPORT
// We are building the shared lib -> EXPORT symbols
#define DUMMY_CONTROLLER_PUBLIC DUMMY_CONTROLLER_EXPORT
#else
// We are linking to the shared lib -> IMPORT symbols
#define DUMMY_CONTROLLER_PUBLIC DUMMY_CONTROLLER_IMPORT
#endif

#define DUMMY_CONTROLLER_PRIVATE DUMMY_CONTROLLER_LOCAL

#else  // DUMMY_CONTROLLER_IS_SHARED

// LFC lib is static (.a)
#define DUMMY_CONTROLLER_PRIVATE
#define DUMMY_CONTROLLER_PUBLIC

#endif

#endif  // DUMMY_CONTROLLER__VISIBILITY_HPP_
