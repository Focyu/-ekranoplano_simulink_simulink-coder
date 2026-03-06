#ifndef PID_CONTROL_V1__VISIBILITY_CONTROL_H_
#define PID_CONTROL_V1__VISIBILITY_CONTROL_H_
#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define PID_CONTROL_V1_EXPORT __attribute__ ((dllexport))
    #define PID_CONTROL_V1_IMPORT __attribute__ ((dllimport))
  #else
    #define PID_CONTROL_V1_EXPORT __declspec(dllexport)
    #define PID_CONTROL_V1_IMPORT __declspec(dllimport)
  #endif
  #ifdef PID_CONTROL_V1_BUILDING_LIBRARY
    #define PID_CONTROL_V1_PUBLIC PID_CONTROL_V1_EXPORT
  #else
    #define PID_CONTROL_V1_PUBLIC PID_CONTROL_V1_IMPORT
  #endif
  #define PID_CONTROL_V1_PUBLIC_TYPE PID_CONTROL_V1_PUBLIC
  #define PID_CONTROL_V1_LOCAL
#else
  #define PID_CONTROL_V1_EXPORT __attribute__ ((visibility("default")))
  #define PID_CONTROL_V1_IMPORT
  #if __GNUC__ >= 4
    #define PID_CONTROL_V1_PUBLIC __attribute__ ((visibility("default")))
    #define PID_CONTROL_V1_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define PID_CONTROL_V1_PUBLIC
    #define PID_CONTROL_V1_LOCAL
  #endif
  #define PID_CONTROL_V1_PUBLIC_TYPE
#endif
#endif  // PID_CONTROL_V1__VISIBILITY_CONTROL_H_
// Generated 06-Mar-2026 14:48:12
 