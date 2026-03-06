/*
 * pid_control_V1_types.h
 *
 * Trial License - for use to evaluate programs for possible purchase as
 * an end-user only.
 *
 * Code generation for model "pid_control_V1".
 *
 * Model version              : 12.56
 * Simulink Coder version : 25.2 (R2025b) 28-Jul-2025
 * C++ source code generated on : Fri Mar  6 14:35:53 2026
 *
 * Target selection: ert.tlc
 * Note: GRT includes extra infrastructure and instrumentation for prototyping
 * Embedded hardware selection: Intel->x86-64 (Linux 64)
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#ifndef pid_control_V1_types_h_
#define pid_control_V1_types_h_
#include "rtwtypes.h"
#ifndef DEFINED_TYPEDEF_FOR_SL_Bus_ROSVariableLengthArrayInfo_
#define DEFINED_TYPEDEF_FOR_SL_Bus_ROSVariableLengthArrayInfo_

struct SL_Bus_ROSVariableLengthArrayInfo
{
  uint32_T CurrentLength;
  uint32_T ReceivedLength;
};

#endif

#ifndef DEFINED_TYPEDEF_FOR_SL_Bus_geometry_msgs_Point_
#define DEFINED_TYPEDEF_FOR_SL_Bus_geometry_msgs_Point_

struct SL_Bus_geometry_msgs_Point
{
  real_T x;
  real_T y;
  real_T z;
};

#endif

#ifndef DEFINED_TYPEDEF_FOR_SL_Bus_geometry_msgs_Quaternion_
#define DEFINED_TYPEDEF_FOR_SL_Bus_geometry_msgs_Quaternion_

struct SL_Bus_geometry_msgs_Quaternion
{
  real_T x;
  real_T y;
  real_T z;
  real_T w;
};

#endif

#ifndef DEFINED_TYPEDEF_FOR_SL_Bus_geometry_msgs_Pose_
#define DEFINED_TYPEDEF_FOR_SL_Bus_geometry_msgs_Pose_

struct SL_Bus_geometry_msgs_Pose
{
  SL_Bus_geometry_msgs_Point position;
  SL_Bus_geometry_msgs_Quaternion orientation;
};

#endif

#ifndef DEFINED_TYPEDEF_FOR_SL_Bus_geometry_msgs_Vector3_
#define DEFINED_TYPEDEF_FOR_SL_Bus_geometry_msgs_Vector3_

struct SL_Bus_geometry_msgs_Vector3
{
  real_T x;
  real_T y;
  real_T z;
};

#endif

#ifndef DEFINED_TYPEDEF_FOR_SL_Bus_geometry_msgs_Twist_
#define DEFINED_TYPEDEF_FOR_SL_Bus_geometry_msgs_Twist_

struct SL_Bus_geometry_msgs_Twist
{
  SL_Bus_geometry_msgs_Vector3 linear;
  SL_Bus_geometry_msgs_Vector3 angular;
};

#endif

#ifndef DEFINED_TYPEDEF_FOR_SL_Bus_gazebo_msgs_EntityState_
#define DEFINED_TYPEDEF_FOR_SL_Bus_gazebo_msgs_EntityState_

struct SL_Bus_gazebo_msgs_EntityState
{
  uint8_T name[128];
  SL_Bus_ROSVariableLengthArrayInfo name_SL_Info;
  SL_Bus_geometry_msgs_Pose pose;
  SL_Bus_geometry_msgs_Twist twist;
  uint8_T reference_frame[128];
  SL_Bus_ROSVariableLengthArrayInfo reference_frame_SL_Info;
};

#endif

#ifndef DEFINED_TYPEDEF_FOR_SL_Bus_gazebo_msgs_SetEntityStateRequest_
#define DEFINED_TYPEDEF_FOR_SL_Bus_gazebo_msgs_SetEntityStateRequest_

struct SL_Bus_gazebo_msgs_SetEntityStateRequest
{
  SL_Bus_gazebo_msgs_EntityState state;
};

#endif

#ifndef DEFINED_TYPEDEF_FOR_SL_Bus_gazebo_msgs_SetEntityStateResponse_
#define DEFINED_TYPEDEF_FOR_SL_Bus_gazebo_msgs_SetEntityStateResponse_

struct SL_Bus_gazebo_msgs_SetEntityStateResponse
{
  boolean_T success;
};

#endif

#ifndef DEFINED_TYPEDEF_FOR_SL_Bus_std_msgs_Float64_
#define DEFINED_TYPEDEF_FOR_SL_Bus_std_msgs_Float64_

struct SL_Bus_std_msgs_Float64
{
  real_T data;
};

#endif

#ifndef DEFINED_TYPEDEF_FOR_SL_Bus_std_msgs_Bool_
#define DEFINED_TYPEDEF_FOR_SL_Bus_std_msgs_Bool_

struct SL_Bus_std_msgs_Bool
{
  boolean_T data;
};

#endif

#ifndef struct_sJ4ih70VmKcvCeguWN0mNVF
#define struct_sJ4ih70VmKcvCeguWN0mNVF

struct sJ4ih70VmKcvCeguWN0mNVF
{
  real_T sec;
  real_T nsec;
};

#endif                                 /* struct_sJ4ih70VmKcvCeguWN0mNVF */

#ifndef struct_ros_slros2_internal_block_Ser_T
#define struct_ros_slros2_internal_block_Ser_T

struct ros_slros2_internal_block_Ser_T
{
  boolean_T matlabCodegenIsDeleted;
  int32_T isInitialized;
  boolean_T isSetupComplete;
  boolean_T QOSAvoidROSNamespaceConventions;
};

#endif                              /* struct_ros_slros2_internal_block_Ser_T */

#ifndef struct_robotics_slcore_internal_bloc_T
#define struct_robotics_slcore_internal_bloc_T

struct robotics_slcore_internal_bloc_T
{
  int32_T isInitialized;
};

#endif                              /* struct_robotics_slcore_internal_bloc_T */

#ifndef struct_ros_slros2_internal_block_Sub_T
#define struct_ros_slros2_internal_block_Sub_T

struct ros_slros2_internal_block_Sub_T
{
  boolean_T matlabCodegenIsDeleted;
  int32_T isInitialized;
  boolean_T isSetupComplete;
  boolean_T QOSAvoidROSNamespaceConventions;
};

#endif                              /* struct_ros_slros2_internal_block_Sub_T */

/* Forward declaration for rtModel */
typedef struct tag_RTM_pid_control_V1_T RT_MODEL_pid_control_V1_T;

#endif                                 /* pid_control_V1_types_h_ */
