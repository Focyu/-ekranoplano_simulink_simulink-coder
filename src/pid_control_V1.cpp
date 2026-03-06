/*
 * pid_control_V1.cpp
 *
 * Trial License - for use to evaluate programs for possible purchase as
 * an end-user only.
 *
 * Code generation for model "pid_control_V1".
 *
 * Model version              : 12.56
 * Simulink Coder version : 25.2 (R2025b) 28-Jul-2025
 * C++ source code generated on : Fri Mar  6 14:47:40 2026
 *
 * Target selection: ert.tlc
 * Note: GRT includes extra infrastructure and instrumentation for prototyping
 * Embedded hardware selection: Intel->x86-64 (Linux 64)
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#include "pid_control_V1.h"
#include "rtwtypes.h"
#include "pid_control_V1_types.h"
#include <string.h>
#include "pid_control_V1_private.h"
#include <emmintrin.h>
#include <math.h>

extern "C"
{

#include "rt_nonfinite.h"

}

#include "rmw/qos_profiles.h"
#include <stddef.h>
#include "rt_defines.h"

uint32_T plook_bincpa(real_T u, const real_T bp[], uint32_T maxIndex, real_T
                      *fraction, uint32_T *prevIndex)
{
  uint32_T bpIndex;

  /* Prelookup - Index and Fraction
     Index Search method: 'binary'
     Extrapolation method: 'Clip'
     Use previous index: 'on'
     Use last breakpoint for index at or above upper limit: 'on'
     Remove protection against out-of-range input in generated code: 'off'
   */
  if (u <= bp[0U]) {
    bpIndex = 0U;
    *fraction = 0.0;
  } else if (u < bp[maxIndex]) {
    bpIndex = binsearch_u32d_prevIdx(u, bp, *prevIndex, maxIndex);
    *fraction = (u - bp[bpIndex]) / (bp[bpIndex + 1U] - bp[bpIndex]);
  } else {
    bpIndex = maxIndex;
    *fraction = 0.0;
  }

  *prevIndex = bpIndex;
  return bpIndex;
}

real_T intrp2d_la_pw(const uint32_T bpIndex[], const real_T frac[], const real_T
                     table[], const uint32_T stride, const uint32_T maxIndex[])
{
  real_T y;
  real_T yL_0d0;
  uint32_T offset_1d;

  /* Column-major Interpolation 2-D
     Interpolation method: 'Linear point-slope'
     Use last breakpoint for index at or above upper limit: 'on'
     Overflow mode: 'portable wrapping'
   */
  offset_1d = bpIndex[1U] * stride + bpIndex[0U];
  if (bpIndex[0U] == maxIndex[0U]) {
    y = table[offset_1d];
  } else {
    yL_0d0 = table[offset_1d];
    y = (table[offset_1d + 1U] - yL_0d0) * frac[0U] + yL_0d0;
  }

  if (bpIndex[1U] == maxIndex[1U]) {
  } else {
    offset_1d += stride;
    if (bpIndex[0U] == maxIndex[0U]) {
      yL_0d0 = table[offset_1d];
    } else {
      yL_0d0 = table[offset_1d];
      yL_0d0 += (table[offset_1d + 1U] - yL_0d0) * frac[0U];
    }

    y += (yL_0d0 - y) * frac[1U];
  }

  return y;
}

uint32_T binsearch_u32d_prevIdx(real_T u, const real_T bp[], uint32_T startIndex,
  uint32_T maxIndex)
{
  uint32_T bpIndex;
  uint32_T found;
  uint32_T iLeft;
  uint32_T iRght;

  /* Binary Search using Previous Index */
  bpIndex = startIndex;
  iLeft = 0U;
  iRght = maxIndex;
  found = 0U;
  while (found == 0U) {
    if (u < bp[bpIndex]) {
      iRght = bpIndex - 1U;
      bpIndex = ((bpIndex + iLeft) - 1U) >> 1U;
    } else if (u < bp[bpIndex + 1U]) {
      found = 1U;
    } else {
      iLeft = bpIndex + 1U;
      bpIndex = ((bpIndex + iRght) + 1U) >> 1U;
    }
  }

  return bpIndex;
}

/*
 * This function updates continuous states using the ODE3 fixed-step
 * solver algorithm
 */
void pid_control_V1::rt_ertODEUpdateContinuousStates(RTWSolverInfo *si )
{
  /* Solver Matrices */
  static const real_T rt_ODE3_A[3] = {
    1.0/2.0, 3.0/4.0, 1.0
  };

  static const real_T rt_ODE3_B[3][3] = {
    { 1.0/2.0, 0.0, 0.0 },

    { 0.0, 3.0/4.0, 0.0 },

    { 2.0/9.0, 1.0/3.0, 4.0/9.0 }
  };

  time_T t = rtsiGetT(si);
  time_T tnew = rtsiGetSolverStopTime(si);
  time_T h = rtsiGetStepSize(si);
  real_T *x = rtsiGetContStates(si);
  ODE3_IntgData *id = static_cast<ODE3_IntgData *>(rtsiGetSolverData(si));
  real_T *y = id->y;
  real_T *f0 = id->f[0];
  real_T *f1 = id->f[1];
  real_T *f2 = id->f[2];
  real_T hB[3];
  int_T i;
  int_T nXc = 39;
  rtsiSetSimTimeStep(si,MINOR_TIME_STEP);

  /* Save the state values at time t in y, we'll use x as ynew. */
  (void) memcpy(y, x,
                static_cast<uint_T>(nXc)*sizeof(real_T));

  /* Assumes that rtsiSetT and ModelOutputs are up-to-date */
  /* f0 = f(t,y) */
  rtsiSetdX(si, f0);
  pid_control_V1_derivatives();

  /* f(:,2) = feval(odefile, t + hA(1), y + f*hB(:,1), args(:)(*)); */
  hB[0] = h * rt_ODE3_B[0][0];
  for (i = 0; i < nXc; i++) {
    x[i] = y[i] + (f0[i]*hB[0]);
  }

  rtsiSetT(si, t + h*rt_ODE3_A[0]);
  rtsiSetdX(si, f1);
  this->step();
  pid_control_V1_derivatives();

  /* f(:,3) = feval(odefile, t + hA(2), y + f*hB(:,2), args(:)(*)); */
  for (i = 0; i <= 1; i++) {
    hB[i] = h * rt_ODE3_B[1][i];
  }

  for (i = 0; i < nXc; i++) {
    x[i] = y[i] + (f0[i]*hB[0] + f1[i]*hB[1]);
  }

  rtsiSetT(si, t + h*rt_ODE3_A[1]);
  rtsiSetdX(si, f2);
  this->step();
  pid_control_V1_derivatives();

  /* tnew = t + hA(3);
     ynew = y + f*hB(:,3); */
  for (i = 0; i <= 2; i++) {
    hB[i] = h * rt_ODE3_B[2][i];
  }

  for (i = 0; i < nXc; i++) {
    x[i] = y[i] + (f0[i]*hB[0] + f1[i]*hB[1] + f2[i]*hB[2]);
  }

  rtsiSetT(si, tnew);
  rtsiSetSimTimeStep(si,MAJOR_TIME_STEP);
}

/*
 * System initialize for enable system:
 *    '<S279>/Enabled Subsystem'
 *    '<S280>/Enabled Subsystem'
 */
void pid_control_V1::pid_contr_EnabledSubsystem_Init
  (B_EnabledSubsystem_pid_contro_T *localB)
{
  /* SystemInitialize for SignalConversion generated from: '<S320>/In1' */
  memset(&localB->In1, 0, sizeof(SL_Bus_std_msgs_Bool));
}

/*
 * Output and update for enable system:
 *    '<S279>/Enabled Subsystem'
 *    '<S280>/Enabled Subsystem'
 */
void pid_control_V1::pid_control_V1_EnabledSubsystem(boolean_T rtu_Enable, const
  SL_Bus_std_msgs_Bool *rtu_In1, B_EnabledSubsystem_pid_contro_T *localB)
{
  /* Outputs for Enabled SubSystem: '<S279>/Enabled Subsystem' incorporates:
   *  EnablePort: '<S320>/Enable'
   */
  if (rtu_Enable) {
    /* SignalConversion generated from: '<S320>/In1' */
    localB->In1 = *rtu_In1;
  }

  /* End of Outputs for SubSystem: '<S279>/Enabled Subsystem' */
}

void pid_control_V1::pid_contro_Subscriber_setupImpl(const
  ros_slros2_internal_block_Sub_T *obj)
{
  rmw_qos_profile_t qos_profile;
  sJ4ih70VmKcvCeguWN0mNVF lifespan;
  sJ4ih70VmKcvCeguWN0mNVF liveliness_lease_duration;
  char_T b_zeroDelimTopic[14];
  static const char_T b_zeroDelimTopic_0[14] = "/setpoint/yaw";
  qos_profile = rmw_qos_profile_default;

  /* Start for MATLABSystem: '<S10>/SourceBlock' */
  pid_control_V1_B.deadline.sec = 0.0;
  pid_control_V1_B.deadline.nsec = 0.0;
  lifespan.sec = 0.0;
  lifespan.nsec = 0.0;
  liveliness_lease_duration.sec = 0.0;
  liveliness_lease_duration.nsec = 0.0;
  SET_QOS_VALUES(qos_profile, RMW_QOS_POLICY_HISTORY_KEEP_LAST, (size_t)10.0,
                 RMW_QOS_POLICY_DURABILITY_VOLATILE,
                 RMW_QOS_POLICY_RELIABILITY_RELIABLE, pid_control_V1_B.deadline,
                 lifespan, RMW_QOS_POLICY_LIVELINESS_AUTOMATIC,
                 liveliness_lease_duration, (bool)
                 obj->QOSAvoidROSNamespaceConventions);
  for (int32_T i = 0; i < 14; i++) {
    /* Start for MATLABSystem: '<S10>/SourceBlock' */
    b_zeroDelimTopic[i] = b_zeroDelimTopic_0[i];
  }

  Sub_pid_control_V1_377.createSubscriber(&b_zeroDelimTopic[0], qos_profile);
}

void pid_control_V1::pid_con_ServiceCaller_setupImpl(const
  ros_slros2_internal_block_Ser_T *obj)
{
  rmw_qos_profile_t qos_profile;
  sJ4ih70VmKcvCeguWN0mNVF deadline;
  sJ4ih70VmKcvCeguWN0mNVF lifespan;
  sJ4ih70VmKcvCeguWN0mNVF liveliness_lease_duration;
  static const char_T b_zeroDelimTopic[25] = "/gazebo/set_entity_state";
  qos_profile = rmw_qos_profile_default;

  /* Start for MATLABSystem: '<S2>/ServiceCaller' */
  deadline.sec = 0.0;
  deadline.nsec = 0.0;
  lifespan.sec = 0.0;
  lifespan.nsec = 0.0;
  liveliness_lease_duration.sec = 0.0;
  liveliness_lease_duration.nsec = 0.0;
  SET_QOS_VALUES(qos_profile, RMW_QOS_POLICY_HISTORY_KEEP_LAST, (size_t)1.0,
                 RMW_QOS_POLICY_DURABILITY_VOLATILE,
                 RMW_QOS_POLICY_RELIABILITY_RELIABLE, deadline, lifespan,
                 RMW_QOS_POLICY_LIVELINESS_AUTOMATIC, liveliness_lease_duration,
                 (bool)obj->QOSAvoidROSNamespaceConventions);
  for (int32_T i = 0; i < 25; i++) {
    /* Start for MATLABSystem: '<S2>/ServiceCaller' */
    pid_control_V1_B.b_zeroDelimTopic[i] = b_zeroDelimTopic[i];
  }

  ServCall_pid_control_V1_326.createServiceCaller
    (&pid_control_V1_B.b_zeroDelimTopic[0], qos_profile);
}

void pid_control_V1::pid_cont_Subscriber_setupImpl_o(const
  ros_slros2_internal_block_Sub_T *obj)
{
  rmw_qos_profile_t qos_profile;
  sJ4ih70VmKcvCeguWN0mNVF deadline;
  sJ4ih70VmKcvCeguWN0mNVF lifespan;
  sJ4ih70VmKcvCeguWN0mNVF liveliness_lease_duration;
  static const char_T b_zeroDelimTopic[22] = "/setpoint/turbulencia";
  qos_profile = rmw_qos_profile_default;

  /* Start for MATLABSystem: '<S279>/SourceBlock' */
  deadline.sec = 0.0;
  deadline.nsec = 0.0;
  lifespan.sec = 0.0;
  lifespan.nsec = 0.0;
  liveliness_lease_duration.sec = 0.0;
  liveliness_lease_duration.nsec = 0.0;
  SET_QOS_VALUES(qos_profile, RMW_QOS_POLICY_HISTORY_KEEP_LAST, (size_t)10.0,
                 RMW_QOS_POLICY_DURABILITY_VOLATILE,
                 RMW_QOS_POLICY_RELIABILITY_RELIABLE, deadline, lifespan,
                 RMW_QOS_POLICY_LIVELINESS_AUTOMATIC, liveliness_lease_duration,
                 (bool)obj->QOSAvoidROSNamespaceConventions);
  for (int32_T i = 0; i < 22; i++) {
    /* Start for MATLABSystem: '<S279>/SourceBlock' */
    pid_control_V1_B.b_zeroDelimTopic_c[i] = b_zeroDelimTopic[i];
  }

  Sub_pid_control_V1_417.createSubscriber(&pid_control_V1_B.b_zeroDelimTopic_c[0],
    qos_profile);
}

void pid_control_V1::pid_con_Subscriber_setupImpl_on(const
  ros_slros2_internal_block_Sub_T *obj)
{
  rmw_qos_profile_t qos_profile;
  sJ4ih70VmKcvCeguWN0mNVF deadline;
  sJ4ih70VmKcvCeguWN0mNVF lifespan;
  sJ4ih70VmKcvCeguWN0mNVF liveliness_lease_duration;
  static const char_T b_zeroDelimTopic[22] = "/setpoint/turbulencia";
  qos_profile = rmw_qos_profile_default;

  /* Start for MATLABSystem: '<S280>/SourceBlock' */
  deadline.sec = 0.0;
  deadline.nsec = 0.0;
  lifespan.sec = 0.0;
  lifespan.nsec = 0.0;
  liveliness_lease_duration.sec = 0.0;
  liveliness_lease_duration.nsec = 0.0;
  SET_QOS_VALUES(qos_profile, RMW_QOS_POLICY_HISTORY_KEEP_LAST, (size_t)10.0,
                 RMW_QOS_POLICY_DURABILITY_VOLATILE,
                 RMW_QOS_POLICY_RELIABILITY_RELIABLE, deadline, lifespan,
                 RMW_QOS_POLICY_LIVELINESS_AUTOMATIC, liveliness_lease_duration,
                 (bool)obj->QOSAvoidROSNamespaceConventions);
  for (int32_T i = 0; i < 22; i++) {
    /* Start for MATLABSystem: '<S280>/SourceBlock' */
    pid_control_V1_B.b_zeroDelimTopic_m[i] = b_zeroDelimTopic[i];
  }

  Sub_pid_control_V1_423.createSubscriber(&pid_control_V1_B.b_zeroDelimTopic_m[0],
    qos_profile);
}

real_T rt_urand_Upu32_Yd_f_pw_snf(uint32_T *u)
{
  uint32_T hi;
  uint32_T lo;

  /* Uniform random number generator (random number between 0 and 1)

     #define IA      16807                      magic multiplier = 7^5
     #define IM      2147483647                 modulus = 2^31-1
     #define IQ      127773                     IM div IA
     #define IR      2836                       IM modulo IA
     #define S       4.656612875245797e-10      reciprocal of 2^31-1
     test = IA * (seed % IQ) - IR * (seed/IQ)
     seed = test < 0 ? (test + IM) : test
     return (seed*S)
   */
  lo = *u % 127773U * 16807U;
  hi = *u / 127773U * 2836U;
  if (lo < hi) {
    *u = 2147483647U - (hi - lo);
  } else {
    *u = lo - hi;
  }

  return static_cast<real_T>(*u) * 4.6566128752457969E-10;
}

real_T rt_nrand_Upu32_Yd_f_pw_snf(uint32_T *u)
{
  real_T si;
  real_T sr;
  real_T y;

  /* Normal (Gaussian) random number generator */
  do {
    sr = 2.0 * rt_urand_Upu32_Yd_f_pw_snf(u) - 1.0;
    si = 2.0 * rt_urand_Upu32_Yd_f_pw_snf(u) - 1.0;
    si = sr * sr + si * si;
  } while (si > 1.0);

  y = sqrt(-2.0 * log(si) / si) * sr;
  return y;
}

real_T rt_atan2d_snf(real_T u0, real_T u1)
{
  real_T y;
  if (rtIsNaN(u0) || rtIsNaN(u1)) {
    y = (rtNaN);
  } else if (rtIsInf(u0) && rtIsInf(u1)) {
    int32_T tmp;
    int32_T tmp_0;
    if (u0 > 0.0) {
      tmp = 1;
    } else {
      tmp = -1;
    }

    if (u1 > 0.0) {
      tmp_0 = 1;
    } else {
      tmp_0 = -1;
    }

    y = atan2(static_cast<real_T>(tmp), static_cast<real_T>(tmp_0));
  } else if (u1 == 0.0) {
    if (u0 > 0.0) {
      y = RT_PI / 2.0;
    } else if (u0 < 0.0) {
      y = -(RT_PI / 2.0);
    } else {
      y = 0.0;
    }
  } else {
    y = atan2(u0, u1);
  }

  return y;
}

real_T pid_control_V1::pid_control_V1_rt_powd_snf(real_T u0, real_T u1)
{
  real_T y;
  if (rtIsNaN(u0) || rtIsNaN(u1)) {
    y = (rtNaN);
  } else {
    pid_control_V1_B.d = fabs(u0);
    pid_control_V1_B.d1 = fabs(u1);
    if (rtIsInf(u1)) {
      if (pid_control_V1_B.d == 1.0) {
        y = 1.0;
      } else if (pid_control_V1_B.d > 1.0) {
        if (u1 > 0.0) {
          y = (rtInf);
        } else {
          y = 0.0;
        }
      } else if (u1 > 0.0) {
        y = 0.0;
      } else {
        y = (rtInf);
      }
    } else if (pid_control_V1_B.d1 == 0.0) {
      y = 1.0;
    } else if (pid_control_V1_B.d1 == 1.0) {
      if (u1 > 0.0) {
        y = u0;
      } else {
        y = 1.0 / u0;
      }
    } else if (u1 == 2.0) {
      y = u0 * u0;
    } else if ((u1 == 0.5) && (u0 >= 0.0)) {
      y = sqrt(u0);
    } else if ((u0 < 0.0) && (u1 > floor(u1))) {
      y = (rtNaN);
    } else {
      y = pow(u0, u1);
    }
  }

  return y;
}

/* Model step function */
void pid_control_V1::step()
{
  /* local block i/o variables */
  SL_Bus_std_msgs_Bool rtb_SourceBlock_o2_d;
  SL_Bus_std_msgs_Bool rtb_SourceBlock_o2_j;
  __m128d tmp_1;
  __m128d tmp_2;
  __m128d tmp_3;
  SL_Bus_gazebo_msgs_SetEntityStateResponse tmp;
  int8_T rtAction;
  int8_T rtPrevAction;
  boolean_T serverAvailableOnTime;
  boolean_T tmp_0;
  static const uint8_T b[11] = { 101U, 107U, 114U, 97U, 110U, 111U, 112U, 108U,
    97U, 110U, 111U };

  static const uint8_T b_0[5] = { 119U, 111U, 114U, 108U, 100U };

  if (rtmIsMajorTimeStep((&pid_control_V1_M))) {
    /* set solver stop time */
    if (!((&pid_control_V1_M)->Timing.clockTick0+1)) {
      rtsiSetSolverStopTime(&(&pid_control_V1_M)->solverInfo,
                            (((&pid_control_V1_M)->Timing.clockTickH0 + 1) *
        (&pid_control_V1_M)->Timing.stepSize0 * 4294967296.0));
    } else {
      rtsiSetSolverStopTime(&(&pid_control_V1_M)->solverInfo,
                            (((&pid_control_V1_M)->Timing.clockTick0 + 1) *
        (&pid_control_V1_M)->Timing.stepSize0 + (&pid_control_V1_M)
        ->Timing.clockTickH0 * (&pid_control_V1_M)->Timing.stepSize0 *
        4294967296.0));
    }
  }                                    /* end MajorTimeStep */

  /* Update absolute time of base rate at minor time step */
  if (rtmIsMinorTimeStep((&pid_control_V1_M))) {
    (&pid_control_V1_M)->Timing.t[0] = rtsiGetT(&(&pid_control_V1_M)->solverInfo);
  }

  /* Integrator: '<S11>/Integrator' */
  memcpy(&pid_control_V1_B.x[0], &pid_control_V1_X.Integrator_CSTATE[0], 12U *
         sizeof(real_T));

  /* Gain: '<Root>/Gain' */
  pid_control_V1_B.Gain = -pid_control_V1_B.x[11];

  /* Gain: '<S261>/Filter Coefficient' incorporates:
   *  Constant: '<Root>/Constant3'
   *  Gain: '<S251>/Derivative Gain'
   *  Integrator: '<S253>/Filter'
   *  Sum: '<Root>/Sum3'
   *  Sum: '<S253>/SumD'
   */
  pid_control_V1_B.FilterCoefficient = ((28.0 - pid_control_V1_B.x[0]) * 0.005 -
    pid_control_V1_X.Filter_CSTATE) * 100.0;

  /* Sum: '<S267>/Sum' incorporates:
   *  Constant: '<Root>/Constant3'
   *  Gain: '<S263>/Proportional Gain'
   *  Integrator: '<S258>/Integrator'
   *  Sum: '<Root>/Sum3'
   */
  pid_control_V1_B.SignPreSat = ((28.0 - pid_control_V1_B.x[0]) * 0.05 +
    pid_control_V1_X.Integrator_CSTATE_f) + pid_control_V1_B.FilterCoefficient;

  /* Saturate: '<S265>/Saturation' */
  if (pid_control_V1_B.SignPreSat > 1.0) {
    /* Saturate: '<S265>/Saturation' */
    pid_control_V1_B.Saturation = 1.0;
  } else if (pid_control_V1_B.SignPreSat < 0.0) {
    /* Saturate: '<S265>/Saturation' */
    pid_control_V1_B.Saturation = 0.0;
  } else {
    /* Saturate: '<S265>/Saturation' */
    pid_control_V1_B.Saturation = pid_control_V1_B.SignPreSat;
  }

  /* End of Saturate: '<S265>/Saturation' */

  /* Gain: '<Root>/Gain6' incorporates:
   *  Constant: '<Root>/Constant1'
   *  Sum: '<Root>/Sum6'
   */
  pid_control_V1_B.Saturation1 = (0.0 - pid_control_V1_B.x[10]) * 0.02;

  /* Saturate: '<Root>/Saturation1' */
  if (pid_control_V1_B.Saturation1 > 0.35) {
    /* Gain: '<Root>/Gain6' incorporates:
     *  Saturate: '<Root>/Saturation1'
     */
    pid_control_V1_B.Saturation1 = 0.35;
  } else if (pid_control_V1_B.Saturation1 < -0.35) {
    /* Gain: '<Root>/Gain6' incorporates:
     *  Saturate: '<Root>/Saturation1'
     */
    pid_control_V1_B.Saturation1 = -0.35;
  }

  /* End of Saturate: '<Root>/Saturation1' */

  /* Sum: '<Root>/Sum4' */
  pid_control_V1_B.Sum4 = pid_control_V1_B.Saturation1 - pid_control_V1_B.x[6];

  /* Gain: '<S49>/Filter Coefficient' incorporates:
   *  Gain: '<S39>/Derivative Gain'
   *  Integrator: '<S41>/Filter'
   *  Sum: '<S41>/SumD'
   */
  pid_control_V1_B.FilterCoefficient_c = (-10.0 * pid_control_V1_B.Sum4 -
    pid_control_V1_X.Filter_CSTATE_g) * 100.0;

  /* Sum: '<S55>/Sum' incorporates:
   *  Gain: '<S51>/Proportional Gain'
   *  Integrator: '<S46>/Integrator'
   */
  pid_control_V1_B.Sum_ks = (-15.0 * pid_control_V1_B.Sum4 +
    pid_control_V1_X.Integrator_CSTATE_m) + pid_control_V1_B.FilterCoefficient_c;

  /* Saturate: '<S53>/Saturation' */
  if (pid_control_V1_B.Sum_ks > 0.3490658503988659) {
    /* Saturate: '<S53>/Saturation' */
    pid_control_V1_B.Saturation_k = 0.3490658503988659;
  } else if (pid_control_V1_B.Sum_ks < -0.3490658503988659) {
    /* Saturate: '<S53>/Saturation' */
    pid_control_V1_B.Saturation_k = -0.3490658503988659;
  } else {
    /* Saturate: '<S53>/Saturation' */
    pid_control_V1_B.Saturation_k = pid_control_V1_B.Sum_ks;
  }

  /* End of Saturate: '<S53>/Saturation' */

  /* Outputs for Enabled SubSystem: '<S283>/Hugw(s)' incorporates:
   *  EnablePort: '<S296>/Enable'
   */
  /* Outputs for Enabled SubSystem: '<S282>/Hrgw' incorporates:
   *  EnablePort: '<S295>/Enable'
   */
  /* Outputs for Enabled SubSystem: '<S283>/Hvgw(s)' incorporates:
   *  EnablePort: '<S297>/Enable'
   */
  /* Outputs for Enabled SubSystem: '<S282>/Hqgw' incorporates:
   *  EnablePort: '<S294>/Enable'
   */
  /* Outputs for Enabled SubSystem: '<S283>/Hwgw(s)' incorporates:
   *  EnablePort: '<S298>/Enable'
   */
  /* Outputs for Enabled SubSystem: '<S282>/Hpgw' incorporates:
   *  EnablePort: '<S293>/Enable'
   */
  tmp_0 = rtmIsMajorTimeStep((&pid_control_V1_M));

  /* End of Outputs for SubSystem: '<S282>/Hpgw' */
  /* End of Outputs for SubSystem: '<S283>/Hwgw(s)' */
  /* End of Outputs for SubSystem: '<S282>/Hqgw' */
  /* End of Outputs for SubSystem: '<S283>/Hvgw(s)' */
  /* End of Outputs for SubSystem: '<S282>/Hrgw' */
  /* End of Outputs for SubSystem: '<S283>/Hugw(s)' */
  if (tmp_0) {
    /* MATLABSystem: '<S10>/SourceBlock' */
    serverAvailableOnTime = Sub_pid_control_V1_377.getLatestMessage
      (&pid_control_V1_B.rtb_SourceBlock_o2_p);

    /* Outputs for Enabled SubSystem: '<S10>/Enabled Subsystem' incorporates:
     *  EnablePort: '<S276>/Enable'
     */
    /* Switch: '<Root>/Switch2' incorporates:
     *  MATLABSystem: '<S10>/SourceBlock'
     * */
    if (serverAvailableOnTime) {
      /* SignalConversion generated from: '<S276>/In1' */
      pid_control_V1_B.In1 = pid_control_V1_B.rtb_SourceBlock_o2_p;

      /* Switch: '<Root>/Switch2' */
      pid_control_V1_B.Switch2 = pid_control_V1_B.In1.data;
    } else {
      /* Switch: '<Root>/Switch2' incorporates:
       *  UnitDelay: '<Root>/Unit Delay2'
       */
      pid_control_V1_B.Switch2 = pid_control_V1_DW.UnitDelay2_DSTATE;
    }

    /* End of Switch: '<Root>/Switch2' */
    /* End of Outputs for SubSystem: '<S10>/Enabled Subsystem' */
  }

  /* Sum: '<Root>/Sum5' */
  pid_control_V1_B.Sum5 = pid_control_V1_B.Switch2 - pid_control_V1_B.x[8];

  /* Gain: '<S207>/Filter Coefficient' incorporates:
   *  Gain: '<S197>/Derivative Gain'
   *  Integrator: '<S199>/Filter'
   *  Sum: '<S199>/SumD'
   */
  pid_control_V1_B.FilterCoefficient_p = (-5.0 * pid_control_V1_B.Sum5 -
    pid_control_V1_X.Filter_CSTATE_f) * 100.0;

  /* Sum: '<S213>/Sum' incorporates:
   *  Gain: '<S209>/Proportional Gain'
   *  Integrator: '<S204>/Integrator'
   */
  pid_control_V1_B.Saturation_m = (-15.0 * pid_control_V1_B.Sum5 +
    pid_control_V1_X.Integrator_CSTATE_d) + pid_control_V1_B.FilterCoefficient_p;

  /* Saturate: '<S211>/Saturation' */
  if (pid_control_V1_B.Saturation_m > 0.26179938779914941) {
    /* Sum: '<S213>/Sum' incorporates:
     *  Saturate: '<S211>/Saturation'
     */
    pid_control_V1_B.Saturation_m = 0.26179938779914941;
  } else if (pid_control_V1_B.Saturation_m < -0.26179938779914941) {
    /* Sum: '<S213>/Sum' incorporates:
     *  Saturate: '<S211>/Saturation'
     */
    pid_control_V1_B.Saturation_m = -0.26179938779914941;
  }

  /* End of Saturate: '<S211>/Saturation' */

  /* Gain: '<S101>/Filter Coefficient' incorporates:
   *  Constant: '<Root>/Constant2'
   *  Gain: '<Root>/Gain4'
   *  Integrator: '<S93>/Filter'
   *  Sum: '<Root>/Sum2'
   *  Sum: '<S93>/SumD'
   */
  pid_control_V1_B.FilterCoefficient_j = ((1.0 - (-pid_control_V1_B.x[11])) -
    pid_control_V1_X.Filter_CSTATE_fb) * 100.0;

  /* Sum: '<S107>/Sum' incorporates:
   *  Constant: '<Root>/Constant2'
   *  Gain: '<Root>/Gain4'
   *  Gain: '<S103>/Proportional Gain'
   *  Integrator: '<S98>/Integrator'
   *  Sum: '<Root>/Sum2'
   */
  pid_control_V1_B.Saturation_i = ((1.0 - (-pid_control_V1_B.x[11])) * 2.5 +
    pid_control_V1_X.Integrator_CSTATE_n) + pid_control_V1_B.FilterCoefficient_j;

  /* Saturate: '<Root>/Saturation' */
  if (pid_control_V1_B.Saturation_i > 0.13962634015954636) {
    /* Sum: '<S107>/Sum' incorporates:
     *  Saturate: '<Root>/Saturation'
     */
    pid_control_V1_B.Saturation_i = 0.13962634015954636;
  } else if (pid_control_V1_B.Saturation_i < -0.034906585039886591) {
    /* Sum: '<S107>/Sum' incorporates:
     *  Saturate: '<Root>/Saturation'
     */
    pid_control_V1_B.Saturation_i = -0.034906585039886591;
  }

  /* End of Saturate: '<Root>/Saturation' */

  /* Gain: '<S149>/Integral Gain' incorporates:
   *  Sum: '<Root>/Sum1'
   */
  pid_control_V1_B.Switch = pid_control_V1_B.Saturation_i - pid_control_V1_B.x[7];

  /* Gain: '<S155>/Filter Coefficient' incorporates:
   *  Gain: '<S145>/Derivative Gain'
   *  Integrator: '<S147>/Filter'
   *  Sum: '<S147>/SumD'
   */
  pid_control_V1_B.FilterCoefficient_m = (-12.5 * pid_control_V1_B.Switch -
    pid_control_V1_X.Filter_CSTATE_m) * 100.0;

  /* Sum: '<S161>/Sum' incorporates:
   *  Gain: '<S157>/Proportional Gain'
   *  Integrator: '<S152>/Integrator'
   */
  pid_control_V1_B.SignPreSat_j = (-25.0 * pid_control_V1_B.Switch +
    pid_control_V1_X.Integrator_CSTATE_p) + pid_control_V1_B.FilterCoefficient_m;

  /* Saturate: '<S159>/Saturation' */
  if (pid_control_V1_B.SignPreSat_j > 0.3490658503988659) {
    /* Saturate: '<S159>/Saturation' */
    pid_control_V1_B.Saturation_f = 0.3490658503988659;
  } else if (pid_control_V1_B.SignPreSat_j < -0.3490658503988659) {
    /* Saturate: '<S159>/Saturation' */
    pid_control_V1_B.Saturation_f = -0.3490658503988659;
  } else {
    /* Saturate: '<S159>/Saturation' */
    pid_control_V1_B.Saturation_f = pid_control_V1_B.SignPreSat_j;
  }

  /* End of Saturate: '<S159>/Saturation' */
  if (tmp_0) {
    /* MATLAB Function: '<Root>/MATLAB Function' */
    memset(&pid_control_V1_B.stringOut_l[0], 0, sizeof(uint8_T) << 7U);
    for (pid_control_V1_B.i = 0; pid_control_V1_B.i < 11; pid_control_V1_B.i++)
    {
      pid_control_V1_B.stringOut_l[pid_control_V1_B.i] = b[pid_control_V1_B.i];
    }

    pid_control_V1_B.lengthOut_e = 11U;

    /* End of MATLAB Function: '<Root>/MATLAB Function' */

    /* MATLAB Function: '<Root>/MATLAB Function1' */
    memset(&pid_control_V1_B.stringOut[0], 0, sizeof(uint8_T) << 7U);
    for (pid_control_V1_B.i = 0; pid_control_V1_B.i < 5; pid_control_V1_B.i++) {
      pid_control_V1_B.stringOut[pid_control_V1_B.i] = b_0[pid_control_V1_B.i];
    }

    pid_control_V1_B.lengthOut = 5U;

    /* End of MATLAB Function: '<Root>/MATLAB Function1' */
  }

  /* BusAssignment: '<Root>/Bus Assignment' */
  memset(&pid_control_V1_B.BusAssignment, 0, sizeof
         (SL_Bus_gazebo_msgs_SetEntityStateRequest));

  /* Gain: '<Root>/Gain2' incorporates:
   *  Gain: '<Root>/Gain1'
   *  MATLABSystem: '<Root>/Coordinate Transformation Conversion'
   */
  _mm_storeu_pd(&pid_control_V1_B.wbe_b[0], _mm_div_pd(_mm_set_pd
    (-pid_control_V1_B.x[7], -pid_control_V1_B.x[8]), _mm_set1_pd(2.0)));

  /* MATLABSystem: '<Root>/Coordinate Transformation Conversion' incorporates:
   *  Constant: '<Root>/Constant'
   *  Sum: '<Root>/Sum'
   */
  pid_control_V1_B.wbe_b[2] = (pid_control_V1_B.x[6] + 1.5707963267948966) / 2.0;
  pid_control_V1_B.sina = sin(pid_control_V1_B.wbe_b[0]);
  pid_control_V1_B.sinb = sin(pid_control_V1_B.wbe_b[1]);
  pid_control_V1_B.sinc = sin(pid_control_V1_B.wbe_b[2]);
  pid_control_V1_B.cosa = cos(pid_control_V1_B.wbe_b[0]);
  pid_control_V1_B.cosb = cos(pid_control_V1_B.wbe_b[1]);
  pid_control_V1_B.cosc = cos(pid_control_V1_B.wbe_b[2]);

  /* BusAssignment: '<Root>/Bus Assignment' incorporates:
   *  Gain: '<Root>/Gain3'
   */
  pid_control_V1_B.BusAssignment.state.pose.position.x = pid_control_V1_B.x[9];
  pid_control_V1_B.BusAssignment.state.pose.position.y = -pid_control_V1_B.x[10];
  pid_control_V1_B.BusAssignment.state.pose.position.z = pid_control_V1_B.Gain;

  /* Start for MATLABSystem: '<Root>/Coordinate Transformation Conversion' */
  pid_control_V1_B.Va = pid_control_V1_B.cosa * pid_control_V1_B.cosb;

  /* BusAssignment: '<Root>/Bus Assignment' incorporates:
   *  MATLABSystem: '<Root>/Coordinate Transformation Conversion'
   * */
  pid_control_V1_B.BusAssignment.state.pose.orientation.w =
    pid_control_V1_B.sina * pid_control_V1_B.sinb * pid_control_V1_B.sinc +
    pid_control_V1_B.Va * pid_control_V1_B.cosc;
  pid_control_V1_B.BusAssignment.state.pose.orientation.z = pid_control_V1_B.Va *
    pid_control_V1_B.sinc - pid_control_V1_B.cosc * pid_control_V1_B.sina *
    pid_control_V1_B.sinb;
  pid_control_V1_B.BusAssignment.state.pose.orientation.y =
    pid_control_V1_B.cosa * pid_control_V1_B.cosc * pid_control_V1_B.sinb +
    pid_control_V1_B.cosb * pid_control_V1_B.sina * pid_control_V1_B.sinc;
  pid_control_V1_B.BusAssignment.state.pose.orientation.x =
    pid_control_V1_B.cosb * pid_control_V1_B.cosc * pid_control_V1_B.sina -
    pid_control_V1_B.cosa * pid_control_V1_B.sinb * pid_control_V1_B.sinc;
  memcpy(&pid_control_V1_B.BusAssignment.state.name[0],
         &pid_control_V1_B.stringOut_l[0], sizeof(uint8_T) << 7U);
  memcpy(&pid_control_V1_B.BusAssignment.state.reference_frame[0],
         &pid_control_V1_B.stringOut[0], sizeof(uint8_T) << 7U);
  pid_control_V1_B.BusAssignment.state.name_SL_Info.CurrentLength =
    pid_control_V1_B.lengthOut_e;
  pid_control_V1_B.BusAssignment.state.reference_frame_SL_Info.CurrentLength =
    pid_control_V1_B.lengthOut;

  /* Outputs for Atomic SubSystem: '<Root>/Call Service' */
  /* MATLABSystem: '<S2>/ServiceCaller' */
  serverAvailableOnTime = ServCall_pid_control_V1_326.waitForServer(5.0);
  if (serverAvailableOnTime) {
    ServCall_pid_control_V1_326.call(&pid_control_V1_B.BusAssignment, &tmp);
  }

  /* End of MATLABSystem: '<S2>/ServiceCaller' */
  /* End of Outputs for SubSystem: '<Root>/Call Service' */

  /* Sum: '<S38>/SumI4' incorporates:
   *  Gain: '<S43>/Integral Gain'
   *  Sum: '<S38>/SumI2'
   */
  pid_control_V1_B.SumI4 = (pid_control_V1_B.Saturation_k -
    pid_control_V1_B.Sum_ks) - pid_control_V1_B.Sum4;

  /* Gain: '<S95>/Integral Gain' incorporates:
   *  Constant: '<Root>/Constant2'
   *  Gain: '<Root>/Gain4'
   *  Sum: '<Root>/Sum2'
   */
  pid_control_V1_B.IntegralGain = (1.0 - (-pid_control_V1_B.x[11])) * 0.1;

  /* Gain: '<S142>/ZeroGain' */
  pid_control_V1_B.Sum4 = 0.0 * pid_control_V1_B.SignPreSat_j;

  /* DeadZone: '<S144>/DeadZone' */
  if (pid_control_V1_B.SignPreSat_j > 0.3490658503988659) {
    pid_control_V1_B.SignPreSat_j -= 0.3490658503988659;
  } else if (pid_control_V1_B.SignPreSat_j >= -0.3490658503988659) {
    pid_control_V1_B.SignPreSat_j = 0.0;
  } else {
    pid_control_V1_B.SignPreSat_j -= -0.3490658503988659;
  }

  /* End of DeadZone: '<S144>/DeadZone' */

  /* Gain: '<S149>/Integral Gain' */
  pid_control_V1_B.Switch *= -5.0;

  /* Signum: '<S142>/SignPreSat' */
  if (rtIsNaN(pid_control_V1_B.SignPreSat_j)) {
    /* DataTypeConversion: '<S142>/DataTypeConv1' */
    pid_control_V1_B.i = 0;
  } else {
    if (pid_control_V1_B.SignPreSat_j < 0.0) {
      /* DataTypeConversion: '<S142>/DataTypeConv1' */
      pid_control_V1_B.Va = -1.0;
    } else {
      /* DataTypeConversion: '<S142>/DataTypeConv1' */
      pid_control_V1_B.Va = (pid_control_V1_B.SignPreSat_j > 0.0);
    }

    /* DataTypeConversion: '<S142>/DataTypeConv1' */
    pid_control_V1_B.i = static_cast<int32_T>(fmod(pid_control_V1_B.Va, 256.0));
  }

  /* End of Signum: '<S142>/SignPreSat' */

  /* Signum: '<S142>/SignPreIntegrator' */
  if (rtIsNaN(pid_control_V1_B.Switch)) {
    /* DataTypeConversion: '<S142>/DataTypeConv2' */
    pid_control_V1_B.i_c = 0;
  } else {
    if (pid_control_V1_B.Switch < 0.0) {
      /* DataTypeConversion: '<S142>/DataTypeConv2' */
      pid_control_V1_B.Va = -1.0;
    } else {
      /* DataTypeConversion: '<S142>/DataTypeConv2' */
      pid_control_V1_B.Va = (pid_control_V1_B.Switch > 0.0);
    }

    /* DataTypeConversion: '<S142>/DataTypeConv2' */
    pid_control_V1_B.i_c = static_cast<int32_T>(fmod(pid_control_V1_B.Va, 256.0));
  }

  /* End of Signum: '<S142>/SignPreIntegrator' */

  /* DataTypeConversion: '<S142>/DataTypeConv1' */
  if (pid_control_V1_B.i < 0) {
    pid_control_V1_B.i = static_cast<int8_T>(-static_cast<int8_T>
      (static_cast<uint8_T>(-static_cast<real_T>(pid_control_V1_B.i))));
  }

  /* DataTypeConversion: '<S142>/DataTypeConv2' */
  if (pid_control_V1_B.i_c < 0) {
    pid_control_V1_B.i_c = static_cast<int8_T>(-static_cast<int8_T>(static_cast<
      uint8_T>(-static_cast<real_T>(pid_control_V1_B.i_c))));
  }

  /* Logic: '<S142>/AND3' incorporates:
   *  DataTypeConversion: '<S142>/DataTypeConv1'
   *  DataTypeConversion: '<S142>/DataTypeConv2'
   *  RelationalOperator: '<S142>/Equal1'
   *  RelationalOperator: '<S142>/NotEqual'
   */
  pid_control_V1_B.AND3 = ((pid_control_V1_B.Sum4 !=
    pid_control_V1_B.SignPreSat_j) && (pid_control_V1_B.i ==
    pid_control_V1_B.i_c));
  if (tmp_0) {
    /* Memory: '<S142>/Memory' */
    pid_control_V1_B.Memory_n = pid_control_V1_DW.Memory_PreviousInput_d;
  }

  /* Switch: '<S142>/Switch' */
  if (pid_control_V1_B.Memory_n) {
    /* Gain: '<S149>/Integral Gain' incorporates:
     *  Constant: '<S142>/Constant1'
     *  Switch: '<S142>/Switch'
     */
    pid_control_V1_B.Switch = 0.0;
  }

  /* End of Switch: '<S142>/Switch' */

  /* Gain: '<S201>/Integral Gain' */
  pid_control_V1_B.IntegralGain_a = -0.5 * pid_control_V1_B.Sum5;

  /* Gain: '<S248>/ZeroGain' */
  pid_control_V1_B.Sum5 = 0.0 * pid_control_V1_B.SignPreSat;

  /* DeadZone: '<S250>/DeadZone' */
  if (pid_control_V1_B.SignPreSat > 1.0) {
    pid_control_V1_B.SignPreSat--;
  } else if (pid_control_V1_B.SignPreSat >= 0.0) {
    pid_control_V1_B.SignPreSat = 0.0;
  }

  /* End of DeadZone: '<S250>/DeadZone' */

  /* Gain: '<S255>/Integral Gain' incorporates:
   *  Constant: '<Root>/Constant3'
   *  Sum: '<Root>/Sum3'
   */
  pid_control_V1_B.Switch_j = (28.0 - pid_control_V1_B.x[0]) * 0.01;

  /* Signum: '<S248>/SignPreSat' */
  if (rtIsNaN(pid_control_V1_B.SignPreSat)) {
    /* DataTypeConversion: '<S248>/DataTypeConv1' */
    pid_control_V1_B.i = 0;
  } else {
    if (pid_control_V1_B.SignPreSat < 0.0) {
      /* DataTypeConversion: '<S248>/DataTypeConv1' */
      pid_control_V1_B.Va = -1.0;
    } else {
      /* DataTypeConversion: '<S248>/DataTypeConv1' */
      pid_control_V1_B.Va = (pid_control_V1_B.SignPreSat > 0.0);
    }

    /* DataTypeConversion: '<S248>/DataTypeConv1' */
    pid_control_V1_B.i = static_cast<int32_T>(fmod(pid_control_V1_B.Va, 256.0));
  }

  /* End of Signum: '<S248>/SignPreSat' */

  /* Signum: '<S248>/SignPreIntegrator' */
  if (rtIsNaN(pid_control_V1_B.Switch_j)) {
    /* DataTypeConversion: '<S248>/DataTypeConv2' */
    pid_control_V1_B.i_c = 0;
  } else {
    if (pid_control_V1_B.Switch_j < 0.0) {
      /* DataTypeConversion: '<S248>/DataTypeConv2' */
      pid_control_V1_B.Va = -1.0;
    } else {
      /* DataTypeConversion: '<S248>/DataTypeConv2' */
      pid_control_V1_B.Va = (pid_control_V1_B.Switch_j > 0.0);
    }

    /* DataTypeConversion: '<S248>/DataTypeConv2' */
    pid_control_V1_B.i_c = static_cast<int32_T>(fmod(pid_control_V1_B.Va, 256.0));
  }

  /* End of Signum: '<S248>/SignPreIntegrator' */

  /* DataTypeConversion: '<S248>/DataTypeConv1' */
  if (pid_control_V1_B.i < 0) {
    pid_control_V1_B.i = static_cast<int8_T>(-static_cast<int8_T>
      (static_cast<uint8_T>(-static_cast<real_T>(pid_control_V1_B.i))));
  }

  /* DataTypeConversion: '<S248>/DataTypeConv2' */
  if (pid_control_V1_B.i_c < 0) {
    pid_control_V1_B.i_c = static_cast<int8_T>(-static_cast<int8_T>(static_cast<
      uint8_T>(-static_cast<real_T>(pid_control_V1_B.i_c))));
  }

  /* Logic: '<S248>/AND3' incorporates:
   *  DataTypeConversion: '<S248>/DataTypeConv1'
   *  DataTypeConversion: '<S248>/DataTypeConv2'
   *  RelationalOperator: '<S248>/Equal1'
   *  RelationalOperator: '<S248>/NotEqual'
   */
  pid_control_V1_B.AND3_c = ((pid_control_V1_B.Sum5 !=
    pid_control_V1_B.SignPreSat) && (pid_control_V1_B.i == pid_control_V1_B.i_c));
  if (tmp_0) {
    /* Memory: '<S248>/Memory' */
    pid_control_V1_B.Memory_h = pid_control_V1_DW.Memory_PreviousInput_a;

    /* Memory: '<S11>/Memory' */
    pid_control_V1_B.Memory[0] = pid_control_V1_DW.Memory_PreviousInput[0];

    /* Memory: '<S11>/Memory1' */
    pid_control_V1_B.Memory1[0] = pid_control_V1_DW.Memory1_PreviousInput[0];

    /* Memory: '<S11>/Memory' */
    pid_control_V1_B.Memory[1] = pid_control_V1_DW.Memory_PreviousInput[1];

    /* Memory: '<S11>/Memory1' */
    pid_control_V1_B.Memory1[1] = pid_control_V1_DW.Memory1_PreviousInput[1];

    /* Memory: '<S11>/Memory' */
    pid_control_V1_B.Memory[2] = pid_control_V1_DW.Memory_PreviousInput[2];

    /* Memory: '<S11>/Memory1' */
    pid_control_V1_B.Memory1[2] = pid_control_V1_DW.Memory1_PreviousInput[2];
  }

  /* Switch: '<S248>/Switch' */
  if (pid_control_V1_B.Memory_h) {
    /* Gain: '<S255>/Integral Gain' incorporates:
     *  Constant: '<S248>/Constant1'
     *  Switch: '<S248>/Switch'
     */
    pid_control_V1_B.Switch_j = 0.0;
  }

  /* End of Switch: '<S248>/Switch' */

  /* SignalConversion generated from: '<S278>/ SFunction ' incorporates:
   *  MATLAB Function: '<S11>/MATLAB Function'
   */
  pid_control_V1_B.TmpSignalConversionAtSFunct[0] =
    pid_control_V1_B.Saturation_k;
  pid_control_V1_B.TmpSignalConversionAtSFunct[1] =
    pid_control_V1_B.Saturation_f;
  pid_control_V1_B.TmpSignalConversionAtSFunct[2] =
    pid_control_V1_B.Saturation_m;
  pid_control_V1_B.TmpSignalConversionAtSFunct[3] = pid_control_V1_B.Saturation;
  pid_control_V1_B.TmpSignalConversionAtSFunct[4] = pid_control_V1_B.Saturation;

  /* MATLAB Function: '<S11>/MATLAB Function' incorporates:
   *  Memory: '<S11>/Memory'
   */
  if (pid_control_V1_B.TmpSignalConversionAtSFunct[1] <= 0.3490658503988659) {
    pid_control_V1_B.u2 = pid_control_V1_B.TmpSignalConversionAtSFunct[1];
  } else {
    pid_control_V1_B.u2 = 0.3490658503988659;
  }

  if (!(pid_control_V1_B.u2 >= -0.3490658503988659)) {
    pid_control_V1_B.u2 = -0.3490658503988659;
  }

  tmp_2 = _mm_add_pd(_mm_loadu_pd(&pid_control_V1_B.x[0]), _mm_loadu_pd
                     (&pid_control_V1_B.Memory[0]));
  _mm_storeu_pd(&pid_control_V1_B.dv[0], tmp_2);

  /* MATLAB Function: '<S11>/MATLAB Function' incorporates:
   *  Memory: '<S11>/Memory'
   *  Memory: '<S11>/Memory1'
   */
  pid_control_V1_B.SignPreSat = pid_control_V1_B.x[2] + pid_control_V1_B.Memory
    [2];
  pid_control_V1_B.Va = sqrt((pid_control_V1_B.dv[0] * pid_control_V1_B.dv[0] +
    pid_control_V1_B.dv[1] * pid_control_V1_B.dv[1]) +
    pid_control_V1_B.SignPreSat * pid_control_V1_B.SignPreSat);
  if (pid_control_V1_B.Va == 0.0) {
    pid_control_V1_B.Va = 0.001;
  }

  pid_control_V1_B.SignPreSat = rt_atan2d_snf(pid_control_V1_B.SignPreSat,
    pid_control_V1_B.dv[0]);
  pid_control_V1_B.Sum5 = asin(pid_control_V1_B.dv[1] / pid_control_V1_B.Va);
  if ((-pid_control_V1_B.x[11] - 0.363 <= 0.001) || rtIsNaN(-pid_control_V1_B.x
       [11] - 0.363)) {
    pid_control_V1_B.Sum_ks = 0.001;
  } else {
    pid_control_V1_B.Sum_ks = -pid_control_V1_B.x[11] - 0.363;
  }

  pid_control_V1_B.Q = pid_control_V1_B.Va * pid_control_V1_B.Va * 0.6125;
  pid_control_V1_B.wbe_b[0] = pid_control_V1_B.x[3];
  pid_control_V1_B.wbe_b[1] = pid_control_V1_B.x[4];
  pid_control_V1_B.wbe_b[2] = pid_control_V1_B.x[5];
  pid_control_V1_B.SignPreSat_j = ((pid_control_V1_B.SignPreSat -
    -0.065449846949787352) + 0.043633231299858237) * 4.9604094530365153;
  pid_control_V1_B.Sum4 = ((pid_control_V1_B.SignPreSat - -0.074176493209759012)
    + 0.026179938779914941) * 4.8387748917360032;
  pid_control_V1_B.Cn = pid_control_V1_B.Sum_ks / 5.02;
  pid_control_V1_B.Sum_ks = (pid_control_V1_rt_powd_snf(pid_control_V1_B.Cn,
    0.787) * 288.0 * exp(pid_control_V1_rt_powd_snf(pid_control_V1_B.Cn, 0.327) *
    -9.14) * 0.97986308862072491 / 5.9129476540958859 + 1.0) *
    pid_control_V1_B.SignPreSat_j;
  pid_control_V1_B.cosa = pid_control_V1_B.u2 * pid_control_V1_B.u2;
  pid_control_V1_B.sina = (pid_control_V1_rt_powd_snf(fabs((-pid_control_V1_B.x
    [11] + 0.72) / 2.74), 0.787) * 288.0 * exp(pid_control_V1_rt_powd_snf(fabs((
    -pid_control_V1_B.x[11] + 0.72) / 2.74), 0.327) * -9.14) *
    0.95628590200128227 / 5.35300902982722 + 1.0) * ((pid_control_V1_B.cosa *
    -0.00141 + 0.0307 * pid_control_V1_B.u2) + pid_control_V1_B.Sum4);
  pid_control_V1_B.sinb = (1.0 - exp(pid_control_V1_rt_powd_snf
    (pid_control_V1_B.Cn, 0.686) * -10.1)) * (pid_control_V1_B.SignPreSat_j *
    pid_control_V1_B.SignPreSat_j / 21.205750411731103);
  pid_control_V1_B.sinc = (1.0 - exp(pid_control_V1_rt_powd_snf(fabs
    ((-pid_control_V1_B.x[11] + 0.72) / 2.74), 0.686) * -10.1)) *
    (pid_control_V1_B.Sum4 * pid_control_V1_B.Sum4 / 18.943803701146454);
  pid_control_V1_B.cosa = ((pid_control_V1_B.cosa * -1.08E-5 + 0.000715 *
    pid_control_V1_B.u2) * 1.128 + ((pid_control_V1_B.sinb * 3.334 + 0.1020204)
    + pid_control_V1_B.sinc * 1.128)) * pid_control_V1_B.Q;
  pid_control_V1_B.cosb = (pid_control_V1_B.Sum_ks * 3.334 +
    pid_control_V1_B.sina * 1.128) * pid_control_V1_B.Q;
  pid_control_V1_B.cosc = -0.019 * pid_control_V1_B.Sum5 * 180.0 /
    3.1415926535897931;
  pid_control_V1_B.FA_b_tmp_k = sin(pid_control_V1_B.SignPreSat);
  pid_control_V1_B.FA_b_tmp_c = cos(pid_control_V1_B.SignPreSat);
  pid_control_V1_B.FA_b_tmp[0] = pid_control_V1_B.FA_b_tmp_c;
  pid_control_V1_B.FA_b_tmp[3] = 0.0;
  pid_control_V1_B.FA_b_tmp[6] = -pid_control_V1_B.FA_b_tmp_k;
  pid_control_V1_B.FA_b_tmp[2] = pid_control_V1_B.FA_b_tmp_k;
  pid_control_V1_B.FA_b_tmp[5] = 0.0;
  pid_control_V1_B.FA_b_tmp[8] = pid_control_V1_B.FA_b_tmp_c;
  pid_control_V1_B.Dtot[0] = -pid_control_V1_B.cosa;
  pid_control_V1_B.Dtot[1] = pid_control_V1_B.cosc * pid_control_V1_B.Q * 3.334;
  pid_control_V1_B.Dtot[2] = -pid_control_V1_B.cosb;
  pid_control_V1_B.FA_b_tmp[1] = 0.0;
  pid_control_V1_B.FA_b_tmp[4] = 1.0;
  pid_control_V1_B.FA_b_tmp[7] = 0.0;
  pid_control_V1_B.FA_b_tmp_k = 0.0;
  pid_control_V1_B.FA_b_tmp_c = 0.0;
  pid_control_V1_B.FA_b_b = 0.0;
  for (pid_control_V1_B.i = 0; pid_control_V1_B.i < 3; pid_control_V1_B.i++) {
    tmp_2 = _mm_add_pd(_mm_mul_pd(_mm_loadu_pd(&pid_control_V1_B.FA_b_tmp[3 *
      pid_control_V1_B.i]), _mm_set1_pd(pid_control_V1_B.Dtot[pid_control_V1_B.i])),
                       _mm_set_pd(pid_control_V1_B.FA_b_tmp_c,
      pid_control_V1_B.FA_b_tmp_k));
    _mm_storeu_pd(&pid_control_V1_B.dv[0], tmp_2);
    pid_control_V1_B.FA_b_tmp_k = pid_control_V1_B.dv[0];
    pid_control_V1_B.FA_b_tmp_c = pid_control_V1_B.dv[1];
    pid_control_V1_B.FA_b_b += pid_control_V1_B.FA_b_tmp[3 * pid_control_V1_B.i
      + 2] * pid_control_V1_B.Dtot[pid_control_V1_B.i];
  }

  if (pid_control_V1_B.TmpSignalConversionAtSFunct[0] <= 0.26179938779914941) {
    pid_control_V1_B.Vd1 = pid_control_V1_B.TmpSignalConversionAtSFunct[0];
  } else {
    pid_control_V1_B.Vd1 = 0.26179938779914941;
  }

  if (!(pid_control_V1_B.Vd1 >= -0.3490658503988659)) {
    pid_control_V1_B.Vd1 = -0.3490658503988659;
  }

  pid_control_V1_B.FE1_b_idx_1 = 2.0 * pid_control_V1_B.Va;
  pid_control_V1_B.Cl = ((pid_control_V1_B.Memory1[0] + pid_control_V1_B.x[3]) *
    5.02 / pid_control_V1_B.FE1_b_idx_1 * -2.0 + -0.0005 * pid_control_V1_B.Sum5
    * 180.0 / 3.1415926535897931) + -0.5 * pid_control_V1_B.Vd1;
  pid_control_V1_B.u2 = ((pid_control_V1_B.Memory1[1] + pid_control_V1_B.x[4]) *
    0.646 / pid_control_V1_B.FE1_b_idx_1 * -5.0 + (exp(pid_control_V1_B.Cn *
    -4.0) * -0.05 + -1.14 * pid_control_V1_B.SignPreSat)) + -3.0 *
    pid_control_V1_B.u2;
  if (pid_control_V1_B.TmpSignalConversionAtSFunct[2] <= 0.26179938779914941) {
    pid_control_V1_B.Vd1 = pid_control_V1_B.TmpSignalConversionAtSFunct[2];
  } else {
    pid_control_V1_B.Vd1 = 0.26179938779914941;
  }

  if (!(pid_control_V1_B.Vd1 >= -0.26179938779914941)) {
    pid_control_V1_B.Vd1 = -0.26179938779914941;
  }

  pid_control_V1_B.Cn = ((pid_control_V1_B.Memory1[2] + pid_control_V1_B.x[5]) *
    5.02 / pid_control_V1_B.FE1_b_idx_1 * -1.5 + -0.002 * pid_control_V1_B.Sum5 *
    180.0 / 3.1415926535897931) + -0.3 * pid_control_V1_B.Vd1;
  if (pid_control_V1_B.TmpSignalConversionAtSFunct[3] <= 0.000125) {
    pid_control_V1_B.Vd1 = pid_control_V1_B.TmpSignalConversionAtSFunct[3];
  } else {
    pid_control_V1_B.Vd1 = 0.000125;
  }

  if (!(pid_control_V1_B.Vd1 >= 0.0)) {
    pid_control_V1_B.Vd1 = 0.0;
  }

  pid_control_V1_B.Vd1 = pid_control_V1_B.Vd1 / 0.000125 * (37.42 -
    pid_control_V1_B.Va) + pid_control_V1_B.Va;
  pid_control_V1_B.Tp1 = 0.11056515539994617 * pid_control_V1_B.Vd1 *
    (pid_control_V1_B.Vd1 - pid_control_V1_B.Va);
  if (pid_control_V1_B.TmpSignalConversionAtSFunct[4] <= 0.000125) {
    pid_control_V1_B.Vd1 = pid_control_V1_B.TmpSignalConversionAtSFunct[4];
  } else {
    pid_control_V1_B.Vd1 = 0.000125;
  }

  if (!(pid_control_V1_B.Vd1 >= 0.0)) {
    pid_control_V1_B.Vd1 = 0.0;
  }

  pid_control_V1_B.Vd1 = pid_control_V1_B.Vd1 / 0.000125 * (37.42 -
    pid_control_V1_B.Va) + pid_control_V1_B.Va;
  pid_control_V1_B.Tp2 = 0.11056515539994617 * pid_control_V1_B.Vd1 *
    (pid_control_V1_B.Vd1 - pid_control_V1_B.Va);
  pid_control_V1_B.Vd1 = pid_control_V1_B.Tp1 * 0.99619469809174555;
  pid_control_V1_B.FE1_b_idx_1 = 0.0;
  pid_control_V1_B.Tp1 *= 0.087155742747658166;
  pid_control_V1_B.FE2_b_idx_0 = pid_control_V1_B.Tp2 * 0.99619469809174555;
  pid_control_V1_B.FE2_b_idx_2 = pid_control_V1_B.Tp2 * 0.087155742747658166;
  pid_control_V1_B.Tp2 = -9.81 * sin(pid_control_V1_B.x[7]) * 112.0;
  _mm_storeu_pd(&pid_control_V1_B.dv[0], _mm_mul_pd(_mm_mul_pd(_mm_mul_pd
    (_mm_set1_pd(9.81), _mm_set1_pd(cos(pid_control_V1_B.x[7]))), _mm_set_pd(cos
    (pid_control_V1_B.x[6]), sin(pid_control_V1_B.x[6]))), _mm_set1_pd(112.0)));
  pid_control_V1_B.Fg_b_idx_1 = pid_control_V1_B.dv[0];
  pid_control_V1_B.Fg_b_idx_2 = pid_control_V1_B.dv[1];
  pid_control_V1_B.Mcg_b_idx_2 = 5.02 * pid_control_V1_B.Q * 3.334;
  pid_control_V1_B.Mcg_b_idx_0 = (0.6 * pid_control_V1_B.Tp1 + -0.6 *
    pid_control_V1_B.FE2_b_idx_2) + pid_control_V1_B.Mcg_b_idx_2 *
    pid_control_V1_B.Cl;
  pid_control_V1_B.Q = 0.646 * pid_control_V1_B.Q * 3.334 * pid_control_V1_B.u2
    + ((-0.285 * pid_control_V1_B.Vd1 - 0.519 * pid_control_V1_B.Tp1) + (-0.285 *
        pid_control_V1_B.FE2_b_idx_0 - 0.519 * pid_control_V1_B.FE2_b_idx_2));
  pid_control_V1_B.Mcg_b_idx_2 = ((0.0 - 0.6 * pid_control_V1_B.Vd1) + (0.0 -
    -0.6 * pid_control_V1_B.FE2_b_idx_0)) + pid_control_V1_B.Mcg_b_idx_2 *
    pid_control_V1_B.Cn;
  pid_control_V1_B.FA_b_tmp[0] = cos(pid_control_V1_B.x[7]) * cos
    (pid_control_V1_B.x[8]);
  tmp_2 = _mm_set_pd(1.0, -1.0);

  /* MATLAB Function: '<S11>/MATLAB Function' */
  _mm_storeu_pd(&pid_control_V1_B.dv[0], _mm_add_pd(_mm_mul_pd(_mm_mul_pd
    (_mm_set_pd(cos(pid_control_V1_B.x[6]), sin(pid_control_V1_B.x[6])),
     _mm_set1_pd(sin(pid_control_V1_B.x[7]))), _mm_set1_pd(cos
    (pid_control_V1_B.x[8]))), _mm_mul_pd(_mm_mul_pd(_mm_set_pd(sin
    (pid_control_V1_B.x[6]), cos(pid_control_V1_B.x[6])), _mm_set1_pd(sin
    (pid_control_V1_B.x[8]))), tmp_2)));
  pid_control_V1_B.FA_b_tmp[3] = pid_control_V1_B.dv[0];
  pid_control_V1_B.FA_b_tmp[6] = pid_control_V1_B.dv[1];
  pid_control_V1_B.FA_b_tmp[1] = cos(pid_control_V1_B.x[7]) * sin
    (pid_control_V1_B.x[8]);
  tmp_3 = _mm_set_pd(-1.0, 1.0);

  /* MATLAB Function: '<S11>/MATLAB Function' */
  _mm_storeu_pd(&pid_control_V1_B.dv[0], _mm_add_pd(_mm_mul_pd(_mm_mul_pd
    (_mm_set_pd(cos(pid_control_V1_B.x[6]), sin(pid_control_V1_B.x[6])),
     _mm_set1_pd(sin(pid_control_V1_B.x[7]))), _mm_set1_pd(sin
    (pid_control_V1_B.x[8]))), _mm_mul_pd(_mm_mul_pd(_mm_set_pd(sin
    (pid_control_V1_B.x[6]), cos(pid_control_V1_B.x[6])), _mm_set1_pd(cos
    (pid_control_V1_B.x[8]))), tmp_3)));
  pid_control_V1_B.FA_b_tmp[4] = pid_control_V1_B.dv[0];
  pid_control_V1_B.FA_b_tmp[7] = pid_control_V1_B.dv[1];
  pid_control_V1_B.FA_b_tmp[2] = -sin(pid_control_V1_B.x[7]);
  _mm_storeu_pd(&pid_control_V1_B.dv[0], _mm_mul_pd(_mm_set_pd(cos
    (pid_control_V1_B.x[6]), sin(pid_control_V1_B.x[6])), _mm_set1_pd(cos
    (pid_control_V1_B.x[7]))));
  pid_control_V1_B.FA_b_tmp[5] = pid_control_V1_B.dv[0];
  pid_control_V1_B.FA_b_tmp[8] = pid_control_V1_B.dv[1];
  pid_control_V1_B.Dtot[0] = pid_control_V1_B.x[0];
  pid_control_V1_B.Dtot[1] = pid_control_V1_B.x[1];
  pid_control_V1_B.Dtot[2] = pid_control_V1_B.x[2];
  pid_control_V1_B.FE2_b_idx_0 += pid_control_V1_B.Vd1;
  pid_control_V1_B.FE_b_idx_0 = pid_control_V1_B.FE2_b_idx_0;
  pid_control_V1_B.FA_b[0] = (pid_control_V1_B.Tp2 +
    pid_control_V1_B.FE2_b_idx_0) + pid_control_V1_B.FA_b_tmp_k;
  pid_control_V1_B.Vd1 = 0.0;
  pid_control_V1_B.FA_b[1] = pid_control_V1_B.Fg_b_idx_1 +
    pid_control_V1_B.FA_b_tmp_c;
  pid_control_V1_B.FE2_b_idx_0 = pid_control_V1_B.Tp1 +
    pid_control_V1_B.FE2_b_idx_2;
  pid_control_V1_B.FA_b[2] = (pid_control_V1_B.Fg_b_idx_2 +
    pid_control_V1_B.FE2_b_idx_0) + pid_control_V1_B.FA_b_b;
  pid_control_V1_B.Tp1 = 0.0;
  for (pid_control_V1_B.i = 0; pid_control_V1_B.i < 3; pid_control_V1_B.i++) {
    tmp_1 = _mm_add_pd(_mm_mul_pd(_mm_loadu_pd(&pid_control_V1_B.FA_b_tmp[3 *
      pid_control_V1_B.i]), _mm_set1_pd(pid_control_V1_B.Dtot[pid_control_V1_B.i])),
                       _mm_set_pd(pid_control_V1_B.FE1_b_idx_1,
      pid_control_V1_B.Vd1));
    _mm_storeu_pd(&pid_control_V1_B.dv[0], tmp_1);
    pid_control_V1_B.Vd1 = pid_control_V1_B.dv[0];
    pid_control_V1_B.FE1_b_idx_1 = pid_control_V1_B.dv[1];
    pid_control_V1_B.Tp1 += pid_control_V1_B.FA_b_tmp[3 * pid_control_V1_B.i + 2]
      * pid_control_V1_B.Dtot[pid_control_V1_B.i];
  }

  tmp_1 = _mm_sub_pd(_mm_mul_pd(_mm_set_pd(pid_control_V1_B.x[0],
    pid_control_V1_B.x[2]), _mm_loadu_pd(&pid_control_V1_B.x[4])), _mm_mul_pd
                     (_mm_loadu_pd(&pid_control_V1_B.x[1]), _mm_set_pd
                      (pid_control_V1_B.x[3], pid_control_V1_B.x[5])));
  _mm_storeu_pd(&pid_control_V1_B.Dtot[0], tmp_1);
  pid_control_V1_B.Dtot[2] = pid_control_V1_B.x[1] * pid_control_V1_B.x[3] -
    pid_control_V1_B.x[0] * pid_control_V1_B.x[4];
  pid_control_V1_B.FA_b_tmp[0] = 1.0;
  _mm_storeu_pd(&pid_control_V1_B.dv[0], _mm_mul_pd(_mm_set_pd(cos
    (pid_control_V1_B.x[6]), sin(pid_control_V1_B.x[6])), _mm_set1_pd(tan
    (pid_control_V1_B.x[7]))));
  pid_control_V1_B.FA_b_tmp[3] = pid_control_V1_B.dv[0];
  pid_control_V1_B.FA_b_tmp[6] = pid_control_V1_B.dv[1];
  pid_control_V1_B.FA_b_tmp[1] = 0.0;
  pid_control_V1_B.FA_b_tmp[4] = cos(pid_control_V1_B.x[6]);
  pid_control_V1_B.FA_b_tmp[7] = -sin(pid_control_V1_B.x[6]);
  pid_control_V1_B.FA_b_tmp[2] = 0.0;
  _mm_storeu_pd(&pid_control_V1_B.dv[0], _mm_div_pd(_mm_set_pd(cos
    (pid_control_V1_B.x[6]), sin(pid_control_V1_B.x[6])), _mm_set1_pd(cos
    (pid_control_V1_B.x[7]))));
  pid_control_V1_B.FA_b_tmp[5] = pid_control_V1_B.dv[0];
  pid_control_V1_B.FA_b_tmp[8] = pid_control_V1_B.dv[1];
  pid_control_V1_B.FE2_b_idx_2 = 0.0;
  pid_control_V1_B.unnamed_idx_1 = 0.0;
  pid_control_V1_B.unnamed_idx_2 = 0.0;
  for (pid_control_V1_B.i = 0; pid_control_V1_B.i < 3; pid_control_V1_B.i++) {
    tmp_1 = _mm_add_pd(_mm_mul_pd(_mm_loadu_pd(&pid_control_V1_B.FA_b_tmp[3 *
      pid_control_V1_B.i]), _mm_set1_pd
      (pid_control_V1_B.wbe_b[pid_control_V1_B.i])), _mm_set_pd
                       (pid_control_V1_B.unnamed_idx_1,
                        pid_control_V1_B.FE2_b_idx_2));
    _mm_storeu_pd(&pid_control_V1_B.dv[0], tmp_1);
    pid_control_V1_B.FE2_b_idx_2 = pid_control_V1_B.dv[0];
    pid_control_V1_B.unnamed_idx_1 = pid_control_V1_B.dv[1];
    _mm_storeu_pd(&pid_control_V1_B.dv[0], _mm_add_pd(_mm_mul_pd(_mm_set_pd
      (0.0089285714285714281, pid_control_V1_B.FA_b_tmp[3 * pid_control_V1_B.i +
       2]), _mm_set_pd(pid_control_V1_B.FA_b[pid_control_V1_B.i],
                       pid_control_V1_B.wbe_b[pid_control_V1_B.i])), _mm_mul_pd
      (_mm_set_pd(pid_control_V1_B.Dtot[pid_control_V1_B.i],
                  pid_control_V1_B.unnamed_idx_2), tmp_3)));
    pid_control_V1_B.unnamed_idx_2 = pid_control_V1_B.dv[0];
    pid_control_V1_B.XDOT[pid_control_V1_B.i] = pid_control_V1_B.dv[1];
  }

  pid_control_V1_B.XDOT[3] = ((114.39 * pid_control_V1_B.Mcg_b_idx_0 + 8.97 *
    pid_control_V1_B.Mcg_b_idx_2) - (-615.2523000000001 * pid_control_V1_B.x[3]
    + 3384.0440999999996 * pid_control_V1_B.x[5]) * pid_control_V1_B.x[4]) /
    4461.966;
  pid_control_V1_B.XDOT[4] = ((pid_control_V1_B.Q - -74.68 * pid_control_V1_B.x
    [3] * pid_control_V1_B.x[5]) - (pid_control_V1_B.x[3] * pid_control_V1_B.x[3]
    - pid_control_V1_B.x[5] * pid_control_V1_B.x[5]) * 8.97) / 85.51;
  pid_control_V1_B.XDOT[5] = ((-615.2523000000001 * pid_control_V1_B.x[5] +
    -1738.2571000000003 * pid_control_V1_B.x[3]) * pid_control_V1_B.x[4] + (8.97
    * pid_control_V1_B.Mcg_b_idx_0 + 39.71 * pid_control_V1_B.Mcg_b_idx_2)) /
    4461.966;
  pid_control_V1_B.XDOT[9] = pid_control_V1_B.Vd1;
  pid_control_V1_B.XDOT[10] = pid_control_V1_B.FE1_b_idx_1;
  pid_control_V1_B.XDOT[11] = pid_control_V1_B.Tp1;
  pid_control_V1_B.XDOT[12] = pid_control_V1_B.cosb / pid_control_V1_B.cosa;
  pid_control_V1_B.XDOT[19] = pid_control_V1_B.cosc;
  pid_control_V1_B.XDOT[20] = pid_control_V1_B.Cl;
  pid_control_V1_B.XDOT[21] = pid_control_V1_B.u2;
  pid_control_V1_B.XDOT[22] = pid_control_V1_B.Cn;
  pid_control_V1_B.XDOT[23] = pid_control_V1_B.SignPreSat;
  pid_control_V1_B.XDOT[24] = pid_control_V1_B.Sum5;
  pid_control_V1_B.XDOT[25] = pid_control_V1_B.SignPreSat_j;
  pid_control_V1_B.XDOT[26] = pid_control_V1_B.Sum4;
  pid_control_V1_B.XDOT[27] = pid_control_V1_B.Sum_ks;
  pid_control_V1_B.XDOT[28] = pid_control_V1_B.sina;
  pid_control_V1_B.XDOT[29] = pid_control_V1_B.sinb;
  pid_control_V1_B.XDOT[30] = pid_control_V1_B.sinc;
  pid_control_V1_B.XDOT[6] = pid_control_V1_B.FE2_b_idx_2;
  pid_control_V1_B.XDOT[13] = pid_control_V1_B.FA_b[0];
  pid_control_V1_B.XDOT[16] = pid_control_V1_B.Mcg_b_idx_0;
  pid_control_V1_B.XDOT[31] = pid_control_V1_B.Tp2;
  pid_control_V1_B.XDOT[34] = pid_control_V1_B.FE_b_idx_0;
  pid_control_V1_B.XDOT[37] = pid_control_V1_B.FA_b_tmp_k;
  pid_control_V1_B.XDOT[7] = pid_control_V1_B.unnamed_idx_1;
  pid_control_V1_B.XDOT[14] = pid_control_V1_B.FA_b[1];
  pid_control_V1_B.XDOT[17] = pid_control_V1_B.Q;
  pid_control_V1_B.XDOT[32] = pid_control_V1_B.Fg_b_idx_1;
  pid_control_V1_B.XDOT[35] = 0.0;
  pid_control_V1_B.XDOT[38] = pid_control_V1_B.FA_b_tmp_c;
  pid_control_V1_B.XDOT[8] = pid_control_V1_B.unnamed_idx_2;
  pid_control_V1_B.XDOT[15] = pid_control_V1_B.FA_b[2];
  pid_control_V1_B.XDOT[18] = pid_control_V1_B.Mcg_b_idx_2;
  pid_control_V1_B.XDOT[33] = pid_control_V1_B.Fg_b_idx_2;
  pid_control_V1_B.XDOT[36] = pid_control_V1_B.FE2_b_idx_0;
  pid_control_V1_B.XDOT[39] = pid_control_V1_B.FA_b_b;

  /* Math: '<S11>/Square2' */
  pid_control_V1_B.Sum5 = pid_control_V1_B.x[1] * pid_control_V1_B.x[1];

  /* Product: '<S11>/Product2' incorporates:
   *  Math: '<S11>/Square'
   *  Math: '<S11>/Square1'
   *  Sqrt: '<S11>/Sqrt'
   *  Sum: '<S11>/Sum2'
   */
  pid_control_V1_B.Power = sqrt((pid_control_V1_B.x[0] * pid_control_V1_B.x[0] +
    pid_control_V1_B.Sum5) + pid_control_V1_B.x[2] * pid_control_V1_B.x[2]) *
    pid_control_V1_B.XDOT[34];

  /* Gain: '<S11>/Gain3' */
  pid_control_V1_B.Gain3 = 0.001 * pid_control_V1_B.Power;
  if (tmp_0) {
  }

  /* Gain: '<S11>/Gain1' incorporates:
   *  Integrator: '<S11>/Integrator1'
   */
  pid_control_V1_B.EnergykWh = 2.7777777777777776E-7 *
    pid_control_V1_X.Integrator1_CSTATE;
  if (tmp_0) {
    /* SignalConversion generated from: '<S11>/To Workspace1' */
    pid_control_V1_B.TmpSignalConversionAtSFunct[0] =
      pid_control_V1_B.Saturation_k;
    pid_control_V1_B.TmpSignalConversionAtSFunct[1] =
      pid_control_V1_B.Saturation_f;
    pid_control_V1_B.TmpSignalConversionAtSFunct[2] =
      pid_control_V1_B.Saturation_m;
    pid_control_V1_B.TmpSignalConversionAtSFunct[3] =
      pid_control_V1_B.Saturation;
    pid_control_V1_B.TmpSignalConversionAtSFunct[4] =
      pid_control_V1_B.Saturation;
  }

  /* Product: '<S11>/Divide' incorporates:
   *  Constant: '<S11>/thrust efficiency Cp?'
   */
  pid_control_V1_B.powerdemand = pid_control_V1_B.Gain3 / 0.248;
  if (tmp_0) {
  }

  /* Product: '<S11>/Divide1' */
  pid_control_V1_B.loadtorque = pid_control_V1_B.powerdemand /
    pid_control_V1_ConstB.motorspeed;
  if (tmp_0) {
  }

  /* UnitConversion: '<S284>/Unit Conversion' incorporates:
   *  Gain: '<S11>/Gain4'
   */
  /* Unit Conversion - from: m to: ft
     Expression: output = (3.28084*input) + (0) */
  pid_control_V1_B.SignPreSat = 3.280839895013123 * -pid_control_V1_B.x[11];

  /* Saturate: '<S317>/Limit Function 10ft to 1000ft' */
  if (pid_control_V1_B.SignPreSat > 1000.0) {
    pid_control_V1_B.Sum_ks = 1000.0;
  } else if (pid_control_V1_B.SignPreSat < 10.0) {
    pid_control_V1_B.Sum_ks = 10.0;
  } else {
    pid_control_V1_B.Sum_ks = pid_control_V1_B.SignPreSat;
  }

  /* End of Saturate: '<S317>/Limit Function 10ft to 1000ft' */

  /* Gain: '<S289>/Lw' */
  pid_control_V1_B.SignPreSat_j = pid_control_V1_ConstB.UnitConversion_c;

  /* Interpolation_n-D: '<S299>/Medium//High Altitude Intensity' incorporates:
   *  PreLookup: '<S299>/PreLook-Up Index Search  (altitude)'
   */
  pid_control_V1_B.bpIndex[0] = plook_bincpa(pid_control_V1_B.SignPreSat,
    pid_control_V1_ConstP.PreLookUpIndexSearchaltitude_Br, 11U,
    &pid_control_V1_B.Sum5, &pid_control_V1_DW.PreLookUpIndexSearchaltitude_DW);
  pid_control_V1_B.frac[0] = pid_control_V1_B.Sum5;
  pid_control_V1_B.frac[1] = pid_control_V1_ConstB.PreLookUpIndexSearchprobofe;
  pid_control_V1_B.bpIndex[1] =
    pid_control_V1_ConstB.PreLookUpIndexSearchprobo_g;
  pid_control_V1_B.Sum5 = intrp2d_la_pw(pid_control_V1_B.bpIndex,
    pid_control_V1_B.frac, pid_control_V1_ConstP.MediumHighAltitudeIntensity_Tab,
    12U, pid_control_V1_ConstP.MediumHighAltitudeIntensity_max);

  /* UnitConversion: '<S290>/Unit Conversion' incorporates:
   *  MATLAB Function: '<S11>/MATLAB Function'
   */
  /* Unit Conversion - from: m/s to: ft/s
     Expression: output = (3.28084*input) + (0) */
  pid_control_V1_B.Sum4 = 3.280839895013123 * pid_control_V1_B.Va;

  /* Outputs for Enabled SubSystem: '<S282>/Hpgw' incorporates:
   *  EnablePort: '<S293>/Enable'
   */
  if (tmp_0) {
    /* Product: '<S292>/Divide' incorporates:
     *  Product: '<S292>/Product'
     *  RandomNumber: '<S292>/White Noise'
     */
    tmp_3 = _mm_mul_pd(_mm_loadu_pd(&pid_control_V1_ConstB.Divide[0]),
                       _mm_loadu_pd(&pid_control_V1_DW.NextOutput[0]));

    /* RandomNumber: '<S292>/White Noise' incorporates:
     *  Product: '<S292>/Product'
     */
    _mm_storeu_pd(&pid_control_V1_B.Product[0], tmp_3);

    /* Product: '<S292>/Divide' incorporates:
     *  Product: '<S292>/Product'
     *  RandomNumber: '<S292>/White Noise'
     */
    tmp_3 = _mm_mul_pd(_mm_loadu_pd(&pid_control_V1_ConstB.Divide[2]),
                       _mm_loadu_pd(&pid_control_V1_DW.NextOutput[2]));

    /* RandomNumber: '<S292>/White Noise' incorporates:
     *  Product: '<S292>/Product'
     */
    _mm_storeu_pd(&pid_control_V1_B.Product[2], tmp_3);
    if (rtsiIsModeUpdateTimeStep(&(&pid_control_V1_M)->solverInfo) &&
        (!pid_control_V1_DW.Hpgw_MODE)) {
      (void) memset(&(pid_control_V1_XDis.pgw_p_CSTATE), 0,
                    2*sizeof(boolean_T));

      /* InitializeConditions for Integrator: '<S293>/pgw_p' */
      pid_control_V1_X.pgw_p_CSTATE[0] = 0.0;
      pid_control_V1_X.pgw_p_CSTATE[1] = 0.0;
      pid_control_V1_DW.Hpgw_MODE = true;
    }
  }

  if (pid_control_V1_DW.Hpgw_MODE) {
    /* Fcn: '<S293>/sqrt(0.8//V)' */
    pid_control_V1_B.Va = sqrt(0.8 / pid_control_V1_B.Sum4);

    /* Product: '<S293>/w3' */
    pid_control_V1_B.sina = pid_control_V1_B.Sum4 * pid_control_V1_ConstB.w4;

    /* Product: '<S293>/w' incorporates:
     *  Fcn: '<S293>/sqrt(0.8//V)'
     *  Gain: '<S289>/Lw'
     *  Integrator: '<S293>/pgw_p'
     *  Math: '<S293>/L^1//3'
     *  Product: '<S293>/Lug//V1'
     *  Product: '<S293>/w1'
     *  Product: '<S293>/w2'
     *  Sum: '<S293>/Sum'
     */
    pid_control_V1_B.w_o[0] = (pid_control_V1_B.Va / pid_control_V1_rt_powd_snf
      (pid_control_V1_B.Sum_ks, 0.33333333333333331) * pid_control_V1_ConstB.u16
      * pid_control_V1_B.Product[3] - pid_control_V1_X.pgw_p_CSTATE[0]) *
      pid_control_V1_B.sina;

    /* Math: '<S293>/L^1//3' */
    if (pid_control_V1_B.SignPreSat_j < 0.0) {
      pid_control_V1_B.sinb = -pid_control_V1_rt_powd_snf
        (-pid_control_V1_B.SignPreSat_j, 0.33333333333333331);
    } else {
      pid_control_V1_B.sinb = pid_control_V1_rt_powd_snf
        (pid_control_V1_B.SignPreSat_j, 0.33333333333333331);
    }

    /* Product: '<S293>/w' incorporates:
     *  Fcn: '<S293>/sqrt(0.8//V)'
     *  Integrator: '<S293>/pgw_p'
     *  Math: '<S293>/L^1//3'
     *  Product: '<S293>/Lug//V1'
     *  Product: '<S293>/w1'
     *  Product: '<S293>/w2'
     *  Sum: '<S293>/Sum'
     */
    pid_control_V1_B.w_o[1] = (pid_control_V1_B.Va / pid_control_V1_B.sinb *
      pid_control_V1_ConstB.u16 * pid_control_V1_B.Product[3] -
      pid_control_V1_X.pgw_p_CSTATE[1]) * pid_control_V1_B.sina;

    /* Product: '<S293>/sigma_w' incorporates:
     *  Integrator: '<S293>/pgw_p'
     */
    tmp_3 = _mm_mul_pd(_mm_set_pd(pid_control_V1_B.Sum5,
      pid_control_V1_ConstB.sigma_wg), _mm_loadu_pd
                       (&pid_control_V1_X.pgw_p_CSTATE[0]));

    /* Product: '<S293>/sigma_w' */
    _mm_storeu_pd(&pid_control_V1_B.sigma_w[0], tmp_3);
  }

  /* End of Outputs for SubSystem: '<S282>/Hpgw' */

  /* Outputs for Enabled SubSystem: '<S283>/Hwgw(s)' incorporates:
   *  EnablePort: '<S298>/Enable'
   */
  if (tmp_0 && rtsiIsModeUpdateTimeStep(&(&pid_control_V1_M)->solverInfo) &&
      (!pid_control_V1_DW.Hwgws_MODE)) {
    (void) memset(&(pid_control_V1_XDis.wg_p1_CSTATE), 0,
                  4*sizeof(boolean_T));

    /* InitializeConditions for Integrator: '<S298>/wg_p1' */
    pid_control_V1_X.wg_p1_CSTATE[0] = 0.0;

    /* InitializeConditions for Integrator: '<S298>/wg_p2' */
    pid_control_V1_X.wg_p2_CSTATE[0] = 0.0;

    /* InitializeConditions for Integrator: '<S298>/wg_p1' */
    pid_control_V1_X.wg_p1_CSTATE[1] = 0.0;

    /* InitializeConditions for Integrator: '<S298>/wg_p2' */
    pid_control_V1_X.wg_p2_CSTATE[1] = 0.0;
    pid_control_V1_DW.Hwgws_MODE = true;
  }

  if (pid_control_V1_DW.Hwgws_MODE) {
    /* Product: '<S298>/Lwg//V' incorporates:
     *  Gain: '<S289>/Lw'
     */
    pid_control_V1_B.Va = pid_control_V1_B.Sum_ks / pid_control_V1_B.Sum4;

    /* Product: '<S298>/w' incorporates:
     *  Gain: '<S298>/1//pi'
     *  Integrator: '<S298>/wg_p1'
     *  Product: '<S298>/Lug//V1'
     *  Sqrt: '<S298>/sqrt1'
     *  Sum: '<S298>/Sum'
     */
    pid_control_V1_B.sina = (sqrt(0.31830988618379069 * pid_control_V1_B.Va) *
      pid_control_V1_B.Product[2] - pid_control_V1_X.wg_p1_CSTATE[0]) /
      pid_control_V1_B.Va;
    pid_control_V1_B.w[0] = pid_control_V1_B.sina;

    /* Product: '<S298>/w ' incorporates:
     *  Integrator: '<S298>/wg_p1'
     *  Integrator: '<S298>/wg_p2'
     *  Product: '<S298>/Lwg//V '
     *  Sum: '<S298>/Sum1'
     */
    pid_control_V1_B.w_a[0] = (pid_control_V1_B.sina *
      pid_control_V1_ConstB.sqrt_a * pid_control_V1_B.Va +
      (pid_control_V1_X.wg_p1_CSTATE[0] - pid_control_V1_X.wg_p2_CSTATE[0])) /
      pid_control_V1_B.Va;

    /* Product: '<S298>/Lwg//V' */
    pid_control_V1_B.Va = pid_control_V1_B.SignPreSat_j / pid_control_V1_B.Sum4;

    /* Product: '<S298>/w' incorporates:
     *  Gain: '<S298>/1//pi'
     *  Integrator: '<S298>/wg_p1'
     *  Product: '<S298>/Lug//V1'
     *  Sqrt: '<S298>/sqrt1'
     *  Sum: '<S298>/Sum'
     */
    pid_control_V1_B.sina = (sqrt(0.31830988618379069 * pid_control_V1_B.Va) *
      pid_control_V1_B.Product[2] - pid_control_V1_X.wg_p1_CSTATE[1]) /
      pid_control_V1_B.Va;
    pid_control_V1_B.w[1] = pid_control_V1_B.sina;

    /* Product: '<S298>/w ' incorporates:
     *  Integrator: '<S298>/wg_p1'
     *  Integrator: '<S298>/wg_p2'
     *  Product: '<S298>/Lwg//V '
     *  Sum: '<S298>/Sum1'
     */
    pid_control_V1_B.w_a[1] = (pid_control_V1_B.sina *
      pid_control_V1_ConstB.sqrt_a * pid_control_V1_B.Va +
      (pid_control_V1_X.wg_p1_CSTATE[1] - pid_control_V1_X.wg_p2_CSTATE[1])) /
      pid_control_V1_B.Va;

    /* Product: '<S298>/Lwg//V 1' incorporates:
     *  Integrator: '<S298>/wg_p2'
     */
    tmp_3 = _mm_mul_pd(_mm_set_pd(pid_control_V1_B.Sum5,
      pid_control_V1_ConstB.sigma_wg), _mm_loadu_pd
                       (&pid_control_V1_X.wg_p2_CSTATE[0]));

    /* Product: '<S298>/Lwg//V 1' */
    _mm_storeu_pd(&pid_control_V1_B.LwgV1[0], tmp_3);
  }

  /* End of Outputs for SubSystem: '<S283>/Hwgw(s)' */

  /* Outputs for Enabled SubSystem: '<S282>/Hqgw' incorporates:
   *  EnablePort: '<S294>/Enable'
   */
  if (tmp_0 && rtsiIsModeUpdateTimeStep(&(&pid_control_V1_M)->solverInfo) &&
      (!pid_control_V1_DW.Hqgw_MODE)) {
    (void) memset(&(pid_control_V1_XDis.qgw_p_CSTATE), 0,
                  2*sizeof(boolean_T));

    /* InitializeConditions for Integrator: '<S294>/qgw_p' */
    pid_control_V1_X.qgw_p_CSTATE[0] = 0.0;
    pid_control_V1_X.qgw_p_CSTATE[1] = 0.0;
    pid_control_V1_DW.Hqgw_MODE = true;
  }

  if (pid_control_V1_DW.Hqgw_MODE) {
    /* Gain: '<S294>/pi//4' */
    pid_control_V1_B.Va = 0.78539816339744828 * pid_control_V1_B.Sum4;

    /* Product: '<S294>/w' incorporates:
     *  Integrator: '<S294>/qgw_p'
     *  Product: '<S294>/wg//V'
     *  Sum: '<S294>/Sum'
     */
    pid_control_V1_B.SignPreSat_j = (pid_control_V1_B.LwgV1[0] /
      pid_control_V1_B.Sum4 - pid_control_V1_X.qgw_p_CSTATE[0]) *
      (pid_control_V1_B.Va / pid_control_V1_ConstB.UnitConversion_n);
    pid_control_V1_B.w_e0[0] = pid_control_V1_B.SignPreSat_j;

    /* UnaryMinus: '<S294>/Unary Minus' */
    pid_control_V1_B.UnaryMinus[0] = -pid_control_V1_B.SignPreSat_j;

    /* Product: '<S294>/w' incorporates:
     *  Integrator: '<S294>/qgw_p'
     *  Product: '<S294>/wg//V'
     *  Sum: '<S294>/Sum'
     */
    pid_control_V1_B.SignPreSat_j = (pid_control_V1_B.LwgV1[1] /
      pid_control_V1_B.Sum4 - pid_control_V1_X.qgw_p_CSTATE[1]) *
      (pid_control_V1_B.Va / pid_control_V1_ConstB.UnitConversion_n);
    pid_control_V1_B.w_e0[1] = pid_control_V1_B.SignPreSat_j;

    /* UnaryMinus: '<S294>/Unary Minus' */
    pid_control_V1_B.UnaryMinus[1] = -pid_control_V1_B.SignPreSat_j;
  }

  /* End of Outputs for SubSystem: '<S282>/Hqgw' */

  /* Saturate: '<S300>/Limit Height h<1000ft' */
  if (pid_control_V1_B.SignPreSat > 1000.0) {
    pid_control_V1_B.Va = 1000.0;
  } else if (pid_control_V1_B.SignPreSat < 0.0) {
    pid_control_V1_B.Va = 0.0;
  } else {
    pid_control_V1_B.Va = pid_control_V1_B.SignPreSat;
  }

  /* Product: '<S300>/sigma_ug, sigma_vg' incorporates:
   *  Fcn: '<S300>/Low Altitude Intensity'
   *  Saturate: '<S300>/Limit Height h<1000ft'
   */
  pid_control_V1_B.sina = 1.0 / pid_control_V1_rt_powd_snf(0.000823 *
    pid_control_V1_B.Va + 0.177, 0.4) * pid_control_V1_ConstB.sigma_wg;

  /* Fcn: '<S317>/Low Altitude Scale Length' */
  pid_control_V1_B.Sum_ks /= pid_control_V1_rt_powd_snf(0.000823 *
    pid_control_V1_B.Sum_ks + 0.177, 1.2);

  /* Gain: '<S289>/Lv' */
  pid_control_V1_B.SignPreSat_j = pid_control_V1_ConstB.UnitConversion_c;

  /* Outputs for Enabled SubSystem: '<S283>/Hvgw(s)' incorporates:
   *  EnablePort: '<S297>/Enable'
   */
  if (tmp_0 && rtsiIsModeUpdateTimeStep(&(&pid_control_V1_M)->solverInfo) &&
      (!pid_control_V1_DW.Hvgws_MODE)) {
    (void) memset(&(pid_control_V1_XDis.vg_p1_CSTATE), 0,
                  4*sizeof(boolean_T));

    /* InitializeConditions for Integrator: '<S297>/vg_p1' */
    pid_control_V1_X.vg_p1_CSTATE[0] = 0.0;

    /* InitializeConditions for Integrator: '<S297>/vgw_p2' */
    pid_control_V1_X.vgw_p2_CSTATE[0] = 0.0;

    /* InitializeConditions for Integrator: '<S297>/vg_p1' */
    pid_control_V1_X.vg_p1_CSTATE[1] = 0.0;

    /* InitializeConditions for Integrator: '<S297>/vgw_p2' */
    pid_control_V1_X.vgw_p2_CSTATE[1] = 0.0;
    pid_control_V1_DW.Hvgws_MODE = true;
  }

  if (pid_control_V1_DW.Hvgws_MODE) {
    /* Product: '<S297>/Lvg//V' incorporates:
     *  Gain: '<S289>/Lv'
     */
    pid_control_V1_B.Va = pid_control_V1_B.Sum_ks / pid_control_V1_B.Sum4;

    /* Product: '<S297>/w' incorporates:
     *  Gain: '<S297>/(1//pi)'
     *  Integrator: '<S297>/vg_p1'
     *  Product: '<S297>/Lug//V1'
     *  Sqrt: '<S297>/sqrt'
     *  Sum: '<S297>/Sum'
     */
    pid_control_V1_B.sinb = (sqrt(0.31830988618379069 * pid_control_V1_B.Va) *
      pid_control_V1_B.Product[1] - pid_control_V1_X.vg_p1_CSTATE[0]) /
      pid_control_V1_B.Va;
    pid_control_V1_B.w_g[0] = pid_control_V1_B.sinb;

    /* Product: '<S297>/w ' incorporates:
     *  Gain: '<S297>/sqrt(3)'
     *  Integrator: '<S297>/vg_p1'
     *  Integrator: '<S297>/vgw_p2'
     *  Product: '<S297>/Lvg//V '
     *  Sum: '<S297>/Sum1'
     */
    pid_control_V1_B.w_e[0] = (pid_control_V1_B.sinb * pid_control_V1_B.Va *
      1.7320508075688772 + (pid_control_V1_X.vg_p1_CSTATE[0] -
      pid_control_V1_X.vgw_p2_CSTATE[0])) / pid_control_V1_B.Va;

    /* Product: '<S297>/Lvg//V' */
    pid_control_V1_B.Va = pid_control_V1_B.SignPreSat_j / pid_control_V1_B.Sum4;

    /* Product: '<S297>/w' incorporates:
     *  Gain: '<S297>/(1//pi)'
     *  Integrator: '<S297>/vg_p1'
     *  Product: '<S297>/Lug//V1'
     *  Sqrt: '<S297>/sqrt'
     *  Sum: '<S297>/Sum'
     */
    pid_control_V1_B.sinb = (sqrt(0.31830988618379069 * pid_control_V1_B.Va) *
      pid_control_V1_B.Product[1] - pid_control_V1_X.vg_p1_CSTATE[1]) /
      pid_control_V1_B.Va;
    pid_control_V1_B.w_g[1] = pid_control_V1_B.sinb;

    /* Product: '<S297>/w ' incorporates:
     *  Gain: '<S297>/sqrt(3)'
     *  Integrator: '<S297>/vg_p1'
     *  Integrator: '<S297>/vgw_p2'
     *  Product: '<S297>/Lvg//V '
     *  Sum: '<S297>/Sum1'
     */
    pid_control_V1_B.w_e[1] = (pid_control_V1_B.sinb * pid_control_V1_B.Va *
      1.7320508075688772 + (pid_control_V1_X.vg_p1_CSTATE[1] -
      pid_control_V1_X.vgw_p2_CSTATE[1])) / pid_control_V1_B.Va;

    /* Product: '<S297>/w 1' incorporates:
     *  Integrator: '<S297>/vgw_p2'
     */
    tmp_3 = _mm_mul_pd(_mm_set_pd(pid_control_V1_B.Sum5, pid_control_V1_B.sina),
                       _mm_loadu_pd(&pid_control_V1_X.vgw_p2_CSTATE[0]));

    /* Product: '<S297>/w 1' */
    _mm_storeu_pd(&pid_control_V1_B.w1[0], tmp_3);
  }

  /* End of Outputs for SubSystem: '<S283>/Hvgw(s)' */

  /* Outputs for Enabled SubSystem: '<S282>/Hrgw' incorporates:
   *  EnablePort: '<S295>/Enable'
   */
  if (tmp_0 && rtsiIsModeUpdateTimeStep(&(&pid_control_V1_M)->solverInfo) &&
      (!pid_control_V1_DW.Hrgw_MODE)) {
    (void) memset(&(pid_control_V1_XDis.rgw_p_CSTATE), 0,
                  2*sizeof(boolean_T));

    /* InitializeConditions for Integrator: '<S295>/rgw_p' */
    pid_control_V1_X.rgw_p_CSTATE[0] = 0.0;
    pid_control_V1_X.rgw_p_CSTATE[1] = 0.0;
    pid_control_V1_DW.Hrgw_MODE = true;
  }

  if (pid_control_V1_DW.Hrgw_MODE) {
    /* Product: '<S295>/vg//V' incorporates:
     *  Gain: '<S295>/pi//3'
     *  Integrator: '<S295>/rgw_p'
     *  Product: '<S295>/w'
     */
    tmp_3 = _mm_mul_pd(_mm_sub_pd(_mm_div_pd(_mm_loadu_pd(&pid_control_V1_B.w1[0]),
      _mm_set1_pd(pid_control_V1_B.Sum4)), _mm_loadu_pd
      (&pid_control_V1_X.rgw_p_CSTATE[0])), _mm_div_pd(_mm_set1_pd
      (1.0471975511965976 * pid_control_V1_B.Sum4), _mm_set1_pd
      (pid_control_V1_ConstB.UnitConversion_n)));

    /* Product: '<S295>/w' */
    _mm_storeu_pd(&pid_control_V1_B.w_d[0], tmp_3);
  }

  /* End of Outputs for SubSystem: '<S282>/Hrgw' */

  /* Outputs for Enabled SubSystem: '<S283>/Hugw(s)' incorporates:
   *  EnablePort: '<S296>/Enable'
   */
  if (tmp_0 && rtsiIsModeUpdateTimeStep(&(&pid_control_V1_M)->solverInfo) &&
      (!pid_control_V1_DW.Hugws_MODE)) {
    (void) memset(&(pid_control_V1_XDis.ug_p_CSTATE), 0,
                  2*sizeof(boolean_T));

    /* InitializeConditions for Integrator: '<S296>/ug_p' */
    pid_control_V1_X.ug_p_CSTATE[0] = 0.0;
    pid_control_V1_X.ug_p_CSTATE[1] = 0.0;
    pid_control_V1_DW.Hugws_MODE = true;
  }

  if (pid_control_V1_DW.Hugws_MODE) {
    /* Product: '<S296>/Lug//V' */
    pid_control_V1_B.Va = pid_control_V1_B.Sum_ks / pid_control_V1_B.Sum4;
    pid_control_V1_B.SignPreSat_j = pid_control_V1_ConstB.UnitConversion_c /
      pid_control_V1_B.Sum4;

    /* Sqrt: '<S296>/sqrt' incorporates:
     *  Gain: '<S296>/(2//pi)'
     *  Integrator: '<S296>/ug_p'
     *  Product: '<S296>/Lug//V1'
     */
    tmp_3 = _mm_div_pd(_mm_sub_pd(_mm_mul_pd(_mm_set_pd(sqrt(0.63661977236758138
      * pid_control_V1_B.SignPreSat_j), sqrt(0.63661977236758138 *
      pid_control_V1_B.Va)), _mm_set1_pd(pid_control_V1_B.Product[0])),
      _mm_loadu_pd(&pid_control_V1_X.ug_p_CSTATE[0])), _mm_set_pd
                       (pid_control_V1_B.SignPreSat_j, pid_control_V1_B.Va));

    /* Product: '<S296>/w' */
    _mm_storeu_pd(&pid_control_V1_B.w_n[0], tmp_3);

    /* Integrator: '<S296>/ug_p' incorporates:
     *  Product: '<S296>/w1'
     */
    tmp_3 = _mm_mul_pd(_mm_loadu_pd(&pid_control_V1_X.ug_p_CSTATE[0]),
                       _mm_set_pd(pid_control_V1_B.Sum5, pid_control_V1_B.sina));

    /* Product: '<S296>/w1' */
    _mm_storeu_pd(&pid_control_V1_B.w1_c[0], tmp_3);
  }

  /* End of Outputs for SubSystem: '<S283>/Hugw(s)' */

  /* Angle2Dcm: '<S11>/Rotation Angles to Direction Cosine Matrix' */
  pid_control_V1_B.Va = cos(pid_control_V1_B.x[6]);
  pid_control_V1_B.Sum5 = sin(pid_control_V1_B.x[6]);
  pid_control_V1_B.SignPreSat_j = -sin(pid_control_V1_B.x[6]);
  pid_control_V1_B.Sum4 = cos(pid_control_V1_B.x[6]);
  pid_control_V1_B.FA_b_tmp_k = cos(pid_control_V1_B.x[7]);
  pid_control_V1_B.u2 = -sin(pid_control_V1_B.x[7]);
  pid_control_V1_B.Sum_ks = sin(pid_control_V1_B.x[7]);
  pid_control_V1_B.sina = cos(pid_control_V1_B.x[7]);
  pid_control_V1_B.sinb = cos(pid_control_V1_B.x[8]);
  pid_control_V1_B.sinc = sin(pid_control_V1_B.x[8]);
  pid_control_V1_B.cosa = -sin(pid_control_V1_B.x[8]);
  pid_control_V1_B.FA_b_tmp_c = cos(pid_control_V1_B.x[8]);
  pid_control_V1_B.cosb = 0.0 * pid_control_V1_B.Sum_ks +
    pid_control_V1_B.FA_b_tmp_k;
  pid_control_V1_B.cosc = 0.0 * pid_control_V1_B.sina + pid_control_V1_B.u2;
  pid_control_V1_B.FA_b_b = pid_control_V1_B.sinb * 0.0;
  pid_control_V1_B.Cl = 0.0 * pid_control_V1_B.FA_b_tmp_k;
  pid_control_V1_B.FA_b_tmp_k = (pid_control_V1_B.Cl + pid_control_V1_B.FA_b_b)
    + pid_control_V1_B.sinc * pid_control_V1_B.Sum_ks;
  pid_control_V1_B.sinb += pid_control_V1_B.sinc * 0.0;
  pid_control_V1_B.u2 *= 0.0;
  pid_control_V1_B.sinc = (pid_control_V1_B.u2 + pid_control_V1_B.FA_b_b) +
    pid_control_V1_B.sinc * pid_control_V1_B.sina;
  pid_control_V1_B.FA_b_b = pid_control_V1_B.cosa * 0.0;
  pid_control_V1_B.Sum_ks = (pid_control_V1_B.Cl + pid_control_V1_B.FA_b_b) +
    pid_control_V1_B.Sum_ks * pid_control_V1_B.FA_b_tmp_c;
  pid_control_V1_B.cosa += pid_control_V1_B.FA_b_tmp_c * 0.0;
  pid_control_V1_B.sina = (pid_control_V1_B.u2 + pid_control_V1_B.FA_b_b) +
    pid_control_V1_B.FA_b_tmp_c * pid_control_V1_B.sina;
  pid_control_V1_B.FA_b_tmp_c = pid_control_V1_B.cosc * 0.0;
  pid_control_V1_B.RotationAnglestoDirectionCo[0] = (pid_control_V1_B.cosb *
    pid_control_V1_B.Va + 0.0 * pid_control_V1_B.SignPreSat_j) +
    pid_control_V1_B.FA_b_tmp_c;
  pid_control_V1_B.FA_b_b = pid_control_V1_B.sinc * 0.0;
  pid_control_V1_B.RotationAnglestoDirectionCo[1] = (pid_control_V1_B.Va *
    pid_control_V1_B.FA_b_tmp_k + pid_control_V1_B.SignPreSat_j *
    pid_control_V1_B.sinb) + pid_control_V1_B.FA_b_b;
  pid_control_V1_B.Cl = pid_control_V1_B.sina * 0.0;
  pid_control_V1_B.RotationAnglestoDirectionCo[2] = (pid_control_V1_B.Va *
    pid_control_V1_B.Sum_ks + pid_control_V1_B.SignPreSat_j *
    pid_control_V1_B.cosa) + pid_control_V1_B.Cl;
  pid_control_V1_B.RotationAnglestoDirectionCo[3] = (pid_control_V1_B.cosb *
    pid_control_V1_B.Sum5 + 0.0 * pid_control_V1_B.Sum4) +
    pid_control_V1_B.FA_b_tmp_c;
  pid_control_V1_B.RotationAnglestoDirectionCo[4] = (pid_control_V1_B.Sum5 *
    pid_control_V1_B.FA_b_tmp_k + pid_control_V1_B.sinb * pid_control_V1_B.Sum4)
    + pid_control_V1_B.FA_b_b;
  pid_control_V1_B.RotationAnglestoDirectionCo[5] = (pid_control_V1_B.Sum5 *
    pid_control_V1_B.Sum_ks + pid_control_V1_B.Sum4 * pid_control_V1_B.cosa) +
    pid_control_V1_B.Cl;
  pid_control_V1_B.RotationAnglestoDirectionCo[6] = pid_control_V1_B.cosb * 0.0
    + pid_control_V1_B.cosc;
  pid_control_V1_B.RotationAnglestoDirectionCo[7] = (pid_control_V1_B.FA_b_tmp_k
    * 0.0 + pid_control_V1_B.sinb * 0.0) + pid_control_V1_B.sinc;
  pid_control_V1_B.RotationAnglestoDirectionCo[8] = (pid_control_V1_B.Sum_ks *
    0.0 + pid_control_V1_B.cosa * 0.0) + pid_control_V1_B.sina;

  /* If: '<S287>/if Height < Max low altitude  elseif Height > Min isotropic altitude ' incorporates:
   *  Constant: '<S301>/max_height_low'
   *  If: '<S288>/if Height < Max low altitude  elseif Height > Min isotropic altitude '
   *  Product: '<S301>/Product1'
   *  Product: '<S306>/Product1'
   *  Product: '<S306>/Product2'
   *  Product: '<S308>/Product1'
   *  Product: '<S308>/Product2'
   *  Sum: '<S301>/Sum1'
   *  Sum: '<S301>/Sum2'
   *  Sum: '<S301>/Sum3'
   *  Sum: '<S306>/Sum'
   *  Sum: '<S308>/Sum'
   */
  rtPrevAction = pid_control_V1_DW.ifHeightMaxlowaltitudeelseifHei;
  serverAvailableOnTime = rtsiIsModeUpdateTimeStep(&(&pid_control_V1_M)
    ->solverInfo);
  if (serverAvailableOnTime) {
    if (pid_control_V1_B.SignPreSat <= 1000.0) {
      rtAction = 0;
    } else if (pid_control_V1_B.SignPreSat >= 2000.0) {
      rtAction = 1;
    } else {
      rtAction = 2;
    }

    pid_control_V1_DW.ifHeightMaxlowaltitudeelseifHei = rtAction;
  } else {
    rtAction = pid_control_V1_DW.ifHeightMaxlowaltitudeelseifHei;
  }

  if (rtPrevAction != rtAction) {
    rtsiSetBlockStateForSolverChangedAtMajorStep(&(&pid_control_V1_M)
      ->solverInfo, true);
  }

  switch (rtAction) {
   case 0:
    /* Outputs for IfAction SubSystem: '<S287>/Low altitude  rates' incorporates:
     *  ActionPort: '<S302>/Action Port'
     */
    /* SignalConversion generated from: '<S307>/Vector Concatenate' */
    pid_control_V1_B.Product_be[2] = pid_control_V1_B.w_d[0];

    /* Trigonometry: '<S308>/Trigonometric Function1' incorporates:
     *  UnitConversion: '<S281>/Unit Conversion'
     */
    pid_control_V1_B.Va = sin(pid_control_V1_ConstB.UnitConversion);
    pid_control_V1_B.Sum5 = cos(pid_control_V1_ConstB.UnitConversion);
    _mm_storeu_pd(&pid_control_V1_B.Product_be[0], _mm_add_pd(_mm_mul_pd
      (_mm_set_pd(pid_control_V1_B.Va, pid_control_V1_B.sigma_w[0]), _mm_set_pd
       (pid_control_V1_B.sigma_w[0], pid_control_V1_B.Sum5)), _mm_mul_pd
      (_mm_mul_pd(_mm_set_pd(pid_control_V1_B.UnaryMinus[0], pid_control_V1_B.Va),
                  _mm_set_pd(pid_control_V1_B.Sum5, pid_control_V1_B.UnaryMinus
      [0])), tmp_2)));

    /* Product: '<S307>/Product' incorporates:
     *  Angle2Dcm: '<S11>/Rotation Angles to Direction Cosine Matrix'
     *  Concatenate: '<S307>/Vector Concatenate'
     *  Product: '<S308>/Product1'
     *  Product: '<S308>/Product2'
     *  Reshape: '<S307>/Reshape1'
     *  Sum: '<S308>/Sum'
     */
    pid_control_V1_B.Va = 0.0;
    pid_control_V1_B.Sum5 = 0.0;
    pid_control_V1_B.SignPreSat_j = 0.0;
    for (pid_control_V1_B.i = 0; pid_control_V1_B.i < 3; pid_control_V1_B.i++) {
      tmp_3 = _mm_add_pd(_mm_mul_pd(_mm_loadu_pd
        (&pid_control_V1_B.RotationAnglestoDirectionCo[3 * pid_control_V1_B.i]),
        _mm_set1_pd(pid_control_V1_B.Product_be[pid_control_V1_B.i])),
                         _mm_set_pd(pid_control_V1_B.Sum5, pid_control_V1_B.Va));
      _mm_storeu_pd(&pid_control_V1_B.dv[0], tmp_3);
      pid_control_V1_B.Va = pid_control_V1_B.dv[0];
      pid_control_V1_B.Sum5 = pid_control_V1_B.dv[1];
      pid_control_V1_B.SignPreSat_j +=
        pid_control_V1_B.RotationAnglestoDirectionCo[3 * pid_control_V1_B.i + 2]
        * pid_control_V1_B.Product_be[pid_control_V1_B.i];
    }

    pid_control_V1_B.wbe_b[2] = pid_control_V1_B.SignPreSat_j;
    pid_control_V1_B.wbe_b[1] = pid_control_V1_B.Sum5;
    pid_control_V1_B.wbe_b[0] = pid_control_V1_B.Va;

    /* End of Product: '<S307>/Product' */
    /* End of Outputs for SubSystem: '<S287>/Low altitude  rates' */
    break;

   case 1:
    /* Outputs for IfAction SubSystem: '<S287>/Medium//High  altitude rates' incorporates:
     *  ActionPort: '<S303>/Action Port'
     */
    /* Gain: '<S303>/Gain' */
    pid_control_V1_B.wbe_b[0] = pid_control_V1_B.sigma_w[1];
    pid_control_V1_B.wbe_b[1] = pid_control_V1_B.UnaryMinus[1];
    pid_control_V1_B.wbe_b[2] = pid_control_V1_B.w_d[1];

    /* End of Outputs for SubSystem: '<S287>/Medium//High  altitude rates' */
    break;

   default:
    /* Outputs for IfAction SubSystem: '<S287>/Interpolate  rates' incorporates:
     *  ActionPort: '<S301>/Action Port'
     */
    /* Trigonometry: '<S306>/Trigonometric Function' incorporates:
     *  UnitConversion: '<S281>/Unit Conversion'
     */
    pid_control_V1_B.Va = sin(pid_control_V1_ConstB.UnitConversion);
    pid_control_V1_B.Sum5 = cos(pid_control_V1_ConstB.UnitConversion);
    _mm_storeu_pd(&pid_control_V1_B.wbe_b[0], _mm_add_pd(_mm_mul_pd(_mm_set_pd
      (pid_control_V1_B.Va, pid_control_V1_B.sigma_w[0]), _mm_set_pd
      (pid_control_V1_B.sigma_w[0], pid_control_V1_B.Sum5)), _mm_mul_pd
      (_mm_mul_pd(_mm_set_pd(pid_control_V1_B.UnaryMinus[0], pid_control_V1_B.Va),
                  _mm_set_pd(pid_control_V1_B.Sum5, pid_control_V1_B.UnaryMinus
      [0])), tmp_2)));

    /* SignalConversion generated from: '<S305>/Vector Concatenate' incorporates:
     *  Product: '<S306>/Product1'
     *  Product: '<S306>/Product2'
     *  Sum: '<S306>/Sum'
     */
    pid_control_V1_B.wbe_b[2] = pid_control_V1_B.w_d[0];

    /* Product: '<S305>/Product' incorporates:
     *  Angle2Dcm: '<S11>/Rotation Angles to Direction Cosine Matrix'
     *  Concatenate: '<S305>/Vector Concatenate'
     */
    pid_control_V1_B.Va = 0.0;
    pid_control_V1_B.Sum5 = 0.0;
    pid_control_V1_B.SignPreSat_j = 0.0;
    for (pid_control_V1_B.i = 0; pid_control_V1_B.i < 3; pid_control_V1_B.i++) {
      tmp_3 = _mm_add_pd(_mm_mul_pd(_mm_loadu_pd
        (&pid_control_V1_B.RotationAnglestoDirectionCo[3 * pid_control_V1_B.i]),
        _mm_set1_pd(pid_control_V1_B.wbe_b[pid_control_V1_B.i])), _mm_set_pd
                         (pid_control_V1_B.Sum5, pid_control_V1_B.Va));
      _mm_storeu_pd(&pid_control_V1_B.dv[0], tmp_3);
      pid_control_V1_B.Va = pid_control_V1_B.dv[0];
      pid_control_V1_B.Sum5 = pid_control_V1_B.dv[1];
      pid_control_V1_B.SignPreSat_j +=
        pid_control_V1_B.RotationAnglestoDirectionCo[3 * pid_control_V1_B.i + 2]
        * pid_control_V1_B.wbe_b[pid_control_V1_B.i];
    }

    pid_control_V1_B.Product_be[2] = pid_control_V1_B.SignPreSat_j;
    pid_control_V1_B.Product_be[1] = pid_control_V1_B.Sum5;
    pid_control_V1_B.Product_be[0] = pid_control_V1_B.Va;
    tmp_3 = _mm_add_pd(_mm_div_pd(_mm_mul_pd(_mm_sub_pd(_mm_set_pd
      (pid_control_V1_B.UnaryMinus[1], pid_control_V1_B.sigma_w[1]),
      _mm_loadu_pd(&pid_control_V1_B.Product_be[0])), _mm_sub_pd(_mm_set1_pd
      (pid_control_V1_B.SignPreSat), _mm_set1_pd(1000.0))), _mm_set1_pd
      (pid_control_V1_ConstB.Sum_a)), _mm_loadu_pd(&pid_control_V1_B.Product_be
      [0]));
    _mm_storeu_pd(&pid_control_V1_B.wbe_b[0], tmp_3);

    /* Sum: '<S301>/Sum3' incorporates:
     *  Constant: '<S301>/max_height_low'
     *  Product: '<S301>/Product1'
     *  Product: '<S305>/Product'
     *  Sum: '<S301>/Sum1'
     *  Sum: '<S301>/Sum2'
     */
    pid_control_V1_B.wbe_b[2] = (pid_control_V1_B.w_d[1] -
      pid_control_V1_B.SignPreSat_j) * (pid_control_V1_B.SignPreSat - 1000.0) /
      pid_control_V1_ConstB.Sum_a + pid_control_V1_B.SignPreSat_j;

    /* End of Outputs for SubSystem: '<S287>/Interpolate  rates' */
    break;
  }

  /* End of If: '<S287>/if Height < Max low altitude  elseif Height > Min isotropic altitude ' */

  /* If: '<S288>/if Height < Max low altitude  elseif Height > Min isotropic altitude ' incorporates:
   *  Constant: '<S309>/max_height_low'
   *  Product: '<S309>/Product1'
   *  Product: '<S314>/Product1'
   *  Product: '<S314>/Product2'
   *  Product: '<S316>/Product1'
   *  Product: '<S316>/Product2'
   *  Sum: '<S309>/Sum1'
   *  Sum: '<S309>/Sum2'
   *  Sum: '<S309>/Sum3'
   *  Sum: '<S314>/Sum'
   *  Sum: '<S316>/Sum'
   *  UnitConversion: '<S277>/Unit Conversion'
   */
  rtPrevAction = pid_control_V1_DW.ifHeightMaxlowaltitudeelseifH_k;
  if (serverAvailableOnTime) {
    if (pid_control_V1_B.SignPreSat <= 1000.0) {
      rtAction = 0;
    } else if (pid_control_V1_B.SignPreSat >= 2000.0) {
      rtAction = 1;
    } else {
      rtAction = 2;
    }

    pid_control_V1_DW.ifHeightMaxlowaltitudeelseifH_k = rtAction;
  } else {
    rtAction = pid_control_V1_DW.ifHeightMaxlowaltitudeelseifH_k;
  }

  if (rtPrevAction != rtAction) {
    rtsiSetBlockStateForSolverChangedAtMajorStep(&(&pid_control_V1_M)
      ->solverInfo, true);
  }

  switch (rtAction) {
   case 0:
    /* Outputs for IfAction SubSystem: '<S288>/Low altitude  velocities' incorporates:
     *  ActionPort: '<S310>/Action Port'
     */
    /* SignalConversion generated from: '<S315>/Vector Concatenate' */
    pid_control_V1_B.Product_be[2] = pid_control_V1_B.LwgV1[0];

    /* Trigonometry: '<S316>/Trigonometric Function' incorporates:
     *  UnitConversion: '<S281>/Unit Conversion'
     */
    pid_control_V1_B.Va = sin(pid_control_V1_ConstB.UnitConversion);
    pid_control_V1_B.SignPreSat = cos(pid_control_V1_ConstB.UnitConversion);
    _mm_storeu_pd(&pid_control_V1_B.Product_be[0], _mm_add_pd(_mm_mul_pd
      (_mm_set_pd(pid_control_V1_B.Va, pid_control_V1_B.w1_c[0]), _mm_set_pd
       (pid_control_V1_B.w1_c[0], pid_control_V1_B.SignPreSat)), _mm_mul_pd
      (_mm_mul_pd(_mm_set_pd(pid_control_V1_B.w1[0], pid_control_V1_B.Va),
                  _mm_set_pd(pid_control_V1_B.SignPreSat, pid_control_V1_B.w1[0])),
       tmp_2)));

    /* Product: '<S315>/Product' incorporates:
     *  Angle2Dcm: '<S11>/Rotation Angles to Direction Cosine Matrix'
     *  Concatenate: '<S315>/Vector Concatenate'
     *  Product: '<S316>/Product1'
     *  Product: '<S316>/Product2'
     *  Reshape: '<S315>/Reshape1'
     *  Sum: '<S316>/Sum'
     *  UnitConversion: '<S277>/Unit Conversion'
     */
    pid_control_V1_B.Switch_p[0] = 0.0;
    pid_control_V1_B.Switch_p[1] = 0.0;
    pid_control_V1_B.Switch_p[2] = 0.0;
    pid_control_V1_B.Va = pid_control_V1_B.Switch_p[0];
    pid_control_V1_B.SignPreSat = pid_control_V1_B.Switch_p[1];
    pid_control_V1_B.Sum5 = pid_control_V1_B.Switch_p[2];
    for (pid_control_V1_B.i = 0; pid_control_V1_B.i < 3; pid_control_V1_B.i++) {
      tmp_2 = _mm_add_pd(_mm_mul_pd(_mm_loadu_pd
        (&pid_control_V1_B.RotationAnglestoDirectionCo[3 * pid_control_V1_B.i]),
        _mm_set1_pd(pid_control_V1_B.Product_be[pid_control_V1_B.i])),
                         _mm_set_pd(pid_control_V1_B.SignPreSat,
        pid_control_V1_B.Va));
      _mm_storeu_pd(&pid_control_V1_B.dv[0], tmp_2);
      pid_control_V1_B.Va = pid_control_V1_B.dv[0];
      pid_control_V1_B.SignPreSat = pid_control_V1_B.dv[1];
      pid_control_V1_B.Sum5 += pid_control_V1_B.RotationAnglestoDirectionCo[3 *
        pid_control_V1_B.i + 2] * pid_control_V1_B.Product_be[pid_control_V1_B.i];
    }

    /* UnitConversion: '<S277>/Unit Conversion' incorporates:
     *  Product: '<S315>/Product'
     *  Reshape: '<S315>/Reshape1'
     */
    pid_control_V1_B.Switch_p[2] = pid_control_V1_B.Sum5;
    pid_control_V1_B.Switch_p[1] = pid_control_V1_B.SignPreSat;
    pid_control_V1_B.Switch_p[0] = pid_control_V1_B.Va;

    /* End of Outputs for SubSystem: '<S288>/Low altitude  velocities' */
    break;

   case 1:
    /* Outputs for IfAction SubSystem: '<S288>/Medium//High  altitude velocities' incorporates:
     *  ActionPort: '<S311>/Action Port'
     */
    /* Gain: '<S311>/Gain' incorporates:
     *  UnitConversion: '<S277>/Unit Conversion'
     */
    pid_control_V1_B.Switch_p[0] = pid_control_V1_B.w1_c[1];
    pid_control_V1_B.Switch_p[1] = pid_control_V1_B.w1[1];
    pid_control_V1_B.Switch_p[2] = pid_control_V1_B.LwgV1[1];

    /* End of Outputs for SubSystem: '<S288>/Medium//High  altitude velocities' */
    break;

   default:
    /* Outputs for IfAction SubSystem: '<S288>/Interpolate  velocities' incorporates:
     *  ActionPort: '<S309>/Action Port'
     */
    /* Trigonometry: '<S314>/Trigonometric Function' incorporates:
     *  UnitConversion: '<S281>/Unit Conversion'
     */
    pid_control_V1_B.Va = sin(pid_control_V1_ConstB.UnitConversion);
    pid_control_V1_B.Sum5 = cos(pid_control_V1_ConstB.UnitConversion);
    _mm_storeu_pd(&pid_control_V1_B.FA_b[0], _mm_add_pd(_mm_mul_pd(_mm_set_pd
      (pid_control_V1_B.Va, pid_control_V1_B.w1_c[0]), _mm_set_pd
      (pid_control_V1_B.w1_c[0], pid_control_V1_B.Sum5)), _mm_mul_pd(_mm_mul_pd
      (_mm_set_pd(pid_control_V1_B.w1[0], pid_control_V1_B.Va), _mm_set_pd
       (pid_control_V1_B.Sum5, pid_control_V1_B.w1[0])), tmp_2)));

    /* SignalConversion generated from: '<S313>/Vector Concatenate' incorporates:
     *  Product: '<S314>/Product1'
     *  Product: '<S314>/Product2'
     *  Sum: '<S314>/Sum'
     */
    pid_control_V1_B.FA_b[2] = pid_control_V1_B.LwgV1[0];

    /* Product: '<S313>/Product' incorporates:
     *  Angle2Dcm: '<S11>/Rotation Angles to Direction Cosine Matrix'
     *  Concatenate: '<S313>/Vector Concatenate'
     */
    pid_control_V1_B.Va = 0.0;
    pid_control_V1_B.Sum5 = 0.0;
    pid_control_V1_B.SignPreSat_j = 0.0;
    for (pid_control_V1_B.i = 0; pid_control_V1_B.i < 3; pid_control_V1_B.i++) {
      tmp_2 = _mm_add_pd(_mm_mul_pd(_mm_loadu_pd
        (&pid_control_V1_B.RotationAnglestoDirectionCo[3 * pid_control_V1_B.i]),
        _mm_set1_pd(pid_control_V1_B.FA_b[pid_control_V1_B.i])), _mm_set_pd
                         (pid_control_V1_B.Sum5, pid_control_V1_B.Va));
      _mm_storeu_pd(&pid_control_V1_B.dv[0], tmp_2);
      pid_control_V1_B.Va = pid_control_V1_B.dv[0];
      pid_control_V1_B.Sum5 = pid_control_V1_B.dv[1];
      pid_control_V1_B.SignPreSat_j +=
        pid_control_V1_B.RotationAnglestoDirectionCo[3 * pid_control_V1_B.i + 2]
        * pid_control_V1_B.FA_b[pid_control_V1_B.i];
    }

    pid_control_V1_B.Product_be[2] = pid_control_V1_B.SignPreSat_j;
    pid_control_V1_B.Product_be[1] = pid_control_V1_B.Sum5;
    pid_control_V1_B.Product_be[0] = pid_control_V1_B.Va;
    tmp_2 = _mm_add_pd(_mm_div_pd(_mm_mul_pd(_mm_sub_pd(_mm_set_pd
      (pid_control_V1_B.w1[1], pid_control_V1_B.w1_c[1]), _mm_loadu_pd
      (&pid_control_V1_B.Product_be[0])), _mm_sub_pd(_mm_set1_pd
      (pid_control_V1_B.SignPreSat), _mm_set1_pd(1000.0))), _mm_set1_pd
      (pid_control_V1_ConstB.Sum)), _mm_loadu_pd(&pid_control_V1_B.Product_be[0]));
    _mm_storeu_pd(&pid_control_V1_B.Switch_p[0], tmp_2);

    /* Sum: '<S309>/Sum3' incorporates:
     *  Constant: '<S309>/max_height_low'
     *  Product: '<S309>/Product1'
     *  Product: '<S313>/Product'
     *  Sum: '<S309>/Sum1'
     *  Sum: '<S309>/Sum2'
     *  UnitConversion: '<S277>/Unit Conversion'
     */
    pid_control_V1_B.Switch_p[2] = (pid_control_V1_B.LwgV1[1] -
      pid_control_V1_B.SignPreSat_j) * (pid_control_V1_B.SignPreSat - 1000.0) /
      pid_control_V1_ConstB.Sum + pid_control_V1_B.SignPreSat_j;

    /* End of Outputs for SubSystem: '<S288>/Interpolate  velocities' */
    break;
  }

  /* UnitConversion: '<S277>/Unit Conversion' */
  /* Unit Conversion - from: ft/s to: m/s
     Expression: output = (0.3048*input) + (0) */
  tmp_2 = _mm_mul_pd(_mm_set1_pd(0.3048), _mm_loadu_pd
                     (&pid_control_V1_B.Switch_p[0]));

  /* UnitConversion: '<S277>/Unit Conversion' */
  _mm_storeu_pd(&pid_control_V1_B.Switch_p[0], tmp_2);

  /* UnitConversion: '<S277>/Unit Conversion' */
  pid_control_V1_B.Switch_p[2] *= 0.3048;
  if (tmp_0) {
    /* MATLABSystem: '<S279>/SourceBlock' */
    pid_control_V1_B.SourceBlock_o1_h = Sub_pid_control_V1_417.getLatestMessage(
      &rtb_SourceBlock_o2_j);

    /* Outputs for Enabled SubSystem: '<S279>/Enabled Subsystem' */
    pid_control_V1_EnabledSubsystem(pid_control_V1_B.SourceBlock_o1_h,
      &rtb_SourceBlock_o2_j, &pid_control_V1_B.EnabledSubsystem_p);

    /* End of Outputs for SubSystem: '<S279>/Enabled Subsystem' */

    /* MATLABSystem: '<S280>/SourceBlock' */
    pid_control_V1_B.SourceBlock_o1 = Sub_pid_control_V1_423.getLatestMessage
      (&rtb_SourceBlock_o2_d);

    /* Outputs for Enabled SubSystem: '<S280>/Enabled Subsystem' */
    pid_control_V1_EnabledSubsystem(pid_control_V1_B.SourceBlock_o1,
      &rtb_SourceBlock_o2_d, &pid_control_V1_B.EnabledSubsystem_g);

    /* End of Outputs for SubSystem: '<S280>/Enabled Subsystem' */
  }

  /* Switch: '<S11>/Switch' */
  if (!pid_control_V1_B.EnabledSubsystem_p.In1.data) {
    /* Switch: '<S11>/Switch' incorporates:
     *  Constant: '<S11>/Constant'
     *  UnitConversion: '<S277>/Unit Conversion'
     */
    pid_control_V1_B.Switch_p[0] = 0.0;
    pid_control_V1_B.Switch_p[1] = 0.0;
    pid_control_V1_B.Switch_p[2] = 0.0;
  }

  /* End of Switch: '<S11>/Switch' */

  /* Switch: '<S11>/Switch1' */
  if (pid_control_V1_B.EnabledSubsystem_g.In1.data) {
    /* Switch: '<S11>/Switch1' incorporates:
     *  Merge: '<S304>/Merge'
     */
    pid_control_V1_B.Switch1[0] = pid_control_V1_B.wbe_b[0];
    pid_control_V1_B.Switch1[1] = pid_control_V1_B.wbe_b[1];
    pid_control_V1_B.Switch1[2] = pid_control_V1_B.wbe_b[2];
  } else {
    /* Switch: '<S11>/Switch1' incorporates:
     *  Constant: '<S11>/Constant2'
     */
    pid_control_V1_B.Switch1[0] = 0.0;
    pid_control_V1_B.Switch1[1] = 0.0;
    pid_control_V1_B.Switch1[2] = 0.0;
  }

  /* End of Switch: '<S11>/Switch1' */
  if (rtmIsMajorTimeStep((&pid_control_V1_M))) {
    if (rtmIsMajorTimeStep((&pid_control_V1_M))) {
      /* Update for UnitDelay: '<Root>/Unit Delay2' */
      pid_control_V1_DW.UnitDelay2_DSTATE = pid_control_V1_B.Switch2;

      /* Update for Memory: '<S142>/Memory' */
      pid_control_V1_DW.Memory_PreviousInput_d = pid_control_V1_B.AND3;

      /* Update for Memory: '<S248>/Memory' */
      pid_control_V1_DW.Memory_PreviousInput_a = pid_control_V1_B.AND3_c;

      /* Update for Memory: '<S11>/Memory' incorporates:
       *  Switch: '<S11>/Switch'
       */
      pid_control_V1_DW.Memory_PreviousInput[0] = pid_control_V1_B.Switch_p[0];

      /* Update for Memory: '<S11>/Memory1' incorporates:
       *  Switch: '<S11>/Switch1'
       */
      pid_control_V1_DW.Memory1_PreviousInput[0] = pid_control_V1_B.Switch1[0];

      /* Update for Memory: '<S11>/Memory' incorporates:
       *  Switch: '<S11>/Switch'
       */
      pid_control_V1_DW.Memory_PreviousInput[1] = pid_control_V1_B.Switch_p[1];

      /* Update for Memory: '<S11>/Memory1' incorporates:
       *  Switch: '<S11>/Switch1'
       */
      pid_control_V1_DW.Memory1_PreviousInput[1] = pid_control_V1_B.Switch1[1];

      /* Update for Memory: '<S11>/Memory' incorporates:
       *  Switch: '<S11>/Switch'
       */
      pid_control_V1_DW.Memory_PreviousInput[2] = pid_control_V1_B.Switch_p[2];

      /* Update for Memory: '<S11>/Memory1' incorporates:
       *  Switch: '<S11>/Switch1'
       */
      pid_control_V1_DW.Memory1_PreviousInput[2] = pid_control_V1_B.Switch1[2];

      /* Update for RandomNumber: '<S292>/White Noise' */
      pid_control_V1_DW.NextOutput[0] = rt_nrand_Upu32_Yd_f_pw_snf
        (&pid_control_V1_DW.RandSeed[0]);
      pid_control_V1_DW.NextOutput[1] = rt_nrand_Upu32_Yd_f_pw_snf
        (&pid_control_V1_DW.RandSeed[1]);
      pid_control_V1_DW.NextOutput[2] = rt_nrand_Upu32_Yd_f_pw_snf
        (&pid_control_V1_DW.RandSeed[2]);
      pid_control_V1_DW.NextOutput[3] = rt_nrand_Upu32_Yd_f_pw_snf
        (&pid_control_V1_DW.RandSeed[3]);
    }
  }                                    /* end MajorTimeStep */

  if (rtmIsMajorTimeStep((&pid_control_V1_M))) {
    rt_ertODEUpdateContinuousStates(&(&pid_control_V1_M)->solverInfo);

    /* Update absolute time for base rate */
    /* The "clockTick0" counts the number of times the code of this task has
     * been executed. The absolute time is the multiplication of "clockTick0"
     * and "Timing.stepSize0". Size of "clockTick0" ensures timer will not
     * overflow during the application lifespan selected.
     * Timer of this task consists of two 32 bit unsigned integers.
     * The two integers represent the low bits Timing.clockTick0 and the high bits
     * Timing.clockTickH0. When the low bit overflows to 0, the high bits increment.
     */
    if (!(++(&pid_control_V1_M)->Timing.clockTick0)) {
      ++(&pid_control_V1_M)->Timing.clockTickH0;
    }

    (&pid_control_V1_M)->Timing.t[0] = rtsiGetSolverStopTime(&(&pid_control_V1_M)
      ->solverInfo);

    {
      /* Update absolute timer for sample time: [0.01s, 0.0s] */
      /* The "clockTick1" counts the number of times the code of this task has
       * been executed. The resolution of this integer timer is 0.01, which is the step size
       * of the task. Size of "clockTick1" ensures timer will not overflow during the
       * application lifespan selected.
       * Timer of this task consists of two 32 bit unsigned integers.
       * The two integers represent the low bits Timing.clockTick1 and the high bits
       * Timing.clockTickH1. When the low bit overflows to 0, the high bits increment.
       */
      (&pid_control_V1_M)->Timing.clockTick1++;
      if (!(&pid_control_V1_M)->Timing.clockTick1) {
        (&pid_control_V1_M)->Timing.clockTickH1++;
      }
    }
  }                                    /* end MajorTimeStep */
}

/* Derivatives for root system: '<Root>' */
void pid_control_V1::pid_control_V1_derivatives()
{
  XDot_pid_control_V1_T *_rtXdot;
  _rtXdot = ((XDot_pid_control_V1_T *) (&pid_control_V1_M)->derivs);

  /* Derivatives for Integrator: '<S11>/Integrator' */
  memcpy(&_rtXdot->Integrator_CSTATE[0], &pid_control_V1_B.XDOT[0], 12U * sizeof
         (real_T));

  /* Derivatives for Integrator: '<S258>/Integrator' */
  _rtXdot->Integrator_CSTATE_f = pid_control_V1_B.Switch_j;

  /* Derivatives for Integrator: '<S253>/Filter' */
  _rtXdot->Filter_CSTATE = pid_control_V1_B.FilterCoefficient;

  /* Derivatives for Integrator: '<S46>/Integrator' */
  _rtXdot->Integrator_CSTATE_m = pid_control_V1_B.SumI4;

  /* Derivatives for Integrator: '<S41>/Filter' */
  _rtXdot->Filter_CSTATE_g = pid_control_V1_B.FilterCoefficient_c;

  /* Derivatives for Integrator: '<S204>/Integrator' */
  _rtXdot->Integrator_CSTATE_d = pid_control_V1_B.IntegralGain_a;

  /* Derivatives for Integrator: '<S199>/Filter' */
  _rtXdot->Filter_CSTATE_f = pid_control_V1_B.FilterCoefficient_p;

  /* Derivatives for Integrator: '<S98>/Integrator' */
  _rtXdot->Integrator_CSTATE_n = pid_control_V1_B.IntegralGain;

  /* Derivatives for Integrator: '<S93>/Filter' */
  _rtXdot->Filter_CSTATE_fb = pid_control_V1_B.FilterCoefficient_j;

  /* Derivatives for Integrator: '<S152>/Integrator' */
  _rtXdot->Integrator_CSTATE_p = pid_control_V1_B.Switch;

  /* Derivatives for Integrator: '<S147>/Filter' */
  _rtXdot->Filter_CSTATE_m = pid_control_V1_B.FilterCoefficient_m;

  /* Derivatives for Integrator: '<S11>/Integrator1' */
  _rtXdot->Integrator1_CSTATE = pid_control_V1_B.Power;

  /* Derivatives for Enabled SubSystem: '<S282>/Hpgw' */
  if (pid_control_V1_DW.Hpgw_MODE) {
    /* Derivatives for Integrator: '<S293>/pgw_p' */
    _rtXdot->pgw_p_CSTATE[0] = pid_control_V1_B.w_o[0];
    _rtXdot->pgw_p_CSTATE[1] = pid_control_V1_B.w_o[1];
  } else {
    {
      real_T *dx;
      int_T i1;
      dx = &(((XDot_pid_control_V1_T *) (&pid_control_V1_M)->derivs)
             ->pgw_p_CSTATE[0]);
      for (i1=0; i1 < 2; i1++) {
        dx[i1] = 0.0;
      }
    }
  }

  /* End of Derivatives for SubSystem: '<S282>/Hpgw' */

  /* Derivatives for Enabled SubSystem: '<S283>/Hwgw(s)' */
  if (pid_control_V1_DW.Hwgws_MODE) {
    /* Derivatives for Integrator: '<S298>/wg_p1' */
    _rtXdot->wg_p1_CSTATE[0] = pid_control_V1_B.w[0];

    /* Derivatives for Integrator: '<S298>/wg_p2' */
    _rtXdot->wg_p2_CSTATE[0] = pid_control_V1_B.w_a[0];

    /* Derivatives for Integrator: '<S298>/wg_p1' */
    _rtXdot->wg_p1_CSTATE[1] = pid_control_V1_B.w[1];

    /* Derivatives for Integrator: '<S298>/wg_p2' */
    _rtXdot->wg_p2_CSTATE[1] = pid_control_V1_B.w_a[1];
  } else {
    {
      real_T *dx;
      int_T i1;
      dx = &(((XDot_pid_control_V1_T *) (&pid_control_V1_M)->derivs)
             ->wg_p1_CSTATE[0]);
      for (i1=0; i1 < 4; i1++) {
        dx[i1] = 0.0;
      }
    }
  }

  /* End of Derivatives for SubSystem: '<S283>/Hwgw(s)' */

  /* Derivatives for Enabled SubSystem: '<S282>/Hqgw' */
  if (pid_control_V1_DW.Hqgw_MODE) {
    /* Derivatives for Integrator: '<S294>/qgw_p' */
    _rtXdot->qgw_p_CSTATE[0] = pid_control_V1_B.w_e0[0];
    _rtXdot->qgw_p_CSTATE[1] = pid_control_V1_B.w_e0[1];
  } else {
    {
      real_T *dx;
      int_T i1;
      dx = &(((XDot_pid_control_V1_T *) (&pid_control_V1_M)->derivs)
             ->qgw_p_CSTATE[0]);
      for (i1=0; i1 < 2; i1++) {
        dx[i1] = 0.0;
      }
    }
  }

  /* End of Derivatives for SubSystem: '<S282>/Hqgw' */

  /* Derivatives for Enabled SubSystem: '<S283>/Hvgw(s)' */
  if (pid_control_V1_DW.Hvgws_MODE) {
    /* Derivatives for Integrator: '<S297>/vg_p1' */
    _rtXdot->vg_p1_CSTATE[0] = pid_control_V1_B.w_g[0];

    /* Derivatives for Integrator: '<S297>/vgw_p2' */
    _rtXdot->vgw_p2_CSTATE[0] = pid_control_V1_B.w_e[0];

    /* Derivatives for Integrator: '<S297>/vg_p1' */
    _rtXdot->vg_p1_CSTATE[1] = pid_control_V1_B.w_g[1];

    /* Derivatives for Integrator: '<S297>/vgw_p2' */
    _rtXdot->vgw_p2_CSTATE[1] = pid_control_V1_B.w_e[1];
  } else {
    {
      real_T *dx;
      int_T i1;
      dx = &(((XDot_pid_control_V1_T *) (&pid_control_V1_M)->derivs)
             ->vg_p1_CSTATE[0]);
      for (i1=0; i1 < 4; i1++) {
        dx[i1] = 0.0;
      }
    }
  }

  /* End of Derivatives for SubSystem: '<S283>/Hvgw(s)' */

  /* Derivatives for Enabled SubSystem: '<S282>/Hrgw' */
  if (pid_control_V1_DW.Hrgw_MODE) {
    /* Derivatives for Integrator: '<S295>/rgw_p' */
    _rtXdot->rgw_p_CSTATE[0] = pid_control_V1_B.w_d[0];
    _rtXdot->rgw_p_CSTATE[1] = pid_control_V1_B.w_d[1];
  } else {
    {
      real_T *dx;
      int_T i1;
      dx = &(((XDot_pid_control_V1_T *) (&pid_control_V1_M)->derivs)
             ->rgw_p_CSTATE[0]);
      for (i1=0; i1 < 2; i1++) {
        dx[i1] = 0.0;
      }
    }
  }

  /* End of Derivatives for SubSystem: '<S282>/Hrgw' */

  /* Derivatives for Enabled SubSystem: '<S283>/Hugw(s)' */
  if (pid_control_V1_DW.Hugws_MODE) {
    /* Derivatives for Integrator: '<S296>/ug_p' */
    _rtXdot->ug_p_CSTATE[0] = pid_control_V1_B.w_n[0];
    _rtXdot->ug_p_CSTATE[1] = pid_control_V1_B.w_n[1];
  } else {
    {
      real_T *dx;
      int_T i1;
      dx = &(((XDot_pid_control_V1_T *) (&pid_control_V1_M)->derivs)
             ->ug_p_CSTATE[0]);
      for (i1=0; i1 < 2; i1++) {
        dx[i1] = 0.0;
      }
    }
  }

  /* End of Derivatives for SubSystem: '<S283>/Hugw(s)' */
}

/* Model initialize function */
void pid_control_V1::initialize()
{
  /* Registration code */

  /* initialize non-finites */
  rt_InitInfAndNaN(sizeof(real_T));

  {
    /* Setup solver object */
    rtsiSetSimTimeStepPtr(&(&pid_control_V1_M)->solverInfo, &(&pid_control_V1_M
                          )->Timing.simTimeStep);
    rtsiSetTPtr(&(&pid_control_V1_M)->solverInfo, &rtmGetTPtr((&pid_control_V1_M)));
    rtsiSetStepSizePtr(&(&pid_control_V1_M)->solverInfo, &(&pid_control_V1_M)
                       ->Timing.stepSize0);
    rtsiSetdXPtr(&(&pid_control_V1_M)->solverInfo, &(&pid_control_V1_M)->derivs);
    rtsiSetContStatesPtr(&(&pid_control_V1_M)->solverInfo, (real_T **)
                         &(&pid_control_V1_M)->contStates);
    rtsiSetNumContStatesPtr(&(&pid_control_V1_M)->solverInfo,
      &(&pid_control_V1_M)->Sizes.numContStates);
    rtsiSetNumPeriodicContStatesPtr(&(&pid_control_V1_M)->solverInfo,
      &(&pid_control_V1_M)->Sizes.numPeriodicContStates);
    rtsiSetPeriodicContStateIndicesPtr(&(&pid_control_V1_M)->solverInfo,
      &(&pid_control_V1_M)->periodicContStateIndices);
    rtsiSetPeriodicContStateRangesPtr(&(&pid_control_V1_M)->solverInfo,
      &(&pid_control_V1_M)->periodicContStateRanges);
    rtsiSetContStateDisabledPtr(&(&pid_control_V1_M)->solverInfo, (boolean_T**)
      &(&pid_control_V1_M)->contStateDisabled);
    rtsiSetErrorStatusPtr(&(&pid_control_V1_M)->solverInfo, (&rtmGetErrorStatus
      ((&pid_control_V1_M))));
    rtsiSetRTModelPtr(&(&pid_control_V1_M)->solverInfo, (&pid_control_V1_M));
  }

  rtsiSetSimTimeStep(&(&pid_control_V1_M)->solverInfo, MAJOR_TIME_STEP);
  rtsiSetIsMinorTimeStepWithModeChange(&(&pid_control_V1_M)->solverInfo, false);
  rtsiSetIsContModeFrozen(&(&pid_control_V1_M)->solverInfo, false);
  (&pid_control_V1_M)->intgData.y = (&pid_control_V1_M)->odeY;
  (&pid_control_V1_M)->intgData.f[0] = (&pid_control_V1_M)->odeF[0];
  (&pid_control_V1_M)->intgData.f[1] = (&pid_control_V1_M)->odeF[1];
  (&pid_control_V1_M)->intgData.f[2] = (&pid_control_V1_M)->odeF[2];
  (&pid_control_V1_M)->contStates = ((X_pid_control_V1_T *) &pid_control_V1_X);
  (&pid_control_V1_M)->contStateDisabled = ((XDis_pid_control_V1_T *)
    &pid_control_V1_XDis);
  (&pid_control_V1_M)->Timing.tStart = (0.0);
  rtsiSetSolverData(&(&pid_control_V1_M)->solverInfo, static_cast<void *>
                    (&(&pid_control_V1_M)->intgData));
  rtsiSetSolverName(&(&pid_control_V1_M)->solverInfo,"ode3");
  rtmSetTPtr((&pid_control_V1_M), &(&pid_control_V1_M)->Timing.tArray[0]);
  (&pid_control_V1_M)->Timing.stepSize0 = 0.01;

  /* Start for MATLABSystem: '<S10>/SourceBlock' */
  pid_control_V1_DW.obj_k.QOSAvoidROSNamespaceConventions = false;
  pid_control_V1_DW.obj_k.matlabCodegenIsDeleted = false;
  pid_control_V1_DW.objisempty_g = true;
  pid_control_V1_DW.obj_k.isSetupComplete = false;
  pid_control_V1_DW.obj_k.isInitialized = 1;
  pid_contro_Subscriber_setupImpl(&pid_control_V1_DW.obj_k);
  pid_control_V1_DW.obj_k.isSetupComplete = true;

  /* Start for MATLABSystem: '<Root>/Coordinate Transformation Conversion' */
  pid_control_V1_DW.objisempty_d = true;
  pid_control_V1_DW.obj_c.isInitialized = 1;

  /* Start for Atomic SubSystem: '<Root>/Call Service' */
  /* Start for MATLABSystem: '<S2>/ServiceCaller' */
  pid_control_V1_DW.obj.QOSAvoidROSNamespaceConventions = false;
  pid_control_V1_DW.obj.matlabCodegenIsDeleted = false;
  pid_control_V1_DW.objisempty_f = true;
  pid_control_V1_DW.obj.isSetupComplete = false;
  pid_control_V1_DW.obj.isInitialized = 1;
  pid_con_ServiceCaller_setupImpl(&pid_control_V1_DW.obj);
  pid_control_V1_DW.obj.isSetupComplete = true;

  /* End of Start for SubSystem: '<Root>/Call Service' */

  /* Start for Enabled SubSystem: '<S282>/Hpgw' */
  (void) memset(&(pid_control_V1_XDis.pgw_p_CSTATE), 1,
                2*sizeof(boolean_T));

  /* End of Start for SubSystem: '<S282>/Hpgw' */

  /* Start for Enabled SubSystem: '<S283>/Hwgw(s)' */
  (void) memset(&(pid_control_V1_XDis.wg_p1_CSTATE), 1,
                4*sizeof(boolean_T));

  /* End of Start for SubSystem: '<S283>/Hwgw(s)' */

  /* Start for Enabled SubSystem: '<S282>/Hqgw' */
  (void) memset(&(pid_control_V1_XDis.qgw_p_CSTATE), 1,
                2*sizeof(boolean_T));

  /* End of Start for SubSystem: '<S282>/Hqgw' */

  /* Start for Enabled SubSystem: '<S283>/Hvgw(s)' */
  (void) memset(&(pid_control_V1_XDis.vg_p1_CSTATE), 1,
                4*sizeof(boolean_T));

  /* End of Start for SubSystem: '<S283>/Hvgw(s)' */

  /* Start for Enabled SubSystem: '<S282>/Hrgw' */
  (void) memset(&(pid_control_V1_XDis.rgw_p_CSTATE), 1,
                2*sizeof(boolean_T));

  /* End of Start for SubSystem: '<S282>/Hrgw' */

  /* Start for Enabled SubSystem: '<S283>/Hugw(s)' */
  (void) memset(&(pid_control_V1_XDis.ug_p_CSTATE), 1,
                2*sizeof(boolean_T));

  /* End of Start for SubSystem: '<S283>/Hugw(s)' */

  /* Start for If: '<S287>/if Height < Max low altitude  elseif Height > Min isotropic altitude ' */
  pid_control_V1_DW.ifHeightMaxlowaltitudeelseifHei = -1;

  /* Start for If: '<S288>/if Height < Max low altitude  elseif Height > Min isotropic altitude ' */
  pid_control_V1_DW.ifHeightMaxlowaltitudeelseifH_k = -1;

  /* Start for MATLABSystem: '<S279>/SourceBlock' */
  pid_control_V1_DW.obj_hq.QOSAvoidROSNamespaceConventions = false;
  pid_control_V1_DW.obj_hq.matlabCodegenIsDeleted = false;
  pid_control_V1_DW.objisempty_a = true;
  pid_control_V1_DW.obj_hq.isSetupComplete = false;
  pid_control_V1_DW.obj_hq.isInitialized = 1;
  pid_cont_Subscriber_setupImpl_o(&pid_control_V1_DW.obj_hq);
  pid_control_V1_DW.obj_hq.isSetupComplete = true;

  /* Start for MATLABSystem: '<S280>/SourceBlock' */
  pid_control_V1_DW.obj_h.QOSAvoidROSNamespaceConventions = false;
  pid_control_V1_DW.obj_h.matlabCodegenIsDeleted = false;
  pid_control_V1_DW.objisempty = true;
  pid_control_V1_DW.obj_h.isSetupComplete = false;
  pid_control_V1_DW.obj_h.isInitialized = 1;
  pid_con_Subscriber_setupImpl_on(&pid_control_V1_DW.obj_h);
  pid_control_V1_DW.obj_h.isSetupComplete = true;

  /* InitializeConditions for Integrator: '<S11>/Integrator' */
  memcpy(&pid_control_V1_X.Integrator_CSTATE[0],
         &pid_control_V1_ConstP.Integrator_IC[0], 12U * sizeof(real_T));

  /* InitializeConditions for Integrator: '<S258>/Integrator' */
  pid_control_V1_X.Integrator_CSTATE_f = 0.0;

  /* InitializeConditions for Integrator: '<S253>/Filter' */
  pid_control_V1_X.Filter_CSTATE = 0.0;

  /* InitializeConditions for Integrator: '<S46>/Integrator' */
  pid_control_V1_X.Integrator_CSTATE_m = 0.0;

  /* InitializeConditions for Integrator: '<S41>/Filter' */
  pid_control_V1_X.Filter_CSTATE_g = 0.0;

  /* InitializeConditions for Integrator: '<S204>/Integrator' */
  pid_control_V1_X.Integrator_CSTATE_d = 0.0;

  /* InitializeConditions for Integrator: '<S199>/Filter' */
  pid_control_V1_X.Filter_CSTATE_f = 0.0;

  /* InitializeConditions for Integrator: '<S98>/Integrator' */
  pid_control_V1_X.Integrator_CSTATE_n = 0.0;

  /* InitializeConditions for Integrator: '<S93>/Filter' */
  pid_control_V1_X.Filter_CSTATE_fb = 0.0;

  /* InitializeConditions for Integrator: '<S152>/Integrator' */
  pid_control_V1_X.Integrator_CSTATE_p = 0.0;

  /* InitializeConditions for Integrator: '<S147>/Filter' */
  pid_control_V1_X.Filter_CSTATE_m = 0.0;

  /* InitializeConditions for Integrator: '<S11>/Integrator1' */
  pid_control_V1_X.Integrator1_CSTATE = 0.0;

  /* InitializeConditions for RandomNumber: '<S292>/White Noise' */
  pid_control_V1_DW.RandSeed[0] = 1529675776U;
  pid_control_V1_DW.NextOutput[0] = rt_nrand_Upu32_Yd_f_pw_snf
    (&pid_control_V1_DW.RandSeed[0]);
  pid_control_V1_DW.RandSeed[1] = 1529741312U;
  pid_control_V1_DW.NextOutput[1] = rt_nrand_Upu32_Yd_f_pw_snf
    (&pid_control_V1_DW.RandSeed[1]);
  pid_control_V1_DW.RandSeed[2] = 1529806848U;
  pid_control_V1_DW.NextOutput[2] = rt_nrand_Upu32_Yd_f_pw_snf
    (&pid_control_V1_DW.RandSeed[2]);
  pid_control_V1_DW.RandSeed[3] = 1529872384U;
  pid_control_V1_DW.NextOutput[3] = rt_nrand_Upu32_Yd_f_pw_snf
    (&pid_control_V1_DW.RandSeed[3]);

  /* SystemInitialize for SignalConversion generated from: '<S276>/In1' */
  memset(&pid_control_V1_B.In1, 0, sizeof(SL_Bus_std_msgs_Float64));

  /* SystemInitialize for Enabled SubSystem: '<S282>/Hpgw' */
  /* InitializeConditions for Integrator: '<S293>/pgw_p' */
  pid_control_V1_X.pgw_p_CSTATE[0] = 0.0;

  /* End of SystemInitialize for SubSystem: '<S282>/Hpgw' */

  /* SystemInitialize for Enabled SubSystem: '<S283>/Hwgw(s)' */
  /* InitializeConditions for Integrator: '<S298>/wg_p1' */
  pid_control_V1_X.wg_p1_CSTATE[0] = 0.0;

  /* InitializeConditions for Integrator: '<S298>/wg_p2' */
  pid_control_V1_X.wg_p2_CSTATE[0] = 0.0;

  /* End of SystemInitialize for SubSystem: '<S283>/Hwgw(s)' */

  /* SystemInitialize for Enabled SubSystem: '<S282>/Hqgw' */
  /* InitializeConditions for Integrator: '<S294>/qgw_p' */
  pid_control_V1_X.qgw_p_CSTATE[0] = 0.0;

  /* End of SystemInitialize for SubSystem: '<S282>/Hqgw' */

  /* SystemInitialize for Enabled SubSystem: '<S283>/Hvgw(s)' */
  /* InitializeConditions for Integrator: '<S297>/vg_p1' */
  pid_control_V1_X.vg_p1_CSTATE[0] = 0.0;

  /* InitializeConditions for Integrator: '<S297>/vgw_p2' */
  pid_control_V1_X.vgw_p2_CSTATE[0] = 0.0;

  /* End of SystemInitialize for SubSystem: '<S283>/Hvgw(s)' */

  /* SystemInitialize for Enabled SubSystem: '<S282>/Hrgw' */
  /* InitializeConditions for Integrator: '<S295>/rgw_p' */
  pid_control_V1_X.rgw_p_CSTATE[0] = 0.0;

  /* End of SystemInitialize for SubSystem: '<S282>/Hrgw' */

  /* SystemInitialize for Enabled SubSystem: '<S283>/Hugw(s)' */
  /* InitializeConditions for Integrator: '<S296>/ug_p' */
  pid_control_V1_X.ug_p_CSTATE[0] = 0.0;

  /* End of SystemInitialize for SubSystem: '<S283>/Hugw(s)' */

  /* SystemInitialize for Enabled SubSystem: '<S282>/Hpgw' */
  /* InitializeConditions for Integrator: '<S293>/pgw_p' */
  pid_control_V1_X.pgw_p_CSTATE[1] = 0.0;

  /* End of SystemInitialize for SubSystem: '<S282>/Hpgw' */

  /* SystemInitialize for Enabled SubSystem: '<S283>/Hwgw(s)' */
  /* InitializeConditions for Integrator: '<S298>/wg_p1' */
  pid_control_V1_X.wg_p1_CSTATE[1] = 0.0;

  /* InitializeConditions for Integrator: '<S298>/wg_p2' */
  pid_control_V1_X.wg_p2_CSTATE[1] = 0.0;

  /* End of SystemInitialize for SubSystem: '<S283>/Hwgw(s)' */

  /* SystemInitialize for Enabled SubSystem: '<S282>/Hqgw' */
  /* InitializeConditions for Integrator: '<S294>/qgw_p' */
  pid_control_V1_X.qgw_p_CSTATE[1] = 0.0;

  /* End of SystemInitialize for SubSystem: '<S282>/Hqgw' */

  /* SystemInitialize for Enabled SubSystem: '<S283>/Hvgw(s)' */
  /* InitializeConditions for Integrator: '<S297>/vg_p1' */
  pid_control_V1_X.vg_p1_CSTATE[1] = 0.0;

  /* InitializeConditions for Integrator: '<S297>/vgw_p2' */
  pid_control_V1_X.vgw_p2_CSTATE[1] = 0.0;

  /* End of SystemInitialize for SubSystem: '<S283>/Hvgw(s)' */

  /* SystemInitialize for Enabled SubSystem: '<S282>/Hrgw' */
  /* InitializeConditions for Integrator: '<S295>/rgw_p' */
  pid_control_V1_X.rgw_p_CSTATE[1] = 0.0;

  /* End of SystemInitialize for SubSystem: '<S282>/Hrgw' */

  /* SystemInitialize for Enabled SubSystem: '<S283>/Hugw(s)' */
  /* InitializeConditions for Integrator: '<S296>/ug_p' */
  pid_control_V1_X.ug_p_CSTATE[1] = 0.0;

  /* End of SystemInitialize for SubSystem: '<S283>/Hugw(s)' */

  /* SystemInitialize for Enabled SubSystem: '<S279>/Enabled Subsystem' */
  pid_contr_EnabledSubsystem_Init(&pid_control_V1_B.EnabledSubsystem_p);

  /* End of SystemInitialize for SubSystem: '<S279>/Enabled Subsystem' */

  /* SystemInitialize for Enabled SubSystem: '<S280>/Enabled Subsystem' */
  pid_contr_EnabledSubsystem_Init(&pid_control_V1_B.EnabledSubsystem_g);

  /* End of SystemInitialize for SubSystem: '<S280>/Enabled Subsystem' */
}

/* Model terminate function */
void pid_control_V1::terminate()
{
  /* Terminate for MATLABSystem: '<S10>/SourceBlock' */
  if (!pid_control_V1_DW.obj_k.matlabCodegenIsDeleted) {
    pid_control_V1_DW.obj_k.matlabCodegenIsDeleted = true;
    if ((pid_control_V1_DW.obj_k.isInitialized == 1) &&
        pid_control_V1_DW.obj_k.isSetupComplete) {
      Sub_pid_control_V1_377.resetSubscriberPtr();//();
    }
  }

  /* End of Terminate for MATLABSystem: '<S10>/SourceBlock' */

  /* Terminate for Atomic SubSystem: '<Root>/Call Service' */
  /* Terminate for MATLABSystem: '<S2>/ServiceCaller' */
  if (!pid_control_V1_DW.obj.matlabCodegenIsDeleted) {
    pid_control_V1_DW.obj.matlabCodegenIsDeleted = true;
    if ((pid_control_V1_DW.obj.isInitialized == 1) &&
        pid_control_V1_DW.obj.isSetupComplete) {
      ServCall_pid_control_V1_326.resetSvcClientPtr();//();
    }
  }

  /* End of Terminate for MATLABSystem: '<S2>/ServiceCaller' */
  /* End of Terminate for SubSystem: '<Root>/Call Service' */
  /* Terminate for MATLABSystem: '<S279>/SourceBlock' */
  if (!pid_control_V1_DW.obj_hq.matlabCodegenIsDeleted) {
    pid_control_V1_DW.obj_hq.matlabCodegenIsDeleted = true;
    if ((pid_control_V1_DW.obj_hq.isInitialized == 1) &&
        pid_control_V1_DW.obj_hq.isSetupComplete) {
      Sub_pid_control_V1_417.resetSubscriberPtr();//();
    }
  }

  /* End of Terminate for MATLABSystem: '<S279>/SourceBlock' */

  /* Terminate for MATLABSystem: '<S280>/SourceBlock' */
  if (!pid_control_V1_DW.obj_h.matlabCodegenIsDeleted) {
    pid_control_V1_DW.obj_h.matlabCodegenIsDeleted = true;
    if ((pid_control_V1_DW.obj_h.isInitialized == 1) &&
        pid_control_V1_DW.obj_h.isSetupComplete) {
      Sub_pid_control_V1_423.resetSubscriberPtr();//();
    }
  }

  /* End of Terminate for MATLABSystem: '<S280>/SourceBlock' */
}

/* Constructor */
pid_control_V1::pid_control_V1() :
  pid_control_V1_B(),
  pid_control_V1_DW(),
  pid_control_V1_X(),
  pid_control_V1_XDis(),
  pid_control_V1_M()
{
  /* Currently there is no constructor body generated.*/
}

/* Destructor */
pid_control_V1::~pid_control_V1()
{
  /* Currently there is no destructor body generated.*/
}

/* Real-Time Model get method */
RT_MODEL_pid_control_V1_T * pid_control_V1::getRTM()
{
  return (&pid_control_V1_M);
}
