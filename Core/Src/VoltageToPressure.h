/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: VoltageToPressure.h
 *
 * Code generated for Simulink model 'VoltageToPressure'.
 *
 * Model version                  : 1.14
 * Simulink Coder version         : 9.4 (R2020b) 29-Jul-2020
 * C/C++ source code generated on : Thu Oct 22 08:43:04 2020
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: ARM Compatible->ARM Cortex
 * Code generation objectives:
 *    1. Safety precaution
 *    2. Debugging
 * Validation result: Passed (8), Warning (1), Error (0)
 */

#ifndef RTW_HEADER_VoltageToPressure_h_
#define RTW_HEADER_VoltageToPressure_h_
#ifndef VoltageToPressure_COMMON_INCLUDES_
#define VoltageToPressure_COMMON_INCLUDES_
#include "rtwtypes.h"
#endif                                 /* VoltageToPressure_COMMON_INCLUDES_ */

/* Model Code Variants */

/* Macros for accessing real-time model data structure */
#ifndef rtmGetErrorStatus
#define rtmGetErrorStatus(rtm)         ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
#define rtmSetErrorStatus(rtm, val)    ((rtm)->errorStatus = (val))
#endif

/* Forward declaration for rtModel */
typedef struct tag_RTM_VoltageToPressure_T RT_MODEL_VoltageToPressure_T;

/* Block signals (default storage) */
typedef struct {
  int32_T NotNegative;                 /* '<S6>/Switch' */
  uint16_T InRangeVoltage;             /* '<S6>/Switch1' */
  uint16_T Dividend;                   /* '<S7>/Multiply1' */
  uint16_T Stability;                  /* '<S1>/Merge' */
  uint16_T uint16Voltage;              /* '<S6>/Data Type Conversion' */
  boolean_T Compare;                   /* '<S8>/Compare' */
  boolean_T Compare_l;                 /* '<S9>/Compare' */
} B_VoltageToPressure_T;

/* Invariant block signals (default storage) */
typedef struct {
  const uint16_T Divisor;              /* '<S7>/Multiply' */
} ConstB_VoltageToPressure_T;

/* External inputs (root inport signals with default storage) */
typedef struct {
  int32_T VSensor;                     /* '<Root>/VSensor' */
  uint16_T VRef;
} ExtU_VoltageToPressure_T;

/* External outputs (root outports fed by signals with default storage) */
typedef struct {
  uint16_T PressureBares;              /* '<Root>/PressureBares' */
  uint16_T Out1;                       /* '<Root>/Out1' */
} ExtY_VoltageToPressure_T;

/* Real-time Model Data Structure */
struct tag_RTM_VoltageToPressure_T {
  const char_T * volatile errorStatus;
};

/* Block signals (default storage) */
extern B_VoltageToPressure_T VoltageToPressure_B;

/* External inputs (root inport signals with default storage) */
extern ExtU_VoltageToPressure_T VoltageToPressure_U;

/* External outputs (root outports fed by signals with default storage) */
extern ExtY_VoltageToPressure_T VoltageToPressure_Y;
extern const ConstB_VoltageToPressure_T VoltageToPressure_ConstB;/* constant block i/o */

/* Model entry point functions */
extern void VoltageToPressure_initialize(void);
extern void VoltageToPressure_step(void);
extern void VoltageToPressure_terminate(void);

/* Real-time Model object */
extern RT_MODEL_VoltageToPressure_T *const VoltageToPressure_M;

/*-
 * The generated code includes comments that allow you to trace directly
 * back to the appropriate location in the model.  The basic format
 * is <system>/block_name, where system is the system number (uniquely
 * assigned by Simulink) and block_name is the name of the block.
 *
 * Use the MATLAB hilite_system command to trace the generated code back
 * to the model.  For example,
 *
 * hilite_system('<S3>')    - opens system 3
 * hilite_system('<S3>/Kp') - opens and selects block Kp which resides in S3
 *
 * Here is the system hierarchy for this model
 *
 * '<Root>' : 'VoltageToPressure'
 * '<S1>'   : 'VoltageToPressure/FaultDetection'
 * '<S2>'   : 'VoltageToPressure/VpoltageToPressure'
 * '<S3>'   : 'VoltageToPressure/FaultDetection/If Action Subsystem'
 * '<S4>'   : 'VoltageToPressure/FaultDetection/If Action Subsystem1'
 * '<S5>'   : 'VoltageToPressure/FaultDetection/If Action Subsystem2'
 * '<S6>'   : 'VoltageToPressure/VpoltageToPressure/CodeProtection'
 * '<S7>'   : 'VoltageToPressure/VpoltageToPressure/TransferFunction'
 * '<S8>'   : 'VoltageToPressure/VpoltageToPressure/CodeProtection/Compare To Constant'
 * '<S9>'   : 'VoltageToPressure/VpoltageToPressure/CodeProtection/Compare To Constant1'
 */

/*-
 * Requirements for '<Root>': VoltageToPressure
 */
#endif                                 /* RTW_HEADER_VoltageToPressure_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
