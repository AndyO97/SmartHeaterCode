/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: VoltageToPressure.c
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

#include "VoltageToPressure.h"
#ifndef UCHAR_MAX
#include <limits.h>
#endif

#if ( UCHAR_MAX != (0xFFU) ) || ( SCHAR_MAX != (0x7F) )
#error Code was generated for compiler with different sized uchar/char. \
Consider adjusting Test hardware word size settings on the \
Hardware Implementation pane to match your compiler word sizes as \
defined in limits.h of the compiler. Alternatively, you can \
select the Test hardware is the same as production hardware option and \
select the Enable portable word sizes option on the Code Generation > \
Verification pane for ERT based targets, which will disable the \
preprocessor word size checks.
#endif

#if ( USHRT_MAX != (0xFFFFU) ) || ( SHRT_MAX != (0x7FFF) )
#error Code was generated for compiler with different sized ushort/short. \
Consider adjusting Test hardware word size settings on the \
Hardware Implementation pane to match your compiler word sizes as \
defined in limits.h of the compiler. Alternatively, you can \
select the Test hardware is the same as production hardware option and \
select the Enable portable word sizes option on the Code Generation > \
Verification pane for ERT based targets, which will disable the \
preprocessor word size checks.
#endif

#if ( UINT_MAX != (0xFFFFFFFFU) ) || ( INT_MAX != (0x7FFFFFFF) )
#error Code was generated for compiler with different sized uint/int. \
Consider adjusting Test hardware word size settings on the \
Hardware Implementation pane to match your compiler word sizes as \
defined in limits.h of the compiler. Alternatively, you can \
select the Test hardware is the same as production hardware option and \
select the Enable portable word sizes option on the Code Generation > \
Verification pane for ERT based targets, which will disable the \
preprocessor word size checks.
#endif

#if ( ULONG_MAX != (0xFFFFFFFFU) ) || ( LONG_MAX != (0x7FFFFFFF) )
#error Code was generated for compiler with different sized ulong/long. \
Consider adjusting Test hardware word size settings on the \
Hardware Implementation pane to match your compiler word sizes as \
defined in limits.h of the compiler. Alternatively, you can \
select the Test hardware is the same as production hardware option and \
select the Enable portable word sizes option on the Code Generation > \
Verification pane for ERT based targets, which will disable the \
preprocessor word size checks.
#endif

/* Invariant block signals (default storage) */
const ConstB_VoltageToPressure_T VoltageToPressure_ConstB = {
  50U                                  /* '<S7>/Multiply' */
};

/* Block signals (default storage) */
B_VoltageToPressure_T VoltageToPressure_B;

/* External inputs (root inport signals with default storage) */
ExtU_VoltageToPressure_T VoltageToPressure_U;

/* External outputs (root outports fed by signals with default storage) */
ExtY_VoltageToPressure_T VoltageToPressure_Y;

/* Real-time model */
static RT_MODEL_VoltageToPressure_T VoltageToPressure_M_;
RT_MODEL_VoltageToPressure_T *const VoltageToPressure_M = &VoltageToPressure_M_;

/* Model step function */
//void VoltageToPressure_step(uint16_T  temp, uint16_T pressure)
void VoltageToPressure_step(void)
{
  int32_T tmp;
  VoltageToPressure_U.VRef = 5U;    /* 'Value for VRef' */
  VoltageToPressure_U.VSensor = 4U; /* 'Value for VSensor' */
  

  /* RelationalOperator: '<S8>/Compare' incorporates:
   *  Constant: '<S8>/Constant'
   *  Inport: '<Root>/VSensor'
   */
  VoltageToPressure_B.Compare = (VoltageToPressure_U.VSensor < 0);

  /* Switch: '<S6>/Switch' */
  if (VoltageToPressure_B.Compare) {
    /* Switch: '<S6>/Switch' incorporates:
     *  Constant: '<S6>/Constant'
     */
    VoltageToPressure_B.NotNegative = 0;
  } else {
    /* Switch: '<S6>/Switch' incorporates:
     *  Inport: '<Root>/VSensor'
     */
    VoltageToPressure_B.NotNegative = VoltageToPressure_U.VSensor;
  }

  /* End of Switch: '<S6>/Switch' */

  /* RelationalOperator: '<S9>/Compare' incorporates:
   *  Constant: '<S9>/Constant'
   */
  VoltageToPressure_B.Compare_l = (VoltageToPressure_B.NotNegative > 5);

  /* Switch: '<S6>/Switch1' */
  if (VoltageToPressure_B.Compare_l) {
    /* Switch: '<S6>/Switch1' incorporates:
     *  Constant: '<S6>/Constant1'
     */
    VoltageToPressure_B.InRangeVoltage = 5U;
  } else {
    /* DataTypeConversion: '<S6>/Data Type Conversion' */
    VoltageToPressure_B.uint16Voltage = (uint16_T)
      VoltageToPressure_B.NotNegative;

    /* Switch: '<S6>/Switch1' */
    VoltageToPressure_B.InRangeVoltage = VoltageToPressure_B.uint16Voltage;
  }

  /* End of Switch: '<S6>/Switch1' */

  /* Gain: '<S7>/Multiply1' */
  VoltageToPressure_B.Dividend = (uint16_T)(2500U *
    VoltageToPressure_B.InRangeVoltage);

  /* Product: '<S7>/Divide' */
  /*tmp = VoltageToPressure_ConstB.Divisor;*/   /* 'For Vref as a constant' */
  tmp = VoltageToPressure_U.VRef * 10U;     /* 'For Vref as a input' */

  /* Outport: '<Root>/PressureBares' incorporates:
   *  Product: '<S7>/Divide'
   */
  VoltageToPressure_Y.PressureBares = (uint16_T)((uint32_T)tmp == 0U ?
    MAX_uint32_T : (uint32_T)VoltageToPressure_B.Dividend / tmp);

  /* If: '<S1>/If' incorporates:
   *  Outport: '<Root>/PressureBares'
   */
  if (VoltageToPressure_Y.PressureBares <= 20) {
    /* Outputs for IfAction SubSystem: '<S1>/If Action Subsystem' incorporates:
     *  ActionPort: '<S3>/Action Port'
     */
    /* Merge: '<S1>/Merge' incorporates:
     *  Constant: '<S1>/Constant'
     *  Inport: '<S3>/Low'
     */
    VoltageToPressure_B.Stability = 2U;

    /* End of Outputs for SubSystem: '<S1>/If Action Subsystem' */
  } else {
    /* Outputs for IfAction SubSystem: '<S1>/If Action Subsystem2' incorporates:
     *  ActionPort: '<S5>/Action Port'
     */
    /* Outputs for IfAction SubSystem: '<S1>/If Action Subsystem1' incorporates:
     *  ActionPort: '<S4>/Action Port'
     */
    /* Merge: '<S1>/Merge' incorporates:
     *  Inport: '<S4>/Stable'
     *  Inport: '<S5>/High'
     */
    VoltageToPressure_B.Stability = (uint16_T)(VoltageToPressure_Y.PressureBares
      >= 150);

    /* End of Outputs for SubSystem: '<S1>/If Action Subsystem1' */
    /* End of Outputs for SubSystem: '<S1>/If Action Subsystem2' */
  }

  /* End of If: '<S1>/If' */

  /* Outport: '<Root>/Out1' */
  VoltageToPressure_Y.Out1 = VoltageToPressure_B.Stability;
  printf("The outputs: %u %u \n", (uint16_T)VoltageToPressure_Y.PressureBares, (uint16_T)VoltageToPressure_Y.Out1);
}

/* Model initialize function */
void VoltageToPressure_initialize(void)
{
  /* (no initialization code required) */
}

/* Model terminate function */
void VoltageToPressure_terminate(void)
{
  /* (no terminate code required) */
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
