/*-----------------------------------------------------------------------------
 * EcMotionControl.h
 * Copyright                acontis technologies GmbH, Ravensburg, Germany
 * Response                 Christoph Widmann
 *                          mdu
 * Description              Motion Control header
 *
 *
 * History:
 * ============================================================================
 * Date    | ID| Description
 * ============================================================================
 * 20Jan23 | 26| Enhancement: Functions blocks for GearIn and GearOut
 * ----------------------------------------------------------------------------
 * 30Oct19 | 25| Fix: MC_Power and DS402StateMachine()
 *         |   | In transition 9 check for axis fault. In case of fault, set error and set DS402State and PLCOpenState
 *         |   | Background. Issue with Beckhoff AX8000.
 *         |   | Transition to "Operation enabled" will fail in case the motor is still moving.
 *         |   | The drive is reporting "Fault" in the status word and this has to be handled.
 * ----------------------------------------------------------------------------
 * 02Sep19 | 24| CSV: Cyclic Synchronous Velocity Mode
 *         |   | MC_DriveSetTargetStep: Output velocity calculated by trajectory generator.
 *         |   | Previous implementation was based on difference between Targetposition and
 *         |   | Actualpostion using a gain factor.
 *         |   | Important hint: The function blocks MoveAbsolute and MoveRelative are not
 *         |   | positioning the drive to the exact position anymore!
 * ----------------------------------------------------------------------------
 * 02Sep19 | 23| MC_MOVE_VELOCITY_T, MC_CheckTargetVelReached
 *         |   | Output InVelocity is set based von Actual Velocity.
 *         |   | Previous implementation was based on Commanded Velocity.
 * ----------------------------------------------------------------------------
 * 02Jul19 | 22| Fix: MC_Power and DS402StateMachine()
 *         |   | Implement transition 9 to state DRV_DEV_STATE_SWITCHON_DIS in drive state "Operation enabled".
 *         |   | Immediately in the same cycle. (bStatus is MC_TRUE)
 *         |   | Use case: Drive will switch from DRV_DEV_STATE_OP_ENABLED to DRV_DEV_STATE_SWITCHON_DIS in case of STO
 * ----------------------------------------------------------------------------
 * 11Jan19 | 21| Fix: MC_Power and DS402StateMachine()
 *         |   | Implement transition 9 to state DRV_DEV_STATE_SWITCHON_DIS in drive state "Operation enabled".
 *         |   | Just in case if bStatus is already MC_FALSE.
 * ----------------------------------------------------------------------------
 * 13Dec18 | 20| New: MC_Power and DS402StateMachine()
 *         |   | Power-Off Sequence: Clear Status flag after DRV_STAT_RDY_SWITCH_ON, DRV_STAT_SWITCHED_ON,
 *         |   | DRV_STAT_OP_ENABLED and DRV_STAT_FAULT are zero
 * ----------------------------------------------------------------------------
 * 25Oct18 | 19| New: MC_Power and DS402StateMachine()
 *         |   | New output MC_POWER_T.FaultReaction signals that the bit DRV_STAT_FAULT is still set by the drive.
 *         |   | The DS402StateMachine stays in the state DRV_DEV_STATE_MALFCT_REACTION until the bit is cleared.
 * ----------------------------------------------------------------------------
 * 25Oct18 | 18| Fix: DS402StateMachine()
 *         |   | In case the drive sets the bit DRV_STAT_QUICK_STOP in the status word,
 *         |   | no error in McPower is set anymore. Also the output Status stays TRUE.
 * ----------------------------------------------------------------------------
 * 05Jun18 | 17| Fix: DS402StateMachine()
 *         |   | In case the drive sets the bit DRV_STAT_QUICK_STOP in the status word,
 *         |   | no error in McPower is set anymore
 * ----------------------------------------------------------------------------
 * 28Mar18 | 16| Fix: MC_TrajSetupTrapezeMove() and MC_TrajSetupJerkMove()
 *         |   | Prevent overflow when calculating "reduced constant vel"
 *         |   | Calc is now floating point and cast to type MC_T_VELACC only for the result
 * ----------------------------------------------------------------------------
 * 05Mar18 | 15| Fix: DS402StateMachine().
 *         |   | State:DRV_DEV_STATE_SWITCHED_ON: Set Error in case drive signals DRV_STAT_FAULT
 *         |   | State:DRV_DEV_STATE_MALFUNCTION: Clear always "Status" flag
 *         |   | Use Case: Drive is switched off by application immediately before fault state
 * ----------------------------------------------------------------------------
 * 20Oct17 | 14| Fix: DS402StateMachine(). Power-off sequence will wait until drive
 *         |   | is ready for lower state.
 * ----------------------------------------------------------------------------
 * 22Aug17 | 13| New: Actual position is now considering overruns.
 *         |   | MC_DriveGetInputs(): Introduced nActualPosOverrunCounter
 * ----------------------------------------------------------------------------
 * 16May17 | 12| Fix in DS402StateMachine: Transition from DRV_DEV_STATE_SWITCHON_DIS to DRV_DEV_STATE_READY_TO_SWITCHON
 *         |   | always commands the value DRV_CTRL_CMD_SHUTDOWN into the control word
 * ----------------------------------------------------------------------------
 * 30Sep16 | 11| Fix in Traj.generator. Buffered Modes with MoveAbsolute and MC_BLENDING_PREVIOUS
 *         |   | Final target pos. now reached
 * ----------------------------------------------------------------------------
 * 09Aug16 | 10| New FB: MC_MOVE_CONT_ABSOLUTE_T, MC_MOVE_CONT_RELATIVE_T
 * ----------------------------------------------------------------------------
 * 27Jul16 |  9| First Release Camming Function Blocks
 * ----------------------------------------------------------------------------
 * 14Jul16 |  8| New API MC_CalcMoveProfileBuffered()
 * ----------------------------------------------------------------------------
 * 22Jan16 |  7| MC_HOME_T
 * ----------------------------------------------------------------------------
 * 14Dec15 |  6| Profile position mode, Digital I/Os
 * ----------------------------------------------------------------------------
 * 11Dec15 |  5| MC_POWER_T::DS402StateMachine: Check for following error in DRV_DEV_STATE_OP_ENABLED
 * ----------------------------------------------------------------------------
 * 11Dec15 |  4| Fix in Traj.generator: IncFactor is not considered anymore for calculating the
 *         |   | maximum possible acc and dec. MC_MoveSetupCheckMaxAcc() and MC_MoveSetupCheckMaxDec()
 * ----------------------------------------------------------------------------
 * 04Dec15 |  3| MC_POWER_T::DS402StateMachine: Clear Error Status in DRV_DEV_STATE_NOT_READY
 *         |   | in case Enable is FALSE
 * ----------------------------------------------------------------------------
 * 01Dec15 |  2| MC_POWER_T::DS402StateMachine: Clear Error Status in case of following error
 * ----------------------------------------------------------------------------
 * 11Jun15 |  1| Support for torque feed forward
 * ----------------------------------------------------------------------------
 *
 *---------------------------------------------------------------------------*/

#ifndef __ECMOTIONCONTROL_H__
#define __ECMOTIONCONTROL_H__   1

//#define MOTIONTEST
/*-INCLUDES------------------------------------------------------------------*/
#include "EcLog.h"

/*-PACK SETTINGS-------------------------------------------------------------*/

#include EC_PACKED_INCLUDESTART(8)

#if defined  __GNUC__
#  undef EC_PACKED /* GCC throws an error if we add the pack attribute. Redefine without pack. */
#  define EC_PACKED(alignment)  __attribute__(( aligned(alignment) ))
#endif

#define _MC_PACKED EC_PACKED(8)

/*-MACROS--------------------------------------------------------------------*/

#define ASSIGNOP(cname) cname & operator=(const cname &) { return *this; }

/*-DEFINES-------------------------------------------------------------------*/

#if defined(ECMOTIONLIB_BUILD) || defined(MOTIONTEST)
# define ACCESSMOD public
#else
# define ACCESSMOD private
#endif

#if defined(EC_DLL)
#if defined(EC_EXPORTS)
#define _MC_API __declspec(dllexport)
#else
#define _MC_API __declspec(dllimport)
#endif
#else
#define _MC_API       /* NOOP */
#endif
#define _MC_THIS_API  /* NOOP */

#define EC_MOTION_SUPPORT_PP_MODE         /* Support for drive mode 1 = Profile Position Mode */
/* #define EC_MOTION_SUPPORT_MC_POSITION_PROFILE */
#define EC_MOTION_SUPPORT_MC_CAMMING
#define EC_MOTION_SUPPORT_MC_GEARING


/* DS402 object identifiers */
#define DRV_OBJ_ERROR_CODE                  0x603F
#define DRV_OBJ_CONTROL_WORD                0x6040
#define DRV_OBJ_STATUS_WORD                 0x6041
#define DRV_OBJ_MODES_OF_OPERATION          0x6060
#define DRV_OBJ_MODES_OF_OPERATION_DISPLAY  0x6061
#define DRV_OBJ_POSITION_ACTUAL_VALUE       0x6064
#define DRV_OBJ_POSITION_WINDOW             0x6067
#define DRV_OBJ_POSITION_WINDOW_TIME        0x6068
#define DRV_OBJ_VELOCITY_ACTUAL_VALUE       0x606C
#define DRV_OBJ_ACTUAL_TORQUE               0x6077
#define DRV_OBJ_TARGET_TORQUE               0x6071
#define DRV_OBJ_TORQUE_ACTUAL_VALUE         0x6077
#define DRV_OBJ_TARGET_POSITION             0x607A
#define DRV_OBJ_POSITION_RANGE_LIMIT        0x607B
#define DRV_IDN_POSITION_RANGE_LIMIT_MIN    1
#define DRV_IDN_POSITION_RANGE_LIMIT_MAX    2
#define DRV_OBJ_SOFTWARE_POSITION_LIMIT     0x607D
#define DRV_IDN_SOFTWARE_POSITION_LIMIT_MIN 1
#define DRV_IDN_SOFTWARE_POSITION_LIMIT_MAX 2

#define DRV_OBJ_PROFILE_VELOCITY            0x6081
#define DRV_OBJ_PROFILE_ACC                 0x6083
#define DRV_OBJ_PROFILE_DEC                 0x6084
#define DRV_OBJ_MOTION_PROFILE_TYPE         0x6086
#define DRV_OBJ_POS_ENCODER_RESOLUTION      0x608F
#define DRV_OBJ_POS_FACTOR                  0x6093

#define DRV_OBJ_HOMING_METHOD               0x6098
#define DRV_OBJ_HOMING_SPEED                0x6099
#define DRV_IDN_HOMING_SEARCH_SPEED_SWITCH  1
#define DRV_IDN_HOMING_SEARCH_SPEED_ZERO    2
#define DRV_OBJ_HOMING_ACCELERATION         0x609A
#define DRV_OBJ_HOMING_OFFSET               0x607C

#define DRV_OBJ_PROFILE_JERK_USE            0x60A3
#define DRV_OBJ_PROFILE_JERK                0x60A4


/* In cyclic synchronous position mode, this object contains the input
value for velocity feed forward. In cyclic synchronous velocity mode it
contains the commanded offset of the drive device. */
#define DRV_OBJ_VELOCITY_OFFSET             0x60B1

/* In cyclic synchronous position mode and cyclic synchronous velocity
mode, this object contains the input value for torque feed forward. In cyclic
synchronous torque mode it contains the commanded additive torque of the
drive, which is added to the target torque value. */
#define DRV_OBJ_TORQUE_OFFSET               0x60B2

#define DRV_OBJ_POS_OPTION_MODE             0x60F2
#define DRV_OBJ_DIGITAL_INPUTS              0x60FD
#define DRV_OBJ_DIGITAL_OUTPUTS             0x60FE

/* This object shall indicate the configured target velocity and shall be used as input for the
trajectory generator. */
#define DRV_OBJ_TARGET_VELOCITY             0x60FF

/* SERCOS IDN's */
#define DRV_IDN_CONTROL_WORD                134  /* S-0-0134 */
#define DRV_IDN_STATUS_WORD                 135  /* S-0-0135 */
#define DRV_IDN_OPMODE                      32   /* S-0-0032 */
#define DRV_IDN_POSITION_ACTUAL_VALUE       51   /* S-0-0051 */
#define DRV_IDN_VELOCITY_ACTUAL_VALUE       40   /* S-0-0040 */
#define DRV_IDN_TARGET_POSITION             47   /* S-0-0047 */
#define DRV_IDN_TARGET_VELOCITY             36   /* S-0-0036 */
#define DRV_IDN_ACTUAL_TORQUE               84   /* S-0-0084 */
#define DRV_IDN_TARGET_TORQUE               80   /* S-0-0080 */
#define DRV_IDN_CLEAR_FAULT                 99   /* S-0-0099 */

/* DS402 object 0x6040: Control word */
#define DRV_CRTL_SWITCH_ON              0x0001          /* Bit 0: */
#define DRV_CRTL_ENABLE_VOLTAGE         0x0002          /* Bit 1: */
#define DRV_CRTL_QUICK_STOP             0x0004          /* Bit 2: */
#define DRV_CRTL_ENABLE_OP              0x0008          /* Bit 3: */
#define DRV_CTRL_INTER_POS_ENA          0x0010          /* Bit 4: Interpolated position mode: enable interpolation */
#define DRV_CTRL_PP_NEW_SET_POINT       0x0010          /* Bit 4: Profile position mode: Set new point */
#define DRV_CTRL_START_HOMING           0x0010          /* Bit 4: Homing mode: Start homing */
#define DRV_CTRL_PP_IMM_SET_POINT       0x0020          /* Bit 5: Profile position mode: Change set point immediately */
#define DRV_CRTL_FAULT_RESET            0x0080          /* Bit 7: */
#define DRV_CRTL_HALT                   0x0100          /* Bit 8: */
#define DRV_CRTL_OP_MODE_SPEC           0x0200          /* Bit 9: */
#define DRV_CRTL_RES_10                 0x0400          /* Bit 10: */
#define DRV_CRTL_MANU_SPEC              0xF800          /* Bit 11-15: */
/* DS402 drive/device control commands */
#define DRV_CTRL_CMD_MASK               0x008F          /* Control commands Mask */
#define DRV_CTRL_CMD_SHUTDOWN           0x0006          /* Shutdown (Transition 2, 6, 8) */
#define DRV_CTRL_CMD_SWITCHON           0x0007          /* Switch On (Transition 3) */
#define DRV_CTRL_CMD_DIS_VOLTAGE        0x0000          /* Disable Voltage (Transition 7, 9, 10, 12) */
#define DRV_CTRL_CMD_DIS_VOLTAGE_MASK   0x0082          /* Disable Voltage Mask */
#define DRV_CTRL_CMD_QUICK_STOP         0x0002          /* Quick Stop (Transition 7, 10, 11) */
#define DRV_CTRL_CMD_QUICK_STOP_MASK    0x0086          /* Disable Voltage Mask */
#define DRV_CTRL_CMD_DIS_OPERATION      0x0007          /* Disable Operation (Transition 5) */
#define DRV_CTRL_CMD_ENA_OPERATION      0x000F          /* Enable Operation (Transition 4) */
#define DRV_CTRL_CMD_RESET_MALFCT       DRV_CRTL_FAULT_RESET          /* Reset Malfunction (0->1 edge ) (Transition 15) */

/* DS402 object 0x6041: Status word */
#define DRV_STAT_RDY_SWITCH_ON          0x0001          /* Bit 0: Ready to switch on */
#define DRV_STAT_SWITCHED_ON            0x0002          /* Bit 1: Switched On */
#define DRV_STAT_OP_ENABLED             0x0004          /* Bit 2: Operation enabled */
#define DRV_STAT_FAULT                  0x0008          /* Bit 3: Fault */
#define DRV_STAT_VOLTAGE_ENABLED        0x0010          /* Bit 4: Optional bit: Voltage enabled */
#define DRV_STAT_QUICK_STOP             0x0020          /* Bit 5: Optional bit: Quick stop      */
#define DRV_STAT_SWITCH_ON_DIS          0x0040          /* Bit 6: Switch on disabled */
#define DRV_STAT_STATUS_TOGGLE          0x0400          /* Bit 10: Optional bit: Status toggle (csp, csv mode) */
#define DRV_STAT_VELOCITY_ZERO          0x0400          /* Bit 10: Optional bit: Velocity 0 (pp and ip mode) */
#define DRV_STAT_TARGET_REACHED         0x0400          /* Bit 10: Homing: Target reached */
#define DRV_STAT_FOLLOW_CMD_VALUE       0x1000          /* Bit 12: Drive follows the command value (csp, csv, cst) */
#define DRV_STAT_OP_MODE_PP_ACK         0x1000          /* Bit 12: PP mode: Set new point acknowledge */
#define DRV_STAT_HOMING_ATTAINED        0x1000          /* Bit 12: Homing mode: attained */
#define DRV_STAT_FOLLOW_ERR             0x2000          /* Bit 13: Optional bit: Following error (csp, csv mode) */
#define DRV_STAT_HOMING_ERROR           0x2000          /* Bit 13: Homing: Homing error */
#define DRV_STAT_RUNNING                0x4000          /* Bit 14: Running */
#define DRV_STAT_IDLE                   0x8000          /* Bit 15: Idle */

/* SERCOS IDN S-0-0134: MDT (Master control word) */
#define DRV_SER_CRTL_HLT                0x2000          /* Bit 13: Halt/restart drive */
#define DRV_SER_CRTL_ENABLED            0x4000          /* Bit 14: Enable drive*/
#define DRV_SER_CRTL_ON                 0x8000          /* Bit 15: Drive On/Off */
#define DRV_SER_CRTLMASK                0xE000          /* Mask bit 13-15 */

/* SERCOS IDN S-0-0135: AT (Drive status word) */
#define DRV_SER_STAT_FAULT                   0x2000          /* Bit 13: Drive shut-down error in C10 */
#define DRV_SER_STAT_OP_NOT_READY            0x0000          /* Bit 14:15: Drive not ready to power up */
#define DRV_SER_STAT_OP_READY_TO_SWITCHON    0x4000          /* Bit 14:15: Drive logic ready for main power on */
#define DRV_SER_STAT_OP_SWITCHED_ON          0x8000          /* Bit 14:15: Drive ready and main power applied */
#define DRV_SER_STAT_OP_ENABLED              0xC000          /* Bit 14:15: Drive ready to operate */
#define DRV_SER_STAT_OPMASK                  0xC000          /* Mask bit 14:15 */


#define PLCOpenStateText(EState)                     \
    ((EState)==MC_PLCOPEN_STATE_UNKNOWN?     "Unknown            ": \
    ((EState)==MC_PLCOPEN_STATE_STAND_STILL? "Standstill         ": \
    ((EState)==MC_PLCOPEN_STATE_HOMING?      "Homing             ": \
    ((EState)==MC_PLCOPEN_STATE_STOPPING?    "Stopping           ": \
    ((EState)==MC_PLCOPEN_STATE_ERROR_STOP?  "Error Stop         ": \
    ((EState)==MC_PLCOPEN_STATE_DISC_MOTION? "Discrete Motion    ": \
    ((EState)==MC_PLCOPEN_STATE_CONT_MOTION? "Continous Motion   ": \
    ((EState)==MC_PLCOPEN_STATE_SYNC_MOTION? "Synchronized Motion": \
    ((EState)==MC_PLCOPEN_STATE_DISABLED?    "Disabled           ": "INVALID STATE VALUE!!!" )))))))))


/* Error IDs found in 'wErrorID' */
#define MC_NO_ERROR                     0       /* no error */
#define MC_SZTXT_NO_ERROR                       "No Error"

#define MC_ERR_INVALID_PARAMETER        1       /* invalid function parameter, e. g., NULL pointer */
#define MC_SZTXT_ERR_INVALID_PARAMETER          "ERROR: Invalid parameter"

#define MC_ERR_INVALID_OPERATION_MODE   2       /* invalid operation mode (MC_T_AXIS_REF.wProfileOperationMode) */
#define MC_SZTXT_ERR_INVALID_OPERATION_MODE     "ERROR: Invalid operation mode"

#define MC_ERR_INVALID_PLCOPEN_STATE    3       /* invalid axis state (MC_T_AXIS_REF.ePLCOpenState) */
#define MC_SZTXT_ERR_INVALID_PLCOPEN_STATE      "ERROR: Invalid axis state"

#define MC_ERR_FOLLOWING                4       /* axis signals a following error (bit 13 in status word) */
#define MC_SZTXT_ERR_FOLLOWING                  "ERROR: Axis signals a following error (bit 13 in status word)"

#define MC_ERR_INVALID_STATUS_WORD      5       /* status word changed while in "operation enabled" */
#define MC_SZTXT_ERR_INVALID_STATUS_WORD        "ERROR: Status word changed while in 'operation enabled'"

#define MC_ERR_AXIS_FAULT               6       /* axis fault/malfunction (bit 3 status word) */
#define MC_SZTXT_ERR_AXIS_FAULT                 "ERROR: Axis fault/malfunction (bit 3 status word)"

#define MC_ERR_BUFFER_OVERFLOW          7       /* axis fault/malfunction (bit 3 status word) */
#define MC_SZTXT_ERR_BUFFER_OVERFLOW            "ERROR: Buffer overflow"

#define MC_ERR_PARA_NOT_SUPPORTED       8       /* parameter number not supported */
#define MC_SZTXT_ERR_PARA_NOT_SUPPORTED         "ERROR: Parameter number not supported"

#define MC_ERR_SW_LIMIT_POS             9       /* software limit positive */
#define MC_SZTXT_ERR_SW_LIMIT_POS               "ERROR: Positive software limit touched"

#define MC_ERR_SW_LIMIT_NEG            10       /* software limit negative */
#define MC_SZTXT_ERR_SW_LIMIT_NEG               "ERROR: Negative software limit touched"

#define MC_ERR_INVALID_DIRECTION       11       /* direction value invalid */
#define MC_SZTXT_ERR_INVALID_DIRECTION          "ERROR: Direction value invalid"

#define MC_ERR_DIST_OUT_OF_RANGE        12
#define MC_SZTXT_ERR_DIST_OUT_OF_RANGE          "ERROR: Distance out of range"

#define MC_ERR_VEL_OUT_OF_RANGE         13
#define MC_SZTXT_ERR_VEL_OUT_OF_RANGE           "ERROR: Velocity out of range"

#define MC_ERR_ACC_OUT_OF_RANGE         14
#define MC_SZTXT_ERR_ACC_OUT_OF_RANGE           "ERROR: Acceleration out of range"

#define MC_ERR_DEC_OUT_OF_RANGE         15
#define MC_SZTXT_ERR_DEC_OUT_OF_RANGE           "ERROR: Deceleration out of range"

#define MC_ERR_JERK_OUT_OF_RANGE        16
#define MC_SZTXT_ERR_JERK_OUT_OF_RANGE          "ERROR: Jerk out of range"

#define MC_ERR_CALC_JERK_ACCDEC_MIN    20       /* calc jerk limited. no solution acc and dec equal one */
#define MC_SZTXT_ERR_CALC_JERK_ACCDEC_MIN       "ERROR: Calc jerk limited - no solution acc and dec equal one"

#define MC_ERR_CALC_JERK_TOMANY_CYC    21       /* calc jerk limited. no solution to many cycles */
#define MC_SZTXT_ERR_CALC_JERK_TOMANY_CYC       "ERROR: Calc jerk limited - no solution to many cycles"

#define MC_ERR_OVERFLOW                22       /* overflow */
#define MC_SZTXT_ERR_OVERFLOW                   "ERROR: Overflow"

#define MC_ERR_BUFFERMODE_NOT_SUPP     23       /* buffer mode not supported */
#define MC_SZTXT_ERR_BUFFERMODE_NOT_SUPP        "ERROR: Buffer mode not supported"

#define MC_ERR_CALC_DIST_NOT_MATCHING  24       /* calc jerk limited. Segment distances not equal to total distance */
#define MC_SZTXT_ERR_CALC_DIST_NOT_MATCHING     "ERROR: Calc jerk limited - segment distances not equal to total distance"

#define MC_ERR_CALC_VEL_NOT_MATCHING   25       /* calc jerk limited. Velocites are not matching. */
#define MC_SZTXT_ERR_CALC_VEL_NOT_MATCHING     "ERROR: Calc jerk limited - Velocites are not matching"

#define MC_ERR_DIR_CHANGED             26       /* change of direction not possible */
#define MC_SZTXT_ERR_DIR_CHANGED                "ERROR: Change of direction not possible"

#define MC_ERR_CONTUPDATE_DIR_CHANGED  27       /* cont. update: change of direction not possible */
#define MC_SZTXT_ERR_CONTUPDATE_DIR_CHANGED     "ERROR: Continuous update - change of direction not possible"

#define MC_ERR_CONTUPDATE_DIST_CHANGED 28       /* cont. update: distance change not possible */
#define MC_SZTXT_ERR_CONTUPDATE_DIST_CHANGED    "ERROR: Continuous update - distance change not possible"

#define MC_ERR_CALC_CONSTANT_VEL_TO_HIGH    30  /* traj. calc: constant velocity to high */
#define MC_SZTXT_ERR_CALC_CONSTANT_VEL_TO_HIGH  "ERROR: Calc: Constant velocity is to high"

#define MC_ERR_CALC_CONSTANT_VEL_TO_LOW    31  /* traj. calc: constant velocity to low */
#define MC_SZTXT_ERR_CALC_CONSTANT_VEL_TO_LOW  "ERROR: Calc: Constant velocity is to low"

#define MC_ERR_CALC_END_VEL_TO_HIGH         32  /* traj. calc: end velocity to high */
#define MC_SZTXT_ERR_CALC_END_VEL_TO_HIGH  "ERROR: Calc: End (final) velocity is to high"

#define MC_ERR_CALC_END_VEL_TO_LOW         33  /* traj. calc: end velocity to low */
#define MC_SZTXT_ERR_CALC_END_VEL_TO_LOW  "ERROR: Calc: End (final) velocity is to low"

#define MC_ERR_CALC_RAMPS_TO_BIG            34  /* traj. calc: total sum of ramps bigger than distance */
#define MC_SZTXT_ERR_CALC_RAMPS_TO_BIG     "ERROR: Calc: Total sum of ramps bigger than distance"

#define MC_ERR_INVALID_RESOLUTION           40  /* initialization not complete: increments per mm not set */
#define MC_SZTXT_ERR_INVALID_RESOLUTION    "ERROR: Parameter 'dwIncPerMM: increments per mm' is zero"

#define MC_ERR_INVALID_CYCLETIME            41  /* initialization not complete: cycle time not set */
#define MC_SZTXT_ERR_INVALID_CYCLETIME     "ERROR: Parameter 'dwCycleTime: bus cycle time' is zero"

#define MC_ERR_BUSY_STOPPING                42  /* FB reports an error indicating the busy MC_Stop command */
#define MC_SZTXT_ERR_BUSY_STOPPING         "ERROR: Emergency stopping of axis in progress. Commands inhibited"

#define MC_ERR_INVALID_INPUT_NUM            43  /* invalid input number */
#define MC_SZTXT_ERR_INVALID_INPUT_NUM     "ERROR: Invalid input number. Valid 0 to 31"

#define MC_ERR_NO_INPUT_DATA_PTR            44  /* invalid input data pointer number */
#define MC_SZTXT_ERR_NO_INPUT_DATA_PTR     "ERROR: Pointer to input data not set. Check InitInputs()"

#define MC_ERR_INVALID_OUTPUT_NUM           45  /* invalid output number */
#define MC_SZTXT_ERR_INVALID_OUTPUT_NUM    "ERROR: Invalid output number. Valid 0 to 31"

#define MC_ERR_NO_OUTPUT_DATA_PTR           46  /* invalid output data pointer number */
#define MC_SZTXT_ERR_NO_OUTPUT_DATA_PTR    "ERROR: Pointer to output data not set. Check InitOutputs()"

#define MC_ERR_NOT_FOLLOW_CMD_VALUE         47  /* axis does not follow commanded value (bit 12 in status word) */
#define MC_SZTXT_NOT_FOLLOW_CMD_VALUE       "ERROR: Axis does not follow commanded value (bit 12 in status word)"


#define MC_ERR_TP_REF_PAIRS_ZERO            50
#define MC_ERR_TP_REF_DELTATIME_ZERO        51
#define MC_ERR_TP_REF_DELTATIME_NEG         52

#define MC_ERR_EXTENDED                     60      /* Drive Controller error */
#define MC_SZTXT_ERR_EXTENDED               "ERROR: Extended error code. Check GetErrorCode()"

#define MC_ERR_HOMING                       61      /* Error during homing */
#define MC_SZTXT_ERR_HOMING                 "ERROR: Homing error"

#if defined(EC_MOTION_SUPPORT_MC_CAMMING) || defined(EC_MOTION_SUPPORT_MC_GEARING)
#define MC_ERR_MASTER_AXIS_INVALID           70      /* Input parameter Master Axis is missing */
#define MC_SZTXT_ERR_MASTER_AXIS_INVALID     "ERROR: Invalid parameter. MasterAxis not specified"

#define MC_ERR_INVALID_PLCOPEN_STATE_SLAVE    73    /* invalid slave axis state (MC_T_AXIS_REF.ePLCOpenState) */
#define MC_SZTXT_ERR_INVALID_PLCOPEN_STATE_SLAVE    "ERROR: Invalid slave axis state"

#endif

#ifdef EC_MOTION_SUPPORT_MC_CAMMING
#define MC_ERR_CAM_TABLE_ID_INVALID         71      /* Input parameter CamTableID is missing */
#define MC_SZTXT_ERR_CAM_TABLE_ID_INVALID   "ERROR: Invalid parameter. CamTableID not specified"

#define MC_ERR_CAM_SPLINE_INVALID           72      /* Memory for spline is missing */
#define MC_SZTXT_ERR_CAM_SPLINE_INVALID     "ERROR: Memory for spline is missing"

#define MC_ERR_START_MODE_INVALID           74      /* Input parameter StartMode not supported */
#define MC_SZTXT_ERR_START_MODE_INVALID     "ERROR: Invalid parameter. StartMode not supported "

#define MC_ERR_CAM_TABLE_ELEM_TOO_LESS         75
#define MC_SZTXT_ERR_CAM_TABLE_ELEM_TOO_LESS  "ERROR: Invalid parameter. CAM Table: To less elements. Minimum is 2"

#define MC_ERR_CAM_TABLE_ELEM_TOO_MANY        76
#define MC_SZTXT_ERR_CAM_TABLE_ELEM_TOO_MANY    "ERROR: Invalid parameter. CAM Table: Number of elements too many"

#define MC_ERR_CAM_TABLE_TYPE_INVALID         77
#define MC_SZTXT_ERR_CAM_TABLE_TYPE_INVALID   "ERROR: Invalid parameter. CAM Table: Type invalid"

#define MC_ERR_CAM_TABLE_VARTYPE_INVALID         78
#define MC_SZTXT_ERR_CAM_TABLE_VARTYPE_INVALID   "ERROR: Invalid parameter. CAM Table: Variable type invalid"

#define MC_ERR_CAM_TABLE_INTERPOL_INVALID         79
#define MC_SZTXT_ERR_CAM_TABLE_INTERPOL_INVALID   "ERROR: Invalid parameter. CAM Table: Interpolation type invalid"

#define MC_ERR_CAM_TABLE_DATA_INVALID             80
#define MC_SZTXT_ERR_CAM_TABLE_DATA_INVALID     "ERROR: Invalid parameter. CAM Table: Data pointer invalid"

#define MC_ERR_CAM_IN_MASPOS_TOO_SMALL             81
#define MC_SZTXT_ERR_CAM_IN_MASPOS_TOO_SMALL     "ERROR: CAM In: Master position below start position"

#define MC_ERR_CAM_IN_MASPOS_TOO_BIG             82
#define MC_SZTXT_ERR_CAM_IN_MASPOS_TOO_BIG      "ERROR: CAM In: Master position above end position"

#define MC_ERR_CAM_IN_MAS_PERIOD_ZERO             83
#define MC_SZTXT_ERR_CAM_IN_MAS_PERIOD_ZERO     "ERROR: CAM In: Master period (start --> end) is zero"

#define MC_ERR_CAM_IN_MAS_PERIOD_NEGA             84
#define MC_SZTXT_ERR_CAM_IN_MAS_PERIOD_NEGA     "ERROR: CAM In: Master period is negative (start > end)"

#define MC_ERR_CAM_IN_MASPOS_NULL_MISSING         85
#define MC_SZTXT_ERR_CAM_IN_MASPOS_NULL_MISSING   "ERROR: CAM In: MasterAbsolute is FALSE, but master range not from zero"

#endif /* EC_MOTION_SUPPORT_MC_CAMMING */

#ifdef EC_MOTION_SUPPORT_MC_GEARING

#define MC_ERR_GEAR_PARA_INVALID         90      /* Input parameter Gear parameter is missing */
#define MC_SZTXT_ERR_GEAR_PARA_INVALID   "ERROR: Invalid parameter. Parameters for gearing not defined"

#define MC_ERR_GEAR_INVALID_MAS_STATE    91      /* Invalid master state for GearIn */
#define MC_SZTXT_ERR_GEAR_INVALID_MAS_STATE   "ERROR: Invalid master state for gearing in."

#endif /* EC_MOTION_SUPPORT_MC_GEARING*/


#define MC_FB_BUFFER_LEN                16
#define STEP_BUFFER_SIZE                16
#define EC_VENDOR_ID_POLLMEIER          0x0000000f
#define EC_SDO_UP_DOWN_TIMEOUT          3000         /* timeout to wait until SDO transfers should have finished */

/* DS402 object 0x60F2: Position Option Code */
#define DRV_POC_MOVE_MODE_MASK          0x00C0          /* Bit 6 and 7: Mask */
#define DRV_POC_MOVE_MODE_NORMAL        0x0000          /* Bit 6 and 7: 00: normal positioning similar to linear axis */
#define DRV_POC_MOVE_MODE_NEGATIVE      0x0040          /* Bit 6 and 7: 01: positioning only in negative direction */
#define DRV_POC_MOVE_MODE_POSITIVE      0x0040          /* Bit 6 and 7: 10: positioning only in positive direction */
#define DRV_POC_MOVE_MODE_SHORTEST      0x00C0          /* Bit 6 and 7: 11: positioning with the shortest way */

/*-ENUMS---------------------------------------------------------------------*/

/* DS402 modes of operation 0x6060 */
enum MC_T_CIA402_OPMODE
{
   DRV_MODE_OP_PROF_POS            = 1,          /* profile position mode */
   DRV_MODE_OP_VELOCITY            = 2,          /* velocity mode (frequency converter) */
   DRV_MODE_OP_PROF_VEL            = 3,          /* profile velocity mode */
   DRV_MODE_OP_PROF_TOR            = 4,          /* profile torque mode */

   DRV_MODE_OP_HOMING              = 6,          /* homing mode */
   DRV_MODE_OP_INTER_POS           = 7,          /* interpolated position mode */
   DRV_MODE_OP_CSP                 = 8,          /* cyclic synchronous position mode */
   DRV_MODE_OP_CSV                 = 9,          /* cyclic synchronous velocity mode */
   DRV_MODE_OP_CST                 = 10          /* cyclic synchronous torque   mode */
};

/* DS402 device control (state machine) states */
enum MC_T_CIA402_STATE
{
   DRV_DEV_STATE_NOT_READY         = 0,          /* Not ready to switch on : Status Word x0xx 0000 */
   DRV_DEV_STATE_SWITCHON_DIS      = 1,          /* Switch on disabled     : Status Word x1xx 0000 */
   DRV_DEV_STATE_READY_TO_SWITCHON = 2,          /* Ready to switch on     : Status Word x01x 0001 */
   DRV_DEV_STATE_SWITCHED_ON       = 3,          /* Switched on            : Status Word x011 0011 */
   DRV_DEV_STATE_OP_ENABLED        = 4,          /* Operation enabled      : Status Word x011 0111 */
   DRV_DEV_STATE_QUICK_STOP        = 5,          /* Quick stop active      : Status Word 0000 0111 */
   DRV_DEV_STATE_MALFCT_REACTION   = 6,          /* Malfunction/Fault reaction active Status Word (xxxx 1111) oder (xx10 1111) */
   DRV_DEV_STATE_MALFUNCTION       = 7           /* Malfunction/Fault                 */
};

/* SERCOS state machine */
enum MC_T_SERCOS_STATE
{
   DRV_SER_STATE_NOT_READY = 0,                     /* Drive not ready to power up / NICHT BEREIT ZUR LEISTUNGSZUSCHALTUNG */
   DRV_SER_STATE_READY_TO_SWITCHON = 2,             /* Drive logic ready for main power on / BEREIT ZUR LEISTUNGSZUSCHALTUNG */
   DRV_SER_STATE_SWITCHED_ON = 3,                   /* Drive ready and main power applied / DREHMOMENTFREI */
   DRV_SER_STATE_OP_ENABLED = 4,                    /* Drive ready to operate / ANTRIEB FOLGT DER SOLLWERTVORGABE */
   DRV_SER_STATE_MALFUNCTION = 7                    /* Shutdown due to C1D error / Antriebsverriegelung, Fehler in Zustandsklasse 1 */
};

enum MC_T_SIMU_STATE
{
   DRV_SIM_STATE_OP_ENABLED = 4                     /* Drive ready to operate / ANTRIEB FOLGT DER SOLLWERTVORGABE */
};

/* Internal operation modes */
enum MC_T_OPMODE
{
   OPMODE_NOTSET = 0,
   OPMODE_CSP = 1,         /* Position control / Lageregelung */
   OPMODE_CSV = 2,         /* Velocity control / Geschwindigkeitsregelung */
   OPMODE_CST = 3,         /* Torque control / Momentenregelung */
   OPMODE_PP = 4,          /* position control: trajectory calculated in drive */
   OPMODE_HOMING = 5       /* Homing mode: trajectory calculated in drive */
};

/* source of target position / velocity generation */
enum MC_T_SETVALUE_SOURCE
{
	SETVAL_NONE = 0,
	SETVAL_TRAJGEN_ECM = 1, /* set value controlled by EC-Motion trajectory generator (default) */
	SETVAL_TRAJGEN_EXT = 2, /* set value controlled by external trajectory generator */
	SETVAL_TRAJGEN_DRV = 3, /* set value controlled by drive */
	SETVAL_MASTER_GEAR = 4, /* set value controlled by master axis in gearing mode */
	SETVAL_MASTER_CAMM = 5, /* set value controlled by master axis in camming mode */
};


/* SERCOS modes of operation: S-0-0032 */
enum MC_T_SERCOS_OPMODE
{
   SER_OPMODE_TORQUE               = 1,        /* Torque  control /
                                                  Momentenregelung */
   SER_OPMODE_VEL                  = 2,        /* Velocity control /
                                                  Geschwindigkeitsregelung */
   SER_OPMODE_POS_FB1              = 3,        /* Position control using motor feedback /
                                                  Lageregelung mit Lageistwert 1 (mit Schleppfehler) */
   SER_OPMODE_POS_FB2              = 4,        /* Position control using external feedback /
                                                  Lageregelung mit Lageistwert 2 (mit Schleppfehler) */
   SER_OPMODE_POS_FB1FB2           = 5,        /* Position control with motor feedback and external feddback /
                                                  Lageregelung mit Lageistwert 1+2 (mit Schleppfehler)*/
   SER_OPMODE_POS_FB1_LAGLESS      = 11,       /* Position control using motor feedback /
                                                  Lageregelung mit Lageistwert 1 (ohne Schleppfehler) */
   SER_OPMODE_POS_FB2_LAGLESS      = 12,       /* Position control using external feedback /
                                                  Lageregelung mit Lageistwert 2 (ohne Schleppfehler) */
   SER_OPMODE_POS_FB1FB2_LAGLESS   = 13,       /* Position control with motor feedback and external feddback /
                                                  Lageregelung mit Lageistwert 1+2 (ohne Schleppfehler) */
};

enum MC_T_DIRECTION
{
   MC_DIR_POSITIVE            = 1,            /* positive direction   */
   MC_DIR_SHORTEST            = 2,            /* shortest distance    */
   MC_DIR_NEGATIVE            = 3,            /* negative direction   */
   MC_DIR_CURRENT             = 4             /* stay with current direction */
};

enum MC_T_START_MODE
{
   MC_SM_ABSOLUTE             = 0,            /* Absolute */
   MC_SM_RELATIVE             = 1,            /* Relative */
   MC_SM_RAMP_IN              = 2,            /* Ramp In */
   MC_SM_RAMP_IN_POS          = 3,            /* Not supported: Ramp In only positive */
   MC_SM_RAMP_IN_NEG          = 4             /* Not supported: Ramp In only negative */
};

enum MC_T_AXIS_TYPE
{
   MC_AXIS_TYPE_VIRTUAL,                      /* Update outputs, read inputs if pointers are provided */
   MC_AXIS_TYPE_REAL_ALL                      /* Control CiA DS402 or SERCOS drive. */
};

enum MC_T_AXIS_PROFILE
{
   MC_T_AXIS_PROFILE_NONE,
   MC_T_AXIS_PROFILE_DS402,                   /* CiA DS402 */
   MC_T_AXIS_PROFILE_SERCOS                   /* SERCOS / SoE */
};

enum MC_T_BUFFER_MODE
{
   MC_ABORTING =                0,            /* start FB immediately (default mode) */
   MC_BUFFERED =                1,            /* start FB after current motion has finished */
   MC_BLENDING_LOW =            2,            /* the velocity is blended with the lowest vel. of both FBs */
   MC_BLENDING_PREVIOUS =       3,            /* the velocity is blended with the vel. of the first FB */
   MC_BLENDING_NEXT =           4,            /* the velocity is blended with the vel. of the second FB */
   MC_BLENDING_HIGH =           5,            /* the velocity is blended with the highest vel. of both FBs */
   MC_BLENDING_PREVIOUS_STOP =  6,            /* like MC_BLENDING_PREVIOUS, but end. velocity of prev. FB maybe reduced to be able to stop */
   MC_BLENDING_NEXT_STOP     =  7,            /* like MC_BLENDING_NEXT, but end. velocity of prev. FB maybe reduced to be able to stop */
   MC_BLENDING_HIGH_STOP     =  8             /* like MC_BLENDING_HIGH, but end. velocity of prev. FB maybe reduced to be able to stop */
};

enum MC_T_EXECUTION_MODE
{
   MC_IMMEDIATELY             = 0,            /* the functionality is immediately valid */
   MC_QUEUED                  = 1             /* same functionality as buffer mode MC_BUFFERED */
};

/* PLC Open axis states */
enum MC_T_PLCOPEN_STATE
{
   MC_PLCOPEN_STATE_UNKNOWN         = 0,               /*< unknown */
   MC_PLCOPEN_STATE_STAND_STILL     = 1,               /*< stand still */
   MC_PLCOPEN_STATE_HOMING          = 2,               /*< homing */
   MC_PLCOPEN_STATE_STOPPING        = 3,               /*< stopping */
   MC_PLCOPEN_STATE_ERROR_STOP      = 4,               /*< error stop */
   MC_PLCOPEN_STATE_DISC_MOTION     = 5,               /*< discrete motion */
   MC_PLCOPEN_STATE_CONT_MOTION     = 6,               /*< continous motion */
   MC_PLCOPEN_STATE_SYNC_MOTION     = 7,               /*< synchronized motion */
   MC_PLCOPEN_STATE_TORQUE_CONTROL  = 10,              /*< torque control */
   MC_PLCOPEN_STATE_DISABLED        = 11               /*< disabled */
};

/* Unique ID for each MCFB (+ implemented, - not implemented) */
enum MC_T_FBID
{
   MCFB_ID_IDLE,                        /*        Axis is idle */
   MCFB_ID_POWER,                       /* + 3.1  MC_Power */
   MCFB_ID_HOME,                        /* - 3.2  MC_Home */
   MCFB_ID_STOP,                        /* + 3.3  MC_Stop */
   MCFB_ID_HALT,                        /* + 3.4  MC_Halt */
   MCFB_ID_MOVE_ABSOLUTE,               /* + 3.5  MC_MoveAbsolute */
   MCFB_ID_MOVE_RELATIVE,               /* + 3.6  MC_MoveRelative */
   MCFB_ID_MOVE_ADDITIVE,               /* - 3.7  MC_MoveAdditive */
   MCFB_ID_MOVE_SUPERIMPOSED,           /* - 3.8  MC_MoveSuperimposed */
   MCFB_ID_HALT_SUPERIMPOSED,           /* - 3.9  MC_HaltSuperimposed */
   MCFB_ID_MOVE_VELOCITY,               /* + 3.10 MC_MoveVelocity */
   MCFB_ID_MOVE_CONTINUOUS_ABSOLUTE,    /* + 3.11 MC_MoveContinuousAbsolute */
   MCFB_ID_MOVE_CONTINUOUS_RELATIVE,    /* + 3.12 MC_MoveContinuousRelative */
   MCFB_ID_TORQUE_CONTROL,              /* - 3.13 MC_TorqueControl */
   MCFB_ID_POSITION_PROFILE,            /* + 3.14 MC_PositionProfile */
   MCFB_ID_VELOCITY_PROFILE,            /* - 3.15 MC_VelocityProfile */
   MCFB_ID_ACCELERATION_PROFILE,        /* - 3.16 MC_AccelerationProfile */
   MCFB_ID_SET_POSITION,                /* + 3.17 MC_SetPosition */
   MCFB_ID_SET_OVERRIDE,                /* - 3.18 MC_SetOverride */
   MCFB_ID_READ_PARAMETER,              /* + 3.19 MC_ReadParameter */
   MCFB_ID_READ_BOOL_PARAMETER,         /* + 3.19 MC_ReadBoolParameter */
   MCFB_ID_WRITE_PARAMETER,             /* + 3.20 MC_WriteParameter */
   MCFB_ID_WRITE_BOOL_PARAMETER,        /* + 3.20 MC_WriteBoolParameter */
   MCFB_ID_READ_DIGITAL_INPUT,          /* + 3.21 MC_ReadDigitalInput */
   MCFB_ID_READ_DIGITAL_OUTPUT,         /* + 3.22 MC_ReadDigitalOutput */
   MCFB_ID_WRITE_DIGITAL_OUTPUT,        /* + 3.23 MC_WriteDigitalOutput */
   MCFB_ID_READ_ACTUAL_POSITION,        /* + 3.24 MC_ReadActualPosition */
   MCFB_ID_READ_ACTUAL_VELOCITY,        /* + 3.25 MC_ReadActualVelocity */
   MCFB_ID_READ_ACTUAL_TORQUE,          /* - 3.26 MC_ReadActualTorque */
   MCFB_ID_READ_STATUS,                 /* - 3.27 MC_ReadStatus */
   MCFB_ID_READ_MOTION_STATE,           /* + 3.28 MC_ReadMotionState */
   MCFB_ID_READ_AXIS_INFO,              /* + 3.39 MC_ReadAxisInfo */
   MCFB_ID_READ_AXIS_ERROR,             /* + 3.30 MC_ReadAxisError */
   MCFB_ID_RESET,                       /* + 3.31 MC_Reset */
   MCFB_ID_DIGITAL_CAM_SWITCH,          /* - 3.32 MC_DigitalCamSwitch */
   MCFB_ID_TOUCH_PROBE,                 /* - 3.33 MC_DigitalTouchProbe */
   MCFB_ID_ABORT_TRIGGER,               /* - 3.34 MC_AbortTrigger */
   MCFB_ID_CAMTABLE_SELECT,             /* + 4.2  MC_CamTableSelect */
   MCFB_ID_CAM_IN,                      /* + 4.3  MC_CamIn */
   MCFB_ID_CAM_OUT,                     /* + 4.4  MC_CamOut */
   MCFB_ID_GEAR_IN,                     /* + 4.5  MC_GearIn */
   MCFB_ID_GEAR_OUT,                    /* + 4.6  MC_GearOut */
   MCFB_ID_CHECK_TARGETPOS_REACHED,     /* Vendor extension */
   MCFB_ID_HALT_RECOVERY                /* Vendor extension */
};

/* Parameter numbers for FB MC_ReadParameter / MC_WriteParameter and boolean versions */
/* Parameter-numbers from 0 to 999 are reserved for the standard */
enum MC_T_PN
{
   MCFB_PN_COMMANDED_POSITION     = 1,           /* REAL: Commanded position */
   MCFB_PN_SW_LIMIT_POS           = 2,           /* REAL: Positive Software limit switch position */
   MCFB_PN_SW_LIMIT_NEG           = 3,           /* REAL: Negative Software limit switch position */
   MCFB_PN_ENA_LIMIT_POS          = 4,           /* BOOL: Enable positive software limit switch */
   MCFB_PN_ENA_LIMIT_NEG          = 5,           /* BOOL: Enable negative software limit switch */
   // MCFB_PN_ENA_POS_LAG_MONITORING = 6,           /* BOOL: Enable monitoring of position lag */
   // MCFB_PN_MAX_POSITION_LAG       = 7,           /* REAL: Maximal position lag */
   // MCFB_PN_MAX_VELOCITY_SYSTEM    = 8,           /* REAL: Maximal allowed velocity of the axis in the motion system */
   // MCFB_PN_MAX_VELOCITY_APPL      = 9,           /* REAL: Maximal allowed velocity of the axis in the application */
   MCFB_PN_ACTUAL_VELOCITY        = 10,          /* REAL: Actual velocity */
   MCFB_PN_COMMANDED_VELOCITY     = 11,          /* REAL: Commanded velocity */
   // MCFB_PN_MAX_ACCELERATION_SYSTEM= 12,          /* REAL: Maximal allowed acceleration of the axis in the motion system */
   // MCFB_PN_MAX_ACCELERATION_APPL  = 13,          /* REAL: Maximal allowed acceleration of the axis in the application   */
   // MCFB_PN_MAX_DECELERATION_SYSTEM= 14,          /* REAL: Maximal allowed deceleration of the axis in the motion system */
   // MCFB_PN_MAX_DECELERATION_APPL  = 15,          /* REAL: Maximal allowed deceleration of the axis in the application */
   // MCFB_PN_MAX_JERKSYSTEM         = 16,          /* REAL: Maximum allowed jerk of the axis in the motion system */
   // MCFB_PN_MAX_JERKAPPL           = 17,          /* REAL: Maximum allowed jerk of the axis in the application   */

   /* Numbers greater than 999 indicate supplier-specific parameters */
   MCFB_PN_COMMANDED_ACCELERATION = 1000,        /* REAL: Commanded acceleration */
   MCFB_PN_COMMANDED_JERK = 1001,                /* REAL: Commanded jerk */
   MCFB_PN_ACTUAL_POSITION = 1002                /* REAL: Actual position */
};

/* Internal state of FB's */
enum MC_T_FBSTATE
{
   FBS_IDLE,
   FBS_BUSY,
   FBS_ACTIVE,
   FBS_DONE,
   FBS_ERROR,
   FBS_COMMAND_ABORTED,
};

/* trajectory generator state */
enum MC_T_TRAJECT_STATE
{
    eTrajStatus_Idle        = 0,                /* idle */
    eTrajStatus_Acc         = 1,                /* acc */
    eTrajStatus_Dec         = 2,                /* dec */
    eTrajStatus_Vel         = 3,                /* constant velocity */
};

#ifdef EC_MOTION_SUPPORT_PP_MODE
/* profile state */
enum MC_T_PROFILE_STATE
{
    eProfileStatus_Idle        = 0,             /* idle */
    eProfileStatus_Error,                       /* error */
    eProfileStatus_NewSetPoint,                 /* new set point */
    eProfileStatus_WaitForAck,                  /* wait for acknowledge */
    eProfileStatus_WaitReached,                 /* wait for target position reached */
    eProfileStatus_WaitQStop,                   /* wait for quick stop executed */
    eProfileStatus_SwitchToHomingMode,          /* switch to homing mode */
    eProfileStatus_StartHoming,                 /* start homing */
    eProfileStatus_WaitHomingDone,              /* wait homing done */
    eProfileStatus_Halt,                        /* halt */
    eProfileStatus_WaitHaltDone                 /* wait for halt done */
};
#endif

/* MC_ReadMotionState (3.28) */
enum MC_T_SOURCE
{
   MC_SOURCE_SET_VALUE =        1,            /* set value       */
   MC_SOURCE_ACTUAL_VALUE =     2,            /* actual value    */
};

enum MC_T_RECOVERY_MODE
{
   MC_RECOVERY_NO_ACTION,
   MC_RECOVERY_ABORT_MOVEMENT      /* Abort current movement */
};

/*-GLOBAL VARIABLES-----------------------------------------------------------*/

/*-FORWARD DECLARATIONS------------------------------------------------------*/

class MC_BUFFERED_FB_T;
class MC_MOVE_VELOCITY_T;
class MC_T_AXIS_REF;
class MC_T_SPLINE;
class MC_POSITION_PROFILE_T;
struct MC_T_BUFFERENTRY;


/*-TYPEDEFS------------------------------------------------------------------*/

typedef EC_T_BOOL   MC_T_BOOL;
#define MC_FALSE    EC_FALSE
#define MC_TRUE     EC_TRUE
typedef EC_T_BYTE   MC_T_BYTE;
typedef EC_T_CHAR   MC_T_CHAR;
typedef EC_T_INT    MC_T_INT;
typedef EC_T_UINT   MC_T_UINT;
typedef double      MC_T_REAL;
typedef EC_T_WORD   MC_T_WORD;
typedef EC_T_SWORD  MC_T_SWORD;
typedef EC_T_DWORD  MC_T_DWORD;
typedef EC_T_SDWORD MC_T_SDWORD;
typedef EC_T_SDWORD MC_T_VELACC;
typedef EC_T_SDWORD MC_T_POSITION;
typedef EC_T_INT64  MC_T_POS_CALC;
typedef EC_T_INT64  MC_T_INT64;
typedef EC_T_UINT64 MC_T_UINT64;
#define MC_NULL     EC_NULL


/* time/position data type for MC_PositionProfile */
/* The content of a Time/Position pair may be expressed in DeltaTime/Pos, where Delta could be the difference
in time between two successive points */
typedef struct
{
   MC_T_REAL   fDeltaTime;
   MC_T_REAL   fPosition;
} MC_T_TP;

typedef struct
{
   MC_T_DWORD  dwNumberOfPairs;
   MC_T_BOOL   bIsAbsolute;
   MC_T_TP*    pTimePos;
} MC_T_TP_REF;

#ifdef EC_MOTION_SUPPORT_MC_CAMMING
/* CAM Table */
enum MC_T_CAM_VAR_TYPE
{
   MC_CAM_VAR_TYPE_INT       = 1,      /* table contains MC_T_INT values   */
   MC_CAM_VAR_TYPE_REAL      = 2       /* table contains MC_T_REAL values */
};

enum MC_T_CAM_INTERPOL_TYPE
{
   MC_CAM_INTERPOL_TYPE_LIN  = 1,      /* interpolation type linear   */
   MC_CAM_INTERPOL_TYPE_CUB  = 2       /* interpolation type cubic */
};

typedef struct _MC_T_CAM_REF
{
   MC_T_CAM_VAR_TYPE    eVarType;         /* variable type */
   MC_T_CAM_INTERPOL_TYPE eInterpolType;  /* interpolation type */
   MC_T_INT             nNumOfElements;   /* number of elements */
   EC_T_VOID*           pData;            /* two dimensional table with master/slave positions */
} MC_T_CAM_REF;

typedef struct _MC_T_CAM_ID
{
    MC_T_AXIS_REF*      MasterAxis;         /* Reference to master axis */
    MC_T_AXIS_REF*      SlaveAxis;          /* Reference to slave axis */
    MC_T_CAM_REF*       CamRef;             /* reference to CAM description */
    MC_T_SPLINE*        pSpline;            /* The curve */
    MC_T_BOOL           Periodic;           /* 1 = periodic, 0 = non periodic (single-shot) */
    MC_T_BOOL           MasterAbsolute;     /* 1 = absolute; 0 = relative coordinates */
    MC_T_BOOL           SlaveAbsolute;      /* 1 = absolute; 0 = relative coordinates */
    MC_T_BOOL           bEndOfProfile;      /* Set in CalcSlavePos() */
    MC_T_REAL           fMasterStart;       /* start position master = first element in pData */
    MC_T_REAL           fMasterEnd;         /* end position master = last element in pData */
    MC_T_REAL           fMasterPeriod;      /* distance between end and start, might be negative */
    MC_T_REAL           fMasterOffset;      /* Offset of the master shaft to cam */
    MC_T_REAL           fMasterStartOffset; /* if MC_CAMTABLE_SELECT_T.MasterAbsolute is 0, this is the actual pos.   */
    MC_T_REAL           fSlaveOffset;       /* Offset of slave table */
    MC_T_REAL           fSlaveStartOffset;  /* if MC_CAMTABLE_SELECT_T.SlaveAbsolute is 0, this is the actual pos.   */
    MC_T_REAL           fMasterScaling;     /* Factor for the master profile (default = 1.0). From the
                                                   slave point of view the master overall profile is multiplied by this factor */
    MC_T_REAL           fSlaveScaling;      /* Factor for the slave profile (default = 1.0). The overall
                                                   slave profile is multiplied by this factor. */
    MC_T_WORD           CalcSlavePos(MC_T_REAL fMasterPos, MC_T_REAL* pfTargetPos, MC_T_BOOL* bValidPos);
} MC_T_CAM_ID, *MC_T_P_CAM_ID;
#endif

#ifdef EC_MOTION_SUPPORT_MC_GEARING
typedef struct _MC_T_GEARING_DATA
{
	MC_T_SOURCE			eMasterValueSource; /* source of synchronization */
	MC_T_INT			nRatioNumerator;	/* ratio numerator */
	MC_T_UINT		    dwRatioDenominator; /* ratio denominator */
} MC_T_GEARING_DATA;
#endif

typedef void (*PerfMeasStartFptr)(EC_T_TSC_MEAS_DESC *pTscMeasDesc, EC_T_DWORD dwIndex);
typedef EC_T_TSC_TIME* (EC_FNCALL *PerfMeasEndFptr)(EC_T_TSC_MEAS_DESC *pTscMeasDesc, EC_T_DWORD dwIndex);

extern "C"
{

typedef EC_T_DWORD (EC_FNCALL *EcatCoeSdoDownloadFptr)(
     EC_T_DWORD     dwSlaveId       /* [in]  slave ID */
    ,EC_T_WORD      wObIndex        /* [in]  SDO index */
    ,EC_T_BYTE      byObSubIndex    /* [in]  SDO sub-index */
    ,EC_T_BYTE*     pbyData         /* [in]  SDO data */
    ,EC_T_DWORD     dwDataLen       /* [in]  length of pbyData buffer */
    ,EC_T_DWORD     dwTimeout       /* [in]  timeout in msec, must not be set to EC_NOWAIT */
    ,EC_T_DWORD     dwFlags         /* [in]  mailbox transfer flags, see EC_MAILBOX_FLAG */
    );
typedef EC_T_DWORD  (EC_FNCALL *EcatCoeSdoUploadFptr)(
     EC_T_DWORD     dwSlaveId       /* [in]  slave ID */
    ,EC_T_WORD      wObIndex        /* [in]  SDO index */
    ,EC_T_BYTE      byObSubIndex    /* [in]  SDO sub-index */
    ,EC_T_BYTE*     pbyData         /* [out] SDO data */
    ,EC_T_DWORD     dwDataLen       /* [in]  length of pbyData buffer */
    ,EC_T_DWORD*    pdwOutDataLen   /* [out] SDO data length (number of received bytes) */
    ,EC_T_DWORD     dwTimeout       /* [in]  timeout in msec, must not be set to EC_NOWAIT */
    ,EC_T_DWORD     dwFlags         /* [in]  mailbox transfer flags, see EC_MAILBOX_FLAG */
    );
typedef EC_T_DWORD (EC_FNCALL *EcatSoeWriteFptr)(
    EC_T_DWORD    dwSlaveId         /**< [in]  ethercat slave ID */
   ,EC_T_BYTE     byDriveNo         /**< [in]  number of drive in slave */
   ,EC_T_BYTE*    pbyElementFlags   /**< [in]  which element of object is written */
   ,EC_T_WORD     wIDN              /**< [in]  IDN of the object */
   ,EC_T_BYTE*    pbyData           /**< [in]  data of the element to be written */
   ,EC_T_DWORD    dwDataLen         /**< [in]  length of pbyData buffer */
   ,EC_T_DWORD    dwTimeout         /**< [in]  timeout in msec, must not be set to EC_NOWAIT */
   );
typedef  EC_T_DWORD (EC_FNCALL *EcatSoeReadFptr)(
    EC_T_DWORD  dwSlaveId         /**< [in]  ethercat slave ID */
   ,EC_T_BYTE   byDriveNo         /**< [in]  number of drive in slave */
   ,EC_T_BYTE*  pbyElementFlags   /**< [in/out]  which element of object is read */
   ,EC_T_WORD   wIDN              /**< [in]  IDN of the object */
   ,EC_T_BYTE*  pbyData           /**< [out] IDN data */
   ,EC_T_DWORD  dwDataLen         /**< [in]  length of pbyData buffer */
   ,EC_T_DWORD* pdwOutDataLen     /**< [out] IDN data length (number of received bytes) */
   ,EC_T_DWORD  dwTimeout         /**< [in]  timeout in msec, must not be set to EC_NOWAIT */
   );
typedef EC_T_DWORD  (EC_FNCALL *EcatGetSlaveStateFptr)(
    EC_T_DWORD              dwSlaveId       /**< [in]   slave id */
   ,EC_T_WORD*              pwCurrDevState  /**< [out]  current device state */
   ,EC_T_WORD*              pwReqDevState   /**< [out]  requested device state */
    );

} /* extern "C" */

// This prototypes are needed for friend decl's.
#if (defined ECMOTIONLIB_BUILD)
static EC_T_VOID   MC_AbortCommand(
     MC_T_AXIS_REF*         pAxis
    ,MC_T_WORD              wNewFbID);
static EC_T_WORD   MC_BufferPut(
     MC_T_AXIS_REF*         pAxis
    ,MC_BUFFERED_FB_T*      pMcfb
    ,MC_T_REAL              fEndVel
    ,MC_T_BUFFERENTRY**     ppPrevFb);
static EC_T_WORD MC_BufferActivateNext(
     MC_T_AXIS_REF*         pAxis,
     MC_BUFFERED_FB_T*      pMcfb,
     MC_T_BOOL*             fbActive);
static MC_T_WORD HandleBlending(
     MC_T_AXIS_REF         *pAxis,
     MC_BUFFERED_FB_T      *pMcfb,
     MC_T_DWORD             dwBufferMode,
     MC_T_REAL              fVelocity,
     MC_T_REAL              fEndVelocity,
     MC_T_BUFFERENTRY**     ppPrevFb);
#endif

/*-INLINES-------------------------------------------------------------------*/
/* This inline's avoiding multi evaluation of val and offer type safety */
static inline void MC_SET_WORD(MC_T_WORD *pDstAligned, MC_T_WORD valUnaligned) { EC_SET_FRM_WORD( pDstAligned, valUnaligned ); }
static inline void MC_SET_DWORD(MC_T_DWORD *pDstAligned, MC_T_DWORD valUnaligned) { EC_SET_FRM_DWORD( pDstAligned, valUnaligned ); }
static inline void MC_SET_SDWORD(MC_T_SDWORD *pDstAligned, MC_T_SDWORD valUnaligned) { EC_SET_FRM_DWORD( pDstAligned, *(EC_T_DWORD *) &valUnaligned ); }
static inline void MC_SET_QWORD(MC_T_UINT64 *pDstAligned, MC_T_UINT64 valUnaligned) { EC_SET_FRM_QWORD( pDstAligned, valUnaligned ); }
static inline void MC_SET_REAL(MC_T_REAL *pDstAligned, MC_T_REAL valUnaligned)
{
    MC_T_BYTE* pbyDst = (MC_T_BYTE*)pDstAligned;
    MC_T_BYTE* pbySrc = (MC_T_BYTE*)&valUnaligned;

#ifdef EC_BIG_ENDIAN
    pbyDst[0] = pbySrc[7];
    pbyDst[1] = pbySrc[6];
    pbyDst[2] = pbySrc[5];
    pbyDst[3] = pbySrc[4];
    pbyDst[4] = pbySrc[3];
    pbyDst[5] = pbySrc[2];
    pbyDst[6] = pbySrc[1];
    pbyDst[7] = pbySrc[0];
#else
    pbyDst[0] = pbySrc[0];
    pbyDst[1] = pbySrc[1];
    pbyDst[2] = pbySrc[2];
    pbyDst[3] = pbySrc[3];
    pbyDst[4] = pbySrc[4];
    pbyDst[5] = pbySrc[5];
    pbyDst[6] = pbySrc[6];
    pbyDst[7] = pbySrc[7];
#endif
}

/*-CLASSES-------------------------------------------------------------------*/

#define  MAX_NUM_OF_SPLINE_POINTS   50

// spline interpolation
class MC_T_SPLINE
{
public:
    enum bd_type {
        first_deriv = 1,
        second_deriv = 2
    };

private:
    // x,y coordinates of points
    MC_T_INT      m_nNumOfPoints;
    MC_T_REAL     m_x[MAX_NUM_OF_SPLINE_POINTS];
    MC_T_REAL     m_y[MAX_NUM_OF_SPLINE_POINTS];
    // interpolation parameters
    // f(x) = a*(x-x_i)^3 + b*(x-x_i)^2 + c*(x-x_i) + y_i

    // spline coefficients
    MC_T_REAL     m_a[MAX_NUM_OF_SPLINE_POINTS];
    MC_T_REAL     m_b[MAX_NUM_OF_SPLINE_POINTS];
    MC_T_REAL     m_c[MAX_NUM_OF_SPLINE_POINTS];

    MC_T_REAL     m_b0, m_c0;                     // for left extrapol
    bd_type       m_left, m_right;
    MC_T_REAL     m_left_value, m_right_value;
    MC_T_BOOL     m_force_linear_extrapolation;

public:
    // set default boundary condition to be zero curvature at both ends
    MC_T_SPLINE(EC_T_VOID): m_left(second_deriv), m_right(second_deriv),
        m_left_value(0.0), m_right_value(0.0),
        m_force_linear_extrapolation(MC_FALSE)
    {
        ;
    }

    // optional, but if called it has to come be before set_points()
    EC_T_DWORD set_boundary(bd_type left, MC_T_REAL left_value,
                      bd_type right, MC_T_REAL right_value,
                      MC_T_BOOL force_linear_extrapolation=MC_FALSE);
    MC_T_WORD set_points(MC_T_INT   nNumOfPoints, MC_T_REAL* pX, MC_T_REAL* pY, MC_T_BOOL cubic_spline=true);
    MC_T_REAL get_result(MC_T_REAL x) const;
};



/* Init parameters for MC_T_AXIS_REF. Initialized by API user. */
struct MC_T_AXIS_INIT
{
    MC_T_AXIS_TYPE         AxisType;
    MC_T_DWORD             CycleTime;            /* [usec] EtherCAT bus cycle time */
    MC_T_DWORD             IncPerMM;             /* Increments per mm */
    MC_T_DWORD             IncFactor;            /* internal position values are x times bigger */
    EC_T_LOG_PARMS         LogParms;
    MC_T_DWORD             VelocityGain;         /* CSV-Mode: Velocity Gain, CSP-Mode: Velocity Gain for Feed Forward Object 0x60B1 */
    MC_T_DWORD             TorqueGain;           /* CSP and CSV-Mode: Torque Gain for Feed Forward Object 0x60B2 */
} _MC_PACKED;

/* Output init parameters for MC_T_AXIS_REF. Initialized by API user. */
struct MC_T_AXIS_INIT_OUTPUTS
{
    MC_T_SDWORD           *pTargetPosition;      /* ptr to target position: Data type DINT (INTEGER32 -2147483648 to +2147483627)
                                                  * In Cyclic Synchronous Position mode, it is always interpreted as an absolute value */
    MC_T_SDWORD           *pTargetVelocity;      /* ptr to target velocity: Data type DINT (INTEGER32 -2147483648 to +2147483627) */
    MC_T_SDWORD           *pVelocityOffset;      /* ptr to velocity offset for feed forward: Data type DINT (INTEGER32 -2147483648 to +2147483627) */

    MC_T_SWORD            *pTargetTorque;        /* ptr to target torque: Data type INT (INTEGER16 -32768 to +32767) */
    MC_T_SWORD            *pTorqueOffset;        /* ptr to target offset for feed forward: Data type INT (INTEGER16 -32768 to +32767) */

#ifdef EC_MOTION_SUPPORT_PP_MODE
    MC_T_DWORD            *pProfileVelocity;     /* ptr to profile velocity: Data type UDINT */
    MC_T_DWORD            *pProfileAcc;          /* ptr to profile acc: Data type UDINT */
    MC_T_DWORD            *pProfileDec;          /* ptr to profile dec: Data type UDINT */
#endif

    MC_T_BYTE             *pModeOfOperation;     /* ptr to mode of operation (DS402 0x6060) */
    MC_T_BYTE             *pModeOfOperationDisplay;     /* ptr to mode of operation (DS402 0x6061) */
    MC_T_DWORD            *pDigitalOutputs;      /* ptr to digital outputs in process data */
} _MC_PACKED;

/* Input init parameters for MC_T_AXIS_REF. Initialized by API user. */
struct MC_T_AXIS_INIT_INPUTS
{
    MC_T_SDWORD           *pActualPosition;      /* ptr to actual position in process data
                                                  * Data type DINT (INTEGER32 -2147483648 to +2147483627) */
    MC_T_SWORD            *pActualTorque;        /* ptr to actual torque in process data (MC_NULL if torque mode is not used)
                                                  * Data type INT (INTEGER16 -32768 to +32767) */
    MC_T_DWORD            *pDigitalInputs;       /* ptr to digital inputs in process data */
} _MC_PACKED;

/* EtherCAT specific init parameters for MC_T_AXIS_REF. Initialized by API user. */
struct MC_T_AXIS_INIT_ECAT
{
    /* EtherCAT specific data */
    MC_T_DWORD             VendorId;
    MC_T_DWORD             ProductCode;
    MC_T_DWORD             SlaveID;
    MC_T_WORD              StationAddress;         /* slave station address */
    MC_T_AXIS_PROFILE      Profile;
    MC_T_BYTE              SercosDriveNo;          /* Adressed drive number within SERCOS servo controller */
    MC_T_WORD              CoeIdxOpMode;           /* Idx of Mode of Operation object (DS402 only). Default: 0x6060 */
    MC_T_WORD              CoeSubIdxOpMode;        /* SubIdx of Mode of Operation object (DS402 only). Default: 0x6060 */

    /* Pointers to EtherCAT process data (drive status / control) */
    MC_T_WORD             *pStatusWord;            /* ptr to status  word in process data */
    MC_T_WORD             *pControlWord;           /* ptr to control word in process data */

    /* Pointers to EC-Master API functions */
    EcatCoeSdoDownloadFptr pEcatCoeSdoDownload;    /* ptr to EC-Master's SDO/CoE-Download function. EC_NULL if not provided. */
    EcatCoeSdoUploadFptr   pEcatCoeSdoUpload;      /* ptr to EC-Master's SDO/CoE-Upload function. EC_NULL if not provided. */
    EcatSoeWriteFptr       pEcatSoeWrite;          /* ptr to EC-Master's SDO/SoE-Write function. EC_NULL if not provided. */
    EcatSoeReadFptr        pEcatSoeRead;           /* ptr to EC-Master's SDO/SoE-Read function. EC_NULL if not provided. */
    EcatGetSlaveStateFptr  pEcatGetSlaveState;     /* ptr to EC-Master's GetSlaveState function. EC_NULL if not provided. */
} _MC_PACKED;

struct MC_T_BUFFERENTRY
{
    MC_BUFFERED_FB_T*           pFB;            /* pointer to FB */
    MC_T_BUFFERENTRY*           pPrev;
    MC_T_REAL                   fEndVel;        /* velocity at end of move */
} _MC_PACKED;

/* Data for a movement */
struct MC_T_MOVEMENT
{
    MC_T_POS_CALC       lTotalDist;             /* [Inc] Total distance  */
    MC_T_POS_CALC       lRemainDist;            /* [Inc] Remaining distance  */
    MC_T_POS_CALC       lDoneDist;              /* [Inc] Done distance  */
    MC_T_POS_CALC       lNextMoveCorrection;    /* [Inc] Correction for next move (remainder previous) */
    MC_T_POS_CALC       lSegDist1to3;           /* [Inc] Distance of segments 1-3  */
    MC_T_POS_CALC       lSegDist5to7;           /* [Inc] Distance of segments 5-7  */
    MC_T_DWORD          dwTotalMoveCycles;      /* total cycles for move */
    MC_T_DWORD          dwDoneCycles;           /* Done cycles since start of move */

    MC_T_VELACC         lStartVel;              /* [Inc/Cycle] Value of velocity and start of movement */
    MC_T_VELACC         lStartAcc;              /* [Inc/Cycle] Value of commanded acc and start of movement */
    MC_T_VELACC         lConstantVel;           /* [Inc/Cycle] Value of the constant (middle) velocity */
    MC_T_VELACC         lEndVel;                /* [Inc/Cycle] Value of velocity and end of movement */
    MC_T_VELACC         lMaxAcc;                /* [Inc/Cycle^2] Value of the max. acceleration  */
    MC_T_VELACC         lMaxDec;                /* [Inc/Cycle^2] Value of max. deceleration  */
    MC_T_VELACC         lJerk;                  /* [Inc/Cycle^3] Value of jerk */
    MC_T_VELACC         lTargetVel;
    MC_T_VELACC         lTargetAcc;

    MC_T_VELACC         lCommandVel;            /* [Inc/Cycle] commanded velocity */
    MC_T_VELACC         lCommandAcc;            /* +/- [Inc/Cycle^2] commanded acceleration */
    MC_T_VELACC         lCommandJerk;           /* +/- [Inc/Cycle^3] commanded jerk */
    MC_T_POS_CALC       lRemainStep;            /* [Inc] Remainder package */
    MC_T_BOOL           bRemainStepCalc;
    MC_T_BOOL           bEndless;               /* if true, endless move */
    MC_T_POSITION       lTargetPosStep;         /* [Inc] change of target pos in current cycle */
    MC_T_POSITION       lLastTargetPosStep;     /* copy of lTargetPosStep just for logging */
    MC_T_VELACC         lLastCommandAcc;        /* for calc of jerk */
    MC_T_INT            nRampIdx;               /* index of ramp which is currently in progress */
    MC_T_POSITION       aStepBuffer[STEP_BUFFER_SIZE]; /* last steps, just for debugging */
    MC_T_POS_CALC       lSegDist[7];            /* distances within movement segments */
    MC_T_INT            lSegCycles[7];          /* cycles within movement segments */
    MC_T_VELACC         lVelReduce[2];          /* velocity to reduce acc/dec within ramp */
    MC_T_VELACC         lVelRemainder[2];       /* velocity remainder acc/dec to reach destination vel */
    MC_T_VELACC         lRampAccMax[2];         /* max acc/dec for ramp */
    MC_T_VELACC         lRamp2AccCorrection;


    //MC_T_INT            lPreStartTime;          /* [usec] pre-start time for next action            */
    //MC_T_POS_CALC       lPreStartDist;          /* [Inc]  pre-start distance (based on start pos)   */
    MC_T_BOOL           bContUpdate;            /* Continuous Update */
    MC_T_REAL           fOrgMaxVelocity;        /* Original/inital max velocity when move was calculated */
    MC_T_REAL           fOrgEndVelocity;        /* Original/inital end velocity when move was calculated */
    MC_T_REAL           fOrgAcceleration;       /* Original/inital acc when move was calculated */
    MC_T_REAL           fOrgDeceleration;       /* Original/inital dec when move was calculated */
    MC_T_REAL           fOrgJerk;               /* Original/inital jerk when move was calculated */
    MC_T_POS_CALC       lTargetPosition;     /* Original/inital target position when move was calculated */
    MC_T_TRAJECT_STATE  eTrajectState;          /* Status of trajectory generator */
#ifdef EC_MOTION_SUPPORT_PP_MODE
    MC_T_PROFILE_STATE  eProfileState;          /* Status of profile in drive */
#endif

#ifdef EC_MOTION_SUPPORT_MC_POSITION_PROFILE
    MC_T_REAL           fTimeInSpline;          /* PositionProfile: Time in Profile */
    MC_T_REAL           fTotalTime;             /* PositionProfile: Total time in profile */
    MC_POSITION_PROFILE_T *pProfile;
#endif
	MC_T_REAL			fSlaveVelKorrection;	/* slave velocity correction (for gearing) */
} _MC_PACKED;

/* State for PID controller
 */
struct MC_T_PID_CONTROLLER
{
   // Constant parameters
   MC_T_REAL Kp, Ki, Kd;
   MC_T_INT ISumMax;

   MC_T_INT ISum;
   MC_T_INT EPrev;
};

/* Reference to the axis */
class _MC_API MC_T_AXIS_REF
{
public:

    MC_T_AXIS_REF();
    MC_T_AXIS_REF(const MC_T_AXIS_INIT &init);

    void _MC_THIS_API Init(const MC_T_AXIS_INIT &init);
    void _MC_THIS_API InitInputs(const MC_T_AXIS_INIT_INPUTS &init);
    void _MC_THIS_API InitOutputs(const MC_T_AXIS_INIT_OUTPUTS &init);
    void _MC_THIS_API InitEcat(const MC_T_AXIS_INIT_ECAT &init);

    MC_T_DWORD _MC_THIS_API SetModeOfOperation(MC_T_WORD wProfileOperationMode);
    MC_T_DWORD _MC_THIS_API SetMotionProfileType(EC_T_SWORD sMotionProfileType);       /* DRV_OBJ_MOTION_PROFILE_TYPE   0x6086 */
    MC_T_DWORD _MC_THIS_API SetProfileOptionCode(EC_T_WORD wPosOptionCode);            /* DRV_OBJ_POS_OPTION_MODE       0x60F2 */
    MC_T_DWORD _MC_THIS_API SetPositionWindow(MC_T_DWORD dwWindow, MC_T_WORD wTime);   /* DRV_OBJ_POSITION_WINDOW       0x6067
                                                                                          DRV_OBJ_POSITION_WINDOW_TIME  0x6068 */
    MC_T_DWORD _MC_THIS_API SetSoftwareLimits(MC_T_BOOL bEnaLimitPos, MC_T_BOOL bEnaLimitNeg,  /* DRV_OBJ_SOFTWARE_POSITION_LIMIT */
                                              MC_T_REAL fLimitPos,    MC_T_REAL fLimitNeg);
    MC_T_DWORD _MC_THIS_API SetPositionRangeLimits(MC_T_BOOL bEnaLimitPos, MC_T_BOOL bEnaLimitNeg,  /* DRV_OBJ_POSITION_RANGE_LIMIT */
                                              MC_T_REAL fLimitPos, MC_T_REAL fLimitNeg);

    MC_T_DWORD SetHomingParameters(MC_T_BYTE byMethod,              /* Homing method */
                                   MC_T_REAL fSpeedSearchSwitch,    /* Speed during search for switch [mm/s] */
                                   MC_T_REAL fSpeedSearchZero,      /* Speed during search for zero [mm/s] */
                                   MC_T_REAL fAcceleration,         /* Homing acceleration [mm/s^2] */
                                   MC_T_REAL fOffset);              /* Home offset [mm] */

    MC_T_DWORD GetErrorCode(EC_T_WORD* pwErrorCode);                /* DRV_OBJ_ERROR_CODE                0x603F */

    MC_T_DWORD _MC_THIS_API GetLogMsg(MC_T_CHAR** pszLog);

ACCESSMOD:

    MC_T_CIA402_STATE       eDS402State;            /* DS402 drive state   */
    MC_T_CIA402_STATE       eLastDS402State;        /* DS402 last drive state   */
    MC_T_SERCOS_STATE       eSercosState;           /* SERCOS drive state   */
    MC_T_SIMU_STATE         eSimuState;             /* State of simulated drive */

#if defined(EC_MOTION_SUPPORT_MC_CAMMING) || defined(EC_MOTION_SUPPORT_MC_GEARING)
    MC_T_AXIS_REF*          pSyncMasterAxis;        /* reference to master axis for sync.motion etc. used by slave */
#endif
#if defined(EC_MOTION_SUPPORT_MC_CAMMING)
	MC_T_CAM_ID*            pCamTableID;            /* only valid for slave axis */
#endif
#if defined(EC_MOTION_SUPPORT_MC_GEARING)
	MC_T_GEARING_DATA       oGearingData;            /* only valid for slave axis */
#endif

    MC_T_PLCOPEN_STATE      ePLCOpenState;          /* PLCOpen state        */
    MC_T_PLCOPEN_STATE      eLastPLCOpenState;      /* previous PLCOpen state (just for print state change)  */
    MC_T_WORD               wStatusWord;            /* Status Word */
    MC_T_WORD               wControlWord;           /* Control Word */
    MC_T_BOOL               bInputsInvalid;         /* if TRUE inputs are invalid */

    MC_T_DWORD              dwCycleCount;           /* just for logging purpose */
public:
    MC_T_INT                dwCalcIncPerMM;         /* calculation (internal) increments per mm */
    MC_T_REAL               fVelToInc;              /* conversion factor from mm/s to inc/cycle */
    MC_T_REAL               fAccToInc;              /* conversion factor from mm/s^2 to inc/cycle^2 */
    MC_T_REAL               fJerkToInc;             /* conversion factor from mm/s^3 to inc/cycle^3 */
ACCESSMOD:
    MC_T_INT                lVeltoAcc;              /* conversion factor from inc/cycle^2 to inc/cycle^3 */
    MC_T_REAL               fMaxDistance;           /* maximum distance in mm */
    MC_T_REAL               fMaxVel;                /* maximum velocity in mm/s */
    MC_T_REAL               fMaxAcc;                /* maximum acc in mm/s^2 */
    MC_T_REAL               fMaxDec;                /* maximum dec in mm/s^2 */
    MC_T_REAL               fMaxJerk;               /* maximum jerk in mm/s^3 */
    MC_T_POS_CALC           lSWLimitPositivePos;    /* SW Limit positive position */
    MC_T_POS_CALC           lSWLimitNegativePos;    /* SW Limit negative position */
    MC_T_BOOL               bSWLimitPositiveEna;    /* SW Limit positive enabled  */
    MC_T_BOOL               bSWLimitNegativeEna;    /* SW Limit negative enabled  */

    MC_T_BOOL               bError;                 /* Indicates if an error has occurred */
    MC_T_WORD               wErrorID;               /* Error identification */

    MC_T_BOOL               bMcPowerStatus;         /* copy of Mc_Power.Status */
    MC_T_BOOL               bMcPowerEnable;         /* copy of Mc_Power.Enable */
public:
    MC_T_DIRECTION          dwDirection;            /* direction for current movement */
    MC_T_POS_CALC           lCommandedPosition;     /* [Inc] target position */
    MC_T_POS_CALC           lActualPosition;        /* [Inc} actual position including offset */
ACCESSMOD:
    MC_T_SWORD              sActualTorque;
    MC_T_POS_CALC           lActualPosRaw;          /* [Inc] actual position raw value */
    MC_T_POS_CALC           lPosOffset;             /* [Inc] position offset */
    MC_T_POS_CALC           lOldActualPosition;     /* last actual position */
    MC_T_VELACC             lActualVelocity;        /* actual velocity calculated in MC_Power */

    MC_T_BOOL               bDriveErrorAckReq;      /* Set by MotionLib if drive error should be acknowledged. Reset by API user */
    MC_T_BOOL               bDriveFollowBitSupp;    /* Set if bit "Drive follow the commanded value" is supported */
    EC_T_SDWORD             lLastActualPosDrive;    /* last read actual position */
    MC_T_INT                nActualPosOverrunCounter; /* Counts 32-Bit overruns to extend value range */

    /* Internal state */
    MC_T_OPMODE             eOperationMode;         /* Internal operation mode */
	MC_T_SETVALUE_SOURCE    eSetValSource;			/* source for set value */
	MC_T_SETVALUE_SOURCE    eSetValSourceOrg;		/* orignal source for set value */
	MC_T_WORD               wActOpMode;             /* Drive mode before homing, s. object 0x6060 */

    MC_T_INT                nFbFillIdx;             /* Fill index for FB buffer */
    MC_T_INT                nFbEmptyIdx;            /* Empty index for FB buffer */
    MC_T_BUFFERENTRY*       pWorkFb;                /* Pointer to current FB buffer */
    MC_T_BUFFERENTRY        apFBBuffer[MC_FB_BUFFER_LEN];

#define MC_LOG_BUFFER_SIZE 256
    MC_T_CHAR               szLog[MC_LOG_BUFFER_SIZE];
    MC_T_WORD               wProfileOperationMode;  /* Operation mode DS402: 0x6060 SoE: S-0-0032 */

    /* Init params. \sa Init() */
    MC_T_AXIS_TYPE          eAxisType;
    MC_T_INT                dwCycleTime;            /* [usec] EtherCAT bus cycle time */

#ifdef MOTIONTEST
public:
#endif

    MC_T_INT                dwIncPerMM;             /* Increments per mm */

ACCESSMOD:

    /* Pointers to EtherCAT process data.  \sa InitEcat() */
    MC_T_WORD*              pwPdStatusWord;         /* ptr to status  word in process data */
    MC_T_WORD*              pwPdControlWord;        /* ptr to control word in process data */

    MC_T_SDWORD*            plPdActualPosition;     /* ptr to actual position in process data */
                                                    /* Data type DINT (INTEGER32 -2147483648 to +2147483627) */
    MC_T_SDWORD*            plPdTargetPosition;     /* ptr to target position: Data type DINT (INTEGER32 -2147483648 to +2147483627)
                                                       In Cyclic Synchronous Position mode it is always interpreted as an absolute value */

    MC_T_SDWORD*            plPdTargetVelocity;     /* ptr to target velocity: Data type DINT (INTEGER32 -2147483648 to +2147483627) */
    MC_T_SDWORD*            plPdVelocityOffset;     /* ptr to velocity offset for feed forward: Data type DINT (INTEGER32 -2147483648 to +2147483627) */
    MC_T_DWORD*             pdwDigitalInputs;       /* ptr to digital inputs */

    MC_T_SWORD*             psPdActualTorque;       /* ptr to actual torque: Data type INT (INTEGER16 -32768 to +32767) */
    MC_T_SWORD*             psPdTargetTorque;       /* ptr to target torque: Data type INT (INTEGER16 -32768 to +32767) */
    MC_T_SWORD*             psPdTorqueOffset;       /* ptr to torque offset for feed forward: Data type INT (INTEGER16 -32768 to +32767) */
    MC_T_BYTE*              pbyPdModeOfOperation;   /* ptr to modes of operation */
    MC_T_BYTE*              pbyPdModeOfOperationDisplay;   /* ptr to modes of operation display */

    MC_T_DWORD*             pdwDigitalOutputs;      /* ptr to digital outputs */

#ifdef EC_MOTION_SUPPORT_PP_MODE
    MC_T_DWORD*             pdwPdProfileVelocity;
    MC_T_DWORD*             pdwPdProfileAcc;
    MC_T_DWORD*             pdwPdProfileDec;
#endif

    /* EtherCAT specific data. \sa InitEcat() */
    MC_T_DWORD              dwVendorId;
    MC_T_DWORD              dwProductCode;
    MC_T_DWORD              dwSlaveID;
    MC_T_WORD               wStationAddress;        /* slave station address */
    MC_T_AXIS_PROFILE       eProfile;
    MC_T_BYTE               bySercosDriveNo;        /* Adressed drive number within SERCOS servo controller */
    MC_T_WORD               wCoeIdxOpMode;          /* Idx of Mode of Operation object (DS402 only). Default: 0x6060 */
    MC_T_WORD               wCoeSubIdxOpMode;       /* Idx of Mode of Operation object (DS402 only). Default: 0x6060 */

    /* Pointers to EC-Master API functions. \sa InitEcat() */
    EcatCoeSdoDownloadFptr  pEcatCoeSdoDownload; /* ptr to EC-Master's SDO/CoE-Download function. EC_NULL if not provided. */
    EcatCoeSdoUploadFptr    pEcatCoeSdoUpload;   /* ptr to EC-Master's SDO/CoE-Upload function. EC_NULL if not provided. */
    EcatSoeWriteFptr        pEcatSoeWrite;       /* ptr to EC-Master's SDO/SoE-Write function. EC_NULL if not provided. */
    EcatSoeReadFptr         pEcatSoeRead;        /* ptr to EC-Master's SDO/SoE-Read function. EC_NULL if not provided. */
    EcatGetSlaveStateFptr   pEcatGetSlaveState;  /* ptr to EC-Master's GetSlaveState function. EC_NULL if not provided. */

    /* Pointers to performance measurement functions / data.  \sa InitPerfMeas() */
    EC_T_TSC_MEAS_DESC*     pTscMeasDesc;
    PerfMeasStartFptr       pPerfMeasStart;      /* ptr to performance measurement start function. NULL if not provided. */
    PerfMeasEndFptr         pPerfMeasEnd;        /* ptr to performance measurement end function. NULL if not provided. */

    MC_T_INT                lIncFactor;             /* internal position values are x times bigger */
                                                         /* Values: 0 = factor = 1 (value<<0) */
                                                         /* Values: 1 = factor = 2 (value<<1) */
                                                         /* Values: 2 = factor = 4 (value<<2) */
                                                         /* Values: 3 = factor = 8 (value<<3) */
    EC_T_LOG_PARMS          LogParms;
    MC_T_DWORD              dwVelocityGain;         /* CSV-Mode: Velocity Gain */
                                                    /* CSP-Mode: Velocity Gain for Feed Forward Object 0x60B1 */
    MC_T_DWORD              dwTorqueGain;           /* CSP and CSV-Mode: Torque Gain for Feed Forward Object 0x60B2 */

public:
    MC_T_MOVEMENT           oMove;                  /* current movement */

ACCESSMOD:

    EC_T_VOID       SetAxisError(MC_T_WORD wErrorID);
    MC_T_BOOL       IsOpModeValid(MC_T_WORD wOpMode);
    MC_T_BOOL       CheckDrvFollowBitSupported(MC_T_WORD wOpMode);
    MC_T_OPMODE     MapOpMode();
} _MC_PACKED;


/* Base class for all FB's */
class _MC_API MC_FB_T
{
public:
    // OUT's
    const MC_T_BOOL     &Error;                  /* OUT(B): Indicates if an error has occurred */
    const MC_T_WORD     &ErrorID;                /* OUT(E): Error identification */

    // IN_OUT's
    MC_T_AXIS_REF       *Axis;                   /* INOUT(B): Reference to the axis */

protected:

    MC_FB_T(MC_T_AXIS_REF *pAxis, MC_T_FBID id)
        : // Init external OUT's
          Error(static_cast<const MC_T_BOOL &>(bError)),
          ErrorID(static_cast<const MC_T_WORD &>(wErrorID)),
          // Init internal IN's
          // Init variables
          Axis(pAxis),
          wFbID(id),
          bError(0),
          wErrorID(0),
          fbState(FBS_IDLE),
          dwStateChCycle(0),
          // Init internal IN_OUT's
          pAxis(Axis)
    {}

    MC_T_FBID           wFbID;          /* ID of FB */

    MC_T_BOOL           bError;         /* OUT: Indicates if an error has occurred */
    MC_T_WORD           wErrorID;       /* OUT: Error identification */

    MC_T_FBSTATE        fbState;        /* Internal: MCFB state machine */
    MC_T_DWORD          dwStateChCycle; /* Internal: Cycle on last state change */

    MC_T_AXIS_REF     *&pAxis;          /* INOUT: Reference to the axis */

    const char * const GetName() const;
    void ChangeState(MC_T_FBSTATE newState);

private:

    ASSIGNOP(MC_FB_T);

    friend void   MC_AbortCommand
        (MC_T_AXIS_REF*         pAxis
        ,MC_T_WORD              wNewFbID);

    // MC_MOVE_VELOCITY_T::DoState_Busy() accesses protected members of MC_FB_T
    friend class MC_MOVE_VELOCITY_T;
} _MC_PACKED;

/* Base class for all buffered FB's */

class _MC_API MC_BUFFERED_FB_T : public MC_FB_T
{
public:

    // OUT's
    const MC_T_BOOL     &Active;                 /* OUT(E): Indicates that the FB has control on the axis */
    const MC_T_BOOL     &CommandAborted;         /* OUT(E): Command is aborted by another command */

    // IN's
    MC_T_BUFFER_MODE    BufferMode;              /* IN(E): Defines the chronological sequence */

protected:

    MC_T_BOOL bActive;                           /* OUT: Indicates that the FB has control on the axis */
    MC_T_BOOL bCommandAborted;                   /* OUT: Command is aborted by another command */

    const MC_T_BUFFER_MODE &dwBufferMode;        /* IN: Start the motion at rising edge */

    MC_BUFFERED_FB_T(MC_T_AXIS_REF *pAxis, MC_T_FBID id)
        : // Init from constructor
          MC_FB_T(pAxis, id),
          // Init external OUT's
          Active(static_cast<const MC_T_BOOL &>(bActive)),
          CommandAborted(static_cast<const MC_T_BOOL &>(bCommandAborted)),
          // Init variables
          BufferMode(MC_ABORTING),
          bActive(0),
          bCommandAborted(0),
          // Init internal IN's
          dwBufferMode(static_cast<const MC_T_BUFFER_MODE &>(BufferMode))
    {}

private:

    ASSIGNOP(MC_BUFFERED_FB_T);

    friend MC_T_WORD HandleBlending(MC_T_AXIS_REF *pAxis,
                        MC_BUFFERED_FB_T *pMcfb, MC_T_DWORD dwBufferMode, MC_T_REAL fVelocity, MC_T_REAL fEndVelocity, MC_T_BUFFERENTRY** ppPrevFb);

    friend EC_T_WORD MC_BufferActivateNext(
         MC_T_AXIS_REF *pAxis, MC_BUFFERED_FB_T *pMcfb, MC_T_BOOL* fbActive);

    friend MC_T_WORD   MC_BufferPut
        (MC_T_AXIS_REF*         pAxis
        ,MC_BUFFERED_FB_T*      pMcfb
        ,MC_T_REAL              fEndVel
        ,MC_T_BUFFERENTRY**     ppPrevFb);
} _MC_PACKED;

/* MC_Power (3.1) */
class _MC_API MC_POWER_T : public MC_FB_T
{
public:

    // OUT's
    const MC_T_BOOL     &Status;                 /* OUT(B): Effective state of the power stage */
    const MC_T_BOOL     &Valid;                  /* OUT(E): If TRUE a valid set of outputs is available */
    const MC_T_BOOL     &FaultReaction;          /* OUT   : If TRUE the drive is in fault reaction state */

    // IN's
    MC_T_BOOL           Enable;                  /* IN(B): As long as is true, power is on */
    MC_T_BOOL           EnablePositive;          /* NOT SUPP: IN(E): As long as is true, permits motion in positive direction only */
    MC_T_BOOL           EnableNegative;          /* NOT SUPP: IN(E): As long as is true, permits motion in negative direction only */

    MC_POWER_T(MC_T_AXIS_REF *pAxis = MC_NULL)
        : // Init from constructor
          MC_FB_T(pAxis, MCFB_ID_POWER),
          // Init external OUT's
          Status(static_cast<const MC_T_BOOL &>(bStatus)),
          Valid(static_cast<const MC_T_BOOL &>(bValid)),
          FaultReaction(static_cast<const MC_T_BOOL &>(bFaultReaction)),
          // Init variables
          Enable(0),
          EnablePositive(0),
          EnableNegative(0),
          bStatus(0),
          bValid(0),
          bFaultReaction(0),
          // Init internal IN's
          bEnable(static_cast<const MC_T_BOOL &>(Enable)),
          bEnable_Positive(static_cast<const MC_T_BOOL &>(EnablePositive)),
          bEnable_Negative(static_cast<const MC_T_BOOL &>(EnableNegative))
    {}

    void _MC_THIS_API OnCycle();

private:

    MC_T_BOOL           bStatus;                 /* OUT: Effective state of the power stage */
    MC_T_BOOL           bValid;                  /* OUT: If TRUE a valid set of outputs is available */
    MC_T_BOOL           bFaultReaction;          /* OUT: If TRUE the drive is in fault reaction state */

    const MC_T_BOOL     &bEnable;                /* IN: As long as is true, power is on */
    const MC_T_BOOL     &bEnable_Positive;       /* NOT SUPP: IN: As long as is true, permits motion in positive direction only */
    const MC_T_BOOL     &bEnable_Negative;       /* NOT SUPP: IN: As long as is true, permits motion in negative direction only */

    ASSIGNOP(MC_POWER_T);
    void DS402StateMachine();
    MC_T_BOOL SercosSwitchToOp();
    void SercosStateMachine();
    void SimuStateMachine();
    void InitAxis();

    friend void   MC_AbortCommand
        (MC_T_AXIS_REF*         pAxis
         ,MC_T_WORD              wNewFbID);
} _MC_PACKED;

/* MC_Home (3.2) */
class _MC_API MC_HOME_T : public MC_BUFFERED_FB_T
{
public:

    // OUT's
    const MC_T_BOOL     &Done;                   /* OUT(B): The axis is within a range close to the target position */
    const MC_T_BOOL     &Busy;                   /* OUT(E): The FB is not finished and new output values are to be expected */

    // IN's
    MC_T_BOOL           Execute;                /* IN(B): Start the motion at rising edge */
    MC_T_REAL           Position;               /* IN(E): Absolute position when the reference signal is detected */
    MC_T_BOOL           EnableSetPosition;      /* IN(E): Absolute position when the reference signal is detected */

    MC_HOME_T(MC_T_AXIS_REF *pAxis = MC_NULL)
        : // Init from constructor
        MC_BUFFERED_FB_T(pAxis, MCFB_ID_HOME),
        // Init external OUT's
        Done(static_cast<const MC_T_BOOL &>(bDone)),
        Busy(static_cast<const MC_T_BOOL &>(bBusy)),
        // Init variables
        Execute(0),
        Position(0.0),
        EnableSetPosition(EC_FALSE),
        bDone(0),
        bBusy(0),
        // Init internal IN's
        bExecute(static_cast<const MC_T_BOOL &>(Execute)),
        fPosition(static_cast<const MC_T_REAL &>(Position)),
        bEnableSetPosition(static_cast<const MC_T_BOOL &>(EnableSetPosition))
    {}

    void _MC_THIS_API OnCycle();

private:

    MC_T_BOOL           bDone;           /* OUT: The axis is within a range close to the target position */
    MC_T_BOOL           bBusy;           /* OUT: The FB is not finished and new output values are to be expected */

    const MC_T_BOOL     &bExecute;              /* IN: Start the motion at rising edge */
    const MC_T_REAL     &fPosition;             /* IN: Absolute position when the reference signal is detected */
    const MC_T_BOOL     &bEnableSetPosition;    /* IN: TRUE = set fPosition after homing */

    ASSIGNOP(MC_HOME_T);
    void DoState_Busy();
    void StateTrans_Active();
    void DoState_Active();
    void StateTrans_Execute();
    MC_T_BOOL StateMachine();

    friend void   MC_AbortCommand
        (MC_T_AXIS_REF*     pAxis
        , MC_T_WORD         wNewFbID);
} _MC_PACKED;

/* MC_Stop (3.3) */
class _MC_API MC_STOP_T : public MC_FB_T
{
public:

    // OUT's
    const MC_T_BOOL     &Done;                   /* OUT(B): The axis is within a range close to the target position */
    const MC_T_BOOL     &Busy;                   /* OUT(E): The FB is not finished and new output values are to be expected */
    const MC_T_BOOL     &CommandAborted;         /* OUT(E): Command is aborted by switching off power */

    // IN's
    MC_T_BOOL           Execute;                 /* IN(B): Start the motion at rising edge */
    MC_T_REAL           Deceleration;            /* IN(E): Value of the deceleration (always positive, decreasing energy of the motor). */
    MC_T_REAL           Jerk;                    /* IN(E): Value of the Jerk (always positive). */

    MC_STOP_T(MC_T_AXIS_REF *pAxis = MC_NULL)
        : // Init from constructor
          MC_FB_T(pAxis, MCFB_ID_STOP),
          // Init external OUT's
          Done(static_cast<const MC_T_BOOL &>(bDone)),
          Busy(static_cast<const MC_T_BOOL &>(bBusy)),
          CommandAborted(static_cast<const MC_T_BOOL &>(bCommandAborted)),
          // Init variables
          Execute(0),
          Deceleration(0.0),
          Jerk(0.0),
          bDone(0),
          bBusy(0),
          bCommandAborted(0),
          // Init internal IN's
          bExecute(static_cast<const MC_T_BOOL &>(Execute)),
          fDeceleration(static_cast<const MC_T_REAL &>(Deceleration)),
          fJerk(static_cast<const MC_T_REAL &>(Jerk))
    {}

    void _MC_THIS_API OnCycle();

private:

    MC_T_BOOL           bDone;                   /* OUT: The axis is within a range close to the target position */
    MC_T_BOOL           bBusy;                   /* OUT: The FB is not finished and new output values are to be expected */
    MC_T_BOOL           bCommandAborted;         /* OUT: Command is aborted by switching off power */

    const MC_T_BOOL     &bExecute;               /* IN: Start the motion at rising edge */
    const MC_T_REAL     &fDeceleration;          /* IN: Value of the deceleration (always positive, decreasing energy of the motor). */
    const MC_T_REAL     &fJerk;                  /* IN: Value of the Jerk (always positive). */

    ASSIGNOP(MC_STOP_T);
    void StateTrans_Execute();
    void DoState_Busy();
    MC_T_BOOL StateMachine();

    friend void   MC_AbortCommand
        (MC_T_AXIS_REF*         pAxis
        ,MC_T_WORD              wNewFbID);
} _MC_PACKED;

/* MC_Halt (3.4) */
class _MC_API MC_HALT_T : public MC_BUFFERED_FB_T
{
public:

    // OUT's
    const MC_T_BOOL     &Done;                   /* OUT(B): The axis is within a range close to the target position */
    const MC_T_BOOL     &Busy;                   /* OUT(E): The FB is not finished and new output values are to be expected */

    // IN's
    MC_T_BOOL           Execute;                 /* IN(B): Start the motion at rising edge */
    MC_T_REAL           Deceleration;            /* IN(E): Value of the deceleration (always positive, decreasing energy of the motor). */
    MC_T_REAL           Jerk;                    /* IN(E): Value of the Jerk (always positive). */

    MC_HALT_T(MC_T_AXIS_REF *pAxis = MC_NULL)
        : // Init from constructor
          MC_BUFFERED_FB_T(pAxis, MCFB_ID_HALT),
          // Init external OUT's
          Done(static_cast<const MC_T_BOOL &>(bDone)),
          Busy(static_cast<const MC_T_BOOL &>(bBusy)),
          // Init variables
          Execute(0),
          Deceleration(0.0),
          Jerk(0.0),
          bDone(0),
          bBusy(0),
          // Init internal IN's
          bExecute(static_cast<const MC_T_BOOL &>(Execute)),
          fDeceleration(static_cast<const MC_T_REAL &>(Deceleration)),
          fJerk(static_cast<const MC_T_REAL &>(Jerk))
    {}

    void _MC_THIS_API OnCycle();

private:

    MC_T_BOOL           bDone;           /* OUT: The axis is within a range close to the target position */
    MC_T_BOOL           bBusy;           /* OUT: The FB is not finished and new output values are to be expected */

    const MC_T_BOOL     &bExecute;       /* IN: Start the motion at rising edge */
    const MC_T_REAL     &fDeceleration;  /* IN: Value of the deceleration (always positive, decreasing energy of the motor). */
    const MC_T_REAL     &fJerk;          /* IN: Value of the Jerk (always positive). */

    ASSIGNOP(MC_HALT_T);
    void DoState_Busy();
    void StateTrans_Active();
    void DoState_Active();
    void StateTrans_Execute();
    MC_T_BOOL StateMachine();

    friend void   MC_AbortCommand
        (MC_T_AXIS_REF*         pAxis
        ,MC_T_WORD              wNewFbID);
} _MC_PACKED;

/* MC_MoveAbsolute (3.5) */
class _MC_API MC_MOVE_ABSOLUTE_T : public MC_BUFFERED_FB_T
{
public:

    // OUT's
    const MC_T_BOOL     &Done;                   /* OUT(B): The axis is within a range close to the target position */
    const MC_T_BOOL     &Busy;                   /* OUT(E): The FB is not finished and new output values are to be expected */

    // IN's
    MC_T_BOOL           Execute;                 /* IN(B): Start the motion at rising edge */
    MC_T_BOOL           ContinuousUpdate;        /* IN(E): Continuous Update (Trapezoid profile only) */
    MC_T_REAL           Position;                /* IN(B): Target position for the motion */
    MC_T_REAL           Velocity;                /* IN(B): Value of the maximum velocity (always positive, not necessarily reached). */
    MC_T_REAL           Acceleration;            /* IN(E): Value of the acceleration (always positive, increasing energy of the motor). */
    MC_T_REAL           Deceleration;            /* IN(E): Value of the deceleration (always positive, decreasing energy of the motor). */
    MC_T_REAL           Jerk;                    /* IN(E): Value of the Jerk (always positive). */
    MC_T_DIRECTION      Direction;               /* IN(B): Direction in which the movement is started:
                                                   1 positive direction
                                                   2 shortest way
                                                   3 negative direction
                                                   4 current direction */

    MC_MOVE_ABSOLUTE_T(MC_T_AXIS_REF *pAxis = MC_NULL)
        : // Init from constructor
          MC_BUFFERED_FB_T(pAxis, MCFB_ID_MOVE_ABSOLUTE),
          // Init external OUT's
          Done(static_cast<const MC_T_BOOL &>(bDone)),
          Busy(static_cast<const MC_T_BOOL &>(bBusy)),
          // Init variables
          Execute(0),
          ContinuousUpdate(0),
          Position(0.0),
          Velocity(0.0),
          Acceleration(0.0),
          Deceleration(0.0),
          Jerk(0.0),
          Direction(MC_DIR_POSITIVE),
          bDone(0),
          bBusy(0),
          // Init internal IN's
          bExecute(static_cast<const MC_T_BOOL &>(Execute)),
          bContUpdate(static_cast<const MC_T_BOOL &>(ContinuousUpdate)),
          fPosition(static_cast<const MC_T_REAL &>(Position)),
          fVelocity(static_cast<const MC_T_REAL &>(Velocity)),
          fAcceleration(static_cast<const MC_T_REAL &>(Acceleration)),
          fDeceleration(static_cast<const MC_T_REAL &>(Deceleration)),
          fJerk(static_cast<const MC_T_REAL &>(Jerk)),
          dwDirection(static_cast<const MC_T_DIRECTION &>(Direction))
    {}

    void _MC_THIS_API OnCycle();

private:

    MC_T_BOOL           bDone;                   /* OUT: The axis is within a range close to the target position */
    MC_T_BOOL           bBusy;                   /* OUT: The FB is not finished and new output values are to be expected */

    const MC_T_BOOL     &bExecute;               /* IN: Start the motion at rising edge */
    const MC_T_BOOL     &bContUpdate;            /* IN: Continuous Update (only trapeze) */
    const MC_T_REAL     &fPosition;              /* IN: Target position for the motion */
    const MC_T_REAL     &fVelocity;              /* IN: Value of the maximum velocity (always positive, not necessarily reached). */
    const MC_T_REAL     &fAcceleration;          /* IN: Value of the acceleration (always positive, increasing energy of the motor). */
    const MC_T_REAL     &fDeceleration;          /* IN: Value of the deceleration (always positive, decreasing energy of the motor). */
    const MC_T_REAL     &fJerk;                  /* IN: Value of the Jerk (always positive). */
    const MC_T_DIRECTION
                        &dwDirection;            /* IN: Direction in which the movement is started:
                                                        1 positive direction
                                                        2 shortest way
                                                        3 negative direction
                                                        4 current direction */

    ASSIGNOP(MC_MOVE_ABSOLUTE_T);
    void StateTrans_Execute();
    void StateTrans_Active();
    void DoState_Active();
    void DoState_Busy();
    MC_T_BOOL StateMachine();

    friend MC_T_WORD HandleBlending(MC_T_AXIS_REF *pAxis,
                        MC_BUFFERED_FB_T *pMcfb, MC_T_DWORD dwBufferMode, MC_T_REAL fVelocity, MC_T_REAL fEndVelocity, MC_T_BUFFERENTRY** ppPrevFb);

    MC_T_WORD BlendingRecalcMove(MC_T_BUFFERENTRY *pPrevFb, MC_T_REAL fDistance);

    friend void   MC_AbortCommand
        (MC_T_AXIS_REF*         pAxis
        ,MC_T_WORD              wNewFbID);
} _MC_PACKED;

/* MC_MoveRelative (3.6) */
class _MC_API MC_MOVE_RELATIVE_T : public MC_BUFFERED_FB_T
{
public:

    // OUT's
    const MC_T_BOOL     &Done;                   /* OUT(B): The axis is within a range close to the target position */
    const MC_T_BOOL     &Busy;                   /* OUT(E): The FB is not finished and new output values are to be expected */

    // IN's
    MC_T_BOOL           Execute;                 /* IN(B): Start the motion at rising edge */
    MC_T_BOOL           ContinuousUpdate;        /* IN(E): Continuous Update (Trapezoid profile only) */
    MC_T_REAL           Distance;                /* IN(B): Relative distance for the motion */
    MC_T_REAL           Velocity;                /* IN(E): Value of the maximum velocity (always positive, not necessarily reached). */
    MC_T_REAL           Acceleration;            /* IN(E): Value of the acceleration (always positive, increasing energy of the motor). */
    MC_T_REAL           Deceleration;            /* IN(E): Value of the deceleration (always positive, decreasing energy of the motor). */
    MC_T_REAL           Jerk;                    /* IN(E): Value of the Jerk (always positive). */
#ifdef MOTIONTEST
    MC_T_REAL           EndVel;
#endif

    MC_MOVE_RELATIVE_T(MC_T_AXIS_REF *pAxis = MC_NULL)
        : // Init from constructor
          MC_BUFFERED_FB_T(pAxis, MCFB_ID_MOVE_RELATIVE),
          // Init external OUT's
          Done(static_cast<const MC_T_BOOL &>(bDone)),
          Busy(static_cast<const MC_T_BOOL &>(bBusy)),
          // Init variables
          Execute(0),
          ContinuousUpdate(0),
          Distance(0.0),
          Velocity(0.0),
          Acceleration(0.0),
          Deceleration(0.0),
          Jerk(0.0),
#ifdef MOTIONTEST
          EndVel(0.0),
#endif
          bDone(0),
          bBusy(0),
          // Init internal IN's
          bExecute(static_cast<const MC_T_BOOL &>(Execute)),
          bContUpdate(static_cast<const MC_T_BOOL &>(ContinuousUpdate)),
          fDistance(static_cast<const MC_T_REAL &>(Distance)),
          fVelocity(static_cast<const MC_T_REAL &>(Velocity)),
          fAcceleration(static_cast<const MC_T_REAL &>(Acceleration)),
          fDeceleration(static_cast<const MC_T_REAL &>(Deceleration)),
          fJerk(static_cast<const MC_T_REAL &>(Jerk))
#ifdef MOTIONTEST
          ,fEndVel(static_cast<const MC_T_REAL &>(EndVel))
#endif
    {}

    void _MC_THIS_API OnCycle();

private:

    MC_T_BOOL           bDone;           /* OUT: The axis is within a range close to the target position */
    MC_T_BOOL           bBusy;           /* OUT: The FB is not finished and new output values are to be expected */

    const MC_T_BOOL     &bExecute;       /* IN: Start the motion at rising edge */
    const MC_T_BOOL     &bContUpdate;    /* IN: Continuous Update (only trapeze) */
    const MC_T_REAL     &fDistance;      /* IN: Relative distance for the motion */
    const MC_T_REAL     &fVelocity;      /* IN: Value of the maximum velocity (always positive, not necessarily reached). */
    const MC_T_REAL     &fAcceleration;  /* IN: Value of the acceleration (always positive, increasing energy of the motor). */
    const MC_T_REAL     &fDeceleration;  /* IN: Value of the deceleration (always positive, decreasing energy of the motor). */
    const MC_T_REAL     &fJerk;          /* IN: Value of the Jerk (always positive). */

#ifdef MOTIONTEST
    const MC_T_REAL     &fEndVel;        /* just for testing purpose */
#endif

    ASSIGNOP(MC_MOVE_RELATIVE_T);
    void StateTrans_Execute();
    void StateTrans_Active();
    void DoState_Active();
    void DoState_Busy();
    MC_T_BOOL StateMachine();

    friend MC_T_WORD HandleBlending(MC_T_AXIS_REF *pAxis,
                        MC_BUFFERED_FB_T *pMcfb, MC_T_DWORD dwBufferMode, MC_T_REAL fVelocity, MC_T_REAL fEndVelocity, MC_T_BUFFERENTRY** ppPrevFb);

    MC_T_WORD BlendingRecalcMove(MC_T_BUFFERENTRY* pPrevFb, MC_T_REAL fDistance);

    friend void   MC_AbortCommand
        (MC_T_AXIS_REF*         pAxis
        ,MC_T_WORD              wNewFbID);
} _MC_PACKED;

/* MC_MoveVelocity (3.10) */
class _MC_API MC_MOVE_VELOCITY_T : public MC_BUFFERED_FB_T
{
public:

    // OUT's
    const MC_T_BOOL     &InVelocity;             /* OUT(B): Command velocity reached */
    const MC_T_BOOL     &Busy;                   /* OUT(E): The FB is not finished and new output values are to be expected */

    // IN's
    MC_T_BOOL           Execute;                 /* IN(B): Start the motion at rising edge */
    MC_T_BOOL           ContinuousUpdate;        /* IN(E): Continuous Update (Trapezoid profile only) */
    MC_T_REAL           Velocity;                /* IN(E): Value of the maximum velocity (always positive) */
    MC_T_REAL           Acceleration;            /* IN(E): Value of the acceleration (always positive, increasing energy of the motor). */
    MC_T_REAL           Deceleration;            /* IN(E): Value of the deceleration (always positive, decreasing energy of the motor). */
    MC_T_REAL           Jerk;                    /* IN(E): Value of the Jerk (always positive). */
    MC_T_DIRECTION      Direction;               /* IN(E): Direction: MC_DIR_POSITIVE, MC_DIR_NEGATIVE, MC_DIR_CURRENT */

    MC_MOVE_VELOCITY_T(MC_T_AXIS_REF *pAxis = MC_NULL)
        : // Init from constructor
          MC_BUFFERED_FB_T(pAxis, MCFB_ID_MOVE_VELOCITY),
          // Init external OUT's
          InVelocity(static_cast<const MC_T_BOOL &>(bInVelocity)),
          Busy(static_cast<const MC_T_BOOL &>(bBusy)),
          // Init variables
          Execute(0),
          ContinuousUpdate(0),
          Velocity(0.0),
          Acceleration(0.0),
          Deceleration(0.0),
          Jerk(0.0),
          Direction(MC_DIR_POSITIVE),
          bInVelocity(0),
          bBusy(0),
          // Init internal IN's
          bExecute(static_cast<const MC_T_BOOL &>(Execute)),
          bContUpdate(static_cast<const MC_T_BOOL &>(ContinuousUpdate)),
          fVelocity(static_cast<const MC_T_REAL &>(Velocity)),
          fAcceleration(static_cast<const MC_T_REAL &>(Acceleration)),
          fDeceleration(static_cast<const MC_T_REAL &>(Deceleration)),
          fJerk(static_cast<const MC_T_REAL &>(Jerk)),
          dwDirection(static_cast<const MC_T_DIRECTION &>(Direction))
    {}

    void _MC_THIS_API OnCycle();

private:

    MC_T_BOOL           bInVelocity;         /* OUT: Command velocity reached */
    MC_T_BOOL           bBusy;               /* OUT: The FB is not finished and new output values are to be expected */

    const MC_T_BOOL     &bExecute;           /* IN: Start the motion at rising edge */
    const MC_T_BOOL     &bContUpdate;        /* IN: Continuous Update (only trapeze) */
    const MC_T_REAL     &fVelocity;          /* IN: Value of the maximum velocity (always positive) */
    const MC_T_REAL     &fAcceleration;      /* IN: Value of the acceleration (always positive, increasing energy of the motor). */
    const MC_T_REAL     &fDeceleration;      /* IN: Value of the deceleration (always positive, decreasing energy of the motor). */
    const MC_T_REAL     &fJerk;              /* IN: Value of the Jerk (always positive). */
    const MC_T_DIRECTION
                        &dwDirection;        /* IN: Direction: MC_DIR_POSITIVE, MC_DIR_NEGATIVE, MC_DIR_CURRENT */

    ASSIGNOP(MC_MOVE_VELOCITY_T);
    void StateTrans_Execute();
    void StateTrans_Active();
    void DoState_Active();
    void DoState_Busy();
    MC_T_BOOL StateMachine();

    friend MC_T_WORD HandleBlending(MC_T_AXIS_REF *pAxis,
                        MC_BUFFERED_FB_T *pMcfb, MC_T_DWORD dwBufferMode, MC_T_REAL fVelocity, MC_T_REAL fEndVelocity, MC_T_BUFFERENTRY** ppPrevFb);

    friend void   MC_AbortCommand
        (MC_T_AXIS_REF*         pAxis
        ,MC_T_WORD              wNewFbID);
} _MC_PACKED;

/* MC_MoveContinuousAbsolute (3.11) */
class _MC_API MC_MOVE_CONT_ABSOLUTE_T : public MC_BUFFERED_FB_T
{
public:

    // OUT's
    const MC_T_BOOL     &InEndVelocity;          /* OUT(B): Commanded distance reached and running at requested end velocity */
    const MC_T_BOOL     &Busy;                   /* OUT(E): The FB is not finished and new output values are to be expected */

    // IN's
    MC_T_BOOL           Execute;                 /* IN(B): Start the motion at rising edge */
    MC_T_BOOL           ContinuousUpdate;        /* IN(E): Continuous Update (Trapezoid profile only) */
    MC_T_REAL           Position;                /* IN(B): Target position for the motion */
    MC_T_REAL           EndVelocity;             /* IN(B): Value of the end velocity. Signed Value */
    MC_T_REAL           Velocity;                /* IN(B): Value of the maximum velocity (always positive, not necessarily reached). */
    MC_T_REAL           Acceleration;            /* IN(E): Value of the acceleration (always positive, increasing energy of the motor). */
    MC_T_REAL           Deceleration;            /* IN(E): Value of the deceleration (always positive, decreasing energy of the motor). */
    MC_T_REAL           Jerk;                    /* IN(E): Value of the Jerk (always positive). */
    MC_T_DIRECTION      Direction;               /* IN(B): Direction in which the movement is started:
                                                   1 positive direction
                                                   2 shortest way
                                                   3 negative direction
                                                   4 current direction */

    MC_MOVE_CONT_ABSOLUTE_T(MC_T_AXIS_REF *pAxis = MC_NULL)
        : // Init from constructor
          MC_BUFFERED_FB_T(pAxis, MCFB_ID_MOVE_CONTINUOUS_ABSOLUTE),
          // Init external OUT's
          InEndVelocity(static_cast<const MC_T_BOOL &>(bInEndVelocity)),
          Busy(static_cast<const MC_T_BOOL &>(bBusy)),
          // Init variables
          Execute(0),
          ContinuousUpdate(0),
          Position(0.0),
          EndVelocity(0.0),
          Velocity(0.0),
          Acceleration(0.0),
          Deceleration(0.0),
          Jerk(0.0),
          Direction(MC_DIR_POSITIVE),
          bInEndVelocity(0),
          bBusy(0),
          // Init internal IN's
          bExecute(static_cast<const MC_T_BOOL &>(Execute)),
          bContUpdate(static_cast<const MC_T_BOOL &>(ContinuousUpdate)),
          fPosition(static_cast<const MC_T_REAL &>(Position)),
          fEndVelocity(static_cast<const MC_T_REAL &>(EndVelocity)),
          fVelocity(static_cast<const MC_T_REAL &>(Velocity)),
          fAcceleration(static_cast<const MC_T_REAL &>(Acceleration)),
          fDeceleration(static_cast<const MC_T_REAL &>(Deceleration)),
          fJerk(static_cast<const MC_T_REAL &>(Jerk)),
          dwDirection(static_cast<const MC_T_DIRECTION &>(Direction))
    {}

    void _MC_THIS_API OnCycle();

private:

    MC_T_BOOL           bInEndVelocity;          /* OUT: Commanded distance reached and running at requested end velocity */
    MC_T_BOOL           bBusy;                   /* OUT: The FB is not finished and new output values are to be expected */

    const MC_T_BOOL     &bExecute;               /* IN: Start the motion at rising edge */
    const MC_T_BOOL     &bContUpdate;            /* IN: Continuous Update (only trapeze) */
    const MC_T_REAL     &fPosition;              /* IN: Target position for the motion */
    const MC_T_REAL     &fEndVelocity;           /* IN: Value of the end velocity. */
    const MC_T_REAL     &fVelocity;              /* IN: Value of the maximum velocity (always positive, not necessarily reached). */
    const MC_T_REAL     &fAcceleration;          /* IN: Value of the acceleration (always positive, increasing energy of the motor). */
    const MC_T_REAL     &fDeceleration;          /* IN: Value of the deceleration (always positive, decreasing energy of the motor). */
    const MC_T_REAL     &fJerk;                  /* IN: Value of the Jerk (always positive). */
    const MC_T_DIRECTION
                        &dwDirection;            /* IN: Direction in which the movement is started:
                                                        1 positive direction
                                                        2 shortest way
                                                        3 negative direction
                                                        4 current direction */

    ASSIGNOP(MC_MOVE_CONT_ABSOLUTE_T);
    void StateTrans_Execute();
    void StateTrans_Active();
    void DoState_Active();
    void DoState_Busy();
    MC_T_BOOL StateMachine();

    friend MC_T_WORD HandleBlending(MC_T_AXIS_REF *pAxis,
                        MC_BUFFERED_FB_T *pMcfb, MC_T_DWORD dwBufferMode, MC_T_REAL fVelocity, MC_T_REAL fEndVelocity, MC_T_BUFFERENTRY** ppPrevFb);

    friend void   MC_AbortCommand
        (MC_T_AXIS_REF*         pAxis
        ,MC_T_WORD              wNewFbID);
} _MC_PACKED;


/* MC_MoveContinuousRelative (3.12) */
class _MC_API MC_MOVE_CONT_RELATIVE_T : public MC_BUFFERED_FB_T
{
public:

    // OUT's
    const MC_T_BOOL     &InEndVelocity;          /* OUT(B): Commanded distance reached and running at requested end velocity */
    const MC_T_BOOL     &Busy;                   /* OUT(E): The FB is not finished and new output values are to be expected */

    // IN's
    MC_T_BOOL           Execute;                 /* IN(B): Start the motion at rising edge */
    MC_T_BOOL           ContinuousUpdate;        /* IN(E): Continuous Update (Trapezoid profile only) */
    MC_T_REAL           Distance;                /* IN(B): Relative distance for the motion */
    MC_T_REAL           EndVelocity;             /* IN(B): Value of the end velocity. Signed Value */
    MC_T_REAL           Velocity;                /* IN(E): Value of the maximum velocity (always positive, not necessarily reached). */
    MC_T_REAL           Acceleration;            /* IN(E): Value of the acceleration (always positive, increasing energy of the motor). */
    MC_T_REAL           Deceleration;            /* IN(E): Value of the deceleration (always positive, decreasing energy of the motor). */
    MC_T_REAL           Jerk;                    /* IN(E): Value of the Jerk (always positive). */

    MC_MOVE_CONT_RELATIVE_T(MC_T_AXIS_REF *pAxis = MC_NULL)
        : // Init from constructor
          MC_BUFFERED_FB_T(pAxis, MCFB_ID_MOVE_CONTINUOUS_RELATIVE),
          // Init external OUT's
          InEndVelocity(static_cast<const MC_T_BOOL &>(bInEndVelocity)),
          Busy(static_cast<const MC_T_BOOL &>(bBusy)),
          // Init variables
          Execute(0),
          ContinuousUpdate(0),
          Distance(0.0),
          EndVelocity(0.0),
          Velocity(0.0),
          Acceleration(0.0),
          Deceleration(0.0),
          Jerk(0.0),
          bInEndVelocity(0),
          bBusy(0),
          // Init internal IN's
          bExecute(static_cast<const MC_T_BOOL &>(Execute)),
          bContUpdate(static_cast<const MC_T_BOOL &>(ContinuousUpdate)),
          fDistance(static_cast<const MC_T_REAL &>(Distance)),
          fEndVelocity(static_cast<const MC_T_REAL &>(EndVelocity)),
          fVelocity(static_cast<const MC_T_REAL &>(Velocity)),
          fAcceleration(static_cast<const MC_T_REAL &>(Acceleration)),
          fDeceleration(static_cast<const MC_T_REAL &>(Deceleration)),
          fJerk(static_cast<const MC_T_REAL &>(Jerk))
    {}

    void _MC_THIS_API OnCycle();

private:

    MC_T_BOOL           bInEndVelocity;  /* OUT: Commanded distance reached and running at requested end velocity */
    MC_T_BOOL           bBusy;           /* OUT: The FB is not finished and new output values are to be expected */

    const MC_T_BOOL     &bExecute;       /* IN: Start the motion at rising edge */
    const MC_T_BOOL     &bContUpdate;    /* IN: Continuous Update (only trapeze) */
    const MC_T_REAL     &fDistance;      /* IN: Relative distance for the motion */
    const MC_T_REAL     &fEndVelocity;           /* IN: Value of the end velocity. */
    const MC_T_REAL     &fVelocity;      /* IN: Value of the maximum velocity (always positive, not necessarily reached). */
    const MC_T_REAL     &fAcceleration;  /* IN: Value of the acceleration (always positive, increasing energy of the motor). */
    const MC_T_REAL     &fDeceleration;  /* IN: Value of the deceleration (always positive, decreasing energy of the motor). */
    const MC_T_REAL     &fJerk;          /* IN: Value of the Jerk (always positive). */

    ASSIGNOP(MC_MOVE_CONT_RELATIVE_T);
    void StateTrans_Execute();
    void StateTrans_Active();
    void DoState_Active();
    void DoState_Busy();
    MC_T_BOOL StateMachine();

    friend MC_T_WORD HandleBlending(MC_T_AXIS_REF *pAxis,
                        MC_BUFFERED_FB_T *pMcfb, MC_T_DWORD dwBufferMode, MC_T_REAL fVelocity, MC_T_REAL fEndVelocity, MC_T_BUFFERENTRY** ppPrevFb);

    friend void   MC_AbortCommand
        (MC_T_AXIS_REF*         pAxis
        ,MC_T_WORD              wNewFbID);
} _MC_PACKED;

/* MC_PositionProfile (3.14) */
#ifdef EC_MOTION_SUPPORT_MC_POSITION_PROFILE
class _MC_API MC_POSITION_PROFILE_T : public MC_BUFFERED_FB_T
{
public:

    // OUT's
    const MC_T_BOOL     &Done;                   /* OUT(B): The axis is within a range close to the target position */
    const MC_T_BOOL     &Busy;                   /* OUT(E): The FB is not finished and new output values are to be expected */

    // IN's
    MC_T_BOOL           Execute;                 /* IN(B): Start the motion at rising edge */
    MC_T_BOOL           ContinuousUpdate;        /* IN(E): Continuous Update: not supported  */
    MC_T_REAL           TimeScale;               /* IN(B): Overall time scaling factor of the profile */
    MC_T_REAL           PositionScale;           /* IN(E): Overall Position scaling factor */
    MC_T_REAL           Offset;                  /* IN(E): Overall offset for profile [u] */

    // IN_OUT's
    MC_T_TP_REF         *TimePosition;           /* INOUT(B): Reference to time/position */

    MC_POSITION_PROFILE_T(MC_T_AXIS_REF *pAxis = MC_NULL, MC_T_TP_REF *pTimePosition = MC_NULL);
    ~MC_POSITION_PROFILE_T();
    void _MC_THIS_API OnCycle();

protected:
    MC_T_TP_REF        *&pTimePosition;  /* INOUT: Reference to time/position */

private:

    MC_T_BOOL           bDone;           /* OUT: The axis is within a range close to the target position */
    MC_T_BOOL           bBusy;           /* OUT: The FB is not finished and new output values are to be expected */

    const MC_T_BOOL     &bExecute;       /* IN: Start the motion at rising edge */
    const MC_T_BOOL     &bContUpdate;    /* IN: Continuous Update */
    const MC_T_REAL     &fTimeScale;     /* IN: Overall time scaling factor of the profile */
    const MC_T_REAL     &fPositionScale; /* IN: Overall Position scaling factor */
    const MC_T_REAL     &fOffset;        /* IN: Overall offset for profile [u] */
    MC_T_DWORD          dwTableIdx;      /* index in Time/Position table */
public:
    MC_T_SPLINE*        pSpline;         /* The curve */
private:
    ASSIGNOP(MC_POSITION_PROFILE_T);
    void StateTrans_Execute();
    void StateTrans_Active();
    void DoState_Active();
    void DoState_Busy();
    MC_T_BOOL StateMachine();

    friend MC_T_WORD HandleBlending(MC_T_AXIS_REF *pAxis,
                        MC_BUFFERED_FB_T *pMcfb, MC_T_DWORD dwBufferMode, MC_T_REAL fVelocity, MC_T_REAL fEndVelocity, MC_T_BUFFERENTRY** ppPrevFb);

    friend void   MC_AbortCommand
        (MC_T_AXIS_REF*         pAxis
        ,MC_T_WORD              wNewFbID);
} _MC_PACKED;
#endif

/* MC_SetPosition (3.17) */
class _MC_API MC_SET_POSITION_T : public MC_FB_T
{
public:

    // OUT's
    const MC_T_BOOL     &Done;                   /* OUT(B): position has new value */
    const MC_T_BOOL     &Busy;                   /* OUT(E): The FB is not finished and new output values are to be expected */

    // IN's
    MC_T_BOOL           Execute;                 /* IN(B): Start the motion at rising edge */
    MC_T_REAL           Position;                /* IN(B): Position or distance if bRelative is EC_TRUE */
    MC_T_BOOL           Relative;                /* IN(E): if EC_TRUE fPosition is a distance */
    MC_T_EXECUTION_MODE ExecutionMode;           /* IN(E): Defines the chronological sequence */

    MC_SET_POSITION_T(MC_T_AXIS_REF *pAxis = MC_NULL)
        : // Init from constructor
          MC_FB_T(pAxis, MCFB_ID_SET_POSITION),
          // Init external OUT's
          Done(static_cast<const MC_T_BOOL &>(bDone)),
          Busy(static_cast<const MC_T_BOOL &>(bBusy)),
          // Init variables
          Execute(0),
          Position(0.0),
          Relative(0),
          ExecutionMode(MC_IMMEDIATELY),
          bLastExecute(0),
          // Init internal IN's
          bExecute(static_cast<const MC_T_BOOL &>(Execute)),
          fPosition(static_cast<const MC_T_REAL &>(Position)),
          bRelative(static_cast<const MC_T_BOOL &>(Relative)),
          dwExecMode(static_cast<const MC_T_EXECUTION_MODE &>(ExecutionMode))
    {}

    void _MC_THIS_API OnCycle();

private:

    MC_T_BOOL           bLastExecute;       /* Internal: Last state of 'Execute' */
    MC_T_BOOL           bDone;              /* OUT: position has new value */
    MC_T_BOOL           bBusy;              /* OUT: The FB is not finished and new output values are to be expected */

    const MC_T_BOOL     &bExecute;          /* IN: Start the motion at rising edge */
    const MC_T_REAL     &fPosition;         /* IN: Position or distance if bRelative is EC_TRUE */
    const MC_T_BOOL     &bRelative;         /* IN: if EC_TRUE fPosition is a distance */
    const MC_T_EXECUTION_MODE
                        &dwExecMode;        /* IN: Defines the chronological sequence */

    ASSIGNOP(MC_SET_POSITION_T);
} _MC_PACKED;

/* MC_ReadParameter (3.19) */
class _MC_API MC_READ_PARAMETER_T : public MC_FB_T
{
public:

    // OUT's
    const MC_T_BOOL     &Valid;                  /* OUT(B): value available */
    const MC_T_BOOL     &Busy;                   /* OUT(E): The FB is not finished and new output values are to be expected */
    const MC_T_REAL     &Value;                  /* OUT(B): Value  */

    // IN's
    MC_T_BOOL           Enable;                  /* IN(B): Get value of the parameter continuously while enabled */
    MC_T_INT            ParameterNumber;         /* IN(B): Parameter ID. \sa MC_T_PN */

    MC_READ_PARAMETER_T(MC_T_AXIS_REF *pAxis = MC_NULL)
        : // Init from constructor
          MC_FB_T(pAxis, MCFB_ID_READ_PARAMETER),
          // Init external OUT's
          Valid(static_cast<const MC_T_BOOL &>(bValid)),
          Busy(static_cast<const MC_T_BOOL &>(bBusy)),
          Value(static_cast<const MC_T_REAL &>(fValue)),
          // Init variables
          Enable(0),
          ParameterNumber(0),
          bValid(0),
          bBusy(0),
          fValue(0.0),
          // Init internal IN's
          bEnable(static_cast<const MC_T_BOOL &>(Enable)),
          nParameterNumber(static_cast<const MC_T_INT &>(ParameterNumber))
    {}

    void _MC_THIS_API OnCycle();

private:

    MC_T_BOOL           bValid;             /* OUT: value available */
    MC_T_BOOL           bBusy;              /* OUT: The FB is not finished and new output values are to be expected */
    MC_T_REAL           fValue;             /* OUT: Value  */

    const MC_T_BOOL     &bEnable;           /* IN: Get value of the parameter continuously while enabled */
    const MC_T_INT      &nParameterNumber;  /* IN: Number of parameter */

    ASSIGNOP(MC_READ_PARAMETER_T);
} _MC_PACKED;

/* MC_ReadBoolParameter (3.19) */
class _MC_API MC_READ_BOOL_PARAMETER_T : public MC_FB_T
{
public:

    // OUT's
    const MC_T_BOOL     &Valid;                  /* OUT(B): value available */
    const MC_T_BOOL     &Busy;                   /* OUT(E): The FB is not finished and new output values are to be expected */
    const MC_T_BOOL     &Value;                  /* OUT(B): Value  */

    // IN's
    MC_T_BOOL           Enable;                  /* IN(B): Get value of the parameter continuously while enabled */
    MC_T_INT            ParameterNumber;         /* IN(B): Number of parameter */

    MC_READ_BOOL_PARAMETER_T(MC_T_AXIS_REF *pAxis = MC_NULL)
        : // Init from constructor
          MC_FB_T(pAxis, MCFB_ID_READ_BOOL_PARAMETER),
          // Init external OUT's
          Valid(static_cast<const MC_T_BOOL &>(bValid)),
          Busy(static_cast<const MC_T_BOOL &>(bBusy)),
          Value(static_cast<const MC_T_BOOL &>(bValue)),
          // Init variables
          Enable(0),
          ParameterNumber(0),
          bValid(0),
          bBusy(0),
          bValue(0),
          // Init internal IN's
          bEnable(static_cast<const MC_T_BOOL &>(Enable)),
          nParameterNumber(static_cast<const MC_T_INT &>(ParameterNumber))
    {}

    void _MC_THIS_API OnCycle();

private:

    MC_T_BOOL           bValid;             /* OUT: value available */
    MC_T_BOOL           bBusy;              /* OUT: The FB is not finished and new output values are to be expected */
    MC_T_BOOL           bValue;             /* OUT: Value  */

    const MC_T_BOOL     &bEnable;            /* IN: Get value of the parameter continuously while enabled */
    const MC_T_INT      &nParameterNumber;   /* IN: Number of parameter */

    ASSIGNOP(MC_READ_BOOL_PARAMETER_T);
} _MC_PACKED;

/* MC_WriteParameter (3.20) */
class _MC_API MC_WRITE_PARAMETER_T : public MC_FB_T
{
public:

    // OUT's
    const MC_T_BOOL     &Busy;                   /* OUT(E): The FB is not finished and new output values are to be expected */

    // IN's
    MC_T_BOOL           Execute;                 /* IN(B): Write the value of the parameter at rising edge */
    MC_T_INT            ParameterNumber;         /* IN(B): Number of parameter */
    MC_T_REAL           Value;                   /* IN(B): Value  */

    MC_WRITE_PARAMETER_T(MC_T_AXIS_REF *pAxis = MC_NULL)
        : // Init from constructor
          MC_FB_T(pAxis, MCFB_ID_WRITE_PARAMETER),
          // Init external OUT's
          Busy(static_cast<const MC_T_BOOL &>(bBusy)),
          // Init variables
          Execute(0),
          ParameterNumber(0),
          Value(0.0),
          bBusy(0),
          bLastExecute(0),
          // Init internal IN's
          bExecute(static_cast<const MC_T_BOOL &>(Execute)),
          nParameterNumber(static_cast<const MC_T_INT &>(ParameterNumber)),
          fValue(static_cast<const MC_T_REAL &>(Value))
    {}

    void _MC_THIS_API OnCycle();

private:

    MC_T_BOOL           bBusy;               /* OUT: The FB is not finished */
    MC_T_BOOL           bLastExecute;        /* Internal: Last state of 'Execute' */

    const MC_T_BOOL     &bExecute;           /* IN: Write the value of the parameter at rising edge */
    const MC_T_INT      &nParameterNumber;   /* IN: Number of parameter */
    const MC_T_REAL     &fValue;             /* IN: Value  */

    ASSIGNOP(MC_WRITE_PARAMETER_T);
} _MC_PACKED;

/* MC_WriteBoolParameter (3.20) */
class _MC_API MC_WRITE_BOOL_PARAMETER_T : public MC_FB_T
{
public:

    // OUT's
    const MC_T_BOOL     &Busy;                   /* OUT(E): The FB is not finished and new output values are to be expected */

    // IN's
    MC_T_BOOL           Execute;                 /* IN(B): Write the value of the parameter at rising edge */
    MC_T_INT            ParameterNumber;         /* IN(B): Number of parameter */
    MC_T_BOOL           Value;                   /* IN(B): Value  */

    MC_WRITE_BOOL_PARAMETER_T(MC_T_AXIS_REF *pAxis = MC_NULL)
        : // Init from constructor
          MC_FB_T(pAxis, MCFB_ID_WRITE_BOOL_PARAMETER),
          // Init external OUT's
          Busy(static_cast<const MC_T_BOOL &>(bBusy)),
          // Init variables
          Execute(0),
          ParameterNumber(0),
          Value(0),
          bBusy(0),
          bLastExecute(0),
          // Init internal IN's
          bExecute(static_cast<const MC_T_BOOL &>(Execute)),
          nParameterNumber(static_cast<const MC_T_INT &>(ParameterNumber)),
          bValue(static_cast<const MC_T_BOOL &>(Value))
    {}

    _MC_THIS_API void OnCycle();

private:

    MC_T_BOOL           bBusy;               /* OUT: The FB is not finished */
    MC_T_BOOL           bLastExecute;        /* Internal: Last state of 'Execute' */

    const MC_T_BOOL     &bExecute;           /* IN: Write the value of the parameter at rising edge */
    const MC_T_INT      &nParameterNumber;   /* IN: Number of parameter */
    const MC_T_BOOL     &bValue;             /* IN: Value  */

    ASSIGNOP(MC_WRITE_BOOL_PARAMETER_T);
} _MC_PACKED;


/* MC_ReadDigitalInput (3.21) */
class _MC_API MC_READ_DIGITAL_INPUT_T : public MC_FB_T
{
public:

    // OUT's
    const MC_T_BOOL     &Valid;                  /* OUT(B): value available */
    const MC_T_BOOL     &Busy;                   /* OUT(E): The FB is not finished and new output values are to be expected */
    const MC_T_BOOL     &Value;                  /* OUT(B): Value  */

    // IN's
    MC_T_BOOL           Enable;                  /* IN(B): Get value of the parameter continuously while enabled */
    MC_T_INT            InputNumber;             /* IN(B): Selects the input number between 0 and 31 */

    MC_READ_DIGITAL_INPUT_T(MC_T_AXIS_REF *pAxis = MC_NULL)
        : // Init from constructor
          MC_FB_T(pAxis, MCFB_ID_READ_DIGITAL_INPUT),
          // Init external OUT's
          Valid(static_cast<const MC_T_BOOL &>(bValid)),
          Busy(static_cast<const MC_T_BOOL &>(bBusy)),
          Value(static_cast<const MC_T_BOOL &>(bValue)),
          // Init variables
          Enable(0),
          InputNumber(0),
          bValid(0),
          bBusy(0),
          bValue(0),
          // Init internal IN's
          bEnable(static_cast<const MC_T_BOOL &>(Enable)),
          nInputNumber(static_cast<const MC_T_INT &>(InputNumber))
    {}

    void _MC_THIS_API OnCycle();

private:

    MC_T_BOOL           bValid;             /* OUT: value available */
    MC_T_BOOL           bBusy;              /* OUT: The FB is not finished and new output values are to be expected */
    MC_T_BOOL           bValue;             /* OUT: Value  */

    const MC_T_BOOL     &bEnable;           /* IN: Get value of the parameter continuously while enabled */
    const MC_T_INT      &nInputNumber;      /* IN: Number of input */

    ASSIGNOP(MC_READ_DIGITAL_INPUT_T);
} _MC_PACKED;


/* MC_ReadDigitalOutput (3.22) */
class _MC_API MC_READ_DIGITAL_OUTPUT_T : public MC_FB_T
{
public:

    // OUT's
    const MC_T_BOOL     &Valid;                  /* OUT(B): value available */
    const MC_T_BOOL     &Busy;                   /* OUT(E): The FB is not finished and new output values are to be expected */
    const MC_T_BOOL     &Value;                  /* OUT(B): Value  */

    // IN's
    MC_T_BOOL           Enable;                  /* IN(B): Get value of the parameter continuously while enabled */
    MC_T_INT            OutputNumber;            /* IN(B): Selects the output number between 0 and 31 */

    MC_READ_DIGITAL_OUTPUT_T(MC_T_AXIS_REF *pAxis = MC_NULL)
        : // Init from constructor
          MC_FB_T(pAxis, MCFB_ID_READ_DIGITAL_OUTPUT),
          // Init external OUT's
          Valid(static_cast<const MC_T_BOOL &>(bValid)),
          Busy(static_cast<const MC_T_BOOL &>(bBusy)),
          Value(static_cast<const MC_T_BOOL &>(bValue)),
          // Init variables
          Enable(0),
          OutputNumber(0),
          bValid(0),
          bBusy(0),
          bValue(0),
          // Init internal IN's
          bEnable(static_cast<const MC_T_BOOL &>(Enable)),
          nOutputNumber(static_cast<const MC_T_INT &>(OutputNumber))
    {}

    void _MC_THIS_API OnCycle();

private:

    MC_T_BOOL           bValid;             /* OUT: value available */
    MC_T_BOOL           bBusy;              /* OUT: The FB is not finished and new output values are to be expected */
    MC_T_BOOL           bValue;             /* OUT: Value  */

    const MC_T_BOOL     &bEnable;           /* IN: Get value of the parameter continuously while enabled */
    const MC_T_INT      &nOutputNumber;     /* IN: Number of output */

    ASSIGNOP(MC_READ_DIGITAL_OUTPUT_T);
} _MC_PACKED;


/* MC_WriteDigitalOutput (3.23) */
class _MC_API MC_WRITE_DIGITAL_OUTPUT_T : public MC_FB_T
{
public:

    // OUT's
    const MC_T_BOOL     &Done;                   /* OUT(B): Writing of the output signal value is done */
    const MC_T_BOOL     &Busy;                   /* OUT(E): The FB is not finished and new output values are to be expected */

    // IN's
    MC_T_BOOL           Execute;                 /* IN(B): Write the value of the selected output */
    MC_T_INT            OutputNumber;            /* IN(B): Selects the output number between 0 and 31 */
    MC_T_BOOL           Value;                   /* IN(B): Value  */
    MC_T_EXECUTION_MODE ExecutionMode;           /* IN(E): Defines the chronological sequence */

    MC_WRITE_DIGITAL_OUTPUT_T(MC_T_AXIS_REF *pAxis = MC_NULL)
        : // Init from constructor
          MC_FB_T(pAxis, MCFB_ID_WRITE_DIGITAL_OUTPUT),
          // Init external OUT's
          Done(static_cast<const MC_T_BOOL &>(bDone)),
          Busy(static_cast<const MC_T_BOOL &>(bBusy)),
          // Init variables
          Execute(0),
          OutputNumber(0),
          Value(0),
          ExecutionMode(MC_IMMEDIATELY),
          bLastExecute(0),
          bDone(0),
          bBusy(0),
          // Init internal IN's
          bExecute(static_cast<const MC_T_BOOL &>(Execute)),
          nOutputNumber(static_cast<const MC_T_INT &>(OutputNumber)),
          bValue(static_cast<const MC_T_BOOL &>(Value)),
          dwExecMode(static_cast<const MC_T_EXECUTION_MODE &>(ExecutionMode))
    {}

    void _MC_THIS_API OnCycle();

private:
    MC_T_BOOL           bLastExecute;       /* Internal: Last state of 'Execute' */
    MC_T_BOOL           bDone;              /* OUT: Done  */
    MC_T_BOOL           bBusy;              /* OUT: The FB is not finished and new output values are to be expected */

    const MC_T_BOOL     &bExecute;          /* IN: Write the value of the selected output  */
    const MC_T_INT      &nOutputNumber;     /* IN: Number of output */
    const MC_T_BOOL     &bValue;            /* IN: Value */
    const MC_T_EXECUTION_MODE
                        &dwExecMode;        /* IN: Defines the chronological sequence */

    ASSIGNOP(MC_WRITE_DIGITAL_OUTPUT_T);
} _MC_PACKED;


/* MC_ReadActualPosition (3.24) */
class _MC_API MC_READ_ACTUAL_POSITION_T : public MC_FB_T
{
public:

    // OUT's
    const MC_T_BOOL     &Valid;                  /* OUT(B): value available */
    const MC_T_REAL     &Position;               /* OUT(B): Actual position of the axis. Unit */

    // IN's
    MC_T_BOOL           Enable;                  /* IN(B): Get value of the parameter continuously while enabled */

    MC_READ_ACTUAL_POSITION_T(MC_T_AXIS_REF *pAxis = MC_NULL)
        : // Init from constructor
          MC_FB_T(pAxis, MCFB_ID_READ_ACTUAL_POSITION),
          // Init external OUT's
          Valid(static_cast<const MC_T_BOOL &>(bValid)),
          Position(static_cast<const MC_T_REAL &>(fPosition)),
          // Init variables
          Enable(0),
          bValid(0),
          fPosition(0.0),
          // Init internal IN's
          bEnable(static_cast<const MC_T_BOOL &>(Enable))
    {}

    _MC_THIS_API void OnCycle();

private:

    MC_T_BOOL           bValid;             /* OUT: value available */
    MC_T_REAL           fPosition;          /* OUT: Actual position of the axis. Unit */

    const MC_T_BOOL     &bEnable;           /* IN: Get value of the parameter continuously while enabled */

    ASSIGNOP(MC_READ_ACTUAL_POSITION_T);
} _MC_PACKED;

/* MC_ReadActualVelocity (3.25) */
class _MC_API MC_READ_ACTUAL_VELOCITY_T : public MC_FB_T
{
public:

    // OUT's
    const MC_T_BOOL     &Valid;                  /* OUT(B): value available */
    const MC_T_REAL     &Velocity;               /* OUT(B): Actual velocity of the axis. Unit */

    // IN's
    MC_T_BOOL           Enable;                  /* IN(B): Get value of the parameter continuously while enabled */

    MC_READ_ACTUAL_VELOCITY_T(MC_T_AXIS_REF *pAxis = MC_NULL)
        : // Init from constructor
          MC_FB_T(pAxis, MCFB_ID_READ_ACTUAL_VELOCITY),
          // Init external OUT's
          Valid(static_cast<const MC_T_BOOL &>(bValid)),
          Velocity(static_cast<const MC_T_REAL &>(fVelocity)),
          // Init variables
          Enable(0),
          bValid(0),
          fVelocity(0.0),
          // Init internal IN's
          bEnable(static_cast<const MC_T_BOOL &>(Enable))
    {}

    _MC_THIS_API void OnCycle();

private:

    MC_T_BOOL           bValid;         /* OUT: value available */
    MC_T_REAL           fVelocity;      /* OUT: Actual velocity of the axis. Unit */

    const MC_T_BOOL     &bEnable;       /* IN: Get value of the parameter continuously while enabled */

    ASSIGNOP(MC_READ_ACTUAL_VELOCITY_T);
} _MC_PACKED;

/* MC_ReadMotionState (3.28) */
class _MC_API MC_READ_MOTION_STATE_T : public MC_FB_T
{
public:

    // OUT's
    const MC_T_BOOL     &Valid;                  /* OUT(B): An axis error is present and will be displayed */
    const MC_T_BOOL     &ConstantVelocity;       /* OUT(E): Velocity is constant. Velocity may be 0. */
    const MC_T_BOOL     &Accelerating;           /* OUT(E): Increasing the absolute value of the velocity */
    const MC_T_BOOL     &Decelerating;           /* OUT(E): Decreasing the absolute value of the velocity */
    const MC_T_BOOL     &DirectionPositive;      /* OUT(E): Signals that the position is increasing */
    const MC_T_BOOL     &DirectionNegative;      /* OUT(E): Signals that the position is decreasing */

    // IN's
    MC_T_BOOL           Enable;                  /* IN(B): Get value of the parameter continuously while enabled */

    MC_READ_MOTION_STATE_T(MC_T_AXIS_REF *pAxis = MC_NULL)
        : // Init from constructor
          MC_FB_T(pAxis, MCFB_ID_READ_MOTION_STATE),
          // Init external OUT's
          Valid(static_cast<const MC_T_BOOL &>(bValid)),
          ConstantVelocity(static_cast<const MC_T_BOOL &>(bConstantVelocity)),
          Accelerating(static_cast<const MC_T_BOOL &>(bAccelerating)),
          Decelerating(static_cast<const MC_T_BOOL &>(bDecelerating)),
          DirectionPositive(static_cast<const MC_T_BOOL &>(bDirPositive)),
          DirectionNegative(static_cast<const MC_T_BOOL &>(bDirNegative)),
          // Init variables
          Enable(0),
          bValid(0),
          bConstantVelocity(0),
          bAccelerating(0),
          bDecelerating(0),
          bDirPositive(0),
          bDirNegative(0),
          // Init internal IN's
          bEnable(static_cast<const MC_T_BOOL &>(Enable))
    {}

    _MC_THIS_API void OnCycle();

private:

    MC_T_BOOL           bValid;             /* OUT: An axis error is present and will be displayed */
    MC_T_BOOL           bConstantVelocity;  /* OUT: Velocity is constant. Velocity may be 0. */
    MC_T_BOOL           bAccelerating;      /* OUT: Increasing the absolute value of the velocity */
    MC_T_BOOL           bDecelerating;      /* OUT: Decreasing the absolute value of the velocity */
    MC_T_BOOL           bDirPositive;       /* OUT: Signals that the position is increasing */
    MC_T_BOOL           bDirNegative;       /* OUT: Signals that the position is decreasing */

    const MC_T_BOOL     &bEnable;           /* IN: Get value of the parameter continuously while enabled */

    ASSIGNOP(MC_READ_MOTION_STATE_T);
} _MC_PACKED;

/* MC_ReadAxisInfo (3.29) */
class _MC_API MC_READ_AXIS_INFO_T : public MC_FB_T
{
public:

    // OUT's
    const MC_T_BOOL           &Valid;            /* OUT(B): An axis error is present and will be displayed */
    const MC_T_BOOL           &Simulation;       /* OUT(E): Axis is in simulation mode */

    const MC_T_INT            &Verbose;          /* OUT(V): verbosity level for debug messages */
    const MC_T_PLCOPEN_STATE  &PLCOpenState;     /* OUT(V): PLCOpen state */
    const MC_T_INT            &DriveState;       /* OUT(V): DS402/SERCOS/SIMU drive state */
    const MC_T_BOOL           &DriveErrorAckReq; /* OUT(V): Set by MotionLib if a drive error should be acknowledged */
    const MC_T_REAL           &CommandedPosition;
                                                 /* OUT(V): Read commanded position */

    // IN's
    MC_T_BOOL                 Enable;            /* IN(B): Get value of the parameter continuously while enabled */
    MC_T_BOOL                 DriveErrorAck;     /* IN(V): Set by user if a drive error has been acknowledged. \sa DriveErrorAckReq */

    MC_READ_AXIS_INFO_T(MC_T_AXIS_REF *pAxis = MC_NULL)
        : // Init from constructor
          MC_FB_T(pAxis, MCFB_ID_READ_AXIS_INFO),
          // Init external OUT's
          Valid(static_cast<const MC_T_BOOL &>(bValid)),
          Simulation(static_cast<const MC_T_BOOL &>(bSimulation)),
          Verbose(static_cast<const MC_T_INT &>(nVerbose)),
          PLCOpenState(static_cast<const MC_T_PLCOPEN_STATE &>(ePLCOpenState)),
          DriveState(static_cast<const MC_T_INT &>(nDriveState)),
          DriveErrorAckReq(static_cast<const MC_T_BOOL &>(bDriveErrorAckReq)),
          CommandedPosition(static_cast<const MC_T_REAL &>(fCommandedPosition)),
          // Init variables
          Enable(0),
          DriveErrorAck(0),
          bValid(0),
          bSimulation(0),
          nVerbose(0),
          ePLCOpenState(MC_PLCOPEN_STATE_UNKNOWN),
          nDriveState(0),
          bDriveErrorAckReq(0),
          fCommandedPosition(0.0),
          // Init internal IN's
          bEnable(static_cast<const MC_T_BOOL &>(Enable)),
          bDriveErrorAck(static_cast<const MC_T_BOOL &>(DriveErrorAck))
    {}

    _MC_THIS_API void OnCycle();

private:

    MC_T_BOOL           bValid;             /* OUT: An axis error is present and will be displayed */
    MC_T_BOOL           bSimulation;        /* OUT: Axis is in simulation mode */
    MC_T_INT            nVerbose;
    MC_T_PLCOPEN_STATE  ePLCOpenState;      /* OUT: PLCOpen state */
    MC_T_INT            nDriveState;
    MC_T_BOOL           bDriveErrorAckReq;
    MC_T_REAL           fCommandedPosition;

    const MC_T_BOOL     &bEnable;            /* IN: Get value of the parameter continuously while enabled */
    const MC_T_BOOL     &bDriveErrorAck;

    ASSIGNOP(MC_READ_AXIS_INFO_T);
} _MC_PACKED;

/* MC_ReadAxisError (3.30) */
class _MC_API MC_READ_ERROR_T : public MC_FB_T
{
public:

    // OUT's
    const MC_T_BOOL     &Valid;                  /* OUT(B): An axis error is present and will be displayed */

    // IN's
    MC_T_BOOL           Enable;                  /* IN(B): Get value of the parameter continuously while enabled */

    MC_READ_ERROR_T(MC_T_AXIS_REF *pAxis = MC_NULL)
        : // Init from constructor
          MC_FB_T(pAxis, MCFB_ID_READ_AXIS_ERROR),
          // Init external OUT's
          Valid(static_cast<const MC_T_BOOL &>(bValid)),
          // Init variables
          Enable(0),
          bValid(0),
          // Init internal IN's
          bEnable(static_cast<const MC_T_BOOL &>(Enable))
    {}

    _MC_THIS_API void OnCycle();

private:

    MC_T_BOOL           bValid;              /* OUT: An axis error is present and will be displayed */
    MC_T_WORD           wAxisErrorID;        /* NOT SUPP: OUT: Vendor specific error */
    const MC_T_BOOL     &bEnable;            /* IN: Get value of the parameter continuously while enabled */

    ASSIGNOP(MC_READ_ERROR_T);
} _MC_PACKED;

/* MC_Reset (3.31) */
class _MC_API MC_RESET_T : public MC_FB_T
{
public:

    // OUT's
    const MC_T_BOOL     &Done;                   /* OUT(B): An axis error is present and will be displayed */
    const MC_T_BOOL     &Busy;                   /* OUT(E): The FB is not finished and new output values are to be expected */

    // IN's
    MC_T_BOOL           Execute;                 /* IN(B): Get the values of the parameters continuously while enabled */

    MC_RESET_T(MC_T_AXIS_REF *pAxis = MC_NULL)
        : // Init from constructor
          MC_FB_T(pAxis, MCFB_ID_RESET),
          // Init external OUT's
          Done(static_cast<const MC_T_BOOL &>(bDone)),
          Busy(static_cast<const MC_T_BOOL &>(bBusy)),
          // Init variables
          Execute(0),
          bDone(0),
          bBusy(0),
          bLastExecute(0),
          // Init internal IN's
          bExecute(static_cast<const MC_T_BOOL &>(Execute))
    {}

    _MC_THIS_API void OnCycle();

private:

    MC_T_BOOL           bDone;               /* OUT: An axis error is present and will be displayed */
    MC_T_BOOL           bBusy;               /* OUT: The FB is not finished and new output values are to be expected */
    MC_T_BOOL           bLastExecute;        /* Internal: Last state of 'Execute' */

    const MC_T_BOOL     &bExecute;           /* IN: Get the values of the parameters continuously while enabled */

    ASSIGNOP(MC_RESET_T);
} _MC_PACKED;


#ifdef EC_MOTION_SUPPORT_MC_CAMMING
/* MC_CamTableSelect 4.2 */
class _MC_API MC_CAMTABLE_SELECT_T : public MC_FB_T
{
public:

    // OUT's
    const MC_T_BOOL     &Done;                   /* OUT(B): Pre-selection done */
    const MC_T_BOOL     &Busy;                   /* OUT(E): The FB is not finished and new output values are to be expected */
    const MC_T_P_CAM_ID &CamTableIDPtr;          /* OUT(E): Identifier of CAM Table to be used in the MC_CAM_IN_T */

    // IN's
    MC_T_BOOL           Execute;                 /* IN(B): Start the motion at rising edge */
    MC_T_BOOL           Periodic;                /* IN(E): 1 = periodic, 0 = non periodic (single-shot)  */
    MC_T_BOOL           MasterAbsolute;          /* IN(E): 1 = absolute; 0 = relative coordinates  */
    MC_T_BOOL           SlaveAbsolute;           /* IN(E): 1 = absolute; 0 = relative coordinates  */
    MC_T_EXECUTION_MODE ExecutionMode;           /* IN(E):Defines the chronological sequence of the FB.
                                                  *       Only mcImmediately is supported  */

    // IN_OUT's
    MC_T_AXIS_REF*      MasterAxis;             /* INOUT(B): Reference to the master axis */
    MC_T_CAM_REF*       CamTable;               /* INOUT(B): Reference to CAM description */

    MC_CAMTABLE_SELECT_T(MC_T_AXIS_REF *pAxis = MC_NULL, MC_T_AXIS_REF *pMasterAxis = MC_NULL, MC_T_CAM_REF *pCamTable = MC_NULL);
    ~MC_CAMTABLE_SELECT_T();

    void _MC_THIS_API OnCycle();

protected:
    MC_T_AXIS_REF      *&pMasterAxis;  /* INOUT: Reference to the master axis */
    MC_T_CAM_REF        *&pCamTable;  /* INOUT: Reference to CAM table */

private:

    MC_T_BOOL           bDone;           /* OUT: The axis is within a range close to the target position */
    MC_T_BOOL           bBusy;           /* OUT: The FB is not finished and new output values are to be expected */

    const MC_T_BOOL     &bExecute;       /* IN: Start the motion at rising edge */

    MC_T_CAM_ID          oCamTableID;       /* Identifier of CAM Table to be used in the MC_CamIn FB */
    MC_T_P_CAM_ID        pCamTableID;
    MC_T_SPLINE*         pSpline;           /* The curve */

    ASSIGNOP(MC_CAMTABLE_SELECT_T);
    void StateTrans_Execute();
    void DoState_Busy();
    MC_T_BOOL StateMachine();

} _MC_PACKED;

/* MC_CamIn 4.3 */
/* Restrictions: SlaveAxis has to be in state MC_PLCOPEN_STATE_STAND_STILL */
class _MC_API MC_CAM_IN_T : public MC_BUFFERED_FB_T
{
public:
    // OUT's
    const MC_T_BOOL     &InSync;                 /* OUT(B): Is TRUE if the set value = the commanded value */
    const MC_T_BOOL     &Busy;                   /* OUT(E): The FB is not finished and new output values are to be expected */
    const MC_T_BOOL     &EndOfProfile;           /* OUT(E): Pulsed output signaling the cyclic end of the CAM Profile
                                                            It is displayed every time the end of the cam profile is reached */

    // IN's
    MC_T_BOOL           Execute;                 /* IN(B): Start the motion at rising edge */
    MC_T_REAL           MasterOffset;            /* IN(E): Offset of the master shaft to cam */
    MC_T_REAL           SlaveOffset;             /* IN(E): Offset of slave table */
    MC_T_REAL           MasterScaling;           /* IN(E): Factor for the master profile (default = 1.0). From the
                                                           slave point of view the master overall profile is multiplied by this factor */
    MC_T_REAL           SlaveScaling;            /* IN(E): Factor for the slave profile (default = 1.0). The overall
                                                           slave profile is multiplied by this factor. */
    MC_T_START_MODE     StartMode;               /* IN(E): Start Mode */
    MC_T_CAM_ID         *CamTableID;             /* IN(E): Identifier of CAM Table to be used, linked to output of MC_CamTableSelect */
    MC_T_REAL           Velocity;                /* IN(B): Only for StartMode=MC_SM_RAMP_IN: Value of the maximum velocity (always positive, not necessarily reached). */
    MC_T_REAL           Acceleration;            /* IN(E): Only for StartMode=MC_SM_RAMP_IN: Value of the acceleration (always positive). */
    MC_T_REAL           Deceleration;            /* IN(E): Only for StartMode=MC_SM_RAMP_IN: Value of the deceleration (always positive). */

    // IN_OUT's
    MC_T_AXIS_REF*      MasterAxis;              /* INOUT(B): Reference to the master axis */

    MC_CAM_IN_T(MC_T_AXIS_REF *pAxis = MC_NULL, MC_T_AXIS_REF *pMasterAxisParm = MC_NULL)
        : // Init from constructor
          MC_BUFFERED_FB_T(pAxis, MCFB_ID_CAM_IN),
          // Init external OUT's
          InSync(static_cast<const MC_T_BOOL &>(bInSync)),
          Busy(static_cast<const MC_T_BOOL &>(bBusy)),
          EndOfProfile(static_cast<const MC_T_BOOL &>(bEndOfProfile)),
          // Init variables
          Execute(0),
          MasterOffset(0.0),
          SlaveOffset(0.0),
          MasterScaling(0.0),
          SlaveScaling(0.0),
          StartMode(MC_SM_ABSOLUTE),
          CamTableID(MC_NULL),
          Velocity(0.0),
          Acceleration(0.0),
          Deceleration(0.0),
          MasterAxis(pMasterAxisParm),
          pMasterAxis(MasterAxis),
          bInSync(0),
          bBusy(0),
          bEndOfProfile(0),
          // Init internal IN's
          bExecute(static_cast<const MC_T_BOOL &>(Execute)),
          fMasterOffset(static_cast<const MC_T_REAL &>(MasterOffset)),
          fSlaveOffset(static_cast<const MC_T_REAL &>(SlaveOffset)),
          fMasterScaling(static_cast<const MC_T_REAL &>(MasterScaling)),
          fSlaveScaling(static_cast<const MC_T_REAL &>(SlaveScaling)),
          eStartMode(static_cast<const MC_T_START_MODE &>(StartMode)),
          fVelocity(static_cast<const MC_T_REAL &>(Velocity)),
          fAcceleration(static_cast<const MC_T_REAL &>(Acceleration)),
          fDeceleration(static_cast<const MC_T_REAL &>(Deceleration)),
          pCamTableID(static_cast<const MC_T_P_CAM_ID &>(CamTableID))
          // Init internal IN_OUT's
          {}

    void _MC_THIS_API OnCycle();

protected:
    MC_T_AXIS_REF      *&pMasterAxis;  /* INOUT: Reference to the master axis */

private:

    MC_T_BOOL           bInSync;         /* OUT: Is TRUE if the set value = the commanded value */
    MC_T_BOOL           bBusy;           /* OUT: The FB is not finished and new output values are to be expected */
    MC_T_BOOL           bEndOfProfile;   /* OUT: Pulsed output signaling the cyclic end of the CAM Profile */

    const MC_T_BOOL     &bExecute;       /* IN: Start the motion at rising edge */

    const MC_T_REAL     &fMasterOffset;  /* IN(E): Offset of the master shaft to cam */
    const MC_T_REAL     &fSlaveOffset;   /* IN(E): Offset of slave table */
    const MC_T_REAL     &fMasterScaling; /* IN(E): Factor for the master profile (default = 1.0). From the
                                                   slave point of view the master overall profile is multiplied by this factor */
    const MC_T_REAL     &fSlaveScaling;  /* IN(E): Factor for the slave profile (default = 1.0). The overall
                                                   slave profile is multiplied by this factor. */
    const MC_T_START_MODE &eStartMode;   /* IN(E): Start Mode */
    const MC_T_REAL     &fVelocity;              /* IN: Value of the maximum velocity (always positive, not necessarily reached). */
    const MC_T_REAL     &fAcceleration;          /* IN: Value of the acceleration (always positive, increasing energy of the motor). */
    const MC_T_REAL     &fDeceleration;          /* IN: Value of the deceleration (always positive, decreasing energy of the motor). */
    const   MC_T_P_CAM_ID  &pCamTableID;

    ASSIGNOP(MC_CAM_IN_T);
    void StateTrans_Execute();
    void StateTrans_Active();
    void DoState_Active();
    void DoState_Busy();
    MC_T_BOOL StateMachine();

    friend MC_T_WORD HandleBlending(MC_T_AXIS_REF *pAxis,
                        MC_BUFFERED_FB_T *pMcfb, MC_T_DWORD dwBufferMode, MC_T_REAL fVelocity, MC_T_REAL fEndVelocity, MC_T_BUFFERENTRY** ppPrevFb);

    friend void   MC_AbortCommand
        (MC_T_AXIS_REF*         pAxis
        ,MC_T_WORD              wNewFbID);

} _MC_PACKED;

/* MC_CamOut 4.3 */
class _MC_API MC_CAM_OUT_T : public MC_FB_T
{
public:

    // OUT's
    const MC_T_BOOL     &Done;                   /* OUT(B): The axis is within a range close to the target position */
    const MC_T_BOOL     &Busy;                   /* OUT(E): The FB is not finished and new output values are to be expected */

    // IN's
    MC_T_BOOL           Execute;                 /* IN(B): Start the motion at rising edge */
    MC_T_REAL           Deceleration;            /* IN(E): Value of the deceleration (always positive, decreasing energy of the motor). */
    MC_T_REAL           Jerk;                    /* IN(E): Value of the Jerk (always positive). */

    MC_CAM_OUT_T(MC_T_AXIS_REF *pSlaveAxis = MC_NULL)
        : // Init from constructor
          MC_FB_T(pSlaveAxis, MCFB_ID_CAM_OUT),
          // Init external OUT's
          Done(static_cast<const MC_T_BOOL &>(bDone)),
          Busy(static_cast<const MC_T_BOOL &>(bBusy)),
          // Init variables
          Execute(0),
          Deceleration(0.0),
          Jerk(0.0),
          bDone(0),
          bBusy(0),
          // Init internal IN's
          bExecute(static_cast<const MC_T_BOOL &>(Execute)),
          fDeceleration(static_cast<const MC_T_REAL &>(Deceleration)),
          fJerk(static_cast<const MC_T_REAL &>(Jerk))

          {}

    void _MC_THIS_API OnCycle();

protected:

private:

    MC_T_BOOL           bDone;           /* OUT: The axis is within a range close to the target position */
    MC_T_BOOL           bBusy;           /* OUT: The FB is not finished and new output values are to be expected */
    MC_T_BOOL           bActive;         /* Indicates that the FB has control on the axis */

    const MC_T_BOOL     &bExecute;       /* IN: Start the motion at rising edge */
    const MC_T_REAL     &fDeceleration;          /* IN: Value of the deceleration (always positive, decreasing energy of the motor). */
    const MC_T_REAL     &fJerk;                  /* IN: Value of the Jerk (always positive). */

    ASSIGNOP(MC_CAM_OUT_T);
    void StateTrans_Execute();
    void DoState_Busy();
    void StateTrans_Active();
    MC_T_BOOL StateMachine();
    void DoState_Active();

} _MC_PACKED;
#endif /* EC_MOTION_SUPPORT_MC_CAMMING */

#ifdef EC_MOTION_SUPPORT_MC_GEARING
/* MC_GearIn 4.5 */
/* Restrictions: SlaveAxis has to be in state MC_PLCOPEN_STATE_STAND_STILL */
/* Function block has to be executed for the slave axis and will put the state to MC_PLCOPEN_STATE_SYNC_MOTION*/
class _MC_API MC_GEAR_IN_T : public MC_BUFFERED_FB_T
{
public:
	// OUT's
	const MC_T_BOOL     &InGear;                 /* OUT(B): Is TRUE if the set value = the commanded value */
	const MC_T_BOOL     &Busy;                   /* OUT(E): The FB is not finished and new output values are to be expected */

												 // IN's
	MC_T_BOOL           Execute;                 /* IN(B): Start the motion at rising edge */
	MC_T_BOOL           ContinuousUpdate;        /* IN(E): NOT SUPPORTED:Continuous Update */
	MC_T_INT            RatioNumerator;          /* IN(B): Gear ratio Numerator */
	MC_T_UINT           RatioDenominator;        /* IN(B): Gear ratio Denominator */
	MC_T_SOURCE			MasterValueSource;		 /* IN(E): Source of synchronization */
	MC_T_REAL           Acceleration;            /* IN(E): NOT SUPPORTED: Acceleration for gearing in */
	MC_T_REAL           Deceleration;            /* IN(E): NOT SUPPORTED: Deceleration for gearing in */
	MC_T_REAL           Jerk;                    /* IN(E): NOT SUPPORTED: Jerk for gearing in */

	// IN_OUT's
	MC_T_AXIS_REF*      MasterAxis;              /* INOUT(B): Reference to the master axis */

	MC_GEAR_IN_T(MC_T_AXIS_REF *pAxis = MC_NULL, MC_T_AXIS_REF *pMasterAxisParm = MC_NULL)
		: // Init from constructor
		MC_BUFFERED_FB_T(pAxis, MCFB_ID_GEAR_IN),
		// Init external OUT's
		InGear(static_cast<const MC_T_BOOL &>(bInGear)),
		Busy(static_cast<const MC_T_BOOL &>(bBusy)),
		// Init variables
		Execute(0),
		ContinuousUpdate(0),
		RatioNumerator(0),
		RatioDenominator(0),
		MasterValueSource(MC_SOURCE_SET_VALUE),
		Acceleration(0.0),
		Deceleration(0.0),
		Jerk(0.0),
		MasterAxis(pMasterAxisParm),
		pMasterAxis(MasterAxis),
		bInGear(0),
		bBusy(0),
		// Init internal IN's
		bExecute(static_cast<const MC_T_BOOL &>(Execute)),
		nRatioNumerator(static_cast<const MC_T_INT &>(RatioNumerator)),
		nRatioDenominator(static_cast<const MC_T_UINT &>(RatioDenominator)),
		eMasterValueSource(static_cast<const MC_T_SOURCE &>(MasterValueSource)),
		fAcceleration(static_cast<const MC_T_REAL &>(Acceleration)),
		fDeceleration(static_cast<const MC_T_REAL &>(Deceleration)),
		fJerk(static_cast<const MC_T_REAL &>(Jerk))
		// Init internal IN_OUT's
	{}

	void _MC_THIS_API OnCycle();

protected:
	MC_T_AXIS_REF      *&pMasterAxis;  /* INOUT: Reference to the master axis */

private:

	MC_T_BOOL           bInGear;         /* OUT: Is TRUE if the set value = the commanded value */
	MC_T_BOOL           bBusy;           /* OUT: The FB is not finished and new output values are to be expected */

	const MC_T_BOOL     &bExecute;       /* IN: Start the motion at rising edge */

	const MC_T_INT		&nRatioNumerator;	/* IN: Gear ratio numerator */
	const MC_T_UINT		&nRatioDenominator; /* IN: Gear ratio denominator */
	const MC_T_SOURCE   &eMasterValueSource; /* IN: Source of synchronization */
	const MC_T_REAL     &fAcceleration;     /* IN: Acceleration for gearing in */
	const MC_T_REAL     &fDeceleration;     /* IN: Deceleration for gearing in */
	const MC_T_REAL     &fJerk;             /* IN: Jerk for gearing in */

	ASSIGNOP(MC_GEAR_IN_T);
	void StateTrans_Execute();
	void StateTrans_Active();
	void DoState_Active();
	void DoState_Busy();
	MC_T_BOOL StateMachine();

	friend MC_T_WORD HandleBlending(MC_T_AXIS_REF *pAxis,
		MC_BUFFERED_FB_T *pMcfb, MC_T_DWORD dwBufferMode, MC_T_REAL fVelocity, MC_T_REAL fEndVelocity, MC_T_BUFFERENTRY** ppPrevFb);

	friend void   MC_AbortCommand
	(MC_T_AXIS_REF*         pAxis
	, MC_T_WORD              wNewFbID);

} _MC_PACKED;

/* MC_GearOut 4.6 */
class _MC_API MC_GEAR_OUT_T : public MC_FB_T
{
public:

	// OUT's
	const MC_T_BOOL     &Done;                   /* OUT(B): The axis is within a range close to the target position */
	const MC_T_BOOL     &Busy;                   /* OUT(E): The FB is not finished and new output values are to be expected */

												 // IN's
	MC_T_BOOL           Execute;                 /* IN(B): Start the motion at rising edge */
	MC_T_REAL           Deceleration;            /* IN(E): Value of the deceleration (always positive, decreasing energy of the motor). */
	MC_T_REAL           Jerk;                    /* IN(E): Value of the Jerk (always positive). */

	MC_GEAR_OUT_T(MC_T_AXIS_REF *pAxis = MC_NULL)
		: // Init from constructor
		MC_FB_T(pAxis, MCFB_ID_GEAR_OUT),
		// Init external OUT's
		Done(static_cast<const MC_T_BOOL &>(bDone)),
		Busy(static_cast<const MC_T_BOOL &>(bBusy)),
		// Init variables
		Execute(0),
		Deceleration(0.0),
		Jerk(0.0),
		bDone(0),
		bBusy(0),
		// Init internal IN's
		bExecute(static_cast<const MC_T_BOOL &>(Execute)),
		fDeceleration(static_cast<const MC_T_REAL &>(Deceleration)),
		fJerk(static_cast<const MC_T_REAL &>(Jerk))
	{}

	void _MC_THIS_API OnCycle();

protected:

private:

	MC_T_BOOL           bDone;           /* OUT: The axis is within a range close to the target position */
	MC_T_BOOL           bBusy;           /* OUT: The FB is not finished and new output values are to be expected */
	MC_T_BOOL           bActive;         /* Indicates that the FB has control on the axis */

	const MC_T_BOOL     &bExecute;       /* IN: Start the motion at rising edge */
	const MC_T_REAL     &fDeceleration;          /* IN: Value of the deceleration (always positive, decreasing energy of the motor). */
	const MC_T_REAL     &fJerk;                  /* IN: Value of the Jerk (always positive). */

	ASSIGNOP(MC_GEAR_OUT_T);
	void StateTrans_Execute();
	void DoState_Busy();
	void StateTrans_Active();
	MC_T_BOOL StateMachine();
	void DoState_Active();

} _MC_PACKED;

#endif /* EC_MOTION_SUPPORT_MC_GEARING */


/* Vendor extension */
class _MC_API AMC_CHECK_TARGETPOS_REACHED_T : public MC_FB_T
{
public:

    // OUT's
    const MC_T_BOOL           &InPosition;       /* OUT(V): Actual position near Commanded position. Tolerance window: InPositionWindow */
    const MC_T_BOOL           &Busy;             /* OUT(V): The FB is not finished and new output values are to be expected */

    // IN's
    MC_T_BOOL                 Execute;           /* IN(V): Get the values of the parameters continuously while enabled */
    MC_T_DWORD                InPositionWindow;  /* IN(V): In-Position window in drive increments */

    AMC_CHECK_TARGETPOS_REACHED_T(MC_T_AXIS_REF *pAxis = MC_NULL)
        : // Init from constructor
          MC_FB_T(pAxis, MCFB_ID_CHECK_TARGETPOS_REACHED),
          // Init external OUT's
          InPosition(static_cast<const MC_T_BOOL &>(bInPosition)),
          Busy(static_cast<const MC_T_BOOL &>(bBusy)),
          // Init variables
          Execute(0),
          InPositionWindow(0),
          bInPosition(0),
          bBusy(0),
          // Init internal IN's
          bExecute(static_cast<const MC_T_BOOL &>(Execute)),
          dwPositionWindow(static_cast<const MC_T_DWORD &>(InPositionWindow))
    {}

    _MC_THIS_API void OnCycle();

private:

    MC_T_BOOL           bInPosition;        /* OUT(B): Commanded position reached */
    MC_T_BOOL           bBusy;              /* OUT: The FB is not finished and new output values are to be expected */

    const MC_T_BOOL     &bExecute;          /* IN: Get value of the parameter continuously while enabled */
    const MC_T_DWORD    &dwPositionWindow;  /* IN(V): In-Position window in drive increments */

    MC_T_BOOL StateMachine();
    ASSIGNOP(AMC_CHECK_TARGETPOS_REACHED_T);
} _MC_PACKED;

/* Vendor extension */
class _MC_API AMC_HALT_RECOVERY_T : public MC_FB_T
{
public:

    // OUT's
    const MC_T_BOOL           &Busy;             /* OUT(V): The FB is not finished and new output values are to be expected */
    const MC_T_BOOL           &Done;             /* OUT(V): Recovery done */

    // IN's
    MC_T_BOOL                 Execute;           /* IN(V): Start recovery */
    MC_T_RECOVERY_MODE        RecoveryMode;      /* IN(V): Recovery mode */

    AMC_HALT_RECOVERY_T(MC_T_AXIS_REF *pAxis = MC_NULL)
        : // Init from constructor
          MC_FB_T(pAxis, MCFB_ID_HALT_RECOVERY),
          // Init external OUT's
          Busy(static_cast<const MC_T_BOOL &>(bBusy)),
          Done(static_cast<const MC_T_BOOL &>(bDone)),
          // Init variables
          Execute(0),
          RecoveryMode(MC_RECOVERY_NO_ACTION),
          bBusy(0),
          bDone(0),
          // Init internal IN's
          bExecute(static_cast<const MC_T_BOOL &>(Execute)),
          recoveryMode(static_cast<const MC_T_RECOVERY_MODE &>(RecoveryMode))
    {}

    _MC_THIS_API void OnCycle();

private:

    MC_T_BOOL                 bBusy;              /* OUT: The FB is not finished and new output values are to be expected */
    MC_T_BOOL                 bDone;              /* OUT: Recovery done */

    const MC_T_BOOL           &bExecute;          /* IN: Start recovery */
    const MC_T_RECOVERY_MODE  &recoveryMode;      /* IN(V): Recovery mode */

    MC_T_BOOL StateMachine();
    ASSIGNOP(AMC_HALT_RECOVERY_T);
    void StateTrans_Execute();
} _MC_PACKED;


/*
 * Calculate move times and segment distances
 * Example code:
 *       MC_T_MOVEMENT   oMove;
 *       pAxis->dwCycleTime = 1000;
 *       pAxis->dwIncPerMM =  10000;
 *       MC_CalcMoveProfile(pAxis, 100, 250, 1000, 1000, 0, &oMove);
 */
_MC_API MC_T_WORD MC_CalcMoveProfile               /* Vendor extension */
    (MC_T_AXIS_REF*         pAxis
    ,MC_T_REAL              fDistance       /* IN: distance for the motion */
    ,MC_T_REAL              fVelocity       /* IN: Value of the maximum velocity (always positive, not necessarily reached). */
    ,MC_T_REAL              fAcceleration   /* IN: Value of the acceleration (always positive, increasing energy of the motor). */
    ,MC_T_REAL              fDeceleration   /* IN: Value of the deceleration (always positive, decreasing energy of the motor). */
    ,MC_T_REAL              fJerk           /* IN: Value of the Jerk (always positive). */
    ,MC_T_MOVEMENT*         pMove);


_MC_API MC_T_WORD MC_CalcMoveProfileBuffered      /* Vendor extension */
    (MC_T_AXIS_REF*         pAxis
    ,MC_T_REAL              fDistance       /* IN: distance for the motion */
    ,MC_T_REAL              fVelocity       /* IN: Value of the maximum velocity (always positive, not necessarily reached). */
    ,MC_T_REAL              fAcceleration   /* IN: Value of the acceleration (always positive, increasing energy of the motor). */
    ,MC_T_REAL              fDeceleration   /* IN: Value of the deceleration (always positive, decreasing energy of the motor). */
    ,MC_T_REAL              fJerk           /* IN: Value of the Jerk (always positive). */
    ,EC_T_DWORD             dwBufferMode    /* IN: Defines the chronological sequence */
    ,EC_T_DWORD             dwBufferModeNext /* IN: Defines the chronological sequence for next FB */
    ,MC_T_REAL              fVelocityPrev   /* IN: Velocity of FB before: Only if BufferMode == MC_BLENDING_ */
    ,MC_T_REAL              fVelocityNext   /* IN: Velocity of FB after:  Only if BufferMode == MC_BLENDING_ */
    ,MC_T_MOVEMENT*         pMove);


/*
 * Calculate time until a certain position is reached. MC_CalcMoveProfile has to be called prior calling this function
 * Example code:
 *       MC_T_INT64      ulTriggerTime;
 *
 *       MC_CalcMoveTimeAtPos(pAxis, &oMove, 15, &ulTriggerTime);
 *       MC_CalcMoveTimeAtPos(pAxis, &oMove, 25, &ulTriggerTime);
 *       MC_CalcMoveTimeAtPos(pAxis, &oMove, 55, &ulTriggerTime);
 */
_MC_API MC_T_WORD MC_CalcMoveTimeAtPos              /* Vendor extension */
    (MC_T_AXIS_REF*         pAxis           /* IN: Axis parameters */
    ,MC_T_MOVEMENT*         pMove           /* IN: Move parameters */
    ,MC_T_REAL              fTriggerDist    /* IN: Trigger distance from start */
    ,MC_T_INT64*            pTriggerTime);  /* OUT: Trigger time */


/* set velocity without using trajectory generator from EC-Motion                        */
/* Don't use this function in parallel to MoveAbsolute, MoveRelative, MoveVelocity, etc. */
/* lTargetStep: pos. or negative value is added to  current target position */
_MC_API EC_T_WORD   MC_DriveSetTargetStep
    (MC_T_AXIS_REF*         pAxis               /* IN: Axis parameters */
    ,MC_T_POSITION          lTargetStep         /* IN: Velocity +/- */
    ,MC_T_VELACC            lTargetVel = 0      /* IN: Optional: Target velocity for feed forward */
    ,MC_T_VELACC            lTargetAcc = 0);    /* IN: Optional: Target torque for feed forward */

_MC_API const MC_T_CHAR* MC_GetErrorText( /* Vendor extension */
    MC_T_WORD wErrorID );

/* Logging stuff */
_MC_API MC_T_DWORD MC_GetLogMsgHeader               /* Vendor extension */
    (MC_T_CHAR*            pszLog
    ,MC_T_INT              nMaxSize);

/*-PACK SETTINGS-------------------------------------------------------------*/

#include EC_PACKED_INCLUDESTOP

#endif /*__ECMOTIONCONTROL_H__*/

/*-END OF SOURCE FILE--------------------------------------------------------*/
