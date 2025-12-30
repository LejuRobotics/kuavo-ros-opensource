/*-----------------------------------------------------------------------------
 * EcDemoApp.h
 * Copyright                acontis technologies GmbH, Ravensburg, Germany
 * Response                 Holger Oelhaf
 * Description              Application specific settings for EC-Master demo
 *---------------------------------------------------------------------------*/

#ifndef INC_ECDEMOAPP_H
#define INC_ECDEMOAPP_H 1

/*-LOGGING-------------------------------------------------------------------*/
#ifndef pEcLogParms
#define pEcLogParms (&(pAppContext->LogParms))
#endif

#define INCLUDE_EC_MASTER

/*-INCLUDES------------------------------------------------------------------*/
#include "AtEthercat.h"
#include "EcDemoPlatform.h"
#include "EcDemoParms.h"
#include "EcLogging.h"
#include "EcNotification.h"
#include "EcSdoServices.h"
#include "EcSelectLinkLayer.h"
#include "EcSlaveInfo.h"
#include <iostream>
#include <math.h>
#include <mutex>
#include <functional>
#include <vector>
#include <atomic>
/*-DEFINES-------------------------------------------------------------------*/
#define EC_DEMO_APP_NAME (EC_T_CHAR *)"EcMasterDemoDc"

/* the RAS server is necessary to support the EC-Engineer or other remote applications */
#if (!defined INCLUDE_RAS_SERVER) && (defined EC_SOCKET_SUPPORTED)
#define INCLUDE_RAS_SERVER
#endif

#if (defined INCLUDE_RAS_SERVER)
#include "AtEmRasSrv.h"
#define ATEMRAS_MAX_WATCHDOG_TIMEOUT 10000
#define ATEMRAS_CYCLE_TIME 2
#endif

#define MOTOR_STATUS_NO_ERROR 0
#define MOTOR_STATUS_ERROR 1
#define MOTOR_STATUS_REINIT 2

/***********************************************/
/* static EtherCAT master configuration values */
/***********************************************/
#define ETHERCAT_DC_TIMEOUT 12000          /* DC initialization timeout in ms */
#define ETHERCAT_DC_ARMW_BURSTCYCLES 10000 /* DC burst cycles (static drift compensation) */
#define ETHERCAT_DC_ARMW_BURSTSPP 12       /* DC burst bulk (static drift compensation) */
#define ETHERCAT_DC_DEV_LIMIT 13           /* DC deviation limit (highest bit tolerate by the broadcast read) */
#define ETHERCAT_DC_SETTLE_TIME 1500       /* DC settle time in ms */

/*--------------------------------------------------------------------------*/
/* Performance measurements of jobs                                         */
/* This is only available on CPUs with TSC support                          */
/*--------------------------------------------------------------------------*/
#define PERF_MEASURE_JOBS_INIT(numJobs)                             \
  ecatPerfMeasInit(&pAppContext->TscMeasDesc, 0, numJobs, EC_NULL); \
  ecatPerfMeasEnable(&pAppContext->TscMeasDesc)
#define PERF_MEASURE_JOBS_DEINIT() ecatPerfMeasDeinit(&pAppContext->TscMeasDesc)
#define PERF_MEASURE_JOBS_RESET() ecatPerfMeasReset(&pAppContext->TscMeasDesc, 0xFFFFFFFF);
#define PERF_MEASURE_JOBS_SHOW() ecatPerfMeasShow(&pAppContext->TscMeasDesc, 0xFFFFFFFF, S_aszMeasInfo)
#define PERF_JOB_START(nJobIndex) ecatPerfMeasStart(&pAppContext->TscMeasDesc, (EC_T_DWORD)(nJobIndex))
#define PERF_JOB_END(nJobIndex) ecatPerfMeasEnd(&pAppContext->TscMeasDesc, (EC_T_DWORD)(nJobIndex))

#pragma pack(1)
typedef struct
{
  int32_t position_actual_value;
  int16_t torque_actual_value;
  uint16_t status_word;
  int16_t mode_of_opration_display;
  int32_t position_demand_raw;
  int32_t velocity_demand_raw;
  int32_t velocity_actual_value;
  int16_t torque_demand_raw;
  uint16_t error_code;
} ELMO_SlaveRead_t;

typedef struct
{
  int32_t target_position;
  int32_t target_velocity;
  int16_t target_torque;
  uint16_t max_torque;
  uint16_t control_word;
  int16_t mode_of_opration;
  int32_t position_offset;
  int32_t velocit_offset;
  int16_t torque_offset;
} ELMO_SlaveWrite_t;


/* yd */
typedef struct
{
  uint16_t status_word;
  int32_t position_actual_value;
  int32_t velocity_actual_value;
  int16_t torque_actual_value;
  int8_t mode_of_opration_display;
  uint16_t error_code;
  int16_t torque_demand_raw;
  int32_t velocity_demand_raw;
} YD_SlaveRead_t;

typedef struct
{
  uint16_t control_word;
  int32_t target_position;
  int32_t target_velocity;
  int16_t target_torque;
  int32_t velocity_offset;
  int16_t torque_offset;
  int8_t mode_of_opration;//选模式，例：CSP、CSV等。。。
} YD_SlaveWrite_t;
#pragma pack()

// 定义input区结构
struct SlaveBuffersIn {
    ELMO_SlaveRead_t* elmo_slave_input;
    YD_SlaveRead_t* yd_slave_input;
};

// 定义out结构
struct SlaveBuffersOut {
    ELMO_SlaveWrite_t* elmo_slave_output;
    YD_SlaveWrite_t* yd_slave_output;
};

enum EcMasterType
{
  ELMO = 0,
  YD = 1,
  LEJU = 2
};

typedef struct
{
  double position = 0.0;
  double velocity = 0.0;
  double torque = 0.0;
  double maxTorque = 0.0;
  double positionOffset = 0.0;
  double velocityOffset = 0.0;
  double torqueOffset = 0.0;
  double acceleration = 0.0;
  double kp = 0.0;
  double kd = 0.0;
  uint8_t status = 0;
  uint16_t status_word = 0;
  uint16_t error_code = 0;
  double torque_demand_trans = 0.0;
} MotorParam_t;

extern enum EcMasterType driver_type[30];

/*-FUNCTION DECLARATIONS-----------------------------------------------------*/
EC_T_VOID ShowSyntaxAppUsage(T_EC_DEMO_APP_CONTEXT *pAppContext);
EC_T_VOID ShowSyntaxApp(T_EC_DEMO_APP_CONTEXT *pAppContext);
EC_T_VOID ShowSyntaxLinkLayer(EC_T_VOID);
EC_T_DWORD EcDemoApp(T_EC_DEMO_APP_CONTEXT *pAppContext);

using logger_callback_stdvector = std::function<void(const std::string &name, const std::vector<double> &vec)>;
void set_EClogger_callback(logger_callback_stdvector callback);

extern bool isEcMasterExit();
void setEcMasterExit();

bool motorIsEnable(const uint16_t id);

uint8_t motorStatus(const uint16_t id);

extern void motorGetData(const uint16_t *ids, const EcMasterType* driver, uint32_t num, MotorParam_t *data);

extern void motorSetPosition(const uint16_t *ids, const EcMasterType* driver, uint32_t num, MotorParam_t *params);
extern void motorSetVelocity(const uint16_t *ids, const EcMasterType* driver, uint32_t num, MotorParam_t *params);
extern void motorSetTorque(const uint16_t *ids, const EcMasterType* driver, uint32_t num, MotorParam_t *params);
extern void motorSetTorqueWithFeedback(const uint16_t *ids, const EcMasterType* driver, uint32_t num, MotorParam_t *params);

extern void motorToPosition(T_EC_DEMO_APP_CONTEXT *pAppContext, const uint16_t *ids, uint32_t num_id, const double *q_d, double speed, double dt);
extern void motorToPosition(const uint16_t *ids, uint32_t num_id, const double *q_d, double speed, double dt);


extern void motorSetkp(const std::vector<int32_t>& joint_kp);
extern void motorSetkd(const std::vector<int32_t>& joint_kd);

/**
 * @brief 读取电机Kp/Kd
 * 
 * @param ids 
 * @param driver_type 
 * @param joint_kp 
 * @return int 0 表示成功，1 表示驱动类型不支持该操作，2 表示读取失败
 */
extern int motorReadKp(const std::vector<uint16_t> &ids, EcMasterType driver_type, std::vector<int32_t>& joint_kp);
extern int motorReadKd(const std::vector<uint16_t> &ids, EcMasterType driver_type, std::vector<int32_t>& joint_kd);

/**
 * @brief 写入电机Kp/Kd
 * 
 * @param ids 
 * @param driver_type 
 * @param joint_kp 
 * @return int 0 表示成功，1 表示驱动类型不支持该操作，2 表示写入失败
 */ 
extern int motorWriteKp(const std::vector<uint16_t> &ids, EcMasterType driver_type, const std::vector<int32_t>& joint_kp);
extern int motorWriteKd(const std::vector<uint16_t> &ids, EcMasterType driver_type, const std::vector<int32_t>& joint_kd);

extern bool isMotorEnable(void);
extern uint32_t getNumSlave(void);
void setEcEncoderRange(uint32_t *encoder_range_set, uint16_t num);

// void motorGetCurrent(double *current_actual);
bool loadOffset();
bool saveOffset();

void getMotorPositionOffset(std::vector<double>& output);
void setMotorPositionOffset(const std::vector<double>& offsets);

/* function in EcNotification */
void fixEmergencyRequest(const int slaveAddr, const int errorCode);

// 忽略/屏蔽电机
extern void disableMotor(const uint16_t *ids, uint32_t num_id);

#define PRINT_PERF_MEAS() ((EC_NULL != pEcLogContext) ? ((CAtEmLogging *)pEcLogContext)->PrintPerfMeas(pAppContext->dwInstanceId, 0, pEcLogContext) : 0)
#define PRINT_HISTOGRAM() ((EC_NULL != pEcLogContext) ? ((CAtEmLogging *)pEcLogContext)->PrintHistogramAsCsv(pAppContext->dwInstanceId) : 0)

#endif /* INC_ECDEMOAPP_H */

/*-END OF SOURCE FILE--------------------------------------------------------*/
