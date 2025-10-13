/*-----------------------------------------------------------------------------
 * EcDemoApp.h
 * Copyright                acontis technologies GmbH, Ravensburg, Germany
 * Response                 Holger Oelhaf
 * Description              Application specific settings for EC-Master demo
 *---------------------------------------------------------------------------*/

#ifndef INC_ECDEMOAPP_H
#define INC_ECDEMOAPP_H

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

/*-DEFINES-------------------------------------------------------------------*/
#define EC_DEMO_APP_NAME (EC_T_CHAR*)"EcMasterDemo"

/* the RAS server is necessary to support the EC-Engineer or other remote applications */
#if (!defined INCLUDE_RAS_SERVER) && (defined EC_SOCKET_SUPPORTED)
#define INCLUDE_RAS_SERVER
#endif

#if (defined INCLUDE_RAS_SERVER)
#include "AtEmRasSrv.h"
#define ATEMRAS_MAX_WATCHDOG_TIMEOUT    10000
#define ATEMRAS_CYCLE_TIME              2
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

// #pragma pack(1)

// #pragma pack()

/*-FUNCTION DECLARATIONS-----------------------------------------------------*/
EC_T_VOID  ShowSyntaxAppUsage(T_EC_DEMO_APP_CONTEXT* pAppContext);
EC_T_VOID  ShowSyntaxApp(T_EC_DEMO_APP_CONTEXT* pAppContext);
EC_T_VOID  ShowSyntaxLinkLayer(EC_T_VOID);
EC_T_DWORD EcDemoApp(T_EC_DEMO_APP_CONTEXT* pAppContext);

#define PRINT_PERF_MEAS() ((EC_NULL != pEcLogContext)?((CAtEmLogging*)pEcLogContext)->PrintPerfMeas(pAppContext->dwInstanceId, 0, pEcLogContext) : 0)
#define PRINT_HISTOGRAM() ((EC_NULL != pEcLogContext)?((CAtEmLogging*)pEcLogContext)->PrintHistogramAsCsv(pAppContext->dwInstanceId) : 0)


/* User*/
void setEcMasterExit();
bool isEcMasterExit();
void setMotorEnable(bool en);
bool isMotorEnable(void);
void setPdoRwEn(bool en);
bool isPdoRwEn(void);

#endif /* INC_ECDEMOAPP_H */

/*-END OF SOURCE FILE--------------------------------------------------------*/
