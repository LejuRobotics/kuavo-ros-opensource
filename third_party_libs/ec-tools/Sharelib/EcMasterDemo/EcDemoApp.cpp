/*-----------------------------------------------------------------------------
 * EcDemoApp.cpp
 * Copyright                acontis technologies GmbH, Ravensburg, Germany
 * Response                 Holger Oelhaf
 * Description              EC-Master demo application
 *---------------------------------------------------------------------------*/

/*-INCLUDES------------------------------------------------------------------*/
#include "EcDemoApp.h"
#include "EcMotor.h"
#include "EcSdo.h"
#include "UserAppWorkpd.h"
/*-DEFINES-------------------------------------------------------------------*/
#define PERF_myAppWorkpd       0
#define MAX_JOB_NUM            1

static bool pdo_rw_en = true;
static bool motor_enabled = false;
static bool ecMasterExit = true;

/*-LOCAL VARIABLES-----------------------------------------------------------*/
static EC_T_PERF_MEAS_INFO_PARMS S_aPerfMeasInfos[MAX_JOB_NUM] =
{
    {"myAppWorkPd                    ", 0}
};

/*-FUNCTION DECLARATIONS-----------------------------------------------------*/
static EC_T_VOID  EcMasterJobTask(EC_T_VOID* pvAppContext);
static EC_T_DWORD EcMasterNotifyCallback(EC_T_DWORD dwCode, EC_T_NOTIFYPARMS* pParms);
#if (defined INCLUDE_RAS_SERVER)
static EC_T_DWORD RasNotifyCallback(EC_T_DWORD dwCode, EC_T_NOTIFYPARMS* pParms);
#endif

/*-MYAPP---------------------------------------------------------------------*/
typedef struct _T_MY_APP_DESC
{
    EC_T_DWORD dwFlashPdBitSize; /* Size of process data memory */
    EC_T_DWORD dwFlashPdBitOffs; /* Process data offset of data */
    EC_T_DWORD dwFlashTimer;
    EC_T_DWORD dwFlashInterval;
    EC_T_BYTE  byFlashVal;          /* flash pattern */
    EC_T_BYTE* pbyFlashBuf;         /* flash buffer */
    EC_T_DWORD dwFlashBufSize;      /* flash buffer size */
} T_MY_APP_DESC;
static EC_T_DWORD myAppInit(T_EC_DEMO_APP_CONTEXT* pAppContext);
static EC_T_DWORD myAppPrepare(T_EC_DEMO_APP_CONTEXT* pAppContext);
static EC_T_DWORD myAppSetup(T_EC_DEMO_APP_CONTEXT* pAppContext);
static EC_T_DWORD myAppWorkpd(T_EC_DEMO_APP_CONTEXT* pAppContext);
static EC_T_DWORD myAppDiagnosis(T_EC_DEMO_APP_CONTEXT* pAppContext);
static EC_T_DWORD myAppNotify(EC_T_DWORD dwCode, EC_T_NOTIFYPARMS* pParms);

/*-FUNCTION DEFINITIONS------------------------------------------------------*/

/********************************************************************************/

void setEcMasterExit()
{
  ecMasterExit = true;
}

bool isEcMasterExit()
{
  return ecMasterExit;
}

/** \brief EC-Master demo application.
*
* This is an EC-Master demo application.
*
* \return  Status value.
*/
EC_T_DWORD EcDemoApp(T_EC_DEMO_APP_CONTEXT *pAppContext)
{

  EC_T_DWORD dwRetVal = EC_E_NOERROR;
  EC_T_DWORD dwRes = EC_E_NOERROR;

  T_EC_DEMO_APP_PARMS *pAppParms = &pAppContext->AppParms;

  EC_T_VOID *pvJobTaskHandle = EC_NULL;

  EC_T_REGISTERRESULTS RegisterClientResults;
  OsMemset(&RegisterClientResults, 0, sizeof(EC_T_REGISTERRESULTS));

  CEcTimer oAppDuration;
  EC_T_BOOL bFirstDcmStatus = EC_TRUE;
  CEcTimer oDcmStatusTimer;

#if (defined INCLUDE_RAS_SERVER)
  EC_T_VOID *pvRasServerHandle = EC_NULL;
#endif

#if (defined INCLUDE_PCAP_RECORDER)
  CPcapRecorder *pPcapRecorder = EC_NULL;
#endif

  /* check link layer parameter */
  if (EC_NULL == pAppParms->apLinkParms[0])
  {
    dwRetVal = EC_E_INVALIDPARM;
    EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "ERROR: Missing link layer parameter\n"));
    goto Exit;
  }

  /* check if polling mode is selected */
  if (pAppParms->apLinkParms[0]->eLinkMode != EcLinkMode_POLLING)
  {
    dwRetVal = EC_E_INVALIDPARM;
    EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "ERROR: Link layer in 'interrupt' mode is not supported by %s. Please select 'polling' mode.\n", EC_DEMO_APP_NAME));
    goto Exit;
  }

  pAppContext->pNotificationHandler = EC_NEW(CEmNotification(pAppContext));
  if (EC_NULL == pAppContext->pNotificationHandler)
  {
    dwRetVal = EC_E_NOMEMORY;
    EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "ERROR: Cannot create notification handler\n"));
    goto Exit;
  }

  pAppContext->pMyAppDesc = (T_MY_APP_DESC *)OsMalloc(sizeof(T_MY_APP_DESC));
  if (EC_NULL == pAppContext->pMyAppDesc)
  {
    dwRetVal = EC_E_NOMEMORY;
    EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "ERROR: Cannot create myApp descriptor\n"));
    goto Exit;
  }
  OsMemset(pAppContext->pMyAppDesc, 0, sizeof(T_MY_APP_DESC));
  
  dwRes = myAppInit(pAppContext);
  if (EC_E_NOERROR != dwRes)
  {
    dwRetVal = dwRes;
    EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "ERROR: myAppInit failed: %s (0x%lx))\n", ecatGetText(dwRes), dwRes));
    goto Exit;
  }

#ifdef INCLUDE_RAS_SERVER
  /* start RAS server if enabled */
  if (pAppParms->bStartRasServer)
  {
    ATEMRAS_T_SRVPARMS oRemoteApiConfig;

    OsMemset(&oRemoteApiConfig, 0, sizeof(ATEMRAS_T_SRVPARMS));
    oRemoteApiConfig.dwSignature = ATEMRASSRV_SIGNATURE;
    oRemoteApiConfig.dwSize = sizeof(ATEMRAS_T_SRVPARMS);
    oRemoteApiConfig.oAddr.dwAddr = 0; /* INADDR_ANY */
    oRemoteApiConfig.wPort = pAppParms->wRasServerPort;
    oRemoteApiConfig.dwCycleTime = ATEMRAS_CYCLE_TIME;
    oRemoteApiConfig.dwCommunicationTimeout = ATEMRAS_MAX_WATCHDOG_TIMEOUT;
    oRemoteApiConfig.oAcceptorThreadCpuAffinityMask = pAppParms->CpuSet;
    oRemoteApiConfig.dwAcceptorThreadPrio = MAIN_THREAD_PRIO;
    oRemoteApiConfig.dwAcceptorThreadStackSize = JOBS_THREAD_STACKSIZE;
    oRemoteApiConfig.oClientWorkerThreadCpuAffinityMask = pAppParms->CpuSet;
    oRemoteApiConfig.dwClientWorkerThreadPrio = MAIN_THREAD_PRIO;
    oRemoteApiConfig.dwClientWorkerThreadStackSize = JOBS_THREAD_STACKSIZE;
    oRemoteApiConfig.pfnRasNotify = RasNotifyCallback;                    /* RAS notification callback function */
    oRemoteApiConfig.pvRasNotifyCtxt = pAppContext->pNotificationHandler; /* RAS notification callback function context */
    oRemoteApiConfig.dwMaxQueuedNotificationCnt = 100;                    /* pre-allocation */
    oRemoteApiConfig.dwMaxParallelMbxTferCnt = 50;                        /* pre-allocation */
    oRemoteApiConfig.dwCycErrInterval = 500;                              /* span between to consecutive cyclic notifications of same type */

    if (1 <= pAppParms->nVerbose)
    {
      OsMemcpy(&oRemoteApiConfig.LogParms, &pAppContext->LogParms, sizeof(EC_T_LOG_PARMS));
      oRemoteApiConfig.LogParms.dwLogLevel = EC_LOG_LEVEL_ERROR;
    }
    EcLogMsg(EC_LOG_LEVEL_INFO, (pEcLogContext, EC_LOG_LEVEL_INFO, "Start Remote API Server now\n"));
    dwRes = emRasSrvStart(&oRemoteApiConfig, &pvRasServerHandle);
    if (EC_E_NOERROR != dwRes)
    {
      EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "ERROR: Cannot spawn Remote API Server\n"));
    }
  }
#endif

  /* initialize EtherCAT master */
  {
    EC_T_INIT_MASTER_PARMS oInitParms;

    OsMemset(&oInitParms, 0, sizeof(EC_T_INIT_MASTER_PARMS));
    oInitParms.dwSignature = ATECAT_SIGNATURE;
    oInitParms.dwSize = sizeof(EC_T_INIT_MASTER_PARMS);
    oInitParms.pOsParms = &pAppParms->Os;
    oInitParms.pLinkParms = pAppParms->apLinkParms[0];
    oInitParms.pLinkParmsRed = pAppParms->apLinkParms[1];
    oInitParms.dwBusCycleTimeUsec = pAppParms->dwBusCycleTimeUsec;
    oInitParms.dwMaxBusSlaves = MASTER_CFG_ECAT_MAX_BUS_SLAVES;
    oInitParms.dwMaxAcycFramesQueued = MASTER_CFG_MAX_ACYC_FRAMES_QUEUED;
    if (oInitParms.dwBusCycleTimeUsec >= 1000)
    {
      oInitParms.dwMaxAcycBytesPerCycle = MASTER_CFG_MAX_ACYC_BYTES_PER_CYC;
    }
    else
    {
      oInitParms.dwMaxAcycBytesPerCycle = 1500;
      oInitParms.dwMaxAcycFramesPerCycle = 1;
      oInitParms.dwMaxAcycCmdsPerCycle = 20;
    }
    oInitParms.dwEcatCmdMaxRetries = MASTER_CFG_MAX_ACYC_CMD_RETRIES;

    OsMemcpy(&oInitParms.LogParms, &pAppContext->LogParms, sizeof(EC_T_LOG_PARMS));
    oInitParms.LogParms.dwLogLevel = pAppParms->dwMasterLogLevel;

    if (pAppParms->dwPerfMeasLevel > 0)
    {
      oInitParms.PerfMeasInternalParms.bEnabled = EC_TRUE;

      if (pAppParms->dwPerfMeasLevel > 1)
      {
        oInitParms.PerfMeasInternalParms.HistogramParms.dwBinCount = 200;
      }
    }
    else
    {
      oInitParms.PerfMeasInternalParms.bEnabled = EC_FALSE;
    }

    dwRes = ecatInitMaster(&oInitParms);
    if (dwRes != EC_E_NOERROR)
    {
      dwRetVal = dwRes;
      EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "Cannot initialize EtherCAT-Master: %s (0x%lx))\n", ecatGetText(dwRes), dwRes));
      goto Exit;
    }
    if (0 != OsStrlen(pAppParms->szLicenseKey))
    {
      EcLogMsg(EC_LOG_LEVEL_INFO, (pEcLogContext, EC_LOG_LEVEL_INFO, "Try to verify EtherCAT license!\n"));
      dwRes = ecatSetLicenseKey(pAppParms->szLicenseKey);
      if (dwRes != EC_E_NOERROR)
      {
        dwRetVal = dwRes;
        EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "ERROR: Cannot set license key: %s (0x%lx))\n", ecatGetText(dwRes), dwRes));
        goto Exit;
      }
      else
      {
        EcLogMsg(EC_LOG_LEVEL_INFO, (pEcLogContext, EC_LOG_LEVEL_INFO, "EtherCAT license is verified!\n"));
      }
    }
  }

  /* initalize performance measurement */
  if (pAppParms->dwPerfMeasLevel > 0)
  {
    EC_T_PERF_MEAS_APP_PARMS oPerfMeasAppParms;
    OsMemset(&oPerfMeasAppParms, 0, sizeof(EC_T_PERF_MEAS_APP_PARMS));
    oPerfMeasAppParms.dwNumMeas = MAX_JOB_NUM;
    oPerfMeasAppParms.aPerfMeasInfos = S_aPerfMeasInfos;

    dwRes = ecatPerfMeasAppCreate(&oPerfMeasAppParms, &pAppContext->pvPerfMeas);
    if (dwRes != EC_E_NOERROR)
    {
      dwRetVal = dwRes;
      EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "Cannot initialize app performance measurement: %s (0x%lx))\n", ecatGetText(dwRes), dwRes));
      goto Exit;
    }
    pAppContext->dwPerfMeasLevel = pAppParms->dwPerfMeasLevel;
  }

  /* print MAC address */
  {
    ETHERNET_ADDRESS oSrcMacAddress;
    OsMemset(&oSrcMacAddress, 0, sizeof(ETHERNET_ADDRESS));

    dwRes = ecatGetSrcMacAddress(&oSrcMacAddress);
    if (dwRes != EC_E_NOERROR)
    {
      EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "Cannot get MAC address: %s (0x%lx))\n", ecatGetText(dwRes), dwRes));
    }
    EcLogMsg(EC_LOG_LEVEL_INFO, (pEcLogContext, EC_LOG_LEVEL_INFO, "EtherCAT network adapter MAC: %02X-%02X-%02X-%02X-%02X-%02X\n",
                                 oSrcMacAddress.b[0], oSrcMacAddress.b[1], oSrcMacAddress.b[2], oSrcMacAddress.b[3], oSrcMacAddress.b[4], oSrcMacAddress.b[5]));
  }

  /* EtherCAT traffic logging */
#if (defined INCLUDE_PCAP_RECORDER)
  if (pAppParms->bPcapRecorder)
  {
    pPcapRecorder = EC_NEW(CPcapRecorder());
    if (EC_NULL == pPcapRecorder)
    {
      dwRetVal = EC_E_NOMEMORY;
      EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "ERROR: %d: Creating PcapRecorder failed: %s (0x%lx)\n", pAppContext->dwInstanceId, ecatGetText(dwRetVal), dwRetVal));
      goto Exit;
    }
    dwRes = pPcapRecorder->InitInstance(pAppContext->dwInstanceId, pAppParms->dwPcapRecorderBufferFrameCnt, pAppParms->szPcapRecorderFileprefix);
    if (dwRes != EC_E_NOERROR)
    {
      dwRetVal = dwRes;
      EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "ERROR: %d: Initialize PcapRecorder failed: %s (0x%lx)\n", pAppContext->dwInstanceId, ecatGetText(dwRes), dwRes));
      goto Exit;
    }
  }
#endif /* INCLUDE_PCAP_RECORDER */

#if (defined INCLUDE_SLAVE_STATISTICS)
  /* Slave statistics polling for error diagnostic */
  {
    EC_T_DWORD dwPeriodMs = 1000;

    dwRes = ecatIoCtl(EC_IOCTL_SET_SLVSTAT_PERIOD, (EC_T_BYTE *)&dwPeriodMs, sizeof(EC_T_DWORD), EC_NULL, 0, EC_NULL);
    if (dwRes != EC_E_NOERROR)
    {
      EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "ecatIoControl(EC_IOCTL_SET_SLVSTAT_PERIOD) returns with error=0x%x\n", dwRes));
      goto Exit;
    }
  }
#endif /* INCLUDE_SLAVE_STATISTICS */

  /* create cyclic task to trigger jobs */
  {
    CEcTimer oTimeout(2000);

    pAppContext->bJobTaskRunning = EC_FALSE;
    pAppContext->bJobTaskShutdown = EC_FALSE;
    pvJobTaskHandle = OsCreateThread((EC_T_CHAR *)"EcMasterJobTask", EcMasterJobTask, pAppParms->CpuSet,
                                     JOBS_THREAD_PRIO, JOBS_THREAD_STACKSIZE, (EC_T_VOID *)pAppContext);

    /* wait until thread is running */
    while (!oTimeout.IsElapsed() && !pAppContext->bJobTaskRunning)
    {
      OsSleep(10);
    }
    if (!pAppContext->bJobTaskRunning)
    {
      dwRetVal = EC_E_TIMEOUT;
      EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "ERROR: Timeout starting JobTask\n"));
      goto Exit;
    }
  }

  /* set OEM key if available */
  if (0 != pAppParms->qwOemKey)
  {
    dwRes = ecatSetOemKey(pAppParms->qwOemKey);
    if (dwRes != EC_E_NOERROR)
    {
      dwRetVal = dwRes;
      EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "Cannot set OEM key at master: %s (0x%lx))\n", ecatGetText(dwRes), dwRes));
      goto Exit;
    }
  }

  /* configure master */
  dwRes = ecatConfigureMaster(pAppParms->eCnfType, pAppParms->pbyCnfData, pAppParms->dwCnfDataLen);
  if (dwRes != EC_E_NOERROR)
  {
    dwRetVal = dwRes;
    EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "Cannot configure EtherCAT-Master: %s (0x%lx))\n", ecatGetText(dwRes), dwRes));
    goto Exit;
  }

  /* register client */
  dwRes = ecatRegisterClient(EcMasterNotifyCallback, pAppContext, &RegisterClientResults);
  if (dwRes != EC_E_NOERROR)
  {
    dwRetVal = dwRes;
    EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "ERROR: Cannot register client: %s (0x%lx))\n", ecatGetText(dwRes), dwRes));
    goto Exit;
  }
  pAppContext->pNotificationHandler->SetClientID(RegisterClientResults.dwClntId);

  /* configure DC/DCM master is started with ENI */
  if (EC_NULL != pAppParms->pbyCnfData)
  {
    /* configure DC */
    {
      EC_T_DC_CONFIGURE oDcConfigure;

      OsMemset(&oDcConfigure, 0, sizeof(EC_T_DC_CONFIGURE));
      oDcConfigure.dwTimeout = ETHERCAT_DC_TIMEOUT;
      oDcConfigure.dwDevLimit = ETHERCAT_DC_DEV_LIMIT;
      oDcConfigure.dwSettleTime = ETHERCAT_DC_SETTLE_TIME;
      if (eDcmMode_MasterRefClock == pAppParms->eDcmMode)
      {
        oDcConfigure.dwTotalBurstLength = 10000;
        oDcConfigure.dwBurstBulk = 1;
      }
      else
      {
        oDcConfigure.dwTotalBurstLength = ETHERCAT_DC_ARMW_BURSTCYCLES;
        if (pAppParms->dwBusCycleTimeUsec < 1000)
        {
          /* if the cycle time is below 1000 usec, we have to reduce the number of frames sent within one cycle */
          oDcConfigure.dwBurstBulk = ETHERCAT_DC_ARMW_BURSTSPP / 2;
        }
        else
        {
          oDcConfigure.dwBurstBulk = ETHERCAT_DC_ARMW_BURSTSPP;
        }
      }
#if (defined INCLUDE_DCX)
      if (eDcmMode_Dcx == pAppParms->eDcmMode)
      {
        oDcConfigure.bAcycDistributionDisabled = EC_FALSE; /* Enable acyclic distribution if cycle time is above 1000 usec to get DCX in sync */
      }
      else
      {
        oDcConfigure.bAcycDistributionDisabled = EC_TRUE;
      }
#else
      oDcConfigure.bAcycDistributionDisabled = EC_TRUE;
#endif

      dwRes = ecatDcConfigure(&oDcConfigure);
      if (dwRes != EC_E_NOERROR)
      {
        dwRetVal = dwRes;
        EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "Cannot configure DC! (Result = 0x%x)\n", dwRes));
        goto Exit;
      }
    }
    /* configure DCM */
    if (pAppParms->bDcmLogEnabled && !pAppParms->bDcmConfigure)
    {
      EC_T_BOOL bBusShiftConfiguredByEni = EC_FALSE;
      dwRes = ecatDcmGetBusShiftConfigured(&bBusShiftConfiguredByEni);
      if (dwRes != EC_E_NOERROR)
      {
        EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "Cannot check if BusShift is configured  (Result = 0x%x)\n", dwRes));
      }
      if (bBusShiftConfiguredByEni)
      {
        pAppParms->bDcmConfigure = EC_TRUE;
        pAppParms->eDcmMode = eDcmMode_BusShift;
      }
    }
    if (pAppParms->bDcmConfigure)
    {
      EC_T_DWORD dwCycleTimeNsec = pAppParms->dwBusCycleTimeUsec * 1000; /* cycle time in nsec */
      EC_T_INT nCtlSetValNsec = dwCycleTimeNsec * 2 / 3 /* 66% */;       /* distance between cyclic frame send time and DC base on bus */
      EC_T_DWORD dwInSyncLimitNsec = dwCycleTimeNsec / 5 /* 20% */;      /* limit for DCM InSync monitoring */

      EcLogMsg(EC_LOG_LEVEL_INFO, (pEcLogContext, EC_LOG_LEVEL_INFO, "cycle time: %d nsec  \n", dwCycleTimeNsec));
      EcLogMsg(EC_LOG_LEVEL_INFO, (pEcLogContext, EC_LOG_LEVEL_INFO, "distance between cyclic frame: %d nsec  \n", nCtlSetValNsec));
      EcLogMsg(EC_LOG_LEVEL_INFO, (pEcLogContext, EC_LOG_LEVEL_INFO, "sync limit time: %d nsec  \n", dwInSyncLimitNsec));

      EC_T_DCM_CONFIG oDcmConfig;
      OsMemset(&oDcmConfig, 0, sizeof(EC_T_DCM_CONFIG));

      switch (pAppParms->eDcmMode)
      {
      case eDcmMode_Off:
        oDcmConfig.eMode = eDcmMode_Off;
        break;
      case eDcmMode_BusShift:
        oDcmConfig.eMode = eDcmMode_BusShift;
        oDcmConfig.u.BusShift.nCtlSetVal = nCtlSetValNsec;
        oDcmConfig.u.BusShift.dwInSyncLimit = dwInSyncLimitNsec;
        oDcmConfig.u.BusShift.bLogEnabled = pAppParms->bDcmLogEnabled;
        if (pAppParms->bDcmControlLoopDisabled)
        {
          EcLogMsg(EC_LOG_LEVEL_INFO, (pEcLogContext, EC_LOG_LEVEL_INFO, "DCM control loop disabled for diagnosis!\n"));
          oDcmConfig.u.BusShift.bCtlOff = EC_TRUE;
        }
        break;
      case eDcmMode_MasterShift:
        oDcmConfig.eMode = eDcmMode_MasterShift;
        oDcmConfig.u.MasterShift.nCtlSetVal = nCtlSetValNsec;
        oDcmConfig.u.MasterShift.dwInSyncLimit = dwInSyncLimitNsec;
        oDcmConfig.u.MasterShift.bLogEnabled = pAppParms->bDcmLogEnabled;
        if (pAppParms->bDcmControlLoopDisabled)
        {
          EcLogMsg(EC_LOG_LEVEL_INFO, (pEcLogContext, EC_LOG_LEVEL_INFO, "DCM control loop disabled for diagnosis!\n"));
          oDcmConfig.u.MasterShift.bCtlOff = EC_TRUE;
        }
        break;
      case eDcmMode_MasterRefClock:
        oDcmConfig.eMode = eDcmMode_MasterRefClock;
        oDcmConfig.u.MasterRefClock.nCtlSetVal = nCtlSetValNsec;
        oDcmConfig.u.MasterRefClock.bLogEnabled = pAppParms->bDcmLogEnabled;
        break;
      case eDcmMode_LinkLayerRefClock:
        oDcmConfig.eMode = eDcmMode_LinkLayerRefClock;
        oDcmConfig.u.LinkLayerRefClock.nCtlSetVal = nCtlSetValNsec;
        oDcmConfig.u.LinkLayerRefClock.bLogEnabled = pAppParms->bDcmLogEnabled;
        break;
#if (defined INCLUDE_DCX)
      case eDcmMode_Dcx:
        oDcmConfig.eMode = eDcmMode_Dcx;
        /* Mastershift */
        oDcmConfig.u.Dcx.MasterShift.nCtlSetVal = nCtlSetValNsec;
        oDcmConfig.u.Dcx.MasterShift.dwInSyncLimit = dwInSyncLimitNsec;
        oDcmConfig.u.Dcx.MasterShift.bLogEnabled = pAppParms->bDcmLogEnabled;
        /* Dcx Busshift */
        oDcmConfig.u.Dcx.nCtlSetVal = nCtlSetValNsec;
        oDcmConfig.u.Dcx.dwInSyncLimit = dwInSyncLimitNsec;
        oDcmConfig.u.Dcx.bLogEnabled = pAppParms->bDcmLogEnabled;
        oDcmConfig.u.Dcx.dwExtClockTimeout = 1000;
        oDcmConfig.u.Dcx.wExtClockFixedAddr = 0; /* 0 only when clock adjustment in external mode configured by EcEngineer */
        if (pAppParms->bDcmControlLoopDisabled)
        {
          EcLogMsg(EC_LOG_LEVEL_INFO, (pEcLogContext, EC_LOG_LEVEL_INFO, "DCM control loop disabled for diagnosis!\n"));

          oDcmConfig.u.Dcx.MasterShift.bCtlOff = EC_TRUE;
          oDcmConfig.u.Dcx.bCtlOff = EC_TRUE;
        }
        break;
#endif
      default:
        dwRetVal = EC_E_NOTSUPPORTED;
        EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "DCM mode is not supported!\n"));
        goto Exit;
      }
      dwRes = ecatDcmConfigure(&oDcmConfig, 0);
      switch (dwRes)
      {
      case EC_E_NOERROR:
        break;
      case EC_E_FEATURE_DISABLED:
        EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "Cannot configure DCM mode!\n"));
        EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "Start with -dcmmode off to run the DC demo without DCM, or prepare the ENI file to support the requested DCM mode\n"));
        EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "In ET9000 for example, select under "
                                                                         "Advanced settings\\Distributed clocks"
                                                                         " "
                                                                         "DC in use"
                                                                         " and "
                                                                         "Slave Mode"
                                                                         "\n"));
        EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "to support BusShift and MasterRefClock modes.\n"));
        EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "Please refer to the class A manual for further information\n"));
        dwRetVal = dwRes;
        goto Exit;
      default:
        EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "Cannot configure DCM mode! %s (Result = 0x%x)\n", ecatGetText(dwRes), dwRes));
        dwRetVal = dwRes;
        goto Exit;
      }
    }
  }

  /* print found slaves */
  if (pAppParms->dwAppLogLevel >= EC_LOG_LEVEL_VERBOSE)
  {
    dwRes = ecatScanBus(ETHERCAT_SCANBUS_TIMEOUT);
    pAppContext->pNotificationHandler->ProcessNotificationJobs();
    switch (dwRes)
    {
    case EC_E_NOERROR:
    case EC_E_BUSCONFIG_MISMATCH:
    case EC_E_LINE_CROSSED:
      PrintSlaveInfos(pAppContext);
      break;
    default:
      EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "Cannot scan bus: %s (0x%lx)\n", ecatGetText(dwRes), dwRes));
      break;
    }
    if (dwRes != EC_E_NOERROR)
    {
      dwRetVal = dwRes;
      goto Exit;
    }
  }

  /* set master to INIT */
  dwRes = ecatSetMasterState(ETHERCAT_STATE_CHANGE_TIMEOUT, eEcatState_INIT);
  pAppContext->pNotificationHandler->ProcessNotificationJobs();
  if (dwRes != EC_E_NOERROR)
  {
    EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "Cannot start set master state to INIT: %s (0x%lx))\n", ecatGetText(dwRes), dwRes));
    dwRetVal = dwRes;
    goto Exit;
  }

  dwRes = myAppPrepare(pAppContext);
  if (EC_E_NOERROR != dwRes)
  {
    EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "ERROR: myAppPrepare failed: %s (0x%lx))\n", ecatGetText(dwRes), dwRes));
    dwRetVal = dwRes;
    goto Exit;
  }

  /* set master to PREOP */
  dwRes = ecatSetMasterState(ETHERCAT_STATE_CHANGE_TIMEOUT, eEcatState_PREOP);
  pAppContext->pNotificationHandler->ProcessNotificationJobs();
  if (dwRes != EC_E_NOERROR)
  {
    EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "Cannot start set master state to PREOP: %s (0x%lx))\n", ecatGetText(dwRes), dwRes));
    dwRetVal = dwRes;
    goto Exit;
  }

  /* skip this step if demo started without ENI */
  if (EC_NULL != pAppParms->pbyCnfData)
  {
    dwRes = myAppSetup(pAppContext);
    if (EC_E_NOERROR != dwRes)
    {
      EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "myAppSetup failed: %s (0x%lx))\n", ecatGetText(dwRes), dwRes));
      dwRetVal = dwRes;
      goto Exit;
    }

    if(!isPdoRwEn())
    {
      // printf("PDO is not need writing, exit program\n");
      goto Exit;
    }

    /* set master to SAFEOP */
    dwRes = ecatSetMasterState(ETHERCAT_STATE_CHANGE_TIMEOUT, eEcatState_SAFEOP);
    pAppContext->pNotificationHandler->ProcessNotificationJobs();
    if (dwRes != EC_E_NOERROR)
    {
      EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "Cannot start set master state to SAFEOP: %s (0x%lx))\n", ecatGetText(dwRes), dwRes));

      /* most of the time SAFEOP is not reachable due to DCM */
      if ((eDcmMode_Off != pAppParms->eDcmMode) && (eDcmMode_LinkLayerRefClock != pAppParms->eDcmMode))
      {
        EC_T_DWORD dwStatus = 0;
        EC_T_INT nDiffCur = 0, nDiffAvg = 0, nDiffMax = 0;

        dwRes = ecatDcmGetStatus(&dwStatus, &nDiffCur, &nDiffAvg, &nDiffMax);
        if (dwRes == EC_E_NOERROR)
        {
          if (dwStatus != EC_E_NOERROR)
          {
            EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "DCM Status: %s (0x%08X)\n", ecatGetText(dwStatus), dwStatus));
          }
        }
        else
        {
          EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "Cannot get DCM status! %s (0x%08X)\n", ecatGetText(dwRes), dwRes));
        }
      }
      dwRetVal = dwRes;
      goto Exit;
    }

    /* set master to OP */
    dwRes = ecatSetMasterState(ETHERCAT_STATE_CHANGE_TIMEOUT, eEcatState_OP);
    pAppContext->pNotificationHandler->ProcessNotificationJobs();
    if (dwRes != EC_E_NOERROR)
    {
      EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "Cannot start set master state to OP: %s (0x%lx))\n", ecatGetText(dwRes), dwRes));
      dwRetVal = dwRes;
      goto Exit;
    }
  }
  else
  {
    EcLogMsg(EC_LOG_LEVEL_INFO, (pEcLogContext, EC_LOG_LEVEL_INFO, "No ENI file provided. EC-Master started with generated ENI file.\n"));
  }

  {
    bRun = EC_TRUE;
    uint32_t i = 0, count = 0;
    uint32_t enalbe_elmo_number = 0,enalbe_yd_number = 0;
   
    EcLogMsg(EC_LOG_LEVEL_INFO, (pEcLogContext, EC_LOG_LEVEL_INFO, "Wait for %d motors to enable\n", num_slave));
    while (bRun)
    {
      if (motorEnable(i + 1, driver_type))
      {
        EcLogMsg(EC_LOG_LEVEL_INFO, (pEcLogContext, EC_LOG_LEVEL_INFO, "Motor %d enabled successfully\n", i + 1));
        i++;
        if(driver_type[i] == ELMO)
        {
          enalbe_elmo_number++;
        }
        else if(driver_type[i] == YD)
        {
          enalbe_yd_number++;
        }
        if (i >= num_slave)
        {
          break;
        }
        count = 0;
      }
      OsSleep(1); // 1000 usec
      count++;
      if (count >= 5000)
      {
        EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "Failed to enable motor %d\n", i + 1));
        if(driver_type[i] == ELMO)
        {
            EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "elmo_Status word 0x%x\n", elmo_slave_input[enalbe_elmo_number]->status_word));
            EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "elmo_Error code 0x%x\n", elmo_slave_input[enalbe_elmo_number]->error_code));
        }
        else if (driver_type[i] == YD)
        {
            EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "yd_Status word 0x%x\n", yd_slave_input[enalbe_yd_number]->status_word));
            EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "yd_Error code 0x%x\n", yd_slave_input[enalbe_yd_number]->error_code));
        }
        goto Exit;
      }
    }

    // 启动回到0位置，有死区 编码器不会回到0.0
    // motorToPosition(pAppContext, ids, num_slave, pos_offset, 90, 1e-3);

    setMotorEnable(true);
    EcLogMsg(EC_LOG_LEVEL_INFO, (pEcLogContext, EC_LOG_LEVEL_INFO, "motor enabled\n"));
  }
  OsSleep(10);

  if (pAppContext->dwPerfMeasLevel > 0)
  {
    EcLogMsg(EC_LOG_LEVEL_INFO, (pEcLogContext, EC_LOG_LEVEL_INFO, "\nJob times during startup <INIT> to <%s>:\n", ecatStateToStr(ecatGetMasterState())));
    PRINT_PERF_MEAS();
    EcLogMsg(EC_LOG_LEVEL_INFO, (pEcLogContext, EC_LOG_LEVEL_INFO, "\n"));
    /* clear job times of startup phase */
    ecatPerfMeasAppReset(pAppContext->pvPerfMeas, EC_PERF_MEAS_ALL);
    ecatPerfMeasInternalReset(EC_PERF_MEAS_ALL);
  }

#if (defined DEBUG) && (defined EC_VERSION_XENOMAI)
  /* Enabling mode switch warnings for shadowed task (mallocs before may have switched mode) */
  dwRes = rt_task_set_mode(0, T_WARNSW, NULL);
  if (0 != dwRes)
  {
    EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "EnableRealtimeEnvironment: rt_task_set_mode returned error 0x%lx\n", dwRes));
    OsDbgAssert(EC_FALSE);
  }
#endif

  /* run the demo */
  if (pAppParms->dwDemoDuration != 0)
  {
    EcLogMsg(EC_LOG_LEVEL_INFO, (pEcLogContext, EC_LOG_LEVEL_INFO, "%s will stop in %ds...\n", EC_DEMO_APP_NAME, pAppParms->dwDemoDuration / 1000));
    oAppDuration.Start(pAppParms->dwDemoDuration);
  }
  bRun = EC_TRUE;
  {
    CEcTimer oPerfMeasPrintTimer;

    if (pAppParms->bPerfMeasShowCyclic)
    {
      oPerfMeasPrintTimer.Start(2000);
    }

    while (bRun)
    {
      if (oPerfMeasPrintTimer.IsElapsed())
      {
        PRINT_PERF_MEAS();
        oPerfMeasPrintTimer.Restart();
      }

      /* check if demo shall terminate */
      if (bRun) {  // 只有在bRun为true时才检查其他退出条件
        bRun = !(OsTerminateAppRequest() || oAppDuration.IsElapsed());
      }

      myAppDiagnosis(pAppContext);

      if (EC_NULL != pAppParms->pbyCnfData)
      {
        if ((eDcmMode_Off != pAppParms->eDcmMode) && (eDcmMode_LinkLayerRefClock != pAppParms->eDcmMode))
        {
          EC_T_DWORD dwStatus = 0;
          EC_T_BOOL bWriteDiffLog = EC_FALSE;
          EC_T_INT nDiffCur = 0, nDiffAvg = 0, nDiffMax = 0;

          if (!oDcmStatusTimer.IsStarted() || oDcmStatusTimer.IsElapsed())
          {
            bWriteDiffLog = EC_TRUE;
            oDcmStatusTimer.Start(5000);
          }

          dwRes = ecatDcmGetStatus(&dwStatus, &nDiffCur, &nDiffAvg, &nDiffMax);
          if (dwRes == EC_E_NOERROR)
          {
            if (bFirstDcmStatus)
            {
              EcLogMsg(EC_LOG_LEVEL_INFO, (pEcLogContext, EC_LOG_LEVEL_INFO, "DCM during startup (<INIT> to <%s>)\n", ecatStateToStr(ecatGetMasterState())));
            }
            if ((dwStatus != EC_E_NOTREADY) && (dwStatus != EC_E_BUSY) && (dwStatus != EC_E_NOERROR))
            {
              EcLogMsg(EC_LOG_LEVEL_INFO, (pEcLogContext, EC_LOG_LEVEL_INFO, "DCM Status: %s (0x%08X)\n", ecatGetText(dwStatus), dwStatus));
            }
            if (bWriteDiffLog && pAppParms->bDcmLogEnabled)
            {
              EcLogMsg(EC_LOG_LEVEL_INFO, (pEcLogContext, EC_LOG_LEVEL_INFO, "DCM Diff (cur/avg/max) [nsec]: %7d/ %7d/ %7d\n", nDiffCur, nDiffAvg, nDiffMax));
            }
          }
          else
          {
            if ((eEcatState_OP == ecatGetMasterState()) || (eEcatState_SAFEOP == ecatGetMasterState()))
            {
              EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "Cannot get DCM status! %s (0x%08X)\n", ecatGetText(dwRes), dwRes));
            }
          }
#if (defined INCLUDE_DCX)
          if (eDcmMode_Dcx == pAppParms->eDcmMode && EC_E_NOERROR == dwRes)
          {
            EC_T_INT64 nTimeStampDiff = 0;
            dwRes = ecatDcxGetStatus(&dwStatus, &nDiffCur, &nDiffAvg, &nDiffMax, &nTimeStampDiff);
            if (EC_E_NOERROR == dwRes)
            {
              if (bFirstDcmStatus)
              {
                EcLogMsg(EC_LOG_LEVEL_INFO, (pEcLogContext, EC_LOG_LEVEL_INFO, "DCX during startup (<INIT> to <%s>)\n", ecatStateToStr(ecatGetMasterState())));
              }
              if ((dwStatus != EC_E_NOTREADY) && (dwStatus != EC_E_BUSY) && (dwStatus != EC_E_NOERROR))
              {
                EcLogMsg(EC_LOG_LEVEL_INFO, (pEcLogContext, EC_LOG_LEVEL_INFO, "DCX Status: %s (0x%08X)\n", ecatGetText(dwStatus), dwStatus));
              }
              if (bWriteDiffLog && pAppParms->bDcmLogEnabled)
              {
                EcLogMsg(EC_LOG_LEVEL_INFO, (pEcLogContext, EC_LOG_LEVEL_INFO, "DCX Diff(ns): Cur=%7d, Avg=%7d, Max=%7d, TimeStamp=%7d\n", nDiffCur, nDiffAvg, nDiffMax, nTimeStampDiff));
              }
            }
            else
            {
              if ((eEcatState_OP == ecatGetMasterState()) || (eEcatState_SAFEOP == ecatGetMasterState()))
              {
                EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "Cannot get DCX status! %s (0x%08X)\n", ecatGetText(dwRes), dwRes));
              }
            }
          }
#endif
          if (bFirstDcmStatus && (EC_E_NOERROR == dwRes))
          {
            bFirstDcmStatus = EC_FALSE;
            ecatDcmResetStatus();
          }
        }
      }
      /* process notification jobs */
      pAppContext->pNotificationHandler->ProcessNotificationJobs();

      OsSleep(5);
    }
  }

  if (pAppParms->dwAppLogLevel >= EC_LOG_LEVEL_VERBOSE)
  {
    EC_T_DWORD dwCurrentUsage = 0;
    EC_T_DWORD dwMaxUsage = 0;
    dwRes = ecatGetMemoryUsage(&dwCurrentUsage, &dwMaxUsage);
    if (EC_E_NOERROR != dwRes)
    {
      EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "ERROR: Cannot read memory usage of master: %s (0x%lx))\n", ecatGetText(dwRes), dwRes));
      goto Exit;
    }
    EcLogMsg(EC_LOG_LEVEL_INFO, (pEcLogContext, EC_LOG_LEVEL_INFO, "Memory Usage Master     (cur/max) [bytes]: %u/%u\n", dwCurrentUsage, dwMaxUsage));

#if (defined INCLUDE_RAS_SERVER)
    if (EC_NULL != pvRasServerHandle)
    {
      dwRes = emRasGetMemoryUsage(pvRasServerHandle, &dwCurrentUsage, &dwMaxUsage);
      if (EC_E_NOERROR != dwRes)
      {
        EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "ERROR: Cannot read memory usage of RAS: %s (0x%lx))\n", ecatGetText(dwRes), dwRes));
        goto Exit;
      }
      EcLogMsg(EC_LOG_LEVEL_INFO, (pEcLogContext, EC_LOG_LEVEL_INFO, "Memory Usage RAS Server (cur/max) [bytes]: %u/%u\n", dwCurrentUsage, dwMaxUsage));
    }
#endif
  }

  ecMasterExit = false;
  // printf("ecMasterExit = false\n");


Exit:
  ecMasterExit = true;

  /* set master state to INIT */
  if (eEcatState_UNKNOWN != ecatGetMasterState())
  {
    if (pAppParms->dwPerfMeasLevel > 0)
    {
      EcLogMsg(EC_LOG_LEVEL_INFO, (pEcLogContext, EC_LOG_LEVEL_INFO, "\nJob times before shutdown\n"));
      PRINT_PERF_MEAS();
    }
    if (pAppParms->dwPerfMeasLevel > 1)
    {
      PRINT_HISTOGRAM();
    }

    dwRes = ecatSetMasterState(ETHERCAT_STATE_CHANGE_TIMEOUT, eEcatState_INIT);
    pAppContext->pNotificationHandler->ProcessNotificationJobs();
    if (EC_E_NOERROR != dwRes)
    {
      EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "Cannot stop EtherCAT-Master: %s (0x%lx))\n", ecatGetText(dwRes), dwRes));
    }
  }

#if (defined INCLUDE_PCAP_RECORDER)
  SafeDelete(pPcapRecorder);
#endif /* INCLUDE_PCAP_RECORDER */

  /* unregister client */
  if (0 != RegisterClientResults.dwClntId)
  {
    dwRes = ecatUnregisterClient(RegisterClientResults.dwClntId);
    if (EC_E_NOERROR != dwRes)
    {
      EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "Cannot unregister client: %s (0x%lx))\n", ecatGetText(dwRes), dwRes));
    }
  }
#if (defined DEBUG) && (defined EC_VERSION_XENOMAI)
  /* disable PRIMARY to SECONDARY MODE switch warning */
  dwRes = rt_task_set_mode(T_WARNSW, 0, NULL);
  if (dwRes != 0)
  {
    EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "ERROR: rt_task_set_mode returned error %d\n", dwRes));
    OsDbgAssert(EC_FALSE);
  }
#endif

#if (defined INCLUDE_RAS_SERVER)
  /* stop RAS server */
  if (EC_NULL != pvRasServerHandle)
  {
    EcLogMsg(EC_LOG_LEVEL_INFO, (pEcLogContext, EC_LOG_LEVEL_INFO, "Stop Remote Api Server\n"));
    dwRes = emRasSrvStop(pvRasServerHandle, 2000);
    if (EC_E_NOERROR != dwRes)
    {
      EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "ERROR: Remote API Server shutdown failed\n"));
    }
  }
#endif

  /* shutdown JobTask */
  {
    CEcTimer oTimeout(2000);
    pAppContext->bJobTaskShutdown = EC_TRUE;
    while (pAppContext->bJobTaskRunning && !oTimeout.IsElapsed())
    {
      OsSleep(10);
    }
    if (EC_NULL != pvJobTaskHandle)
    {
      OsDeleteThreadHandle(pvJobTaskHandle);
    }
  }

  /* deinitialize master */
  dwRes = ecatDeinitMaster();
  if (EC_E_NOERROR != dwRes)
  {
    EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "ERROR: Cannot de-initialize EtherCAT-Master: %s (0x%lx)\n", ecatGetText(dwRes), dwRes));
  }

  SafeDelete(pAppContext->pNotificationHandler);
  if (EC_NULL != pAppContext->pMyAppDesc)
  {
    SafeOsFree(pAppContext->pMyAppDesc->pbyFlashBuf);
    SafeOsFree(pAppContext->pMyAppDesc);
  }

  return dwRetVal;
}
/********************************************************************************/
/** \brief  Trigger jobs to drive master, and update process data.
*
* \return N/A
*/
static EC_T_VOID EcMasterJobTask(EC_T_VOID* pvAppContext)
{
    EC_T_DWORD dwRes = EC_E_ERROR;
    EC_T_INT   nOverloadCounter = 0;               /* counter to check if cycle time is to short */
    T_EC_DEMO_APP_CONTEXT* pAppContext = (T_EC_DEMO_APP_CONTEXT*)pvAppContext;

    EC_T_USER_JOB_PARMS oJobParms;
    EC_T_USER_JOB_PARMS oTaskJobParms;
    OsMemset(&oJobParms, 0, sizeof(EC_T_USER_JOB_PARMS));
    OsMemset(&oTaskJobParms, 0, sizeof(EC_T_USER_JOB_PARMS));

    /* demo loop */
    pAppContext->bJobTaskRunning = EC_TRUE;
    do
    {
        /* wait for next cycle (event from scheduler task) */
        OsWaitForEvent(pAppContext->pvJobTaskEvent, EC_WAITINFINITE);

        /* start Task (required for enhanced performance measurement) */
        dwRes = ecatExecJob(eUsrJob_StartTask, &oTaskJobParms);
        if (EC_E_NOERROR != dwRes && EC_E_INVALIDSTATE != dwRes && EC_E_LINK_DISCONNECTED != dwRes)
        {
            EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "ERROR: ecatExecJob(eUsrJob_StartTask): %s (0x%lx)\n", ecatGetText(dwRes), dwRes));
        }

        /* process all received frames (read new input values) */
        dwRes = ecatExecJob(eUsrJob_ProcessAllRxFrames, &oJobParms);
        if (EC_E_NOERROR != dwRes && EC_E_INVALIDSTATE != dwRes && EC_E_LINK_DISCONNECTED != dwRes)
        {
            EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "ERROR: ecatExecJob(eUsrJob_ProcessAllRxFrames): %s (0x%lx)\n", ecatGetText(dwRes), dwRes));
        }

        if (EC_E_NOERROR == dwRes)
        {
            if (!oJobParms.bAllCycFramesProcessed)
            {
                /* it is not reasonable, that more than 5 continuous frames are lost */
                nOverloadCounter += 10;
                if (nOverloadCounter >= 50)
                {
                    if ((pAppContext->dwPerfMeasLevel > 0) && (nOverloadCounter < 60))
                    {
                        PRINT_PERF_MEAS();
                    }
                    EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "Error: System overload: Cycle time too short or huge jitter!\n"));
                }
                else
                {
                    EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "eUsrJob_ProcessAllRxFrames - not all previously sent frames are received/processed (frame loss)!\n"));
                }
            }
            else
            {
                /* everything o.k.? If yes, decrement overload counter */
                if (nOverloadCounter > 0)    nOverloadCounter--;
            }
        }

        if (pAppContext->dwPerfMeasLevel > 0)
        {
            ecatPerfMeasAppStart(pAppContext->pvPerfMeas, PERF_myAppWorkpd);
        }
        {
            EC_T_STATE eMasterState = ecatGetMasterState();

            if ((eEcatState_SAFEOP == eMasterState) || (eEcatState_OP == eMasterState))
            {
                myAppWorkpd(pAppContext);
            }
        }
        if (pAppContext->dwPerfMeasLevel > 0)
        {
            ecatPerfMeasAppEnd(pAppContext->pvPerfMeas, PERF_myAppWorkpd);
        }

        /* write output values of current cycle, by sending all cyclic frames */
        dwRes = ecatExecJob(eUsrJob_SendAllCycFrames, &oJobParms);
        if (EC_E_NOERROR != dwRes && EC_E_INVALIDSTATE != dwRes && EC_E_LINK_DISCONNECTED != dwRes)
        {
            EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "ecatExecJob( eUsrJob_SendAllCycFrames,    EC_NULL ): %s (0x%lx)\n", ecatGetText(dwRes), dwRes));
        }

        /* remove this code when using licensed version */
        if (EC_E_EVAL_EXPIRED == dwRes)
        {
            bRun = EC_FALSE;
        }

        /* execute some administrative jobs. No bus traffic is performed by this function */
        dwRes = ecatExecJob(eUsrJob_MasterTimer, EC_NULL);
        if (EC_E_NOERROR != dwRes && EC_E_INVALIDSTATE != dwRes)
        {
            EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "ecatExecJob(eUsrJob_MasterTimer, EC_NULL): %s (0x%lx)\n", ecatGetText(dwRes), dwRes));
        }

        /* send queued acyclic EtherCAT frames */
        dwRes = ecatExecJob(eUsrJob_SendAcycFrames, EC_NULL);
        if (EC_E_NOERROR != dwRes && EC_E_INVALIDSTATE != dwRes && EC_E_LINK_DISCONNECTED != dwRes)
        {
            EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "ecatExecJob(eUsrJob_SendAcycFrames, EC_NULL): %s (0x%lx)\n", ecatGetText(dwRes), dwRes));
        }

        /* stop Task (required for enhanced performance measurement) */
        dwRes = ecatExecJob(eUsrJob_StopTask, &oTaskJobParms);
        if (EC_E_NOERROR != dwRes && EC_E_INVALIDSTATE != dwRes && EC_E_LINK_DISCONNECTED != dwRes)
        {
            EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "ERROR: ecatExecJob(eUsrJob_StopTask): %s (0x%lx)\n", ecatGetText(dwRes), dwRes));
        }
#if !(defined NO_OS)
    } while (!pAppContext->bJobTaskShutdown);

    pAppContext->bJobTaskRunning = EC_FALSE;
#else
    /* in case of NO_OS the job task function is called cyclically within the timer ISR */
    } while (EC_FALSE);
    pAppContext->bJobTaskRunning = !pAppContext->bJobTaskShutdown;
#endif

    return;
}

/********************************************************************************/
/** \brief  Handler for master notifications
*
* \return  Status value.
*/
static EC_T_DWORD EcMasterNotifyCallback(
    EC_T_DWORD         dwCode,  /**< [in]   Notification code */
    EC_T_NOTIFYPARMS*  pParms   /**< [in]   Notification parameters */
)
{
    EC_T_DWORD dwRetVal = EC_E_NOERROR;
    CEmNotification* pNotificationHandler = EC_NULL;

    if ((EC_NULL == pParms) || (EC_NULL == pParms->pCallerData))
    {
        dwRetVal = EC_E_INVALIDPARM;
        goto Exit;
    }

    pNotificationHandler = ((T_EC_DEMO_APP_CONTEXT*)pParms->pCallerData)->pNotificationHandler;

    if ((dwCode >= EC_NOTIFY_APP) && (dwCode <= EC_NOTIFY_APP + EC_NOTIFY_APP_MAX_CODE))
    {
        /* notification for application */
        dwRetVal = myAppNotify(dwCode - EC_NOTIFY_APP, pParms);
    }
    else
    {
        /* default notification handler */
        dwRetVal = pNotificationHandler->ecatNotify(dwCode, pParms);
    }

Exit:
    return dwRetVal;
}

/********************************************************************************/
/** \brief  RAS notification handler
 *
 * \return EC_E_NOERROR or error code
 */
#ifdef INCLUDE_RAS_SERVER
static EC_T_DWORD RasNotifyCallback(
    EC_T_DWORD         dwCode,
    EC_T_NOTIFYPARMS*  pParms
)
{
    EC_T_DWORD dwRetVal = EC_E_NOERROR;
    CEmNotification* pNotificationHandler = EC_NULL;

    if ((EC_NULL == pParms) || (EC_NULL == pParms->pCallerData))
    {
        dwRetVal = EC_E_INVALIDPARM;
        goto Exit;
    }

    pNotificationHandler = (CEmNotification*)pParms->pCallerData;
    dwRetVal = pNotificationHandler->emRasNotify(dwCode, pParms);

Exit:
    return dwRetVal;
}
#endif

/*-MYAPP---------------------------------------------------------------------*/

/***************************************************************************************************/
/**
\brief  Initialize Application

\return EC_E_NOERROR on success, error code otherwise.
*/
static EC_T_DWORD myAppInit(T_EC_DEMO_APP_CONTEXT* pAppContext)
{
    EC_UNREFPARM(pAppContext);
     EcLogMsg(EC_LOG_LEVEL_INFO, (pEcLogContext, EC_LOG_LEVEL_INFO, "Enter my app Init\n"));
    return EC_E_NOERROR;
}

/***************************************************************************************************/
/**
\brief  Initialize Slave Instance.

Find slave parameters.
\return EC_E_NOERROR on success, error code otherwise.
*/
static EC_T_DWORD myAppPrepare(T_EC_DEMO_APP_CONTEXT* pAppContext)
{
    EC_T_DWORD          dwRes      = EC_E_NOERROR;
    T_MY_APP_DESC*      pMyAppDesc = pAppContext->pMyAppDesc;
    EC_T_CFG_SLAVE_INFO oCfgSlaveInfo;
    OsMemset(&oCfgSlaveInfo, 0, sizeof(EC_T_CFG_SLAVE_INFO));

    // 根据参商id查找 设备, 并统计数目
    EC_T_DWORD dwSlaveIdx = 0;
    EC_T_DWORD dwNumConfiguredSlaves = emGetNumConfiguredSlaves(pAppContext->dwInstanceId);
    num_slave = dwNumConfiguredSlaves;//总的从站数目
    //初始化从站计数器
    num_elmo_slave = 0;
    num_yd_slave = 0; 

      //识别从站并将个数统计到各自的num_slave
  for(dwSlaveIdx = 0; dwSlaveIdx < dwNumConfiguredSlaves; dwSlaveIdx++)
  {
     EC_T_WORD wAutoIncAddress = (EC_T_WORD)(0 - dwSlaveIdx);

    /* get config slave information */
#if (defined INCLUDE_EC_MASTER) || (defined INCLUDE_EC_MONITOR) 
    dwRes = emGetCfgSlaveInfo(pAppContext->dwInstanceId, EC_FALSE, wAutoIncAddress, &oCfgSlaveInfo);
#elif (defined INCLUDE_EC_SIMULATOR)
    dwRes = esGetCfgSlaveInfo(pAppContext->dwInstanceId, EC_FALSE, wAutoIncAddress, &oCfgSlaveInfo);
#endif
  //根据不同的厂商的id统计从站数目 （Vendor 厂商ID）
    if(oCfgSlaveInfo.dwVendorId == YD_VENDOR_ID)
    {
      num_yd_slave++;
      // printf("num_yd_slave = %d\n",num_yd_slave);   
    }
    else if(oCfgSlaveInfo.dwVendorId == ELMO_VENDOR_ID)
    {      
      num_elmo_slave++;
      // printf("num_elmo_slave = %d\n",num_elmo_slave);
    }
  }
    // std::cout << "[ECmaster]: num_elmo_slave = " << num_elmo_slave << std::endl;
    // std::cout << "[ECmaster]: num_yd_slave = " << num_yd_slave << std::endl;

    // 获取 收发 image
    EC_T_BYTE *pbyPdIn = ecatGetProcessImageInputPtr();
    EC_T_BYTE *pbyPdOut = ecatGetProcessImageOutputPtr();

    uint32_t elmo_rl = sizeof(ELMO_SlaveRead_t);
    uint32_t elmo_wl = sizeof(ELMO_SlaveWrite_t);
    uint32_t elmo_l = elmo_rl >= elmo_wl ? elmo_rl : elmo_wl; // ELMO的需要对齐 ENI BitOffs
    
    uint32_t yd_rl = sizeof(YD_SlaveRead_t);
    uint32_t yd_wl = sizeof(YD_SlaveWrite_t);

    for(uint32_t i = 0;i < num_elmo_slave; i++)
    {
        elmo_slave_input[i] = (ELMO_SlaveRead_t *)(pbyPdIn + elmo_l * i );
        elmo_slave_output[i] = (ELMO_SlaveWrite_t *)(pbyPdOut + elmo_l * i);
    }
    for(uint32_t i = 0;i < num_yd_slave; i++)
    {
        yd_slave_input[i] = (YD_SlaveRead_t *)(pbyPdIn + elmo_l * num_elmo_slave + yd_rl * i);
        yd_slave_output[i] = (YD_SlaveWrite_t *)(pbyPdOut + elmo_l * num_elmo_slave + yd_wl * i);
    }

    if (pAppContext->AppParms.bFlash)
    {
        EC_T_WORD wFlashSlaveAddr = pAppContext->AppParms.wFlashSlaveAddr;

        /* check if slave address is provided */
        if (wFlashSlaveAddr != INVALID_FIXED_ADDR)
        {
            /* get slave's process data offset and some other infos */
            dwRes = ecatGetCfgSlaveInfo(EC_TRUE, wFlashSlaveAddr, &oCfgSlaveInfo);
            if (dwRes != EC_E_NOERROR)
            {
                EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "ERROR: myAppPrepare: ecatGetCfgSlaveInfo() returns with error=0x%x, slave address=%d\n", dwRes, wFlashSlaveAddr));
                goto Exit;
            }

            if (oCfgSlaveInfo.dwPdSizeOut != 0)
            {
                pMyAppDesc->dwFlashPdBitSize = oCfgSlaveInfo.dwPdSizeOut;
                pMyAppDesc->dwFlashPdBitOffs = oCfgSlaveInfo.dwPdOffsOut;
            }
            else
            {
                EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "ERROR: myAppPrepare: Slave address=%d has no outputs, therefore flashing not possible\n", wFlashSlaveAddr));
            }
        }
        else
        {
            /* get complete process data output size */
            EC_T_MEMREQ_DESC oPdMemorySize;
            OsMemset(&oPdMemorySize, 0, sizeof(EC_T_MEMREQ_DESC));

            dwRes = ecatIoCtl(EC_IOCTL_GET_PDMEMORYSIZE, EC_NULL, 0, &oPdMemorySize, sizeof(EC_T_MEMREQ_DESC), EC_NULL);
            if (dwRes != EC_E_NOERROR)
            {
                EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "ERROR: myAppPrepare: ecatIoControl(EC_IOCTL_GET_PDMEMORYSIZE) returns with error=0x%x\n", dwRes));
                goto Exit;
            }
            pMyAppDesc->dwFlashPdBitSize = oPdMemorySize.dwPDOutSize * 8;
        }
        if (pMyAppDesc->dwFlashPdBitSize > 0)
        {
            pMyAppDesc->dwFlashInterval = 20000; /* flash every 20 msec */
            pMyAppDesc->dwFlashBufSize = BIT2BYTE(pMyAppDesc->dwFlashPdBitSize);
            pMyAppDesc->pbyFlashBuf = (EC_T_BYTE*)OsMalloc(pMyAppDesc->dwFlashBufSize);
            OsMemset(pMyAppDesc->pbyFlashBuf, 0 , pMyAppDesc->dwFlashBufSize);
        }
    }

Exit:
    return EC_E_NOERROR;
}


/***************************************************************************************************/
/**
\brief  Setup slave parameters (normally done in PREOP state)

  - SDO up- and Downloads
  - Read Object Dictionary

\return EC_E_NOERROR on success, error code otherwise.
*/
static EC_T_DWORD myAppSetup(T_EC_DEMO_APP_CONTEXT* pAppContext)
{
    // static int32_t yd_reboot = 1;
    EC_UNREFPARM(pAppContext);
    //读取驱动器配置
    if(getSdoStatus() == SDO_STATUS_READ_ALL)
    {
      setPdoRwEn(false);
      real_config_parameter = getRobotAllDriverConfigParameter(num_slave); //读取所有驱动器配置参数,并保存到json文件中
    }
    //写入驱动器配置
    else if(getSdoStatus() == SDO_STATUS_WRITE_ALL)
    {
      setPdoRwEn(false);
      Sdo_RW_Res = robotSetAllConfig(num_slave, g_conJsonPath.c_str()); //按照json设置所有驱动器配置
      
    }
    //单个配置读取
    else if(getSdoStatus() == SDO_STATUS_READ_SINGLE)
    {
      setPdoRwEn(false);
      Sdo_RW_Res = readSingleSdo((SdoRead.SlaveId-1),SdoRead.ObIndex,SdoRead.SubIndex,&SdoRead.read_data); //读取单个驱动器配置
      // printf("Sdo_RW_Res = %d\n",Sdo_RW_Res);
      // printf("SdoRead.SlaveId = %d\n",SdoRead.SlaveId);
      // printf("SdoRead.ObIndex = 0x%x\n",SdoRead.ObIndex);
      // printf("SdoRead.SubIndex = 0x%x\n",SdoRead.SubIndex);
      // printf("SdoRead.read_data = %d\n",SdoRead.read_data);
    }
    //单个配置写入
    else if(getSdoStatus() == SDO_STATUS_WRITE_SINGLE)
    {
      setPdoRwEn(false);
      Sdo_RW_Res = writeSingleSdo((SdoWrite.SlaveId-1),SdoWrite.ObIndex,SdoWrite.SubIndex,&SdoWrite.write_data,SdoWrite.save); //写入单个驱动器配置
      // printf("Sdo_RW_Res = %d\n",Sdo_RW_Res);
      // printf("SdoWrite.SlaveId = %d\n",SdoWrite.SlaveId);
      // printf("SdoWrite.ObIndex = 0x%x\n",SdoWrite.ObIndex);
      // printf("SdoWrite.SubIndex = 0x%x\n",SdoWrite.SubIndex);
      // printf("SdoWrite.write_data = %d\n",SdoWrite.write_data);
      // printf("SdoWrite.save = %d\n",SdoWrite.save);
    }
    //电机自学习
    else if(getSdoStatus() == SDO_STATUS_SELF_LEARN)
    {
      setPdoRwEn(false);
      Sdo_RW_Res = motorSelfLearning(SdoWrite.SlaveId-1,SdoWrite.write_data,20); //电机自学习
    }
    else if(getSdoStatus() == SDO_STATUS_READ_ERROR_CODE)
    {
      setPdoRwEn(false);
      std::vector<int32_t> error_code_list;
      Sdo_RW_Res = readAllErrorCode(num_slave,error_code_list); //读取所有error code
      if(Sdo_RW_Res == true)
      {
        checkErrorCode(error_code_list); //检查error code
      }
    }
    else
    {
      setPdoRwEn(true);
    }

    setSdoStatus(SDO_STATUS_NONE);

    return EC_E_NOERROR;
}

/***************************************************************************************************/
/**
\brief  demo application working process data function.

  This function is called in every cycle after the the master stack is started.

*/
static EC_T_DWORD myAppWorkpd(T_EC_DEMO_APP_CONTEXT* pAppContext)
{
    UserAppWorkpd(pAppContext);
    T_MY_APP_DESC* pMyAppDesc = pAppContext->pMyAppDesc;
    EC_T_BYTE*     pbyPdOut   = ecatGetProcessImageOutputPtr();

    /* demo code flashing */
    if (pMyAppDesc->dwFlashPdBitSize != 0)
    {
        pMyAppDesc->dwFlashTimer += pAppContext->AppParms.dwBusCycleTimeUsec;
        if (pMyAppDesc->dwFlashTimer >= pMyAppDesc->dwFlashInterval)
        {
            pMyAppDesc->dwFlashTimer = 0;

            /* flash with pattern */
            pMyAppDesc->byFlashVal++;
            OsMemset(pMyAppDesc->pbyFlashBuf, pMyAppDesc->byFlashVal, pMyAppDesc->dwFlashBufSize);

            /* update PdOut */
            EC_COPYBITS(pbyPdOut, pMyAppDesc->dwFlashPdBitOffs, pMyAppDesc->pbyFlashBuf, 0, pMyAppDesc->dwFlashPdBitSize);
        }
    }
    return EC_E_NOERROR;
}

/***************************************************************************************************/
/**
\brief  demo application doing some diagnostic tasks

  This function is called in sometimes from the main demo task
*/
static EC_T_DWORD myAppDiagnosis(T_EC_DEMO_APP_CONTEXT* pAppContext)
{
    EC_UNREFPARM(pAppContext);

    // if (ecMasterExit)
    // {
    //   motor_enabled = false;
    // }

    return EC_E_NOERROR;
}

/********************************************************************************/
/** \brief  Handler for application notifications
*
*  !!! No blocking API shall be called within this function!!!
*  !!! Function is called by cylic task                    !!!
*
* \return  Status value.
*/
static EC_T_DWORD myAppNotify(
    EC_T_DWORD        dwCode, /* [in] Application notification code */
    EC_T_NOTIFYPARMS* pParms  /* [in] Notification parameters */
)
{
    EC_T_DWORD dwRetVal = EC_E_ERROR;
    T_EC_DEMO_APP_CONTEXT* pAppContext = (T_EC_DEMO_APP_CONTEXT*)pParms->pCallerData;

    /* dispatch notification code */
    switch (dwCode)
    {
    case 1:
        EcLogMsg(EC_LOG_LEVEL_INFO, (pEcLogContext, EC_LOG_LEVEL_INFO, "myAppNotify: Notification code=0x%lx received\n", dwCode));
        /* dwRetVal = EC_E_NOERROR; */
        break;
    case 2:
        break;
    default:
        break;
    }

    return dwRetVal;
}

EC_T_VOID ShowSyntaxAppUsage(T_EC_DEMO_APP_CONTEXT* pAppContext)
{
    const EC_T_CHAR* szAppUsage = "<LinkLayer> [-f ENI-FileName] [-t time] [-b cycle time] [-a affinity] [-v lvl] [-perf [level]] [-log prefix [msg cnt]] [-lic key] [-oem key] [-flash address]"
#if (defined INCLUDE_RAS_SERVER)
        " [-sp [port]]"
#endif
#if (defined AUXCLOCK_SUPPORTED)
        " [-auxclk period]"
#endif
#if (defined INCLUDE_PCAP_RECORDER)
        " [-rec [prefix [frame cnt]]]"
#endif
        ;
    EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "%s V%s for %s %s\n", EC_DEMO_APP_NAME, ATECAT_FILEVERSIONSTR, ATECAT_PLATFORMSTR, ATECAT_COPYRIGHT));
    EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "Syntax:\n"));
    EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "%s %s", EC_DEMO_APP_NAME, szAppUsage));
}

EC_T_VOID ShowSyntaxApp(T_EC_DEMO_APP_CONTEXT* pAppContext)
{
    EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "   -flash            Flash outputs\n"));
    EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "     address         0=all, >0 = slave station address\n"));
#if (defined AUXCLOCK_SUPPORTED)
    EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "   -auxclk           use auxiliary clock\n"));
    EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "     period          clock period in usec\n"));
#endif
#if (defined INCLUDE_PCAP_RECORDER)
    EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "   -rec              Record network traffic to pcap file\n"));
    EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "    [prefix          pcap file name prefix\n"));
    EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "    [frame cnt]      Frame count for log buffer allocation (default = %d, with %d bytes per message)]\n", PCAP_RECORDER_BUF_FRAME_CNT, ETHERNET_MAX_FRAMEBUF_LEN));
#endif
}

void setMotorEnable(bool en)
{
  motor_enabled = en;
}

bool isMotorEnable(void)
{
  return motor_enabled;
}

void setPdoRwEn(bool en)
{
  pdo_rw_en = en;
}

bool isPdoRwEn(void)
{
  return pdo_rw_en;
}

/*-END OF SOURCE FILE--------------------------------------------------------*/
