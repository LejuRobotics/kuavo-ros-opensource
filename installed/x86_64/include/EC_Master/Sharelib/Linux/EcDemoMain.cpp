/*-----------------------------------------------------------------------------
 * EcDemoMain.cpp
 * Copyright                acontis technologies GmbH, Ravensburg, Germany
 * Response                 Holger Oelhaf
 * Description              EtherCAT demo main entrypoint
 *----------------------------------------------------------------------------*/

/*-LOGGING-------------------------------------------------------------------*/
#ifndef pEcLogParms
#define pEcLogParms G_pEcLogParms
#endif

/*-INCLUDES------------------------------------------------------------------*/
#include "EcDemoApp.h"

#include <sys/mman.h>
#include <sys/utsname.h>
#include <signal.h>

#include <stdio.h>
#include <stdlib.h>

/*-DEFINES-------------------------------------------------------------------*/
#if (!defined EXCLUDE_EC_MASTER_JOB_TASK)
#define INCLUDE_EC_MASTER_JOB_TASK
#endif
#define NSEC_PER_SEC                (1000000000)

/*-TYPEDEFS------------------------------------------------------------------*/
#if (defined INCLUDE_EC_MASTER_JOB_TASK)
typedef struct _EC_T_TIMING_DESC
{
    T_EC_DEMO_APP_CONTEXT*      pAppContext;
    EC_T_BOOL                   bShutdown;          /* EC_TRUE if timing thread shall shut down */
    EC_T_BOOL                   bIsRunning;         /* EC_TRUE if the timing thread is running */
} EC_T_TIMING_DESC;
#endif /* INCLUDE_EC_MASTER_JOB_TASK */

/*-LOCAL FUNCTIONS-----------------------------------------------------------*/
EC_T_DWORD PrepareCommandLine(EC_T_INT nArgc, EC_T_VOID* ppArgv, EC_T_CHAR* szCommandLine)
{
    EC_T_CHAR** ppcArgv = (EC_T_CHAR**)ppArgv;
    EC_T_CHAR* pcStrCur = szCommandLine;
    EC_T_INT   nStrRemain = COMMAND_LINE_BUFFER_LENGTH;
    EC_T_CHAR  szStrFormat[] = "%s";

    /* build szCommandLine from argument list, skipping executable name */
    for (nArgc--, ppcArgv++; nArgc > 0; nArgc--, ppcArgv++)
    {
        EC_T_BOOL bIsFileName = EC_FALSE;

        /* insert next argument */
        OsSnprintf(pcStrCur, nStrRemain - 1, szStrFormat, *ppcArgv);

        /* check for file name */
        if (0 == OsStrcmp(pcStrCur, "-f"))
        {
            bIsFileName = EC_TRUE;
        }
        /* adjust string cursor */
        nStrRemain -= (EC_T_INT)OsStrlen(pcStrCur);
        pcStrCur = pcStrCur + OsStrlen(pcStrCur);

        /* insert space */
        OsStrncpy(pcStrCur, " ", nStrRemain - 1); nStrRemain--; pcStrCur++;

        if (bIsFileName && (1 < nArgc))
        {
            /* move to next arg (ENI file name) */
            nArgc--; ppcArgv++;

            /* insert quotation mark */
            OsStrncpy(pcStrCur, "\"", nStrRemain - 1); nStrRemain--; pcStrCur++;

            /* insert ENI file name */
            OsSnprintf(pcStrCur, nStrRemain - 1, szStrFormat, *ppcArgv); nStrRemain -= (EC_T_INT)OsStrlen(pcStrCur);
            pcStrCur = pcStrCur + OsStrlen(pcStrCur);

            /* insert quotation mark */
            OsStrncpy(pcStrCur, "\" ", nStrRemain - 1); nStrRemain--; pcStrCur++;
        }
    }

    return EC_E_NOERROR;
}

/********************************************************************************/
/** \brief  signal handler.
*
* \return N/A
*/
static void SignalHandler(int nSignal)
{
    bRun = EC_FALSE;
    th_run = EC_FALSE;
}

void EcMasterStop()
{
    bRun = EC_FALSE;
    th_run = EC_FALSE;
}


/********************************************************************************/
/** Enable real-time environment
*
* Return: EC_E_NOERROR in case of success, EC_E_ERROR in case of failure.
*/
EC_T_DWORD EnableRealtimeEnvironment(EC_T_VOID)
{
    struct utsname SystemName;
    int nMaj, nMin, nSub;
    struct timespec ts;
    int nRetval;
    EC_T_DWORD dwResult = EC_E_ERROR;
    EC_T_BOOL bHighResTimerAvail;
    struct sched_param schedParam;

    /* master only tested on >= 2.6 kernel */
    nRetval = uname(&SystemName);
    if (nRetval != 0)
    {
        EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "ERROR calling uname(), required Linux kernel >= 2.6\n"));
        dwResult = EC_E_ERROR;
        goto Exit;
    }
    sscanf(SystemName.release, "%d.%d.%d", &nMaj, &nMin, &nSub);
    if (!(((nMaj == 2) && (nMin == 6)) || (nMaj >= 3)))
    {
        EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "ERROR - detected kernel = %d.%d.%d, required Linux kernel >= 2.6\n", nMaj, nMin, nSub));
        dwResult = EC_E_ERROR;
        goto Exit;
    }

    /* request realtime scheduling for the current process
    * This value is overwritten for each individual task
    */
    schedParam.sched_priority = MAIN_THREAD_PRIO; /* 1 lowest priority, 99 highest priority */
    nRetval = sched_setscheduler(0, SCHED_FIFO, &schedParam);
    if (nRetval == -1)
    {
        EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "ERROR - cannot change scheduling policy!\n"
            "root privilege is required or realtime group has to be joined!\n"));
        goto Exit;
    }

    /* disable paging */
    nRetval = mlockall(MCL_CURRENT | MCL_FUTURE);
    if (nRetval == -1)
    {
        EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "ERROR - cannot disable paging!\n"));
        dwResult = EC_E_ERROR;
        goto Exit;
    }

    /* check if high resolution timers are available */
    if (clock_getres(CLOCK_MONOTONIC, &ts))
    {
        bHighResTimerAvail = EC_FALSE;
    }
    else
    {
        bHighResTimerAvail = !(ts.tv_sec != 0 || ts.tv_nsec != 1);
    }
    if (!bHighResTimerAvail)
    {
        EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "WARNING: High resolution timers not available\n"));
    }

    /* set type of OsSleep implementation  (eSLEEP_USLEEP, eSLEEP_NANOSLEEP or eSLEEP_CLOCK_NANOSLEEP) */
    OsSleepSetType(eSLEEP_CLOCK_NANOSLEEP);

    dwResult = EC_E_NOERROR;
Exit:
    return dwResult;
}

/** Show syntax
*
* Return: N/A
*/
static EC_T_VOID ShowSyntax(T_EC_DEMO_APP_CONTEXT* pAppContext)
{
    ShowSyntaxAppUsage(pAppContext);
    ShowSyntaxCommon(pAppContext);
    ShowSyntaxApp(pAppContext);
#if (!defined ATEMRAS_CLIENT)
    ShowSyntaxLinkLayer();
#endif
    return;
}

#if (defined INCLUDE_EC_MASTER_JOB_TASK)
/********************************************************************************/
/* \brief Set event according to periodical sleep or aux clock
 * Cyclically sets an event for thread synchronization purposes.
 * Either use OsSleep() or use the aux clock by means of:
 * - Enable AUX clock if selected.
 * - Wait for IRQ, aknowledge IRQ, SetEvent in loop until shutdown
 * - Disable AUX clock
 * Return: N/A
 */
static EC_T_VOID tEcTimingTask(EC_T_VOID* pvThreadParamDesc)
{
    EC_T_TIMING_DESC* pTimingDesc = (EC_T_TIMING_DESC*)pvThreadParamDesc;
    T_EC_DEMO_APP_CONTEXT* pAppContext = pTimingDesc->pAppContext;
    T_EC_DEMO_APP_PARMS* pAppParms = &pAppContext->AppParms;
    EC_T_CPUSET CpuSet;
    struct timespec t;

    OsMemset(&CpuSet, 0, sizeof(EC_T_CPUSET));
    EC_CPUSET_ZERO(CpuSet);
    EC_CPUSET_SET(CpuSet, pAppParms->dwCpuIndex);
    OsSetThreadAffinity(EC_NULL, CpuSet);

    OsMemset(&t, 0, sizeof(struct timespec));
    /* get current time */
    clock_gettime(CLOCK_MONOTONIC, &t);
    /* start after one second */
    t.tv_sec = t.tv_sec + 1;

    /* timing task started */
    pTimingDesc->bIsRunning = EC_TRUE;

    /* periodically generate events as long as the application runs */
    while (!pTimingDesc->bShutdown)
    {
        /* Use the Linux high resolution timer. This API offers resolution
         * below the systick (i.e. 50us cycle is possible) if the Linux
         * kernel is patched with the RT-PREEMPT patch.
         */

        /* wait until next shot */
        if (0 != clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &t, NULL))
        {
           perror("clock_nanosleep failed");
           pTimingDesc->bShutdown = EC_TRUE;
        }

        /* trigger jobtask */
        OsSetEvent(pAppContext->pvJobTaskEvent);

        /* calculate next shot. t.tv_nsec is always < 1000000000 */
        t.tv_nsec = t.tv_nsec + (pAppParms->dwBusCycleTimeUsec * 1000);

        /* norm time */
        while (t.tv_nsec >= NSEC_PER_SEC)
        {
           t.tv_nsec = t.tv_nsec - NSEC_PER_SEC;
           t.tv_sec++;
        }
    }
    pTimingDesc->bIsRunning = EC_FALSE;
    return;
}
#endif

/********************************************************************************/
/** \brief  Demo Application entry point.
*
* \return  Value 0 is returned.
*/
int Ec_Master_init(int nArgc, char* ppArgv[])
{
    EC_T_DWORD               dwRetVal = EC_E_ERROR;
    EC_T_DWORD               dwRes = EC_E_ERROR;
    EC_T_DWORD               dwIdx = 0;
    T_EC_DEMO_APP_CONTEXT    AppContext;
    T_EC_DEMO_APP_CONTEXT*   pAppContext = &AppContext;
    T_EC_DEMO_APP_PARMS*     pAppParms = &AppContext.AppParms;
#if (defined INCLUDE_EC_LOGGING)
    CAtEmLogging             oLogging;
    EC_T_BOOL                bLogInitialized = EC_FALSE;
#endif
    EC_T_CHAR                szCommandLine[COMMAND_LINE_BUFFER_LENGTH];
    OsMemset(szCommandLine, '\0', COMMAND_LINE_BUFFER_LENGTH);

#if (defined INCLUDE_EC_MASTER_JOB_TASK)
    EC_T_TIMING_DESC         TimingDesc;
    OsMemset(&TimingDesc, 0, sizeof(EC_T_TIMING_DESC));
    TimingDesc.pAppContext = &AppContext;
#endif

    OsMemset(&AppContext, 0, sizeof(AppContext));

    /* printf logging until logging initialized */
    AppContext.LogParms.dwLogLevel = EC_LOG_LEVEL_ERROR;
#if (defined INCLUDE_EC_LOGGING)
    AppContext.LogParms.pfLogMsg = CAtEmLogging::LogMsgOsPrintf;
#else
    AppContext.LogParms.pfLogMsg = EcDemoLogMsg;
#endif
    AppContext.LogParms.pLogContext = EC_NULL;
    OsMemcpy(G_pEcLogParms, &AppContext.LogParms, sizeof(EC_T_LOG_PARMS));

    AppContext.dwInstanceId = INSTANCE_MASTER_DEFAULT;

    ResetAppParms(&AppContext, pAppParms);
    pAppParms->Os.dwSize = sizeof(EC_T_OS_PARMS);
    pAppParms->Os.dwSignature = EC_OS_PARMS_SIGNATURE;
    pAppParms->Os.dwSupportedFeatures = 0xFFFFFFFF;
    pAppParms->Os.PlatformParms.bConfigMutex = EC_TRUE;
    pAppParms->Os.PlatformParms.nMutexType = PTHREAD_MUTEX_RECURSIVE_NP;
    pAppParms->Os.PlatformParms.nMutexProtocol = PTHREAD_PRIO_NONE;
    OsInit(&pAppParms->Os);

    /* OS specific initialization */
    // EnableRealtimeEnvironment();
    // {
    //     sigset_t SigSet;
    //     int nSigNum = SIGALRM;
    //     sigemptyset(&SigSet);
    //     sigaddset(&SigSet, nSigNum);
    //     sigprocmask(SIG_BLOCK, &SigSet, NULL);
    //     signal(SIGINT, SignalHandler);
    //     signal(SIGTERM, SignalHandler);
    // }
    /* set running flag */
    bRun = EC_TRUE;

    /* handle command line */
    dwRes = PrepareCommandLine(nArgc, ppArgv, szCommandLine);
    if (EC_E_NOERROR != dwRes)
    {
        EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "Failed to prepare command line parameters\n"));
        dwRetVal = EC_E_ERROR;
        goto Exit;
    }
    dwRes = SetAppParmsFromCommandLine(&AppContext, szCommandLine, pAppParms);
    if (EC_E_NOERROR != dwRes)
    {
        dwRetVal = EC_E_INVALIDPARM;
        goto Exit;
    }
    /* initialize logging */
    if ((EC_LOG_LEVEL_SILENT != pAppParms->dwAppLogLevel) || (EC_LOG_LEVEL_SILENT != pAppParms->dwMasterLogLevel))
    {
#if (defined INCLUDE_EC_LOGGING)
        oLogging.InitLogging(INSTANCE_MASTER_DEFAULT, LOG_ROLLOVER, LOG_THREAD_PRIO, pAppParms->CpuSet, pAppParms->szLogFileprefix, LOG_THREAD_STACKSIZE, pAppParms->dwLogBufferMaxMsgCnt);
        AppContext.LogParms.pfLogMsg = CAtEmLogging::LogMsg;
        AppContext.LogParms.pLogContext = (struct _EC_T_LOG_CONTEXT*)&oLogging;
        bLogInitialized = EC_TRUE;
#endif
        AppContext.LogParms.dwLogLevel = pAppParms->dwAppLogLevel;
    }
    else
    {
        AppContext.LogParms.dwLogLevel = EC_LOG_LEVEL_SILENT;
    }
    OsMemcpy(G_pEcLogParms, &AppContext.LogParms, sizeof(EC_T_LOG_PARMS));

    EcLogMsg(EC_LOG_LEVEL_INFO, (pEcLogContext, EC_LOG_LEVEL_INFO, "%s V%s for %s %s\n", EC_DEMO_APP_NAME, ATECAT_FILEVERSIONSTR, ATECAT_PLATFORMSTR, ATECAT_COPYRIGHT));
    EcLogMsg(EC_LOG_LEVEL_INFO, (pEcLogContext, EC_LOG_LEVEL_INFO, "Full command line: %s\n", szCommandLine));

    /* adjust linklayer parameters */
    for (dwIdx = 0; dwIdx < MAX_LINKLAYER; dwIdx++)
    {
        EC_T_LINK_PARMS* pLinkParms = pAppParms->apLinkParms[dwIdx];
        if (EC_NULL == pLinkParms)
        {
            break;
        }
        EC_CPUSET_ZERO(pLinkParms->cpuIstCpuAffinityMask);
        if (!EC_CPUSET_IS_ZERO(pAppParms->CpuSet))
        {
            EC_CPUSET_SET(pLinkParms->cpuIstCpuAffinityMask, pAppParms->dwCpuIndex);
        }
        pLinkParms->dwIstPriority = RECV_THREAD_PRIO;

        OsMemcpy(&pLinkParms->LogParms, &pAppContext->LogParms, sizeof(EC_T_LOG_PARMS));
        pLinkParms->LogParms.dwLogLevel = pAppParms->dwMasterLogLevel;
    }
#if (defined INCLUDE_EMLL_STATIC_LIBRARY)
    OsReplaceGetLinkLayerRegFunc(&DemoGetLinkLayerRegFunc);
#endif

    /* set CPU affinity */
    if (!EC_CPUSET_IS_ZERO(pAppParms->CpuSet))
    {
        dwRes = OsSetThreadAffinity(EC_NULL, pAppParms->CpuSet);
        if (EC_E_NOERROR != dwRes)
        {
            EcLogMsg(EC_LOG_LEVEL_CRITICAL, (pEcLogContext, EC_LOG_LEVEL_CRITICAL, "ERROR: Set Affinity Failed: %s (0x%lx))\n", ecatGetText(dwRes), dwRes));
            dwRetVal = dwRes;
            goto Exit;
        }
    }

#if (defined INCLUDE_EC_MASTER_JOB_TASK)
    if ((eDcmMode_MasterShift == pAppParms->eDcmMode) || (eDcmMode_Dcx == pAppParms->eDcmMode))
    {
        /* current DCM MasterShift implementation based on internal aux clock */
        pAppParms->bUseAuxClock = EC_TRUE;
    }

    /* prepare timing event to trigger the job task */
    if (pAppParms->TtsParms.bEnabled)
    {
        AppContext.pvJobTaskEvent = pAppParms->TtsParms.pvStartCycleContext;
        pAppParms->dwBusCycleTimeUsec = pAppParms->TtsParms.dwCycleTimeUsec;
    }
    else
    {
        AppContext.pvJobTaskEvent = OsCreateEvent();
        if (EC_NULL == AppContext.pvJobTaskEvent)
        {
            EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "ERROR: insufficient memory to create timing event!\n"));
            dwRetVal = EC_E_NOMEMORY;
            goto Exit;
        }

        if (pAppParms->bUseAuxClock)
        {
            dwRes = OsAuxClkInit(pAppParms->dwCpuIndex, 1000000 / pAppParms->dwBusCycleTimeUsec, AppContext.pvJobTaskEvent);
            if (EC_E_NOERROR != dwRes)
            {
                EcLogMsg(EC_LOG_LEVEL_CRITICAL, (pEcLogContext, EC_LOG_LEVEL_CRITICAL, "ERROR at auxiliary clock initialization: %s (0x%lx))\n", ecatGetText(dwRes), dwRes));
                dwRetVal = dwRes;
                goto Exit;
            }
        }
        else
        {
            OsCreateThread((EC_T_CHAR*)"tEcTimingTask", (EC_PF_THREADENTRY)tEcTimingTask, pAppParms->CpuSet, TIMER_THREAD_PRIO, LOG_THREAD_STACKSIZE, (EC_T_VOID*)&TimingDesc);
            while (!TimingDesc.bIsRunning)
            {
                OsSleep(1);
            }
        }
    }
#endif

    dwRes = EcDemoApp(pAppContext);
    if (EC_E_NOERROR != dwRes)
    {
        dwRetVal = dwRes;
        goto Exit;
    }
    /* no errors */
    dwRetVal = EC_E_NOERROR;

Exit:
    setEcMasterExit();

    if (EC_E_INVALIDPARM == dwRetVal)
    {
        ShowSyntax(pAppContext);
    }
    if (EC_E_NOERROR != dwRetVal)
    {
        OsSleep(5000);
    }

    /* stop timing task if running */
#if (defined INCLUDE_EC_MASTER_JOB_TASK)
    if (TimingDesc.bIsRunning)
    {
        TimingDesc.bShutdown = EC_TRUE;
        while (TimingDesc.bIsRunning)
        {
            OsSleep(1);
        }
    }
    /* clean up auxclk */
    if (pAppParms->bUseAuxClock)
    {
        OsAuxClkDeinit();
    }
    /* delete the timing event */
    if (EC_NULL != AppContext.pvJobTaskEvent)
    {
        OsDeleteEvent(AppContext.pvJobTaskEvent);
        AppContext.pvJobTaskEvent = EC_NULL;
    }
#endif
    EcLogMsg(EC_LOG_LEVEL_INFO, (pEcLogContext, EC_LOG_LEVEL_INFO, "%s stop.\n", EC_DEMO_APP_NAME));

    /* de-initialize message logging */
#if (defined INCLUDE_EC_LOGGING)
    if (bLogInitialized)
    {
        AppContext.LogParms.pfLogMsg = CAtEmLogging::LogMsgOsPrintf;
        AppContext.LogParms.pLogContext = EC_NULL;
        OsMemcpy(G_pEcLogParms, &AppContext.LogParms, sizeof(EC_T_LOG_PARMS));
        oLogging.DeinitLogging();
    }
#endif
    /* free app parameters */
    FreeAppParms(&AppContext, pAppParms);

    return (EC_E_NOERROR == dwRetVal) ? 0 : -1;
}

/*-END OF SOURCE FILE--------------------------------------------------------*/
