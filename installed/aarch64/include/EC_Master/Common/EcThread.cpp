/*-----------------------------------------------------------------------------
 * Copyright                acontis technologies GmbH, Ravensburg, Germany
 * Response                 Paul Bussmann
 * Description              Thread class implementation
 *---------------------------------------------------------------------------*/

/*-LOGGING-------------------------------------------------------------------*/
#ifndef pEcLogParms
#define pEcLogParms (GetLogParms())
#endif

/*-INCLUDES------------------------------------------------------------------*/
#include "EcOs.h"
#include "EcThread.h"
#include "EcTimer.h"
#include "EcLog.h"
#include "EcError.h"

/*-CLASS FUNCTIONS-----------------------------------------------------------*/
CEcThread::CEcThread()
    : m_pfThreadEntry(EC_NULL)
    , m_pvParams(EC_NULL)
    , m_hThread(EC_NULL)
    , m_bThreadStop(EC_FALSE)
    , m_bThreadReady(EC_FALSE)
    , m_pszName(EC_NULL)
{
    OsMemset(&m_oLogParms, 0, sizeof(EC_T_LOG_PARMS));
}

CEcThread::~CEcThread()
{
    /* thread should be stopped */
    OsDbgAssert(isStopped());
    SafeOsFree(m_pszName);
    OsDbgAssert(m_hThread == EC_NULL);
}

EC_T_DWORD CEcThread::Start(EC_T_LOG_PARMS* pLogParms, EC_PF_THREADENTRY pfThreadEntry, EC_T_VOID* pvParams, const EC_T_CHAR* szThreadName,
    EC_T_CPUSET cpuIstCpuAffinityMask, EC_T_DWORD dwPrio, EC_T_DWORD dwStackSize, EC_T_DWORD dwTimeout)
{
    EC_T_DWORD dwRetVal = EC_E_NOERROR;
    CEcTimer startTimeout;

    if (EC_NULL == pLogParms)
    {
        return EC_E_INVALIDPARM;
    }
    if ((EC_NULL != m_hThread) || (EC_NULL != m_pszName))
    {
        return EC_E_INVALIDSTATE;
    }

    if (&m_oLogParms != pLogParms)
    {
        OsMemcpy(&m_oLogParms, pLogParms, sizeof(EC_T_LOG_PARMS));
    }

    setThreadProc(pfThreadEntry, pvParams);

    m_pszName = (EC_T_CHAR*)OsMalloc(OsStrlen(szThreadName) + 1);
    if (EC_NULL != m_pszName)
    {
        OsStrcpy(m_pszName, szThreadName);
    }

    m_bThreadStop = EC_FALSE;
    m_hThread = OsCreateThread(szThreadName, (EC_PF_THREADENTRY)threadProc, cpuIstCpuAffinityMask, dwPrio, dwStackSize, this);

    if (EC_NULL == m_hThread)
    {
        /* reset thread entry */
        setThreadProc(EC_NULL, EC_NULL);

        dwRetVal = EC_E_NOMEMORY;
        goto Exit;
    }

    for (startTimeout.Start(dwTimeout); !startTimeout.IsElapsed(); )
    {
        if (isReady())
        {
            break;
        }
        OsSleep(1);
    }
    if (!isReady())
    {
        dwRetVal = EC_E_TIMEOUT;
    }

Exit:
    return dwRetVal;
}

EC_T_DWORD CEcThread::Stop(EC_T_DWORD dwTimeout)
{
    EC_T_DWORD dwRetVal = EC_E_NOERROR;
    CEcTimer stopTimeout;
    CEcTimer stopTimeoutWarnTimout(20000);

    if (m_hThread == EC_NULL)
    {
        return EC_E_NOERROR;
    }

    stopThread();

    if (EC_NOWAIT != dwTimeout)
    {
        if (EC_WAITINFINITE != dwTimeout)
        {
            stopTimeout.Start(dwTimeout);
        }
        while (!isStopped() && !stopTimeout.IsElapsed())
        {
            if (stopTimeoutWarnTimout.IsElapsed())
            {
                stopTimeoutWarnTimout.Stop();
                EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "Stopping thread%s%s takes longer than 20s!\n", (m_pszName ? " ": ""), m_pszName));
            }
            OsSleep(1);
        }
        if (!isStopped())
        {
            dwRetVal = EC_E_TIMEOUT;
            EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "Time-out stopping thread%s%s!\n", (m_pszName ? " ": ""), m_pszName));
        }
    }

    if (m_hThread)
    {
        OsDeleteThreadHandle(m_hThread);
    }
    m_hThread = EC_NULL;

    SafeOsFree(m_pszName);

    return dwRetVal;
}

EC_T_DWORD CEcThread::threadProc(EC_T_PVOID pvParams)
{
    CEcThread* pThis = (CEcThread*)pvParams;
    pThis->m_bThreadReady = EC_TRUE;

    while (!pThis->m_bThreadStop)
    {
        pThis->m_pfThreadEntry(pThis->m_pvParams);
    }

    /* indicate that stopped */
    pThis->setThreadProc(EC_NULL, EC_NULL);

    return 0;
}

EC_T_VOID CEcThread::setThreadProc(EC_PF_THREADENTRY pfThreadEntry, EC_T_VOID* pvParams)
{
    m_pfThreadEntry = pfThreadEntry;
    m_pvParams = pvParams;
}

/*-END OF SOURCE FILE--------------------------------------------------------*/
