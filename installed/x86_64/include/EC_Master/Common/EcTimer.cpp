/*-----------------------------------------------------------------------------
 * Copyright    acontis technologies GmbH, Ravensburg, Germany
 * Response     Paul Bussmann
 * Description  Timer definition
 *---------------------------------------------------------------------------*/

/*-LOGGING-------------------------------------------------------------------*/
#define pEcLogParms G_pLogParmsEcTimer

/*-INCLUDES------------------------------------------------------------------*/
#include "EcOs.h"
#include "EcTimer.h"
#include "EcLog.h"

/*-LOGGING-------------------------------------------------------------------*/
static EC_T_LOG_PARMS  S_oLogParmsEcTimer = { EC_LOG_LEVEL_SILENT, EC_NULL, EC_NULL };
       EC_T_LOG_PARMS* G_pLogParmsEcTimer = &S_oLogParmsEcTimer;

extern "C" EC_T_VOID SetLogContextEcTimer(EC_T_LOG_PARMS* pLogParms)
{
    OsMemcpy(G_pLogParmsEcTimer, pLogParms, sizeof(EC_T_LOG_PARMS));
}

/*-GLOBAL VARIABLES-----------------------------------------------------------*/
#ifdef DEBUG
/* for debugging purposes, disable all Durations */
EC_T_BOOL G_bEcatDisableAllDurations = EC_FALSE;
#endif

/*-CLASS FUNCTIONS-----------------------------------------------------------*/
EC_T_VOID CEcTimer::ResetInstance(EC_T_DWORD dwDurationMsec, EC_T_DWORD* pdwMsecCounter)
{
    /* member variables will be reset by memset() from outside! */
    m_dwStartTime = 0;
    m_dwDurationMsec = 0;
    m_dwTimeElapse = 0;
    m_pdwMsecCounter = pdwMsecCounter;
    m_dwFlags = 0;
    if (dwDurationMsec > 0)
    {
        Start(dwDurationMsec);
    }
}

EC_T_VOID CEcTimer::Stop(EC_T_VOID)
{
    m_dwFlags |= EC_TO_HELPER_STOPPED;
    m_dwFlags &= ~EC_TO_HELPER_STARTED;
}

EC_T_VOID CEcTimer::Start(EC_T_DWORD dwDurationMsec)
{
    SetDuration(dwDurationMsec);

    if (m_pdwMsecCounter != EC_NULL)
    {
        m_dwStartTime = *m_pdwMsecCounter;
    }
    else
    {
        m_dwStartTime = OsQueryMsecCount();
    }

    m_dwTimeElapse = m_dwStartTime + m_dwDurationMsec; /* time when timer will elapse */
    if (m_dwTimeElapse < m_dwStartTime)
    {
        m_dwFlags |= EC_TO_HELPER_ELAPSE_WRAP;
    }
    else
    {
        m_dwFlags &= ~EC_TO_HELPER_ELAPSE_WRAP;
    }

    m_dwFlags |= EC_TO_HELPER_STARTED;
    m_dwFlags &= ~EC_TO_HELPER_STOPPED;
}

EC_T_BOOL CEcTimer::IsElapsed(EC_T_VOID)
{
    EC_T_BOOL  bTimeElapsed = EC_FALSE;

#ifdef DEBUG
    if (G_bEcatDisableAllDurations)
    {
#ifdef INCLUDE_LOG_MESSAGES
        static EC_T_BOOL s_bMsgShown = EC_FALSE;
        if (!s_bMsgShown)
        {
            s_bMsgShown = EC_TRUE;
            EcLogMsg(EC_LOG_LEVEL_INFO, (pEcLogContext, EC_LOG_LEVEL_INFO, "*******************************************\n"));
            EcLogMsg(EC_LOG_LEVEL_INFO, (pEcLogContext, EC_LOG_LEVEL_INFO, "*******************************************\n"));
            EcLogMsg(EC_LOG_LEVEL_INFO, (pEcLogContext, EC_LOG_LEVEL_INFO, "DEBUG SETTING: All Durations are disabled!!!\n"));
            EcLogMsg(EC_LOG_LEVEL_INFO, (pEcLogContext, EC_LOG_LEVEL_INFO, "*******************************************\n"));
            EcLogMsg(EC_LOG_LEVEL_INFO, (pEcLogContext, EC_LOG_LEVEL_INFO, "*******************************************\n"));
        }
#endif /* INCLUDE_LOG_MESSAGES */
        return EC_FALSE;
    }
#endif
    if (m_dwFlags & EC_TO_HELPER_STARTED)
    {
    EC_T_DWORD dwMsecCountCurr = 0;

        if (m_pdwMsecCounter != EC_NULL)
        {
            dwMsecCountCurr =  *m_pdwMsecCounter;
        }
        else
        {
            dwMsecCountCurr =  OsQueryMsecCount();
        }
        if (m_dwFlags & EC_TO_HELPER_ELAPSE_WRAP)
        {
            if ((dwMsecCountCurr < m_dwStartTime) && (dwMsecCountCurr >= m_dwTimeElapse))
            {
                bTimeElapsed = EC_TRUE;
            }
        }
        else
        {
            if (dwMsecCountCurr < m_dwStartTime)
            {
                bTimeElapsed = EC_TRUE;
            }
            if (dwMsecCountCurr >= m_dwTimeElapse)
            {
                bTimeElapsed = EC_TRUE;
            }
        }
    }
    return bTimeElapsed;
}

EC_T_VOID CEcTimer::SetDuration(EC_T_DWORD dwDurationMsec)
{
    if (dwDurationMsec & 0x80000000)
    {
        m_dwFlags |= EC_TO_HELPER_DEFAULT;
        dwDurationMsec = dwDurationMsec & ~0x80000000;
    }
    else
    {
        m_dwFlags &= ~EC_TO_HELPER_DEFAULT;
    }
    m_dwDurationMsec = dwDurationMsec;
}

EC_T_DWORD CEcTimer::GetRemainingTime(EC_T_VOID)
{
    EC_T_DWORD dwMsecCountCurr = 0;

    if (IsElapsed() || !IsStarted())
    {
        return 0;
    }

    if (m_pdwMsecCounter != EC_NULL)
    {
        dwMsecCountCurr = *m_pdwMsecCounter;
    }
    else
    {
        dwMsecCountCurr = OsQueryMsecCount();
    }

    return m_dwTimeElapse - dwMsecCountCurr;
}

/*-END OF SOURCE FILE--------------------------------------------------------*/
