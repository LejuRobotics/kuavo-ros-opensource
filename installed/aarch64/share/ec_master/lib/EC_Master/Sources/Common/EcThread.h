/*-----------------------------------------------------------------------------
 * Copyright                acontis technologies GmbH, Ravensburg, Germany
 * Response                 Paul Bussmann
 * Description              Thread class header
 *---------------------------------------------------------------------------*/

#ifndef INC_ECTHREAD
#define INC_ECTHREAD 1

/*-INCLUDES------------------------------------------------------------------*/
#ifndef INC_ECLOG
#include "EcLog.h"
#endif

/*-CLASS---------------------------------------------------------------------*/
class CEcThread
{
public:
    CEcThread();
    virtual ~CEcThread();

    EC_T_DWORD Start(EC_T_LOG_PARMS* pLogParms, EC_PF_THREADENTRY pfThreadEntry, EC_T_VOID* pvParams,
        const EC_T_CHAR* szThreadName, EC_T_CPUSET oCpuAffinityMask, EC_T_DWORD dwPrio,
        EC_T_DWORD dwStackSize, EC_T_DWORD dwTimeout);

    EC_T_DWORD Start(EC_T_LOG_PARMS* pLogParms, EC_PF_THREADENTRY pfThreadEntry, EC_T_VOID* pvParams,
        const EC_T_CHAR* szThreadName, EC_T_DWORD dwPrio, EC_T_DWORD dwStackSize, EC_T_DWORD dwTimeout)
    {
        EC_T_CPUSET oCpuAffinityMask;
        EC_CPUSET_ZERO(oCpuAffinityMask);
        return Start(pLogParms, pfThreadEntry, pvParams, szThreadName, oCpuAffinityMask, dwPrio,dwStackSize, dwTimeout);
    }

    EC_T_DWORD Stop(EC_T_DWORD dwTimeout = EC_NOWAIT);
    EC_INLINESTART EC_T_BOOL isTerminating() { return m_bThreadReady && m_bThreadStop; } EC_INLINESTOP

    EC_INLINESTART EC_T_LOG_PARMS* GetLogParms() { return &m_oLogParms; } EC_INLINESTOP

protected:
    /* threadProc is run in separate thread and calls listenStep while thread is not stopped */
    static EC_T_DWORD EC_FNCALL threadProc(EC_T_PVOID pvParams);

    EC_INLINESTART EC_T_BOOL isStopped(EC_T_VOID) const { return (EC_NULL == m_pfThreadEntry) && (EC_NULL == m_pvParams); } EC_INLINESTOP
    EC_INLINESTART EC_T_BOOL isReady(EC_T_VOID)   const { return m_bThreadReady; } EC_INLINESTOP

    EC_T_VOID stopThread(EC_T_VOID) { m_bThreadStop = EC_TRUE; }

    EC_T_VOID setThreadProc(EC_PF_THREADENTRY pfThreadEntry, EC_T_VOID* pvParams);

private:
    /* explicitly restrict copy */
    CEcThread(const CEcThread&);
    CEcThread& operator=(const CEcThread&);

protected:
    EC_T_LOG_PARMS m_oLogParms;
    EC_PF_THREADENTRY m_pfThreadEntry;
    EC_T_VOID* m_pvParams;

private:
    EC_T_PVOID m_hThread; /* thread handle */

    EC_T_BOOL m_bThreadStop;    /* indicates that thread should be stopped */
    EC_T_BOOL m_bThreadReady;   /* indicates that thread was really started and ready for operation */
    EC_T_CHAR* m_pszName;
};

#endif /* INC_ECTHREAD */
