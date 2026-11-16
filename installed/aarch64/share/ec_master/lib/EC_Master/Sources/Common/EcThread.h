/** ---------------------------------------------------------------------------
 * \file
 * \brief       Thread class header
 * \author      Paul Bussmann
 * \copyright   acontis technologies GmbH, Weingarten, Germany
 *---------------------------------------------------------------------------*/

#ifndef INC_ECTHREAD
#define INC_ECTHREAD 1

/*-CLASS---------------------------------------------------------------------*/
class CEcThread
{
public:
    CEcThread(const struct _EC_T_LOG_PARMS* poLogParms = EC_NULL, EC_T_DWORD dwMemTrace = 0, struct _EC_T_MEMORY_LOGGER* poMemLog = EC_NULL);
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

    EC_T_VOID SetExceptionCallback(EC_PF_EXCEPTION_CALLBACK pfExceptionCallback, EC_T_VOID* pvCbParams);

    EC_INLINESTART EC_T_BOOL isTerminating() { return m_bThreadReady && m_bThreadStop; } EC_INLINESTOP

    EC_INLINESTART EC_T_LOG_PARMS* GetLogParms() { return m_poLogParms; } EC_INLINESTOP

    virtual EC_T_DWORD GetThisSize() /* EC_OVERRIDE */ { return sizeof(CEcThread); }

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
    /* Logging */
    EC_T_LOG_PARMS          m_oLogParmsStub;
    struct _EC_T_LOG_PARMS* m_poLogParms;

    /* Memory Logger */
    EC_T_DWORD                  m_dwMemTrace;
    struct _EC_T_MEMORY_LOGGER* m_poMemLog;
    EC_INLINESTART struct _EC_T_MEMORY_LOGGER* GetMemLog() { return m_poMemLog; } EC_INLINESTOP

    EC_PF_THREADENTRY m_pfThreadEntry;
    EC_T_VOID* m_pvParams;
    EC_PF_EXCEPTION_CALLBACK m_pfExceptionCallback;
    EC_T_VOID* m_pvCbParams;

private:
    EC_T_PVOID m_hThread; /* thread handle */

    EC_T_BOOL m_bThreadStop;    /* indicates that thread should be stopped */
    EC_T_BOOL m_bThreadReady;   /* indicates that thread was really started and ready for operation */
    EC_T_CHAR* m_pszName;
};

#endif /* INC_ECTHREAD */
