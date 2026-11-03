/*-----------------------------------------------------------------------------
 * Copyright    acontis technologies GmbH, Ravensburg, Germany
 * Response     Paul Bussmann
 * Description  Timer declaration
 *---------------------------------------------------------------------------*/

#ifndef INC_ECTIMER
#define INC_ECTIMER 1

/*-DEFINES-------------------------------------------------------------------*/
#define EC_TO_HELPER_STARTED        0x0001  /* set, if timer is started */
#define EC_TO_HELPER_STOPPED        0x0002  /* set, if timer is stopped */
#define EC_TO_HELPER_DEFAULT        0x0004  /* Default duration is set when timer is started with default value */
#define EC_TO_HELPER_ELAPSE_WRAP    0x0008  /* set, if elapse time is less then start time (if time will wrap) */

/*-CLASS---------------------------------------------------------------------*/
class CEcTimer
{
public:
    CEcTimer()
    {
        ResetInstance(0, EC_NULL);
    }
    CEcTimer(EC_T_DWORD dwDurationMsec)
    {
        ResetInstance(dwDurationMsec, EC_NULL);
    }
    CEcTimer(EC_T_DWORD dwDurationMsec, EC_T_DWORD* pdwMsecCounter)
    {
        ResetInstance(dwDurationMsec, pdwMsecCounter);
    }
    EC_T_VOID ResetInstance(EC_T_DWORD dwDurationMsec, EC_T_DWORD* pdwMsecCounter);

    EC_T_VOID  Start(EC_T_DWORD dwDurationMsec);
    EC_T_VOID  Start(EC_T_DWORD dwDurationMsec, EC_T_DWORD* pdwMsecCounter)
    {
        /* assert that an already set MsecCounter pointer is not reset */
        OsDbgAssert((EC_NULL == m_pdwMsecCounter) || (EC_NULL != pdwMsecCounter));

        m_pdwMsecCounter = pdwMsecCounter;
        Start(dwDurationMsec);
    }
    EC_T_VOID  Stop(EC_T_VOID);
    EC_T_BOOL  IsElapsed(EC_T_VOID);
    EC_T_BOOL  IsStarted(EC_T_VOID)        const { return (m_dwFlags & EC_TO_HELPER_STARTED); }
    EC_T_BOOL  IsStopped(EC_T_VOID)        const { return (m_dwFlags & EC_TO_HELPER_STOPPED); }
    EC_T_BOOL  IsDefaultTimeout(EC_T_VOID) const { return (m_dwFlags & EC_TO_HELPER_DEFAULT); }
    EC_T_VOID  SetDuration(EC_T_DWORD dwDurationMsec);
    EC_T_DWORD GetDuration(EC_T_VOID) const { return m_dwDurationMsec; }
    EC_T_DWORD GetRemainingTime(EC_T_VOID);
    EC_T_VOID  Restart(EC_T_VOID)
    {
        Start(GetDuration() + (IsDefaultTimeout() ? 0x80000000 : 0));
    }

private:
    EC_T_DWORD  m_dwStartTime;      /* millisecond counter when timer was started */
    EC_T_DWORD  m_dwDurationMsec;   /* Duration until timer elapses after start */
    EC_T_DWORD  m_dwTimeElapse;     /* millisecond counter when timer will elapse */
    EC_T_DWORD* m_pdwMsecCounter;   /* pointer to msec counter */
    EC_T_DWORD  m_dwFlags;          /* Flags */
};

#endif /* INC_ECTIMER */
