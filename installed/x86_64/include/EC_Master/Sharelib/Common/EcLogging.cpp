/*-----------------------------------------------------------------------------
 * EcLogging.cpp
 * Copyright                acontis technologies GmbH, Ravensburg, Germany
 * Response                 Stefan Zintgraf
 * Description              EtherCAT Example Logging
 *---------------------------------------------------------------------------*/

/*-INCLUDES------------------------------------------------------------------*/
#include "EcDemoApp.h"

/*-MACROS--------------------------------------------------------------------*/
/*#define NOPRINTF    1*/

/*-DEFINES-------------------------------------------------------------------*/
#if (defined EC_VERSION_RTX) || (defined EC_VERSION_INTIME)
    #define ABSOLUTE_LOG_FILE_PATH       "C:\\"
#else
    #define ABSOLUTE_LOG_FILE_PATH       ""
#endif

#if (defined EC_SIMULATOR)
#if (defined INCLUDE_ATEMRAS)
#ifdef FILESYS_8_3
#define DEFAULT_PREFIX "esr"
#else
#define DEFAULT_PREFIX "ecsimulator_r"
#endif
#else
#ifdef FILESYS_8_3
#define DEFAULT_PREFIX "es"
#else
#define DEFAULT_PREFIX "ecsimulator"
#endif
#endif
#elif (defined INCLUDE_EC_MONITOR)
#ifdef FILESYS_8_3
#define DEFAULT_PREFIX "em"
#else
#define DEFAULT_PREFIX "ecmonitor"
#endif
#else
#if (defined INCLUDE_ATEMRAS)
#ifdef FILESYS_8_3
#define DEFAULT_PREFIX "emr"
#else
#define DEFAULT_PREFIX "ecmaster_r"
#endif
#else
#ifdef FILESYS_8_3
#define DEFAULT_PREFIX "em"
#else
#define DEFAULT_PREFIX "ecmaster"
#endif
#endif
#endif

/* timestamp max. 10 digits, format "%010d: " */
#define LOG_MSG_OFFSET_AFTER_TIMESTAMP 12

/*-GLOBAL VARIABLES-----------------------------------------------------------*/
#ifndef MAX_NUMOF_MASTER_INSTANCES
#define MAX_NUMOF_MASTER_INSTANCES 1
#endif
EC_T_LOG_PARMS G_aLogParms[MAX_NUMOF_MASTER_INSTANCES];

#ifdef INCLUDE_FRAME_SPY
CFrameLogMultiplexer* CFrameLogMultiplexer::G_aLogMultiplexer = EC_NULL;
EC_T_BOOL CFrameLogMultiplexer::G_bAutoDispose = EC_FALSE;
#endif

#if (defined INCLUDE_FILE_LOGGING)
#if (defined EC_VERSION_FREERTOS) || (defined EC_VERSION_VXWORKS) || (defined EC_VERSION_TKERNEL)\
    || (defined EC_VERSION_SYSBIOS) || (defined EC_VERSION_RIN32) || (defined EC_VERSION_XILINX_STANDALONE)\
    || (defined EC_VERSION_ETKERNEL) || (defined EC_VERSION_RTXC) || (defined EC_VERSION_RZT1) || (defined EC_VERSION_RZGNOOS) || (defined EC_VERSION_ECOS)\
    || (defined EC_VERSION_JSLWARE) || (defined EC_VERSION_UC3) || (defined EC_VERSION_UCOS) || (defined EC_VERSION_XMC)
EC_T_BOOL bLogFileEnb = EC_FALSE;
#else
EC_T_BOOL bLogFileEnb = EC_TRUE;
#endif
#endif /* INCLUDE_FILE_LOGGING */

EC_T_BOOL CAtEmLogging::s_bLogParmsArrayInitialized = EC_FALSE;

/*-CLASS FUNCTIONS-----------------------------------------------------------*/
/***************************************************************************************************/
/**
\brief  Create CAtEmLogging instance

\return -
*/
CAtEmLogging::CAtEmLogging(EC_T_VOID)
{
    m_dwInstanceId = 0xffff;
    m_pvLogThreadObj    = EC_NULL;
    m_bLogTaskRunning = EC_FALSE;
    m_bShutdownLogTask = EC_FALSE;
    m_bSettling = EC_FALSE;
    m_dwNumMsgsSinceMsrmt = 0;
    m_poInsertMsgLock = EC_NULL;
    m_poProcessMsgLock = EC_NULL;
    m_pchTempbuffer = EC_NULL;
    m_pFirstMsgBufferDesc = EC_NULL;
    m_pLastMsgBufferDesc = EC_NULL;
    m_pAllMsgBufferDesc = EC_NULL;
    m_pErrorMsgBufferDesc = EC_NULL;
    m_pDcmMsgBufferDesc = EC_NULL;
    m_pHistMsgBufferDesc = EC_NULL;
#if (defined INCLUDE_FILE_LOGGING)
    m_pchLogDir[0] = '\0';
    m_pchLogDir[MAX_PATH_LEN - 1] = '\0';
#endif

#if (!defined EC_EAP)
    m_dwPerfMeasNumOf = 0;
    m_aPerfMeasVal = EC_NULL;
    m_aPerfMeasInfo = EC_NULL;
    OsMemset(m_aData, 0, sizeof(m_aData));
#endif

    if (!s_bLogParmsArrayInitialized)
    {
        OsMemset(G_aLogParms, 0, MAX_NUMOF_MASTER_INSTANCES * sizeof(EC_T_LOG_PARMS));
        s_bLogParmsArrayInitialized = EC_TRUE;
    }
}

/********************************************************************************/
/** \brief Initialize logging
*
* \return N/A
*/
EC_T_VOID CAtEmLogging::InitLogging(
    EC_T_DWORD  dwInstanceId,
    EC_T_WORD   wRollOver,
    EC_T_DWORD  dwPrio,
    EC_T_CPUSET CpuSet,
    EC_T_CHAR*  szFilenamePrefix,
    EC_T_DWORD  dwStackSize,
    EC_T_DWORD  dwLogMsgBufferSize,
    EC_T_DWORD  dwErrMsgBufferSize,
    EC_T_DWORD  dwDcmMsgBufferSize
    )
{
    EC_T_CHAR   szLogFilename[256];

#if (!defined INCLUDE_FILE_LOGGING)
    EC_UNREFPARM(wRollOver);
#endif
    m_dwInstanceId = dwInstanceId;
    m_poInsertMsgLock = OsCreateLockTyped(eLockType_SPIN);
    m_poProcessMsgLock = OsCreateLock();
    m_pchTempbuffer = (EC_T_CHAR*)OsMalloc(MAX_MESSAGE_SIZE);
    if (EC_NULL == m_pchTempbuffer)
    {
        /* TODO: EC_E_NOMEMORY */
    }
    OsMemset(m_pchTempbuffer, 0, MAX_MESSAGE_SIZE);

    if ((EC_NULL == szFilenamePrefix) || ('\0' == szFilenamePrefix[0]))
    {
        szFilenamePrefix = (EC_T_CHAR*)DEFAULT_PREFIX;
    }
    OsSnprintf(szLogFilename, sizeof(szLogFilename) - 1, "%s_err", szFilenamePrefix);

    m_pErrorMsgBufferDesc = (MSG_BUFFER_DESC*)AddLogBuffer(
                                                    dwInstanceId,
#if (defined INCLUDE_FILE_LOGGING)
                                                    wRollOver,
#endif
                                                    dwErrMsgBufferSize,
                                                    EC_TRUE,            /* skip duplicates */
                                                    (EC_T_CHAR*)"Err",  /* name of the logging (identification) */
#if (defined INCLUDE_FILE_LOGGING)
                                                    szLogFilename,
                                                    (EC_T_CHAR*)"log",  /* log file extension */
#endif
                                                    EC_FALSE,           /* print message on console? */
                                                    EC_TRUE );          /* logging with time stamp? */
    if (EC_NULL == m_pErrorMsgBufferDesc)
    {
        /* TODO: EC_E_NOMEMORY */
    }

    OsSnprintf(szLogFilename, sizeof(szLogFilename) - 1, "%s", szFilenamePrefix);

    m_pAllMsgBufferDesc = (MSG_BUFFER_DESC*)AddLogBuffer(
                                                    dwInstanceId,
#if (defined INCLUDE_FILE_LOGGING)
                                                    wRollOver,
#endif
                                                    dwLogMsgBufferSize,
                                                    EC_TRUE,            /* skip duplicates */
                                                    (EC_T_CHAR*)"Log",  /* name of the logging (identification) */
#if (defined INCLUDE_FILE_LOGGING)
                                                    szLogFilename,
                                                    (EC_T_CHAR*)"log",  /* log file extension */
#endif
                                                    EC_TRUE,            /* bPrintConsole */
                                                    EC_TRUE );          /* logging with time stamp? */
    if (EC_NULL == m_pAllMsgBufferDesc)
    {
        /* TODO: EC_E_NOMEMORY */
    }

    OsSnprintf(szLogFilename, sizeof(szLogFilename) - 1, "%s_dcm", szFilenamePrefix);

    m_pDcmMsgBufferDesc = (MSG_BUFFER_DESC*)AddLogBuffer(
                                                    dwInstanceId,
#if (defined INCLUDE_FILE_LOGGING)
                                                    wRollOver,
#endif
                                                    dwDcmMsgBufferSize,
                                                    EC_FALSE,           /* do not skip duplicates */
                                                    (EC_T_CHAR*)"DCM",  /* name of the logging (identification) */
#if (defined INCLUDE_FILE_LOGGING)
                                                    szLogFilename,
                                                    (EC_T_CHAR*)"csv",  /* log file extension */
#endif
                                                    EC_FALSE,           /* print message on console? */
                                                    EC_FALSE );         /* logging with time stamp? */
    if (EC_NULL == m_pDcmMsgBufferDesc)
    {
        /* TODO: EC_E_NOMEMORY */
    }

    OsDbgAssert(!m_bLogTaskRunning);
    m_bShutdownLogTask = EC_FALSE;

#if (defined INCLUDE_LOG_TASK)
        m_pvLogThreadObj = OsCreateThread((EC_T_CHAR*)"tAtEmLog", tAtEmLogWrapper, CpuSet, dwPrio, dwStackSize, this);
    while (!m_bLogTaskRunning) OsSleep(1);
#else
    EC_UNREFPARM(dwStackSize);
    EC_UNREFPARM(CpuSet);
    EC_UNREFPARM(dwPrio);
#endif /* INCLUDE_LOG_TASK */
}

/********************************************************************************/
/** \brief Initialize logging
*
* \return message buffer descriptor
*/
struct _MSG_BUFFER_DESC* CAtEmLogging::AddLogBuffer(
    EC_T_DWORD  dwInstanceId,
#if (defined INCLUDE_FILE_LOGGING)
    EC_T_WORD   wRollOver,
#endif
    EC_T_DWORD  dwBufferSize,       /* [in]  buffer size (number of buffered messages) */
    EC_T_BOOL   bSkipDuplicates,    /* [in]  EC_TRUE if duplicate messages shall be skipped */
    EC_T_CHAR*  szLogName,          /* [in]  name of the logging (identification) */
#if (defined INCLUDE_FILE_LOGGING)
    EC_T_CHAR*  szLogFilename,      /* [in]  log filename */
    EC_T_CHAR*  szLogFileExt,       /* [in]  log file extension */
#endif
    EC_T_BOOL   bPrintConsole,      /* [in]  print message on console? */
    EC_T_BOOL   bPrintTimestamp     /* [in]  logging with time stamp? */
    )
{
#if (defined INCLUDE_FILE_LOGGING)
    EC_T_CHAR   szLogFileNameTmp[MAX_PATH_LEN];
#endif
    MSG_BUFFER_DESC* pNewMsgBufferDesc = EC_NULL;
    EC_T_BOOL bLocked = EC_FALSE;
    EC_T_BOOL bOk = EC_FALSE;

    OsLock(m_poProcessMsgLock);
    bLocked = EC_TRUE;

    /* create buffers */
    pNewMsgBufferDesc = (MSG_BUFFER_DESC*)OsMalloc(sizeof(MSG_BUFFER_DESC));
    if (EC_NULL == pNewMsgBufferDesc)
    {
        /* TODO: EC_E_NOMEMORY */
        goto Exit;
    }
    OsMemset(pNewMsgBufferDesc, 0, sizeof(MSG_BUFFER_DESC));

#if (defined INCLUDE_FILE_LOGGING)
    if (m_pchLogDir[0] != '\0')
    {
        OsSnprintf(szLogFileNameTmp, sizeof(szLogFileNameTmp)-1, "%s%s%d", m_pchLogDir, szLogFilename, dwInstanceId);
    }
    else
    {
        OsSnprintf(szLogFileNameTmp, sizeof(szLogFileNameTmp)-1, "%s%s%d", ABSOLUTE_LOG_FILE_PATH, szLogFilename, dwInstanceId);
    }
#else
    EC_UNREFPARM(dwInstanceId);
#endif
    bOk = InitMsgBuffer( pNewMsgBufferDesc,
                         MAX_MESSAGE_SIZE,
                         dwBufferSize,
                         bSkipDuplicates,
                         bPrintConsole,
                         bPrintTimestamp,
#if (defined INCLUDE_FILE_LOGGING)
                         szLogFileNameTmp,
                         szLogFileExt,
                         wRollOver,
#endif
                         szLogName
                        );
    if (!bOk)
    {
        goto Exit;
    }

    /* link buffer together */
    if (EC_NULL == m_pLastMsgBufferDesc)
    {
        /* create first buffer */
        OsDbgAssert(m_pFirstMsgBufferDesc == EC_NULL);
        m_pFirstMsgBufferDesc = m_pLastMsgBufferDesc = pNewMsgBufferDesc;
        pNewMsgBufferDesc->pNextMsgBuf = EC_NULL;
    }
    else
    {
        /* append to last buffer */
        OsDbgAssert(m_pLastMsgBufferDesc->pNextMsgBuf == EC_NULL);
        m_pLastMsgBufferDesc->pNextMsgBuf = pNewMsgBufferDesc;
        m_pLastMsgBufferDesc = pNewMsgBufferDesc;
    }
    bOk = EC_TRUE;

Exit:
    if (!bOk)
    {
        if (pNewMsgBufferDesc!=EC_NULL) OsFree(pNewMsgBufferDesc);
        pNewMsgBufferDesc = EC_NULL;
    }

    if (bLocked)
        OsUnlock(m_poProcessMsgLock);
    return pNewMsgBufferDesc;
}


/********************************************************************************/
/** \brief set log message buffer
*
* \return N/A
*/
EC_T_VOID CAtEmLogging::SetMsgBuf(
    MSG_BUFFER_DESC* pMsgBufferDesc,
    EC_T_BYTE* pbyLogMem,
    EC_T_DWORD dwSize )
{
    pMsgBufferDesc->pbyNextLogMsg = pbyLogMem;
    pMsgBufferDesc->dwLogMemorySize = dwSize;
    pMsgBufferDesc->bLogBufferFull = EC_FALSE;
    pMsgBufferDesc->bNewLine = EC_FALSE;
    OsMemset(pbyLogMem, 0, dwSize);
    pMsgBufferDesc->pbyLogMemory = pbyLogMem;   /* initialize last as it will be active immediately after! */
}
EC_T_VOID CAtEmLogging::SetLogMsgBuf( EC_T_BYTE* pbyLogMem, EC_T_DWORD dwSize )
{
    SetMsgBuf( m_pAllMsgBufferDesc, pbyLogMem, dwSize );
}
EC_T_VOID CAtEmLogging::SetLogErrBuf( EC_T_BYTE* pbyLogMem, EC_T_DWORD dwSize )
{
    SetMsgBuf( m_pErrorMsgBufferDesc, pbyLogMem, dwSize );
}
EC_T_VOID CAtEmLogging::SetLogDcmBuf( EC_T_BYTE* pbyLogMem, EC_T_DWORD dwSize )
{
    SetMsgBuf( m_pDcmMsgBufferDesc, pbyLogMem, dwSize );
}

/********************************************************************************/
/** \brief De-initialize logging
*
* \return N/A
*/
EC_T_VOID CAtEmLogging::DeinitLogging(EC_T_VOID)
{
    MSG_BUFFER_DESC* pCurrMsgBuf = EC_NULL;
    MSG_BUFFER_DESC* pNextMsgBuf = EC_NULL;

    m_bShutdownLogTask = EC_TRUE;

    while (m_bLogTaskRunning) OsSleep(1);

    /* shutdown all message buffers */
    pNextMsgBuf = m_pFirstMsgBufferDesc;
    while (EC_NULL != pNextMsgBuf)
    {
        DeinitMsgBuffer(pNextMsgBuf);
        pNextMsgBuf = pNextMsgBuf->pNextMsgBuf;
    }

    if (EC_NULL != m_pvLogThreadObj)
    {
        OsDeleteThreadHandle(m_pvLogThreadObj);
        m_pvLogThreadObj = EC_NULL;
    }

    /* free all message buffers */
    pNextMsgBuf = m_pFirstMsgBufferDesc;
    while (EC_NULL != pNextMsgBuf)
    {
        pCurrMsgBuf = pNextMsgBuf;
        pNextMsgBuf = pCurrMsgBuf->pNextMsgBuf;
        OsFree(pCurrMsgBuf);
    }
    /* unlink buffers */
    m_pFirstMsgBufferDesc = EC_NULL;
    m_pLastMsgBufferDesc = EC_NULL;
    m_pAllMsgBufferDesc = EC_NULL;
    m_pErrorMsgBufferDesc = EC_NULL;
    m_pDcmMsgBufferDesc = EC_NULL;

    OsDeleteLock(m_poInsertMsgLock);
    OsDeleteLock(m_poProcessMsgLock);
    SafeOsFree(m_pchTempbuffer);
    m_pchTempbuffer = EC_NULL;

   /* delete performance measurement buffers */
#if (!defined EC_EAP)
    SafeOsFree(m_aPerfMeasVal);
    SafeOsFree(m_aPerfMeasInfo);
    m_dwPerfMeasNumOf = 0;
#endif
}

/********************************************************************************/
/** \brief logging thread
*
* \return N/A
*/
EC_T_VOID CAtEmLogging::tAtEmLogWrapper(EC_T_VOID* pvParm)
{
    CAtEmLogging *pInst = (CAtEmLogging*)pvParm;

    OsDbgAssert(EC_NULL != pInst);
    if (pInst)
    {
        pInst->tAtEmLog(EC_NULL);
    }
}

/********************************************************************************/
/** \brief cyclically process all messages
*
* \return N/A
*/
EC_T_VOID CAtEmLogging::tAtEmLog(EC_T_VOID* pvParm)
{
    EC_UNREFPARM(pvParm);

    m_bLogTaskRunning = EC_TRUE;
    while (!m_bShutdownLogTask)
    {
        ProcessAllMsgs();
        OsSleep(1);
    }
    ProcessAllMsgs();
    m_bLogTaskRunning = EC_FALSE;
#if (defined EC_VERSION_RTEMS)
    rtems_task_delete(RTEMS_SELF);
#endif
}

EC_T_VOID CAtEmLogging::ProcessAllMsgs(EC_T_VOID)
{
    for (MSG_BUFFER_DESC* pNextMsgBuf = m_pFirstMsgBufferDesc; EC_NULL != pNextMsgBuf; pNextMsgBuf = pNextMsgBuf->pNextMsgBuf)
    {
        ProcessMsgs(pNextMsgBuf);
    }
}

/********************************************************************************/
/** \brief Initialize message buffer
*
* \return N/A
*/
EC_T_BOOL CAtEmLogging::InitMsgBuffer
(MSG_BUFFER_DESC*   pMsgBufferDesc      /* [in]  pointer to message buffer descriptor */
,EC_T_DWORD         dwMsgSize           /* [in]  size of a single message */
,EC_T_DWORD         dwNumMsgs           /* [in]  number of messages */
,EC_T_BOOL          bSkipDuplicates     /* [in]  EC_TRUE if duplicate messages shall be skipped */
,EC_T_BOOL          bPrintConsole       /* [in]  print message on console? */
,EC_T_BOOL          bPrintTimestamp     /* [in]  logging with time stamp? */
#if (defined INCLUDE_FILE_LOGGING)
,EC_T_CHAR*         szMsgLogFileName    /* [in]  message log file name */
,EC_T_CHAR*         szMsgLogFileExt     /* [in]  message log file name */
,EC_T_WORD          wRollOver           /* [in]  roll over counter */
#endif
,EC_T_CHAR*         szBufferName        /* [in]  name of the logging buffer */
)
{
    EC_T_BOOL  bOk = EC_FALSE;
    EC_T_CHAR* pchMsgBuffer = EC_NULL;
    EC_T_DWORD dwBufSiz;
    EC_T_DWORD dwCnt;
#if (defined INCLUDE_FILE_LOGGING)
    EC_T_CHAR  szfileNameTemp[MAX_PATH_LEN] = {0};
#endif

    pMsgBufferDesc->dwMsgSize = dwMsgSize;
    pMsgBufferDesc->dwNumMsgs = dwNumMsgs;
    pMsgBufferDesc->bPrintTimestamp = bPrintTimestamp;
    pMsgBufferDesc->bPrintConsole = bPrintConsole;
    pMsgBufferDesc->dwNextEmptyMsgIndex = 0;
    pMsgBufferDesc->dwNextPrintMsgIndex = 0;
    pMsgBufferDesc->wEntryCounter       = 0;
    pMsgBufferDesc->pbyLogMemory = EC_NULL;
    pMsgBufferDesc->dwLogMemorySize = 0;
    pMsgBufferDesc->pbyNextLogMsg = EC_NULL;
    pMsgBufferDesc->bLogBufferFull = EC_FALSE;
    pMsgBufferDesc->bSkipDuplicateMessages = bSkipDuplicates;
    pMsgBufferDesc->dwNumDuplicates = 0;
    pMsgBufferDesc->pszLastMsg = EC_NULL;

    pMsgBufferDesc->paMsg = (LOG_MSG_DESC*)OsMalloc(dwNumMsgs*sizeof(LOG_MSG_DESC));
    if (pMsgBufferDesc->paMsg == EC_NULL)
    {
        OsPrintf("CAtEmLogging::InitMsgBuffer: cannot get memory for logging buffer '%s'\n", szBufferName);
        goto Exit;
    }
    OsMemset(pMsgBufferDesc->paMsg, 0, dwNumMsgs*sizeof(LOG_MSG_DESC));

    dwBufSiz = dwNumMsgs * (dwMsgSize + 1);
    pchMsgBuffer = (EC_T_CHAR*)OsMalloc(dwBufSiz);
    if (pchMsgBuffer == EC_NULL)
    {
        OsPrintf("CAtEmLogging::InitMsgBuffer: cannot get memory for logging buffer '%s'\n", szBufferName);
        goto Exit;
    }

    /* Same as below. Needed to prevent false positive from static code analysis. */
    pMsgBufferDesc->paMsg[0].szMsgBuffer = pchMsgBuffer;

    OsMemset(pchMsgBuffer,0,dwBufSiz);
    for( dwCnt=0; dwCnt < dwNumMsgs; dwCnt++ )
    {
        pMsgBufferDesc->paMsg[dwCnt].szMsgBuffer = &pchMsgBuffer[dwCnt*(dwMsgSize+1)];

        if (pMsgBufferDesc->bPrintTimestamp)
        {
            pMsgBufferDesc->paMsg[dwCnt].szMsg = &pchMsgBuffer[dwCnt*(dwMsgSize+1) + LOG_MSG_OFFSET_AFTER_TIMESTAMP];
        }
        else
        {
            pMsgBufferDesc->paMsg[dwCnt].szMsg = &pchMsgBuffer[dwCnt*(dwMsgSize+1)];
        }
    }

#if (defined INCLUDE_FILE_LOGGING)
    pMsgBufferDesc->wLogFileIndex = 0;
    pMsgBufferDesc->wEntryCounterLimit = wRollOver;
    OsStrncpy(pMsgBufferDesc->szMsgLogFileName, szMsgLogFileName, MAX_PATH_LEN - 1);
    pMsgBufferDesc->szMsgLogFileName[MAX_PATH_LEN - 1] = '\0';
    OsStrncpy(pMsgBufferDesc->szMsgLogFileExt,  szMsgLogFileExt,  MAX_EXT_LEN - 1);
    pMsgBufferDesc->szMsgLogFileExt[MAX_EXT_LEN - 1] = '\0';

    if (0 != pMsgBufferDesc->wEntryCounterLimit )
    {
#ifdef FILESYS_8_3
        OsSnprintf(szfileNameTemp, sizeof(szfileNameTemp) - 1, "%s_%03d.%s", pMsgBufferDesc->szMsgLogFileName, pMsgBufferDesc->wLogFileIndex, pMsgBufferDesc->szMsgLogFileExt);
#else
        OsSnprintf(szfileNameTemp, sizeof(szfileNameTemp) - 1, "%s.%03d.%s", pMsgBufferDesc->szMsgLogFileName, pMsgBufferDesc->wLogFileIndex, pMsgBufferDesc->szMsgLogFileExt);
#endif
    }
    else
    {
        OsSnprintf(szfileNameTemp, sizeof(szfileNameTemp) - 1, "%s.%s", pMsgBufferDesc->szMsgLogFileName, pMsgBufferDesc->szMsgLogFileExt);
    }

    if (bLogFileEnb)
    {
        pMsgBufferDesc->pfMsgFile = OsFopen( szfileNameTemp, "w+");
        if (pMsgBufferDesc->pfMsgFile == EC_NULL)
        {
#if (!defined NOPRINTF)
            OsPrintf("ERROR: cannot create EtherCAT log file %s\n", szfileNameTemp);
#endif
            OsSleep(3000);
        }
    }
    else
    {
        pMsgBufferDesc->pfMsgFile = EC_NULL;
        pMsgBufferDesc->szMsgLogFileName[0] = 0;
    }
    OsStrncpy(pMsgBufferDesc->szLogName, szBufferName, MAX_PATH_LEN - 1);
#endif /* INCLUDE_FILE_LOGGING */

    pMsgBufferDesc->bIsInitialized = EC_TRUE;
    bOk = EC_TRUE;

Exit:
    if (!bOk)
    {
        SafeOsFree(pMsgBufferDesc->paMsg);
        SafeOsFree(pchMsgBuffer);
    }
    return bOk;
}

/********************************************************************************/
/** \brief De-Init message buffer
*
* \return N/A
*/
EC_T_VOID CAtEmLogging::DeinitMsgBuffer
(MSG_BUFFER_DESC*   pMsgBufferDesc
)
{
    CEcTimer oTimeout;

    if (pMsgBufferDesc->bIsInitialized)
    {
        /* let the log task print out all messages */
        if (pMsgBufferDesc->dwNextPrintMsgIndex != pMsgBufferDesc->dwNextEmptyMsgIndex)
        {
            OsPrintf("Store unsaved messages in '%s' message/logging buffer...", pMsgBufferDesc->szLogName);
            oTimeout.Start(3000);
            while (pMsgBufferDesc->dwNextPrintMsgIndex != pMsgBufferDesc->dwNextEmptyMsgIndex)
            {
                ProcessAllMsgs();
                OsSleep(100);
                if (oTimeout.IsElapsed())
                {
                    OsPrintf(".");
                    oTimeout.Start(3000);
                }
            }
            OsPrintf(" done!\n");
        }

        OsFree(pMsgBufferDesc->paMsg[0].szMsgBuffer);
        OsFree(pMsgBufferDesc->paMsg);

#if (defined INCLUDE_FILE_LOGGING)
        if (EC_NULL != pMsgBufferDesc->pfMsgFile)
        {
            OsFclose(pMsgBufferDesc->pfMsgFile);
        }
        pMsgBufferDesc->pfMsgFile = EC_NULL;
#endif

        pMsgBufferDesc->dwNextEmptyMsgIndex = 0;
        pMsgBufferDesc->dwNextPrintMsgIndex = 0;
        pMsgBufferDesc->bIsInitialized = EC_FALSE;
        pMsgBufferDesc->pbyLogMemory = EC_NULL;
        pMsgBufferDesc->dwLogMemorySize = 0;
        pMsgBufferDesc->pbyNextLogMsg = EC_NULL;
        pMsgBufferDesc->bLogBufferFull = EC_FALSE;
        pMsgBufferDesc->pszLastMsg = EC_NULL;
    }
}

/********************************************************************************/
/** \brief Process log message
*
* \return N/A
*/
EC_T_DWORD CAtEmLogging::LogMsg(struct _EC_T_LOG_CONTEXT* pContext, EC_T_DWORD dwLogMsgSeverity, const EC_T_CHAR* szFormat, ...)
{
    EC_T_DWORD  dwRetVal = EC_E_ERROR;
    EC_T_VALIST vaArgs;

    if (!s_bLogParmsArrayInitialized || (EC_NULL == pContext))
    {
        EC_VASTART(vaArgs, szFormat);
#if (!defined NOPRINTF)
        OsVprintf(szFormat, vaArgs);
#endif
        dwRetVal = EC_E_NOERROR;
        EC_VAEND(vaArgs);
    }
    else
    {
        if ((dwLogMsgSeverity == EC_LOG_LEVEL_CRITICAL) || (dwLogMsgSeverity == EC_LOG_LEVEL_ERROR))
        {
            EC_VASTART(vaArgs, szFormat);
            dwRetVal = ((CAtEmLogging*)pContext)->InsertNewMsgVa(((CAtEmLogging*)pContext)->m_pErrorMsgBufferDesc, dwLogMsgSeverity, szFormat, vaArgs);
            EC_VAEND(vaArgs);
        }
        EC_VASTART(vaArgs, szFormat);
        dwRetVal = ((CAtEmLogging*)pContext)->InsertNewMsgVa(((CAtEmLogging*)pContext)->m_pAllMsgBufferDesc, dwLogMsgSeverity, szFormat, vaArgs);
        EC_VAEND(vaArgs);
    }
    return dwRetVal;
}

EC_T_DWORD CAtEmLogging::LogMsgStub(struct _EC_T_LOG_CONTEXT* pContext, EC_T_DWORD dwLogMsgSeverity, const EC_T_CHAR* szFormat, ...)
{
    EC_UNREFPARM(pContext); EC_UNREFPARM(dwLogMsgSeverity); EC_UNREFPARM(szFormat);
    return EC_E_NOERROR;
}

EC_T_DWORD CAtEmLogging::LogMsgOsPrintf(struct _EC_T_LOG_CONTEXT* pContext, EC_T_DWORD dwLogMsgSeverity, const EC_T_CHAR* szFormat, ...)
{
    EC_T_DWORD  dwRetVal = EC_E_ERROR;
    EC_T_VALIST vaArgs;

    EC_UNREFPARM(pContext); EC_UNREFPARM(dwLogMsgSeverity);

    EC_VASTART(vaArgs, szFormat);
    OsVprintf(szFormat, vaArgs);
    EC_VAEND(vaArgs);

    dwRetVal = EC_E_NOERROR;
    return dwRetVal;
}

/********************************************************************************/
/** \brief Insert a new message into message buffer with Log Message Severity
*
* \return N/A
*/
EC_T_DWORD CAtEmLogging::InsertNewMsgVa
(MSG_BUFFER_DESC*   pMsgBufferDesc
 ,EC_T_DWORD        dwLogMsgSeverity
,const
 EC_T_CHAR*         szFormat
,EC_T_VALIST        vaArgs
)
{
    EC_T_DWORD    dwRes       = EC_E_NOERROR;
    EC_T_BOOL     bBufferFull = EC_FALSE;
    LOG_MSG_DESC* pMsgDesc    = EC_NULL;

    /* select default message queue if needed */
    if (pMsgBufferDesc == EC_NULL)
    {
        pMsgBufferDesc = m_pAllMsgBufferDesc;
    }
    /* check state */
    if ((EC_NULL == pMsgBufferDesc) || !pMsgBufferDesc->bIsInitialized || m_bShutdownLogTask)
    {
        dwRes = EC_E_INVALIDSTATE;
        goto Exit;
    }
    /* get message descriptor */
    OsLock(m_poInsertMsgLock);
    {
        EC_T_DWORD dwNewNextEmpty = 0;

        pMsgDesc = &pMsgBufferDesc->paMsg[pMsgBufferDesc->dwNextEmptyMsgIndex];
        pMsgDesc->dwMsgBufferLen = 0;
        pMsgDesc->dwMsgLen = 0;

        dwNewNextEmpty = pMsgBufferDesc->dwNextEmptyMsgIndex + 1;
        if (dwNewNextEmpty >= pMsgBufferDesc->dwNumMsgs)
        {
            dwNewNextEmpty = 0;
        }
        if (dwNewNextEmpty == pMsgBufferDesc->dwNextPrintMsgIndex)
        {
            LOG_MSG_DESC* pDropReportMsg = &pMsgBufferDesc->paMsg[pMsgBufferDesc->dwDropReportMsgIndex];
            if (pDropReportMsg->dwMsgsDropped < 0xFFFFFFFF)
            {
                pDropReportMsg->dwMsgsDropped++;
            }
            bBufferFull = EC_TRUE;
        }
        else
        {
            pMsgBufferDesc->dwDropReportMsgIndex = pMsgBufferDesc->dwNextEmptyMsgIndex;
            pMsgBufferDesc->dwNextEmptyMsgIndex = dwNewNextEmpty;
        }
    }
    OsUnlock(m_poInsertMsgLock);

    if (bBufferFull)
    {
        dwRes = EC_E_NOMEMORY;
        goto Exit;
    }
    /* fill message descriptor */
    pMsgDesc->dwSeverity = dwLogMsgSeverity;

    /* timestamp max. 10 digits, format "%010d: " */
    if (pMsgBufferDesc->bPrintTimestamp)
    {
        pMsgDesc->dwMsgTimestamp = OsQueryMsecCount();
    }
    else
    {
        pMsgDesc->dwMsgTimestamp = 0;
    }

    /* format message */
    pMsgDesc->dwMsgLen = (EC_T_DWORD)EcVsnprintf(pMsgDesc->szMsg, (EC_T_INT)(pMsgBufferDesc->dwMsgSize - (pMsgDesc->szMsg - pMsgDesc->szMsgBuffer)), szFormat, vaArgs);
    pMsgDesc->dwMsgBufferLen = (EC_T_DWORD)(pMsgDesc->dwMsgLen + (pMsgDesc->szMsg - pMsgDesc->szMsgBuffer));

    OnLogMsg(pMsgDesc->szMsg);

    if (FilterMsg(pMsgDesc->szMsg))
    {
        pMsgDesc->szMsg[0] = '\0';
        pMsgDesc->dwMsgBufferLen = 0;
        pMsgDesc->dwMsgLen = 0;
    }

    OsMemoryBarrier();
    pMsgDesc->bValid = EC_TRUE;

Exit:
    return dwRes;
}

/********************************************************************************/
/** \brief Forward to next logging buffer (memory logging)
*
* \return N/A
*/
EC_T_VOID CAtEmLogging::SelectNextLogMemBuffer
(MSG_BUFFER_DESC*   pMsgBufferDesc )
{
    pMsgBufferDesc->pbyNextLogMsg = pMsgBufferDesc->pbyNextLogMsg + OsStrlen(pMsgBufferDesc->pbyNextLogMsg);
    if (pMsgBufferDesc->pbyNextLogMsg >= (pMsgBufferDesc->pbyLogMemory + pMsgBufferDesc->dwLogMemorySize - 3*MAX_MESSAGE_SIZE) )
    {
        /* stop logging if memory is full */
        // LogMsg(pEcLogContext, EC_LOG_LEVEL_INFO, "logging buffer %s is full, logging stopped!\n", pMsgBufferDesc->szLogName); // TODO: replace with logging of lost msg counter in low prio log task
        pMsgBufferDesc->bLogBufferFull = EC_TRUE;
    }

    /* zero-terminate log msg (snprintf doesn't include this) */
    pMsgBufferDesc->pbyNextLogMsg[0] = '\0';
    pMsgBufferDesc->pbyNextLogMsg[MAX_MESSAGE_SIZE - 1] = '\0';
}

/* finalize szMsgBuffer (timestamp, new line) */
EC_T_VOID CAtEmLogging::FinalizeMsg(LOG_MSG_DESC* pMsgDesc)
{
    /* szMsgBuffer contains timestamp, message and new line, because e.g. RTX64 requires terminating new line directly appended to the message */
    if (pMsgDesc->szMsg[pMsgDesc->dwMsgLen - 1] != '\n')
    {
        pMsgDesc->szMsg[pMsgDesc->dwMsgLen] = '\n';
        pMsgDesc->dwMsgLen++;
        pMsgDesc->szMsg[pMsgDesc->dwMsgLen] = '\0';
    }

    /* add timestamp to buffer */
    if (pMsgDesc->szMsg != pMsgDesc->szMsgBuffer)
    {
        /* insert timestamp */
        OsSnprintf(pMsgDesc->szMsgBuffer, LOG_MSG_OFFSET_AFTER_TIMESTAMP, "%010d:", (EC_T_INT)pMsgDesc->dwMsgTimestamp);
        pMsgDesc->szMsgBuffer[LOG_MSG_OFFSET_AFTER_TIMESTAMP - 1] = ' ';

        /* print with timestamp */
        pMsgDesc->dwMsgBufferLen = pMsgDesc->dwMsgLen + LOG_MSG_OFFSET_AFTER_TIMESTAMP;
    }
}

EC_T_VOID CAtEmLogging::PrintConsole(EC_T_CHAR* szMsgBuffer)
{
#if (!defined NOPRINTF)
    OsPrintf("%s", szMsgBuffer);
#else
    EC_UNREFPARM(szMsgBuffer);
#endif
}

EC_T_VOID CAtEmLogging::PrintMsg(LOG_MSG_DESC* pMsgDesc)
{
    FinalizeMsg(pMsgDesc);

#if (defined EC_LOGGING_MAX_LOG_LEVEL_CONSOLE) /* e.g. EC_LOG_LEVEL_INFO */
    if (pMsgDesc->dwSeverity <= EC_LOGGING_MAX_LOG_LEVEL_CONSOLE)
#endif
    {
        PrintConsole(pMsgDesc->szMsgBuffer);
    }
}

#if (!defined EC_EAP)
EC_T_VOID CAtEmLogging::PrintPerfMeasInternal(struct _EC_T_LOG_CONTEXT* pContext, EC_T_DWORD dwNumOf, EC_T_PERF_MEAS_VAL* aPerfMeasVal, EC_T_PERF_MEAS_INFO* aPerfMeasInfo)
{
    EC_T_DWORD dwPerfMeasIdx;
    for (dwPerfMeasIdx = 0; dwPerfMeasIdx < dwNumOf; ++dwPerfMeasIdx)
    {
        EC_T_PERF_MEAS_VAL* pPerfMeasVal = &aPerfMeasVal[dwPerfMeasIdx];
        EC_T_PERF_MEAS_INFO* pPerfMeasInfo = &aPerfMeasInfo[dwPerfMeasIdx];
        EC_T_UINT64 qwFrequency = pPerfMeasInfo->qwFrequency / (10 * 1000) /* 1/10 usec */;
        EC_T_UINT64 qwMin = pPerfMeasVal->qwMinTicks * 1000 / qwFrequency;
        EC_T_UINT64 qwAvg = pPerfMeasVal->qwAvgTicks * 1000 / qwFrequency;
        EC_T_UINT64 qwMax = pPerfMeasVal->qwMaxTicks * 1000 / qwFrequency;
        if ((qwMax > 0) && !(pPerfMeasInfo->dwFlags & EC_T_PERF_MEAS_FLAG_OFFSET))
        {
            LogMsg(pContext, EC_LOG_LEVEL_ANY, "PerfMsmt '%s' (min/avg/max) [usec]: %4d.%d/%4d.%d/%4d.%d\n",
                pPerfMeasInfo->szName,
                (EC_T_DWORD)qwMin / 10, (EC_T_DWORD)qwMin % 10,
                (EC_T_DWORD)qwAvg / 10, (EC_T_DWORD)qwAvg % 10,
                (EC_T_DWORD)qwMax / 10, (EC_T_DWORD)qwMax % 10);
        }
    }
}

EC_T_DWORD CAtEmLogging::PrintPerfMeas(EC_T_DWORD dwPerfMeasInstanceId0, EC_T_DWORD dwPerfMeasInstanceId1, struct _EC_T_LOG_CONTEXT* pContext)
{
    EC_T_DWORD dwRes    = EC_E_ERROR;
    EC_T_DWORD dwRetVal = EC_E_ERROR;

    EC_UNREFPARM(dwPerfMeasInstanceId0);
    EC_UNREFPARM(dwPerfMeasInstanceId1);

    /* (re-)allocate memory if needed */
#if (defined INCLUDE_EC_MASTER) || (defined INCLUDE_EC_MONITOR)
    dwRes = emPerfMeasInternalGetNumOf(dwPerfMeasInstanceId0, &m_aData[0].dwInternalNumOf);
    if (EC_E_NOERROR != dwRes)
    {
        dwRetVal = dwRes;
        goto Exit;
    }
    dwRes = emPerfMeasAppGetNumOf(dwPerfMeasInstanceId0, EC_NULL, &m_aData[0].dwAppNumOf);
    if (EC_E_NOERROR != dwRes)
    {
        dwRetVal = dwRes;
        goto Exit;
    }
#endif /* INCLUDE_EC_MASTER || INCLUDE_EC_MONITOR */
#if (defined INCLUDE_EC_SIMULATOR)
    dwRes = esPerfMeasInternalGetNumOf(dwPerfMeasInstanceId1, &m_aData[1].dwInternalNumOf);
    if (EC_E_NOERROR != dwRes)
    {
        dwRetVal = dwRes;
        goto Exit;
    }
    dwRes = esPerfMeasAppGetNumOf(dwPerfMeasInstanceId1, EC_NULL, &m_aData[1].dwAppNumOf);
    if (EC_E_NOERROR != dwRes)
    {
        dwRetVal = dwRes;
        goto Exit;
    }
#endif /* INCLUDE_EC_SIMULATOR */
    if ((m_aData[0].dwInternalNumOf > m_dwPerfMeasNumOf) || (m_aData[0].dwAppNumOf > m_dwPerfMeasNumOf)
        || (m_aData[1].dwInternalNumOf > m_dwPerfMeasNumOf) || (m_aData[1].dwAppNumOf > m_dwPerfMeasNumOf))
    {
        SafeOsFree(m_aPerfMeasVal);
        SafeOsFree(m_aPerfMeasInfo);
        m_dwPerfMeasNumOf = 0;
    }

    if (0 == m_dwPerfMeasNumOf)
    {
        m_dwPerfMeasNumOf = EC_MAX(EC_MAX(EC_MAX(m_aData[0].dwInternalNumOf, m_aData[0].dwAppNumOf), m_aData[1].dwInternalNumOf), m_aData[1].dwAppNumOf);
        if (m_dwPerfMeasNumOf > 0)
        {
            m_aPerfMeasVal = (EC_T_PERF_MEAS_VAL*)OsMalloc(m_dwPerfMeasNumOf * sizeof(EC_T_PERF_MEAS_VAL));
            m_aPerfMeasInfo = (EC_T_PERF_MEAS_INFO*)OsMalloc(m_dwPerfMeasNumOf * sizeof(EC_T_PERF_MEAS_INFO));

            if ((EC_NULL == m_aPerfMeasVal) || (EC_NULL == m_aPerfMeasInfo))
            {
                SafeOsFree(m_aPerfMeasVal);
                SafeOsFree(m_aPerfMeasInfo);
                m_dwPerfMeasNumOf = 0;

                dwRetVal = EC_E_NOMEMORY;
                goto Exit;
            }
        }
    }

    if ((m_aData[0].dwInternalNumOf > 0) || (m_aData[1].dwInternalNumOf > 0) || (m_aData[0].dwAppNumOf > 0) || (m_aData[1].dwAppNumOf > 0))
    {
        LogMsg(pContext, EC_LOG_LEVEL_ANY, "============================================================================\n");

        /* print internal benchmarks */
#if (defined INCLUDE_EC_MASTER) || (defined INCLUDE_EC_MONITOR)
        if (m_aData[0].dwInternalNumOf > 0)
        {
            dwRes = emPerfMeasInternalGetRaw(dwPerfMeasInstanceId0, EC_PERF_MEAS_ALL, m_aPerfMeasVal, EC_NULL, m_aData[0].dwInternalNumOf);
            if (EC_E_NOERROR != dwRes)
            {
                dwRetVal = dwRes;
                goto Exit;
            }
            dwRes = emPerfMeasInternalGetInfo(dwPerfMeasInstanceId0, EC_PERF_MEAS_ALL, m_aPerfMeasInfo, m_aData[0].dwInternalNumOf);
            if (EC_E_NOERROR != dwRes)
            {
                dwRetVal = dwRes;
                goto Exit;
            }
            PrintPerfMeasInternal(pContext, m_aData[0].dwInternalNumOf, m_aPerfMeasVal, m_aPerfMeasInfo);
        }
#endif /* INCLUDE_EC_MASTER || INCLUDE_EC_MONITOR */

#if (defined INCLUDE_EC_SIMULATOR)
        if (m_aData[1].dwInternalNumOf > 0)
        {
            dwRes = esPerfMeasInternalGetRaw(dwPerfMeasInstanceId1, EC_PERF_MEAS_ALL, m_aPerfMeasVal, EC_NULL, m_aData[1].dwInternalNumOf);
            if (EC_E_NOERROR != dwRes)
            {
                dwRetVal = dwRes;
                goto Exit;
            }
            dwRes = esPerfMeasInternalGetInfo(dwPerfMeasInstanceId1, EC_PERF_MEAS_ALL, m_aPerfMeasInfo, m_aData[1].dwInternalNumOf);
            if (EC_E_NOERROR != dwRes)
            {
                dwRetVal = dwRes;
                goto Exit;
            }
            PrintPerfMeasInternal(pContext, m_aData[1].dwInternalNumOf, m_aPerfMeasVal, m_aPerfMeasInfo);
        }
#endif /* INCLUDE_EC_SIMULATOR */

    /* print Application benchmarks */
#if (defined INCLUDE_EC_MASTER) || (defined INCLUDE_EC_MONITOR)
        if (m_aData[0].dwAppNumOf > 0)
        {
            dwRes = emPerfMeasAppGetRaw(dwPerfMeasInstanceId0, EC_NULL, EC_PERF_MEAS_ALL, m_aPerfMeasVal, EC_NULL, m_aData[0].dwAppNumOf);
            if (EC_E_NOERROR != dwRes)
            {
                dwRetVal = dwRes;
                goto Exit;
            }
            dwRes = emPerfMeasAppGetInfo(dwPerfMeasInstanceId0, EC_NULL, EC_PERF_MEAS_ALL, m_aPerfMeasInfo, m_aData[0].dwAppNumOf);
            if (EC_E_NOERROR != dwRes)
            {
                dwRetVal = dwRes;
                goto Exit;
            }
            PrintPerfMeasInternal(pContext, m_aData[0].dwAppNumOf, m_aPerfMeasVal, m_aPerfMeasInfo);
        }
#endif /* INCLUDE_EC_MASTER || INCLUDE_EC_MONITOR */

#if (defined INCLUDE_EC_SIMULATOR)
        if (m_aData[1].dwAppNumOf > 0)
        {
            dwRes = esPerfMeasAppGetRaw(dwPerfMeasInstanceId1, EC_NULL, EC_PERF_MEAS_ALL, m_aPerfMeasVal, EC_NULL, m_aData[1].dwAppNumOf);
            if (EC_E_NOERROR != dwRes)
            {
                dwRetVal = dwRes;
                goto Exit;
            }
            dwRes = esPerfMeasAppGetInfo(dwPerfMeasInstanceId1, EC_NULL, EC_PERF_MEAS_ALL, m_aPerfMeasInfo, m_aData[1].dwAppNumOf);
            if (EC_E_NOERROR != dwRes)
            {
                dwRetVal = dwRes;
                goto Exit;
            }
            PrintPerfMeasInternal(pContext, m_aData[1].dwAppNumOf, m_aPerfMeasVal, m_aPerfMeasInfo);
        }
#endif /* INCLUDE_EC_SIMULATOR */

        LogMsg(pContext, EC_LOG_LEVEL_ANY, "\n");
    }

    dwRetVal = EC_E_NOERROR;
Exit:
    return dwRetVal;
}

#if (!defined EC_SIMULATOR)
static EC_T_VOID StripStr(EC_T_CHAR* szDst, EC_T_CHAR* szSrc)
{
    EC_T_DWORD dwStartPos = 0;
    EC_T_DWORD dwEndPos = 0;
    EC_T_DWORD dwStrLen = (EC_T_DWORD)OsStrlen(szSrc);
    
    for (EC_T_DWORD i = 0; i < dwStrLen; i++)
    {
        if (' ' != szSrc[i])
        {
            break;
        }
        dwStartPos++;
    }

    dwEndPos = dwStrLen;
    for (EC_T_DWORD j = 1; j < dwStrLen; j++)
    {
        if (' ' != szSrc[dwStrLen - j])
        {
            break;
        }
        dwEndPos--;
    }

    OsStrncpy(szDst, szSrc + dwStartPos, dwEndPos - dwStartPos);
}
#endif /* !EC_SIMULATOR */

EC_T_DWORD CAtEmLogging::PrintHistogramAsCsv(EC_T_DWORD dwInstanceId)
{
#if !(defined EC_SIMULATOR)
    EC_T_DWORD dwRetVal = EC_E_ERROR;
    EC_T_DWORD dwRes = EC_E_NOERROR;
    EC_T_DWORD dwMeasNum = 0;
    EC_T_DWORD dwMeasIdx = 0;
    EC_T_DWORD dwBinIdx = 0;
    EC_T_PERF_MEAS_INFO* aPerfMeasInfo = EC_NULL;
    EC_T_PERF_MEAS_HISTOGRAM* aHistogram = EC_NULL;
    EC_T_DWORD dwBinCntMax = 0;
    EC_T_CHAR* szLineBuf = EC_NULL;
    EC_T_DWORD dwLineBufOffs = 0;

    if (EC_NULL == m_pHistMsgBufferDesc)
    {
#if (defined INCLUDE_FILE_LOGGING)
        m_pHistMsgBufferDesc = AddLogBuffer(dwInstanceId, 0, DEFAULT_LOG_MSG_BUFFER_SIZE, EC_FALSE, (EC_T_CHAR*)"Histogram", (EC_T_CHAR*)"hist", (EC_T_CHAR*)"csv", EC_FALSE, EC_FALSE);
#else
        m_pHistMsgBufferDesc = AddLogBuffer(dwInstanceId, DEFAULT_LOG_MSG_BUFFER_SIZE, EC_FALSE, (EC_T_CHAR*)"Histogram", EC_TRUE, EC_FALSE);
#endif
        if (EC_NULL == m_pHistMsgBufferDesc)
        {
            dwRetVal = EC_E_NOMEMORY;
            goto Exit;
        }
    }

    dwRes = emPerfMeasInternalGetNumOf(dwInstanceId, &dwMeasNum);
    if (EC_E_NOERROR != dwRes)
    {
        dwRetVal = dwRes;
        goto Exit;
    }

    aPerfMeasInfo = (EC_T_PERF_MEAS_INFO*)OsMalloc(sizeof(EC_T_PERF_MEAS_INFO) * dwMeasNum);
    if (EC_NULL == aPerfMeasInfo)
    {
        dwRetVal = EC_E_NOMEMORY;
        goto Exit;
    }
    OsMemset(aPerfMeasInfo, 0, sizeof(EC_T_PERF_MEAS_INFO) * dwMeasNum);

    dwRes = emPerfMeasInternalGetInfo(dwInstanceId, EC_PERF_MEAS_ALL, aPerfMeasInfo, dwMeasNum);
    if (EC_E_NOERROR != dwRes)
    {
        dwRetVal = dwRes;
        goto Exit;
    }

    aHistogram = (EC_T_PERF_MEAS_HISTOGRAM*)OsMalloc(sizeof(EC_T_PERF_MEAS_HISTOGRAM) * dwMeasNum);
    if (EC_NULL == aHistogram)
    {
        dwRetVal = EC_E_NOMEMORY;
        goto Exit;
    }
    OsMemset(aHistogram, 0, sizeof(EC_T_PERF_MEAS_HISTOGRAM) * dwMeasNum);
    for (dwMeasIdx = 0; dwMeasIdx < dwMeasNum; dwMeasIdx++)
    {
        dwBinCntMax = EC_MAX(dwBinCntMax, aPerfMeasInfo[dwMeasIdx].dwBinCountHistogram);

        aHistogram[dwMeasIdx].aBins = (EC_T_DWORD*)OsMalloc(sizeof(EC_T_DWORD) * aPerfMeasInfo[dwMeasIdx].dwBinCountHistogram);
        if (EC_NULL == aHistogram[dwMeasIdx].aBins)
        {
            dwRetVal = EC_E_NOMEMORY;
            goto Exit;
        }
        aHistogram[dwMeasIdx].dwBinCount = aPerfMeasInfo[dwMeasIdx].dwBinCountHistogram;
    }

    dwRes = emPerfMeasInternalGetRaw(dwInstanceId, EC_PERF_MEAS_ALL, EC_NULL, aHistogram, dwMeasNum);
    if (EC_E_NOERROR != dwRes)
    {
        dwRetVal = dwRes;
        goto Exit;
    }

    szLineBuf = (EC_T_CHAR*)OsMalloc(MAX_MESSAGE_SIZE);
    if (EC_NULL == szLineBuf)
    {
        dwRetVal = EC_E_NOMEMORY;
        goto Exit;
    }
    OsMemset(szLineBuf, 0, MAX_MESSAGE_SIZE);

    /* print header */
    dwLineBufOffs = 0;
    for (dwMeasIdx = 0; dwMeasIdx < dwMeasNum; dwMeasIdx++)
    {
        if (0 != aPerfMeasInfo->dwBinCountHistogram)
        {
            EC_T_CHAR szName[MAX_STD_STRLEN];
            OsMemset(szName, 0, MAX_STD_STRLEN);
            StripStr(szName, aPerfMeasInfo[dwMeasIdx].szName);
            
            if (MAX_MESSAGE_SIZE < (dwLineBufOffs + (OsStrlen(szName) * 2) + 10))
            {
                PrintHistogramLine("%s", szLineBuf);
                dwLineBufOffs = 0;
            }

            dwLineBufOffs += (EC_T_DWORD)OsSnprintf(szLineBuf + dwLineBufOffs, MAX_MESSAGE_SIZE - dwLineBufOffs, "%s[us]; %s cnt;",
                szName, szName);
        }
    }
    PrintHistogramLine("%s\n", szLineBuf);

    /* min / max */
    dwLineBufOffs = 0;
    for (dwMeasIdx = 0; dwMeasIdx < dwMeasNum; dwMeasIdx++)
    {
        if (0 != aPerfMeasInfo->dwBinCountHistogram)
        {
            if (MAX_MESSAGE_SIZE < (dwLineBufOffs + OsStrlen("Min: %d us; Max: %d us;") + 10))
            {
                PrintHistogramLine("%s", szLineBuf);
                dwLineBufOffs = 0;
            }

            dwLineBufOffs += (EC_T_DWORD)OsSnprintf(szLineBuf + dwLineBufOffs, MAX_MESSAGE_SIZE - dwLineBufOffs, "Min: %d us; Max: %d us;",
                (EC_T_DWORD)((aHistogram[dwMeasIdx].qwMinTicks * 1000000) / aPerfMeasInfo[dwMeasIdx].qwFrequency),
                (EC_T_DWORD)((aHistogram[dwMeasIdx].qwMaxTicks * 1000000) / aPerfMeasInfo[dwMeasIdx].qwFrequency));
        }
    }
    PrintHistogramLine("%s\n", szLineBuf);

    /* print bins and counts */
    for (dwBinIdx = 0; dwBinIdx < dwBinCntMax; dwBinIdx++)
    {
        dwLineBufOffs = 0;
        for (dwMeasIdx = 0; dwMeasIdx < dwMeasNum; dwMeasIdx++)
        {
            if (dwBinIdx <= aHistogram[dwMeasIdx].dwBinCount)
            {
                EC_T_DWORD dwMinUsec = (EC_T_DWORD)((aHistogram[dwMeasIdx].qwMinTicks * 1000000) / aPerfMeasInfo[dwMeasIdx].qwFrequency);
                EC_T_DWORD dwStepUsec = (EC_T_DWORD)(((aHistogram[dwMeasIdx].qwMaxTicks - aHistogram[dwMeasIdx].qwMinTicks) * 1000000) / aHistogram[dwMeasIdx].dwBinCount / aPerfMeasInfo[dwMeasIdx].qwFrequency);

                if (MAX_MESSAGE_SIZE < (dwLineBufOffs + 16 + 2)) /* assume the max length of each numbers is 8 */
                {
                    PrintHistogramLine("%s", szLineBuf);
                    dwLineBufOffs = 0;
                }

                dwLineBufOffs += (EC_T_DWORD)OsSnprintf(szLineBuf + dwLineBufOffs, MAX_MESSAGE_SIZE - dwLineBufOffs, "%d;%d;",
                    dwMinUsec + (dwStepUsec * dwBinIdx), aHistogram[dwMeasIdx].aBins[dwBinIdx]);
            }
        }
        PrintHistogramLine("%s\n", szLineBuf);
    }

    dwRetVal = EC_E_NOERROR;
Exit:
    SafeOsFree(aPerfMeasInfo);
    if (EC_NULL != aHistogram)
    {
        for (dwMeasIdx = 0; dwMeasIdx < dwMeasNum; dwMeasIdx++)
        {
            SafeOsFree(aHistogram[dwMeasIdx].aBins);
        }
    }
    SafeOsFree(aHistogram);
    SafeOsFree(szLineBuf);

    return dwRetVal;
#else
    EC_UNREFPARM(dwInstanceId);
    return EC_E_NOTSUPPORTED;
#endif
}

EC_T_DWORD CAtEmLogging::PrintHistogramLine(const EC_T_CHAR* szFormat, ...)
{
    EC_T_DWORD  dwRes = EC_E_NOERROR;
    EC_T_VALIST vaArgs;

    EC_VASTART(vaArgs, szFormat);
    dwRes = InsertNewMsgVa(m_pHistMsgBufferDesc, EC_LOG_LEVEL_INFO, szFormat, vaArgs);
    EC_VAEND(vaArgs);

    /* prevent msg buffer overflow */
    if (((m_pHistMsgBufferDesc->dwNextEmptyMsgIndex + 2) % m_pHistMsgBufferDesc->dwNumMsgs) == m_pHistMsgBufferDesc->dwNextPrintMsgIndex)
    {
        OsSleep(500);
    }
    return dwRes;
}
#endif /* !EC_EAP */

/********************************************************************************/
/** \brief Process all messages of a message buffer
*
* \return N/A
*/
EC_T_VOID CAtEmLogging::ProcessMsgs
(MSG_BUFFER_DESC*   pMsgBufferDesc )
{
    EC_T_DWORD    dwNewNextPrint              = 0;
    LOG_MSG_DESC* pCurrMsg                    = EC_NULL;
    EC_T_BOOL     bLocked                     = EC_FALSE;
    EC_T_BOOL     bSkipDuplicate              = EC_FALSE;
    EC_T_DWORD    dwNumDuplicatesBeforeNewMsg = 0;
#if (defined INCLUDE_FILE_LOGGING)
    FILE*         pFileHandle                 = bLogFileEnb ? pMsgBufferDesc->pfMsgFile : EC_NULL;
    EC_T_BOOL     bRollOver                   = EC_FALSE;
    EC_T_CHAR     szfileNameTemp[MAX_PATH_LEN];
    szfileNameTemp[0] = '\0';
#endif

    if (pMsgBufferDesc->bIsInitialized)
    {
        OsLock(m_poProcessMsgLock);
        bLocked = EC_TRUE;

        while (pMsgBufferDesc->dwNextPrintMsgIndex != pMsgBufferDesc->dwNextEmptyMsgIndex)
        {
            EC_T_DWORD dwMsgsDropped = 0;

            OsDbgAssert(pMsgBufferDesc->bIsInitialized);
            pCurrMsg = &pMsgBufferDesc->paMsg[pMsgBufferDesc->dwNextPrintMsgIndex];

            /* wait until message complete */
            if (!pCurrMsg->bValid)
            {
                break;
            }

            /* message complete, not filtered */
            if (pCurrMsg->dwMsgLen != 0)
            {
                EC_T_DWORD dwMsgOffset = 0;
                LOG_MSG_DESC oTmpMsgDesc;
                OsMemcpy(&oTmpMsgDesc, pCurrMsg, sizeof(LOG_MSG_DESC));

                /* set pointers and offsets according for oMsgDesc, oTmpMsgDesc */
                if (pMsgBufferDesc->bPrintTimestamp)
                {
                    dwMsgOffset = LOG_MSG_OFFSET_AFTER_TIMESTAMP;
                    oTmpMsgDesc.szMsg = &m_pchTempbuffer[dwMsgOffset];
                    oTmpMsgDesc.szMsgBuffer = m_pchTempbuffer;
                }
                else
                {
                    dwMsgOffset = 0;
                    oTmpMsgDesc.szMsg = m_pchTempbuffer;
                    oTmpMsgDesc.szMsgBuffer = m_pchTempbuffer;
                }
                oTmpMsgDesc.szMsg = &m_pchTempbuffer[dwMsgOffset];
                oTmpMsgDesc.szMsg[0] = '\0';

                /* skip duplicate */
                bSkipDuplicate = EC_FALSE;
                dwNumDuplicatesBeforeNewMsg = 0;
                if (pMsgBufferDesc->bSkipDuplicateMessages)
                {
                    if (pMsgBufferDesc->pszLastMsg == EC_NULL)
                    {
                        /* first message */
                        pMsgBufferDesc->pszLastMsg = pCurrMsg->szMsg;
                    }
                    else if (OsStrncmp(pMsgBufferDesc->pszLastMsg, pCurrMsg->szMsg, pCurrMsg->dwMsgLen + 1) == 0)
                    {
                        /* same message as before, just increment duplicate pointer */
                        pMsgBufferDesc->dwNumDuplicates++;
                        bSkipDuplicate = EC_TRUE;
                    }
                    else
                    {
                        /* new message */
                        dwNumDuplicatesBeforeNewMsg = pMsgBufferDesc->dwNumDuplicates;
                        pMsgBufferDesc->dwNumDuplicates = 0;
                        pMsgBufferDesc->pszLastMsg = pCurrMsg->szMsg;
                        if (dwNumDuplicatesBeforeNewMsg > 0 && pMsgBufferDesc->pszLastMsg[0] == '\0')
                        {
                            /* ignore empty duplicates... */
                            dwNumDuplicatesBeforeNewMsg = 0;
                        }
                    }
                }
                if (!bSkipDuplicate)
                {
                    /* latch dropped messages count (no empty message buffer available) for reporting and reset info */
                    if (pCurrMsg->dwMsgsDropped > 0)                {
                    
                        OsLock(m_poInsertMsgLock);
                        dwMsgsDropped = pCurrMsg->dwMsgsDropped;
                        pCurrMsg->dwMsgsDropped = 0;
                        OsUnlock(m_poInsertMsgLock);
                    }
#if (defined INCLUDE_FILE_LOGGING)
                    if (EC_NULL != pFileHandle)
                    {
                        pMsgBufferDesc->wEntryCounter++;
                        if (0 != pMsgBufferDesc->wEntryCounterLimit && pMsgBufferDesc->pfMsgFile != EC_NULL)
                        {
                            if (pMsgBufferDesc->wEntryCounter >= pMsgBufferDesc->wEntryCounterLimit)
                            {
                                bRollOver = EC_TRUE;
                                pMsgBufferDesc->wEntryCounter = 0;
                                pMsgBufferDesc->wLogFileIndex++;
#ifdef FILESYS_8_3
                                OsSnprintf(szfileNameTemp, sizeof(szfileNameTemp) - 1, "%s_%03d.%s", pMsgBufferDesc->szMsgLogFileName, pMsgBufferDesc->wLogFileIndex, pMsgBufferDesc->szMsgLogFileExt);
#else
                                OsSnprintf(szfileNameTemp, sizeof(szfileNameTemp) - 1, "%s.%03d.%s", pMsgBufferDesc->szMsgLogFileName, pMsgBufferDesc->wLogFileIndex, pMsgBufferDesc->szMsgLogFileExt);
#endif
                            }
                            else
                            {
                                bRollOver = EC_FALSE;
                            }

                        }
                    }
#endif /* INCLUDE_FILE_LOGGING */
                    if (pMsgBufferDesc->bPrintConsole)
                    {
                        /* print skip messages */
                        if (dwNumDuplicatesBeforeNewMsg > 0)
                        {
                            oTmpMsgDesc.dwMsgLen = EcSnprintf(oTmpMsgDesc.szMsg, MAX_MESSAGE_SIZE - dwMsgOffset - 1, "%d identical messages skipped\n", dwNumDuplicatesBeforeNewMsg);
                            PrintMsg(&oTmpMsgDesc);
                        }
                        /* print message */
                        {
                            PrintMsg(pCurrMsg);
                        }
                        /* print dropped messages */
                        if (dwMsgsDropped > 0)
                        {
                            oTmpMsgDesc.dwMsgLen = EcSnprintf(oTmpMsgDesc.szMsg, MAX_MESSAGE_SIZE - dwMsgOffset - 1, "buffer overflow, %d message(s) dropped, log incomplete\n", dwMsgsDropped);
                            PrintMsg(&oTmpMsgDesc);
                        }
                    }
#if (defined INCLUDE_FILE_LOGGING)
                    if (EC_NULL != pMsgBufferDesc->pbyLogMemory)
                    {
                        EC_T_DWORD dwWritten = 0;

                        /* print skip messages */
                        if (dwNumDuplicatesBeforeNewMsg > 0)
                        {
                            if (pMsgBufferDesc->bLogBufferFull) break;

                            if (pMsgBufferDesc->bPrintTimestamp)
                            {
                                dwWritten = dwWritten + OsSnprintf((EC_T_CHAR*)(pMsgBufferDesc->pbyNextLogMsg + dwWritten), MAX_MESSAGE_SIZE - dwWritten - 1, "%010d: ", (EC_T_INT)pCurrMsg->dwMsgTimestamp);
                            }
                            dwWritten = dwWritten + OsSnprintf((EC_T_CHAR*)(pMsgBufferDesc->pbyNextLogMsg + dwWritten), MAX_MESSAGE_SIZE - dwWritten - 1, "%d identical messages skipped\n", dwNumDuplicatesBeforeNewMsg);
                            SelectNextLogMemBuffer(pMsgBufferDesc);
                        }
                        /* print message */
                        {
                            if (pMsgBufferDesc->bLogBufferFull) break;

                            dwWritten = 0;
                            dwWritten = dwWritten + OsSnprintf((EC_T_CHAR*)(pMsgBufferDesc->pbyNextLogMsg + dwWritten), MAX_MESSAGE_SIZE - dwWritten - 1, "%s", pCurrMsg->szMsgBuffer);
                            SelectNextLogMemBuffer(pMsgBufferDesc);
                        }
                        /* print dropped messages */                    
                        if (dwMsgsDropped > 0)
                        {
                            if (pMsgBufferDesc->bLogBufferFull) break;

                            dwWritten = 0;

                            if (pMsgBufferDesc->bPrintTimestamp)
                            {
                                dwWritten = dwWritten + OsSnprintf((EC_T_CHAR*)(pMsgBufferDesc->pbyNextLogMsg + dwWritten), MAX_MESSAGE_SIZE - dwWritten - 1, "%010d: ", (EC_T_INT)pCurrMsg->dwMsgTimestamp);
                            }
                            dwWritten = dwWritten + OsSnprintf((EC_T_CHAR*)(pMsgBufferDesc->pbyNextLogMsg + dwWritten), MAX_MESSAGE_SIZE - dwWritten - 1, "buffer overflow, %d message(s) dropped, log incomplete\n", dwMsgsDropped);

                            SelectNextLogMemBuffer(pMsgBufferDesc);
                        }
                    }
                    else if (EC_NULL != pFileHandle)
                    {
                        /* don't use fprintf, some platforms don't support it! */

                        /* print skip messages */
                        if (dwNumDuplicatesBeforeNewMsg > 0)
                        {
                            oTmpMsgDesc.dwMsgLen = EcSnprintf(oTmpMsgDesc.szMsg, MAX_MESSAGE_SIZE - dwMsgOffset - 1, "%d identical messages skipped\n", dwNumDuplicatesBeforeNewMsg);
                            FinalizeMsg(&oTmpMsgDesc);
                            OsFwrite(oTmpMsgDesc.szMsgBuffer, oTmpMsgDesc.dwMsgLen, 1, pFileHandle);
                        }
                        /* print message */
                        {
                            FinalizeMsg(pCurrMsg);
                            OsFwrite(pCurrMsg->szMsgBuffer, pCurrMsg->dwMsgBufferLen, 1, pFileHandle);
                        }
                        /* print dropped messages */
                        if (dwMsgsDropped > 0)
                        {
                            oTmpMsgDesc.dwMsgLen = EcSnprintf(oTmpMsgDesc.szMsg, MAX_MESSAGE_SIZE - dwMsgOffset - 1, "buffer overflow, %d message(s) dropped, log incomplete\n", dwMsgsDropped);
                            FinalizeMsg(&oTmpMsgDesc);
                            OsFwrite(oTmpMsgDesc.szMsgBuffer, oTmpMsgDesc.dwMsgLen, 1, pFileHandle);
                        }
                    }
                    OsFflush(pFileHandle);
#endif
                }
            }
            pCurrMsg->bValid = EC_FALSE;
            OsMemoryBarrier();
            dwNewNextPrint = pMsgBufferDesc->dwNextPrintMsgIndex + 1;
            if (dwNewNextPrint >= pMsgBufferDesc->dwNumMsgs)
            {
                dwNewNextPrint = 0;
            }
            pMsgBufferDesc->dwNextPrintMsgIndex = dwNewNextPrint;

#if (defined INCLUDE_FILE_LOGGING)
            if (bRollOver)
            {
                /* do roll over */
                OsFclose(pFileHandle);

                pFileHandle = OsFopen(szfileNameTemp, "w+");
                if (pFileHandle == EC_NULL)
                {
#if (!defined NOPRINTF)
                    OsPrintf("ERROR: cannot create EtherCAT log file %s\n", szfileNameTemp);
#endif
                    OsSleep(3000);
                }

                pMsgBufferDesc->pfMsgFile = pFileHandle;

                bRollOver = EC_FALSE;
            }
#endif /* INCLUDE_FILE_LOGGING */
        }
    }
    if (bLocked)
        OsUnlock(m_poProcessMsgLock);

    return;
}

/********************************************************************************/
/** \brief application DCM message function
*
* \return N/A
*/
EC_T_DWORD CAtEmLogging::LogDcm(const EC_T_CHAR* szFormat, ...)
{
EC_T_VALIST vaArgs;
EC_T_DWORD  dwRes = EC_E_NOERROR;

    EC_VASTART(vaArgs, szFormat);
    dwRes = InsertNewMsgVa(m_pDcmMsgBufferDesc, EC_LOG_LEVEL_INFO, szFormat, vaArgs);
    EC_VAEND(vaArgs);
    return dwRes;
}

#if (defined INCLUDE_FILE_LOGGING)
/********************************************************************************/
/** \brief set log directory
*
* \return EC_E_NOERROR or EC_E_NOMEMORY if szLogDir too long
*/
EC_T_DWORD CAtEmLogging::SetLogDir(EC_T_CHAR* szLogDir)
{
    if (OsStrlen(szLogDir) >= MAX_PATH_LEN)
    {
        return EC_E_NOMEMORY;
    }
    OsStrncpy(m_pchLogDir, szLogDir, MAX_PATH_LEN - 1);
    m_pchLogDir[MAX_PATH_LEN - 1] = '\0';

    return EC_E_NOERROR;
}
#endif /* INCLUDE_FILE_LOGGING */

#ifdef INCLUDE_FRAME_SPY
CFrameLogMultiplexer::CFrameLogMultiplexer()
{
    m_pvLoggerListLock = OsCreateLock();
}

CFrameLogMultiplexer::~CFrameLogMultiplexer()
{
    if (EC_NULL != m_pvLoggerListLock)
    {
        OsLock(m_pvLoggerListLock);
        for (CLoggerDescList::CNode* pNode = m_LoggerList.GetFirstNode(); EC_NULL != pNode; m_LoggerList.GetNext(pNode))
        {
            m_LoggerList.RemoveAt(pNode);
        }
#if (defined EC_SIMULATOR)
        esLogFrameDisable(m_dwInstanceId);
#else
        emLogFrameDisable(m_dwInstanceId);
#endif
        OsUnlock(m_pvLoggerListLock);

        OsDeleteLock(m_pvLoggerListLock);
    }
}

/********************************************************************************/
/** \brief add logger to list
*
* \return N/A
*/
EC_T_VOID CFrameLogMultiplexer::AddFrameLogger(EC_T_DWORD dwInstanceId, EC_T_VOID* pvContext, EC_T_PFLOGFRAME_CB pvLogFrameCallBack)
{
    if (EC_NULL == G_aLogMultiplexer)
    {
        if (EC_E_NOERROR != CreateInstances())
        {
            return;
        }
        G_bAutoDispose = EC_TRUE;
    }
    G_aLogMultiplexer[dwInstanceId].AddFrameLogger(pvContext, pvLogFrameCallBack);
}

EC_T_DWORD CFrameLogMultiplexer::CreateInstances(EC_T_VOID)
{
    EC_T_DWORD dwLogMultiplexerIdx = 0;

    if (EC_NULL != G_aLogMultiplexer)
    {
        return EC_E_INVALIDSTATE;
    }

    G_aLogMultiplexer = EC_NEW(CFrameLogMultiplexer[MAX_NUMOF_MASTER_INSTANCES]);
    if (EC_NULL == G_aLogMultiplexer)
    {
        return EC_E_NOMEMORY;
    }

    for (dwLogMultiplexerIdx = 0; dwLogMultiplexerIdx < MAX_NUMOF_MASTER_INSTANCES; dwLogMultiplexerIdx++)
    {
        G_aLogMultiplexer[dwLogMultiplexerIdx].m_dwInstanceId = dwLogMultiplexerIdx;
    }

    return EC_E_NOERROR;
}

/********************************************************************************/
/** \brief search logger in list and remove
*
* \return N/A
*/
EC_T_VOID CFrameLogMultiplexer::RemoveFrameLogger(EC_T_DWORD dwInstanceId, EC_T_VOID* pvContext, EC_T_PFLOGFRAME_CB pvLogFrameCallBack)
{
    if (EC_NULL != G_aLogMultiplexer)
    {
        G_aLogMultiplexer[dwInstanceId].RemoveFrameLogger(pvContext, pvLogFrameCallBack);

        /* delete array if all multiplexers are unused */
        if (G_bAutoDispose)
        {
            for (EC_T_DWORD i = 0; i < MAX_NUMOF_MASTER_INSTANCES; i++)
            {
                if (!G_aLogMultiplexer[i].m_LoggerList.IsEmpty())
                {
                    return;
                }
            }
            DeleteInstances();
        }
    }
}

EC_T_DWORD CFrameLogMultiplexer::DeleteInstances(EC_T_VOID)
{
    if (EC_NULL == G_aLogMultiplexer)
    {
        return EC_E_INVALIDSTATE;
    }

    SafeDeleteArray(G_aLogMultiplexer);
    G_bAutoDispose = EC_FALSE;

    return EC_E_NOERROR;
}

CFrameLogMultiplexer* CFrameLogMultiplexer::GetInstance(EC_T_DWORD dwInstanceId)
{
    if (EC_NULL == G_aLogMultiplexer)
    {
        return EC_NULL;
    }
    return &G_aLogMultiplexer[dwInstanceId];
}

CFrameSpy::~CFrameSpy()
{
    Uninstall();
}

EC_T_VOID CFrameSpy::Install(EC_T_DWORD dwInstanceId)
{
    if (dwInstanceId > MAX_NUMOF_MASTER_INSTANCES) return;

    if (m_dwInstanceId != dwInstanceId)
    {
        CFrameLogMultiplexer::AddFrameLogger(dwInstanceId, (EC_T_VOID*)this, StaticFrameHandler);
        m_dwInstanceId = dwInstanceId;
    }
}

EC_T_VOID CFrameSpy::Uninstall(EC_T_VOID)
{
    if (0xffff != m_dwInstanceId)
    {
        CFrameLogMultiplexer::RemoveFrameLogger(m_dwInstanceId, (EC_T_VOID*)this, StaticFrameHandler);
        m_dwInstanceId = 0xffff;
    }
}
void CFrameSpy::FrameHandler(EC_T_DWORD dwLogFrameFlags, EC_T_DWORD dwFrameSize, EC_T_BYTE* pbyFrame)
{
    EC_T_WORD wCmdLen = 0;
    EC_T_WORD wEtherCAT_HeaderOffset = ETHERNET_FRAME_LEN;
    EC_T_BYTE* pbyData = pbyFrame;

    /* skip non-EtherCAT frames */
    if (ETHERNET_FRAME_TYPE_BKHF != EC_ETHFRM_GET_FRAMETYPE(pbyFrame)) return;

    /* skip EtherCAT frames without datagrams */
    if (dwFrameSize < wEtherCAT_HeaderOffset + ETYPE_EC_OVERHEAD) return;

    /* parse all commands */
    for (pbyData += wEtherCAT_HeaderOffset + 2; EC_NULL != pbyData;
    pbyData = (0 != EC_AL_CMDHDRLEN_GET_NEXT(&((ETYPE_EC_CMD_HEADER*)pbyData)->uLen)) ? pbyData + wCmdLen : EC_NULL)
    {
        ETYPE_EC_CMD_HEADER* pCmdHdr = (ETYPE_EC_CMD_HEADER*)pbyData;
        wCmdLen = (EC_T_WORD)(EC_CMDHDRLEN_GET_LEN(&pCmdHdr->uLen) + ETYPE_EC_OVERHEAD);

        /* call datagram handler */
        DatagramHandler(dwLogFrameFlags, dwFrameSize, pbyFrame, pCmdHdr, wCmdLen);
    }
}
#endif

#if (defined INCLUDE_PCAP_RECORDER)
/*-LOGGING-------------------------------------------------------------------*/
#undef  pLogMsgCallback
#undef  pEcLogContext
#undef  dwEcLogLevel
#define dwEcLogLevel      (GetLogParms()->dwLogLevel)
#define pLogMsgCallback   (GetLogParms()->pfLogMsg)
#define pEcLogContext     (GetLogParms()->pLogContext)


/*-CLASS FUNCTIONS-----------------------------------------------------------*/
/* legacy */
CPcapRecorder::CPcapRecorder(EC_T_DWORD dwBufCnt, EC_T_DWORD dwPrio /* LOG_THREAD_PRIO */, EC_T_DWORD dwInstanceId, const EC_T_CHAR* szFileName)
    : CPcapFileBufferedWriter(dwBufCnt, 0 /* EC_CPUSET_ZERO */, dwPrio, DEFAULT_LOG_STACK_SIZE), m_nLoggedFrameCount(0), m_dwInstanceId(0xffff)
{
    if (EC_E_NOERROR != InitInstance(dwInstanceId, dwBufCnt, szFileName))
    {
        return;
    }
}

#if (defined LOG_THREAD_PRIO) && (defined DEFAULT_LOG_STACK_SIZE)
CPcapRecorder::CPcapRecorder()
    : CPcapFileBufferedWriter(0 /* not used */, (EC_T_CPUSET)0, LOG_THREAD_PRIO, DEFAULT_LOG_STACK_SIZE), m_nLoggedFrameCount(0), m_dwInstanceId(0xffff)
{
}
CPcapRecorder::CPcapRecorder(EC_T_CPUSET cpuAffinity /* EC_CPUSET_ZERO */, EC_T_DWORD dwPrio /* LOG_THREAD_PRIO */, EC_T_DWORD dwStackSize /* DEFAULT_LOG_STACK_SIZE */)
    : CPcapFileBufferedWriter(0 /* not used */, cpuAffinity, dwPrio, dwStackSize), m_nLoggedFrameCount(0), m_dwInstanceId(0xffff)
{
}
#endif

CPcapRecorder::~CPcapRecorder()
{
    if (0xffff != m_dwInstanceId)
    {
        Uninstall();
    }
    Stop(15000);
    FlushBuffer();
    if (EC_NULL != m_pfHandle)
    {
        Close();
    }
}

EC_T_DWORD CPcapRecorder::InitInstance(EC_T_DWORD dwInstanceId, EC_T_DWORD dwBufCnt,  const EC_T_CHAR* szFileName)
{
    EC_T_DWORD dwRes = EC_E_ERROR;
    EC_T_DWORD dwRetVal = EC_E_ERROR;

    OsDbgAssert(ETHERNET_MAX_FRAME_LEN < ETHERNET_MAX_FRAMEBUF_LEN - sizeof(EC_T_LINK_FRAME_BUF_ENTRY_FRAME_DESC));

    if (0xffff != m_dwInstanceId)
    {
        return EC_E_INVALIDSTATE;
    }

    dwRes = CPcapFileBufferedWriter::InitInstance(dwBufCnt);
    if (EC_E_NOERROR != dwRes)
    {
        dwRetVal = dwRes;
        goto Exit;
    }

    if (EC_NULL != szFileName)
    {
        dwRes = Open(szFileName);
    }
    if (EC_E_NOERROR != dwRes)
    {
        dwRetVal = dwRes;
        goto Exit;
    }

    if (0xffff != dwInstanceId)
    {
        Install(dwInstanceId);
    }

    dwRetVal = EC_E_NOERROR;
Exit:
    return dwRetVal;
}

EC_T_VOID CPcapRecorder::Install(EC_T_DWORD dwInstanceId)
{
    if (dwInstanceId > MAX_NUMOF_MASTER_INSTANCES) return;

    if (isStopped())
    {
        StartThread();
    }

    if (m_dwInstanceId != dwInstanceId)
    {
        CFrameLogMultiplexer::AddFrameLogger(dwInstanceId, (EC_T_VOID*)this, LogFrameStatic);
        m_dwInstanceId = dwInstanceId;
    }
}

EC_T_VOID CPcapRecorder::Uninstall()
{
    if (0xffff != m_dwInstanceId)
    {
        CFrameLogMultiplexer::RemoveFrameLogger(m_dwInstanceId, (EC_T_VOID*)this, LogFrameStatic);
        m_dwInstanceId = 0xffff;
    }
}

EC_T_VOID CPcapFileBufferedWriter::FlushBuffer(EC_T_VOID)
{
    EC_T_LINK_FRAME_BUF_ENTRY Entry;
    OsMemset(&Entry, 0, sizeof(EC_T_LINK_FRAME_BUF_ENTRY));
    while (m_FrameBuffer.RemoveNoLock(Entry))
    {
        struct pcap_pkthdr FrameHeader;
        OsMemset(&FrameHeader, 0, sizeof(struct pcap_pkthdr));

        FrameHeader.TimeStamp.dwSec = (EC_T_DWORD)(Entry.Desc.qwTimestamp / 1000000000);
        FrameHeader.TimeStamp.dwUsec = (EC_T_DWORD)((Entry.Desc.qwTimestamp % 1000000000) / 1000);

        FrameHeader.caplen = FrameHeader.len = Entry.Desc.dwSize;
        WriteFrame(&FrameHeader, Entry.abyData);
    }
}

EC_T_VOID CPcapFileBufferedWriter::AddFrame(EC_T_DWORD dwFrameSize, EC_T_BYTE* pbyFrame)
{
    EC_T_LINK_FRAME_BUF_ENTRY Entry;
    OsMemset(&Entry, 0, sizeof(EC_T_LINK_FRAME_BUF_ENTRY));

    if ((EC_NULL == pbyFrame) || (0 == dwFrameSize)) return;

    if (dwFrameSize > ETHERNET_MAX_FRAMEBUF_LEN - sizeof(EC_T_LINK_FRAME_BUF_ENTRY_FRAME_DESC)) return;

    if (m_FrameBuffer.IsFull())
    {
        EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "CPcapRecorder overflow!"));
        return;
    }
    OsMemcpy(Entry.abyData, pbyFrame, dwFrameSize);

    SetFrameTimestamp(&Entry.Desc.qwTimestamp);

    /* CRC should be already stripped off */
    Entry.Desc.dwSize = dwFrameSize;
    OsDbgAssert(0 != dwFrameSize);
    m_FrameBuffer.Add(Entry);
}

EC_T_VOID CPcapFileBufferedWriter::SetFrameTimestamp(EC_T_UINT64* pqwTimestamp)
{
#if (defined INCLUDE_PCAP_RECORDER_OS_PERF_MEAS)
    if (0 == m_qwStartTimeCounterTicks)
    {
        m_qwStartTimeCounterTicks = OsMeasGetCounterTicks();
        *pqwTimestamp = 0;
    }
    else
    {
        EC_T_DWORD dw100kHzFrequency = OsMeasGet100kHzFrequency();
        EC_T_UINT64 qwDiff = OsMeasGetCounterTicks() - m_qwStartTimeCounterTicks;
        if (0 == dw100kHzFrequency)
        {
            *pqwTimestamp = 0;
        }
        else
        {
            /* convert to nsec */
            if (0 != (EC_HIDWORD(qwDiff) & 0xFFF00000))  /* for big values don't multiply to prevent from overrun */
            {
                *pqwTimestamp = (qwDiff / dw100kHzFrequency) * 10000;
            }
            else
            {
                *pqwTimestamp = (qwDiff * 10000) / dw100kHzFrequency;
            }
        }
    }
#elif (defined EC_VERSION_WINDOWS)
    *pqwTimestamp = OsQueryMsecCount();
    *pqwTimestamp = *pqwTimestamp * 1000000;
#else
    OsSystemTimeGet(pqwTimestamp);
#endif
}

EC_T_VOID CPcapRecorder::LogFrame(EC_T_DWORD dwLogFrameFlags, EC_T_DWORD dwFrameSize, EC_T_BYTE* pbyFrame)
{
    EC_T_LINK_FRAME_BUF_ENTRY Entry;
    OsMemset(&Entry, 0, sizeof(EC_T_LINK_FRAME_BUF_ENTRY));

    if ((EC_NULL == pbyFrame) || (0 == dwFrameSize)) return;

    if (dwFrameSize > ETHERNET_MAX_FRAMEBUF_LEN - sizeof(EC_T_LINK_FRAME_BUF_ENTRY_FRAME_DESC)) return;

    if (m_FrameBuffer.IsFull())
    {
        EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "CPcapRecorder overflow!"));
        return;
    }

    m_nLoggedFrameCount++;
    OsMemcpy(Entry.abyData, pbyFrame, dwFrameSize);

    SetFrameTimestamp(&Entry.Desc.qwTimestamp);

    /* CRC should be already stripped off */
    Entry.Desc.dwSize = dwFrameSize;
    OsDbgAssert(0 != dwFrameSize);
    m_FrameBuffer.Add(Entry);

    EcLogMsg(EC_LOG_LEVEL_VERBOSE_CYC, (pEcLogContext, EC_LOG_LEVEL_VERBOSE_CYC, "%d: CPcapRecorder::LogFrame(%s%s%s %s): frame %d, %d bytes\n",
        m_dwInstanceId, (m_szFileName ? m_szFileName : ""), (m_szFileName ? " " : ""),
        ((0 != (dwLogFrameFlags & EC_LOG_FRAME_FLAG_RED_FRAME)) ? "RED" : "MAIN"),
                      ((0 != (dwLogFrameFlags & EC_LOG_FRAME_FLAG_RX_FRAME)) ? "RX" : "TX"),
        m_nLoggedFrameCount, dwFrameSize));
}
#endif /* INCLUDE_PCAP_RECORDER */

#if (defined INCLUDE_PCAP_READER)
CPcapFileReader::CPcapFileReader()
{
    m_pfHandle = 0;
    m_qwFrameNumber = 0;
    m_lNextFrame = 0;
    m_bLimitedFrameCnt = EC_FALSE;
    m_nMaxFrames = 0;
    m_abyRecvFrameData[0] = '\0';
}

EC_T_DWORD CPcapFileReader::Open(const EC_T_CHAR * szFilename, int nSkipFrames)
{
    if (EC_NULL != m_pfHandle) OsFclose(m_pfHandle);
    m_pfHandle = OsFopen(szFilename, "rb");
    if (EC_NULL == m_pfHandle) return EC_E_OPENFAILED;

    OsFread(&m_FileHeader, sizeof(struct pcap_file_header), 1, m_pfHandle);
    m_qwFrameNumber = 0;
    m_lNextFrame = ftell(m_pfHandle);

    SkipFrames(nSkipFrames);

    m_bLimitedFrameCnt = EC_FALSE;
    m_nMaxFrames = 0;

    return EC_E_NOERROR;
}

EC_T_UINT64 CPcapFileReader::GetNextFrame(EC_T_LINK_FRAMEDESC* pFrame)
{
    if (m_bLimitedFrameCnt && (0 == m_nMaxFrames))
        return EC_FALSE;

    if (EC_NULL == m_pfHandle)
        return EC_FALSE;

    if (0 != fseek(m_pfHandle, m_lNextFrame, SEEK_SET))
        return EC_FALSE;

    if (1 != OsFread(&m_FrameHeader, sizeof(struct pcap_pkthdr), 1, m_pfHandle))
        return EC_FALSE;

    m_FrameHeader.caplen = EC_GET_FRM_DWORD(&m_FrameHeader.caplen);
    m_FrameHeader.len = EC_GET_FRM_DWORD(&m_FrameHeader.len);

    /* only return complete frames (TODO: also check if CRC is included for traces including CRC, s.a. presence information missing) */
    if (m_FrameHeader.len < ETHERNET_MIN_FRAME_LEN)
        return EC_FALSE;

    if (m_FrameHeader.len != (EC_T_DWORD)OsFread(m_abyRecvFrameData, 1, m_FrameHeader.len, m_pfHandle))
        return EC_FALSE;

    pFrame->pbyFrame = m_abyRecvFrameData;

    /* CRC may only be removed if present! (presence information missing) */
    pFrame->dwSize = m_FrameHeader.len /* - 4 */;

    /* timestamp */
    pFrame->qwTimestamp = (EC_T_UINT64)m_FrameHeader.TimeStamp.dwUsec * 1000ul + (EC_T_UINT64)m_FrameHeader.TimeStamp.dwSec * 1000000000ul;

    /* the next frame is directly appended to this frame in pcap */
    m_lNextFrame = m_lNextFrame + sizeof(struct pcap_pkthdr) + m_FrameHeader.len;

    if (m_bLimitedFrameCnt)
        m_nMaxFrames--;

    m_qwFrameNumber++;
    return m_qwFrameNumber;
}

EC_T_VOID CPcapFileReader::EnableAllFrames()
{
    m_bLimitedFrameCnt = EC_FALSE;
}

EC_T_VOID CPcapFileReader::SetMaxFrames(int nMaxFrames)
{
    m_bLimitedFrameCnt = EC_TRUE;
    m_nMaxFrames = nMaxFrames;
}

EC_T_VOID CPcapFileReader::SkipFrames(int nSkipFrames)
{
    for (int i = 0; i < nSkipFrames; i++)
    {
        EC_T_LINK_FRAMEDESC Frame;
        if (!GetNextFrame(&Frame))
        {
            break;
        }
    }
}
#endif /* INCLUDE_PCAP_READER */

/*-END OF SOURCE FILE--------------------------------------------------------*/
