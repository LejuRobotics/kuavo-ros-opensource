/*-----------------------------------------------------------------------------
 * EcLogging.h
 * Copyright                acontis technologies GmbH, Ravensburg, Germany
 * Response                 Stefan Zintgraf
 * Description              EC-Master application logging header
 *---------------------------------------------------------------------------*/

#ifndef INC_LOGGING
#define INC_LOGGING 1

/*-LOGGING-------------------------------------------------------------------*/
#ifndef G_pEcLogParms
extern struct _EC_T_LOG_PARMS G_aLogParms[];
#define G_pEcLogParms (&G_aLogParms[0])
#endif

/*-INCLUDES------------------------------------------------------------------*/
#ifndef INC_ECOS
#include "EcOs.h"
#endif

#include "stdio.h"

#if (!defined ECM_RUNTIME_LINKAGE) && (!defined EXCLUDE_ECM_RUNTIME_LINKAGE)
#define ECM_RUNTIME_LINKAGE
#endif

#if (!defined INCLUDE_PCAP_RECORDER_OS_PERF_MEAS) && (!defined EXCLUDE_PCAP_RECORDER_OS_PERF_MEAS)
#define INCLUDE_PCAP_RECORDER_OS_PERF_MEAS
#endif

/* include Frame Spy for EC-Master and EC-Simulator */
#if (!defined INCLUDE_FRAME_SPY) && (!defined EXCLUDE_FRAME_SPY)
#define INCLUDE_FRAME_SPY
#endif

#if (!defined INCLUDE_FILE_LOGGING) && (!defined EXCLUDE_FILE_LOGGING)
#define INCLUDE_FILE_LOGGING
#endif

#if (!defined NO_OS)
#if (!defined INCLUDE_LOG_TASK) && (!defined EXCLUDE_LOG_TASK)
#define INCLUDE_LOG_TASK
#endif
#endif

#if (defined INCLUDE_EC_SIMULATOR)
#if (!defined INC_ECSIMULATOR)
#include "EcSimulator.h"
#endif
#elif (defined INCLUDE_EC_MONITOR)
#if (!defined INC_ECMONITOR)
#include "EcMonitor.h"
#endif
#elif (defined ECM_RUNTIME_LINKAGE)
#if (!defined INC_ATETHERCAT)
#include "AtEthercat.h"
#endif
#endif

#ifndef INC_ECLOG
#include "EcLog.h"
#endif

#ifndef INC_ECTIMER
#include "EcTimer.h"
#endif

#if (defined INCLUDE_FRAME_SPY)
#ifndef INC_LIST
#include "EcList.h"
#endif
#endif
#if (defined INCLUDE_PCAP_RECORDER)
#ifndef INC_ECFIFO
#include "EcFiFo.h"
#endif
#ifndef INC_ECTHREAD
#include "EcThread.h"
#endif
#endif /* INCLUDE_PCAP_RECORDER */

#ifndef INC_ETHERNETSERVICES
#include "EthernetServices.h"
#endif
#ifndef INC_ECLINK
#include "EcLink.h"
#endif

/*-LOGGING-------------------------------------------------------------------*/
/* set to log level, callback, context to log parms if available */
#ifdef  pEcLogParms
#ifndef dwEcLogLevel
#define dwEcLogLevel  (pEcLogParms->dwLogLevel)
#endif
#ifndef pLogMsgCallback
#define pLogMsgCallback (pEcLogParms->pfLogMsg)
#endif
#ifndef pEcLogContext
#define pEcLogContext (pEcLogParms->pLogContext)
#endif
#endif /* pEcLogParms */

/*-DEFINES-------------------------------------------------------------------*/
#define DEFAULT_LOG_STACK_SIZE     0x4000

#if !(defined EC_DEMO_TINY)

#define MAX_MESSAGE_SIZE              512 /* maximum size of a single message */
#ifndef DEFAULT_LOG_MSG_BUFFER_SIZE
#define DEFAULT_LOG_MSG_BUFFER_SIZE  1000 /* number of buffered messages */
#endif
#define DEFAULT_ERR_MSG_BUFFER_SIZE   500 /* number of buffered messages */
#define DEFAULT_DCM_MSG_BUFFER_SIZE  1000 /* number of buffered messages */

#else

#define MAX_MESSAGE_SIZE              160 /* maximum size of a single message */
#define DEFAULT_LOG_MSG_BUFFER_SIZE    30 /* number of buffered messages */
#define DEFAULT_ERR_MSG_BUFFER_SIZE    20 /* number of buffered messages */
#define DEFAULT_DCM_MSG_BUFFER_SIZE    20 /* number of buffered messages */

#endif /* !(defined EC_DEMO_TINY) */

#if (!defined PCAP_RECORDER_BUF_FRAME_CNT) && (defined INCLUDE_PCAP_RECORDER)
#define PCAP_RECORDER_BUF_FRAME_CNT 50
#endif

/* log thread priority (very low) */
#if defined WIN32 && !defined UNDER_CE && !defined RTOS_32
 #define LOG_ROLLOVER      ((EC_T_WORD)0)
#elif (defined RTOS_32)
 #define LOG_ROLLOVER   ((EC_T_WORD)3000)
#else
 #define LOG_ROLLOVER  ((EC_T_WORD)10000)
#endif

#define MAX_PATH_LEN                  256
#define MAX_EXT_LEN                     4

#ifndef MAX_NUMOF_LOG_INSTANCES
/* MAX_NUMOF_MASTER_INSTANCES defined in AtEthercat.h */
#ifdef  MAX_NUMOF_MASTER_INSTANCES
#define MAX_NUMOF_LOG_INSTANCES MAX_NUMOF_MASTER_INSTANCES
#else
#define MAX_NUMOF_LOG_INSTANCES         1
#endif
#endif

/*-GLOBAL VARIABLES-----------------------------------------------------------*/
#if (defined INCLUDE_FILE_LOGGING)
extern EC_T_BOOL bLogFileEnb;
#endif
extern EC_T_LOG_PARMS G_aLogParms[MAX_NUMOF_LOG_INSTANCES];

/*-TYPEDEFS------------------------------------------------------------------*/
typedef struct _LOG_MSG_DESC
{
    EC_T_BOOL  bValid;                /* entry is valid */
    EC_T_DWORD dwSeverity;            /* logging severity */
    EC_T_CHAR* szMsg;                 /* message without timestamp, format "...\n"  */
    EC_T_DWORD dwMsgLen;              /* message without timestamp length */
    EC_T_CHAR* szMsgBuffer;           /* message with timestamp, format "%10d: ...\n" */
    EC_T_DWORD dwMsgBufferLen;        /* message with timestamp length */
    EC_T_DWORD dwMsgTimestamp;        /* timestamp values */
    EC_T_DWORD dwMsgThreadId;         /* threadId values */
    EC_T_DWORD dwMsgsDropped;         /* count of new messages denied to insert after this message, because no empty message buffer available */
} LOG_MSG_DESC;

typedef struct _MSG_BUFFER_DESC
{
    struct _MSG_BUFFER_DESC* pNextMsgBuf;       /* link to next message buffer */
    LOG_MSG_DESC*   paMsg;                      /* array of messages */
    EC_T_DWORD  dwMsgSize;                      /* message size */
    EC_T_DWORD  dwNumMsgs;                      /* number of messages */
    EC_T_DWORD  dwNextEmptyMsgIndex;            /* index of next empty message buffer */
    EC_T_DWORD  dwNextPrintMsgIndex;            /* index of next message buffer to print */
    EC_T_DWORD  dwDropReportMsgIndex;           /* index message to report buffer full */
    EC_T_CHAR   szMsgLogFileName[MAX_PATH_LEN]; /* message log file name */
    EC_T_CHAR   szMsgLogFileExt[4];             /* message log file extension */
#if (defined INCLUDE_FILE_LOGGING)
    FILE*       pfMsgFile;                      /* file pointer for message log file */
#endif
    EC_T_BOOL   bPrintTimestamp;                /* EC_TRUE if a timestamp shall be printed in the log file */
    EC_T_BOOL   bPrintConsole;                  /* EC_TRUE if the message shall be printed on the console */
    EC_T_BOOL   bIsInitialized;                 /* EC_TRUE if message buffer is initialized */
    EC_T_WORD   wLogFileIndex;                  /* Index of current log file */
    EC_T_WORD   wEntryCounter;                  /* Entries to detect roll over */
    EC_T_WORD   wEntryCounterLimit;             /* Entries before roll over */
    EC_T_WORD   wRes;
    /* logging into memory buffer */
    EC_T_CHAR   szLogName[MAX_PATH_LEN];        /* name of the logging buffer */
    EC_T_BYTE*  pbyLogMemory;                   /* if != EC_NULL then log into memory instead of file */
    EC_T_BYTE*  pbyNextLogMsg;                  /* pointer to next logging message */
    EC_T_DWORD  dwLogMemorySize;                /* size of logging memory */
    EC_T_BOOL   bLogBufferFull;                 /* EC_TRUE if log buffer is full */
    /* skip identical messages */
    EC_T_BOOL   bSkipDuplicateMessages;         /* if set to EC_TRUE, then multiple identical messages will not be printed out */
    EC_T_DWORD  dwNumDuplicates;                /* if 0, the new message is not duplicated */
    EC_T_CHAR*  pszLastMsg;                     /* pointer to last message (points into message buffer) */
    EC_T_BOOL   bNewLine;                       /* EC_TRUE if last message printed with CrLf */
} MSG_BUFFER_DESC;

/*-FORWARD DECLARATIONS------------------------------------------------------*/
struct TETYPE_EC_CMD_HEADER;

/*-CLASS---------------------------------------------------------------------*/
class CAtEmLogging
{
public:
                CAtEmLogging(                   EC_T_VOID                                           );
    virtual    ~CAtEmLogging() {}

    EC_T_DWORD  LogDcm(                         const
                                                EC_T_CHAR*              szFormat,...                );
    EC_T_VOID   InitLogging(                    EC_T_DWORD              dwInstanceId,
                                                EC_T_WORD               wRollOver,
                                                EC_T_DWORD              dwPrio,
                                                EC_T_CPUSET             CpuSet,
                                                EC_T_CHAR*              szFilenamePrefix = EC_NULL,
                                                EC_T_DWORD              dwStackSize = DEFAULT_LOG_STACK_SIZE,
                                                EC_T_DWORD              dwLogMsgBufferSize = DEFAULT_LOG_MSG_BUFFER_SIZE,
                                                EC_T_DWORD              dwErrMsgBufferSize = DEFAULT_ERR_MSG_BUFFER_SIZE,
                                                EC_T_DWORD              dwDcmMsgBufferSize = DEFAULT_DCM_MSG_BUFFER_SIZE);
    /* depreated */
    EC_T_VOID   InitLogging(                    EC_T_DWORD              dwInstanceId,
                                                EC_T_DWORD              dwLogLevel,
                                                EC_T_WORD               wRollOver,
                                                EC_T_DWORD              dwPrio,
                                                EC_T_CPUSET             CpuSet,
                                                EC_T_CHAR*              szFilenamePrefix = EC_NULL,
                                                EC_T_DWORD              dwStackSize = DEFAULT_LOG_STACK_SIZE,
                                                EC_T_DWORD              dwLogMsgBufferSize = DEFAULT_LOG_MSG_BUFFER_SIZE,
                                                EC_T_DWORD              dwErrMsgBufferSize = DEFAULT_ERR_MSG_BUFFER_SIZE,
                                                EC_T_DWORD              dwDcmMsgBufferSize = DEFAULT_DCM_MSG_BUFFER_SIZE)
                                                { EC_UNREFPARM(dwLogLevel); InitLogging(dwInstanceId, wRollOver, dwPrio, CpuSet, szFilenamePrefix, dwStackSize, dwLogMsgBufferSize, dwErrMsgBufferSize, dwDcmMsgBufferSize); }

    EC_T_VOID   SetLogMsgBuf(                   EC_T_BYTE*              pbyLogMem,
                                                EC_T_DWORD              dwSize                      );
    EC_T_VOID   SetLogErrBuf(                   EC_T_BYTE*              pbyLogMem,
                                                EC_T_DWORD              dwSize                      );
    EC_T_VOID   SetLogDcmBuf(                   EC_T_BYTE*              pbyLogMem,
                                                EC_T_DWORD              dwSize                      );
    EC_T_VOID   DeinitLogging(                  EC_T_VOID                                           );
    EC_T_VOID   tAtEmLog(                       EC_T_VOID*              pvParms                     );
    EC_T_VOID   ProcessAllMsgs(                 EC_T_VOID                                           );

    struct _MSG_BUFFER_DESC*  AddLogBuffer(     EC_T_DWORD              dwInstanceId,
#if (defined INCLUDE_FILE_LOGGING)
                                                EC_T_WORD               wRollOver,
#endif
                                                EC_T_DWORD              dwBufferSize,
                                                EC_T_BOOL               bSkipDuplicates,
                                                EC_T_CHAR*              szLogName,
#if (defined INCLUDE_FILE_LOGGING)
                                                EC_T_CHAR*              szLogFilename,
                                                EC_T_CHAR*              szLogFileExt,
#endif
                                                EC_T_BOOL               bPrintConsole,
                                                EC_T_BOOL               bPrintTimestamp             );

    static
    EC_T_VOID   SetMsgBuf(                      MSG_BUFFER_DESC*        pMsgBufferDesc,
                                                EC_T_BYTE*              pbyLogMem,
                                                EC_T_DWORD              dwSize                      );

    EC_T_DWORD  InsertNewMsgVa(                 MSG_BUFFER_DESC*        pMsgBufferDesc,
                                                EC_T_DWORD              dwLogMsgSeverity,
                                                const
                                                EC_T_CHAR*              szFormat,
                                                EC_T_VALIST             vaArgs                      );

    static
    EC_T_DWORD  LogMsg(                         struct _EC_T_LOG_CONTEXT* pContext,
                                                EC_T_DWORD                dwLogMsgSeverity,
                                                const
                                                EC_T_CHAR*                szFormat,
                                                ...                                                 );

    static
    EC_T_DWORD LogMsgStub(                      struct _EC_T_LOG_CONTEXT* pContext,
                                                EC_T_DWORD                dwLogMsgSeverity,
                                                const EC_T_CHAR*          szFormat,
                                                ...                                                 );

    static
    EC_T_DWORD LogMsgOsPrintf(                  struct _EC_T_LOG_CONTEXT* pContext,
                                                EC_T_DWORD                dwLogMsgSeverity,
                                                const EC_T_CHAR*          szFormat,
                                                ...                                                 );

    virtual
    EC_T_VOID   OnLogMsg(EC_T_CHAR* szMsg)      { EC_UNREFPARM(szMsg); }

    virtual
    EC_T_BOOL   FilterMsg(EC_T_CHAR* szMsg)     { EC_UNREFPARM(szMsg); return EC_FALSE; }

#if (defined INCLUDE_FILE_LOGGING)
    EC_T_DWORD  SetLogDir(                      EC_T_CHAR*              szLogDir                    );
#endif

#if (!defined EC_EAP)
    EC_T_DWORD  PrintPerfMeas(                  EC_T_DWORD                dwPerfMeasInstanceId0,
                                                EC_T_DWORD                dwPerfMeasInstanceId1,
                                                struct _EC_T_LOG_CONTEXT* pContext                  );
#endif
    EC_T_DWORD  PrintHistogramAsCsv(            EC_T_DWORD                dwInstanceId              );
    EC_T_DWORD  PrintHistogramLine(             const EC_T_CHAR*          szFormat, ...             );

    EC_T_VOID   FinalizeMsg(                    LOG_MSG_DESC*           pMsgDesc                    );

    virtual
    EC_T_VOID   PrintConsole(                   EC_T_CHAR*              szMsg                       );

    virtual
    EC_T_VOID   PrintMsg(                       LOG_MSG_DESC*           pMsgDesc                    );

private:
    static
    EC_T_BOOL   InitMsgBuffer(                  MSG_BUFFER_DESC*        pMsgBufferDesc,
                                                EC_T_DWORD              dwMsgSize,
                                                EC_T_DWORD              dwNumMsgs,
                                                EC_T_BOOL               bSkipDuplicates,
                                                EC_T_BOOL               bPrintConsole,
                                                EC_T_BOOL               bPrintTimestamp,
#if (defined INCLUDE_FILE_LOGGING)
                                                EC_T_CHAR*              szMsgLogFileName,
                                                EC_T_CHAR*              szMsgLogFileExt,
                                                EC_T_WORD               wRollOver,
#endif
                                                EC_T_CHAR*              szLogName                   );

    EC_T_VOID   DeinitMsgBuffer(                MSG_BUFFER_DESC*        pMsgBufferDesc              );

    EC_T_VOID   ProcessMsgs(                    MSG_BUFFER_DESC*        pMsgBufferDesc              );

    static
    EC_T_VOID   SelectNextLogMemBuffer(         MSG_BUFFER_DESC*        pMsgBufferDesc              );

    static
    EC_T_VOID   tAtEmLogWrapper(                EC_T_VOID*              pvParms                     );

#if (!defined EC_EAP)
    static
    EC_T_VOID   PrintPerfMeasInternal(          struct _EC_T_LOG_CONTEXT* pContext,
                                                EC_T_DWORD                dwNumOf,
                                                EC_T_PERF_MEAS_VAL*       aPerfMeasVal,
                                                EC_T_PERF_MEAS_INFO*      aPerfMeasInfo               );
#endif

    EC_T_PVOID              m_pvLogThreadObj;
    EC_T_BOOL               m_bLogTaskRunning;
    EC_T_BOOL               m_bShutdownLogTask;
    EC_T_DWORD              m_dwInstanceId;
    MSG_BUFFER_DESC*        m_pFirstMsgBufferDesc;          /* pointer to first message buffer */
    MSG_BUFFER_DESC*        m_pLastMsgBufferDesc;           /* link to last message buffer */
    MSG_BUFFER_DESC*        m_pAllMsgBufferDesc;            /* buffer for all messages */
    MSG_BUFFER_DESC*        m_pErrorMsgBufferDesc;          /* buffer for application error messages */
    MSG_BUFFER_DESC*        m_pDcmMsgBufferDesc;            /* DCM buffer */
    MSG_BUFFER_DESC*        m_pHistMsgBufferDesc;           /* Histogram buffer */
    EC_T_CHAR*              m_pchTempbuffer;
    EC_T_VOID*              m_poInsertMsgLock;              /* lock object for inserting new messages */
    EC_T_VOID*              m_poProcessMsgLock;             /* lock object for processing messages */
    EC_T_BOOL               m_bDbgMsgHookEnable;

    CEcTimer                m_oMsgTimeout;
    CEcTimer                m_oSettlingTimeout;
    EC_T_DWORD              m_dwNumMsgsSinceMsrmt;
    EC_T_BOOL               m_bSettling;

#if (defined INCLUDE_FILE_LOGGING)
    EC_T_CHAR               m_pchLogDir[MAX_PATH_LEN];      /* directory for all EtherCAT logging files */
#endif

#if (!defined EC_EAP)
    EC_T_DWORD              m_dwPerfMeasNumOf;              /* number of performance measurements */
    EC_T_PERF_MEAS_VAL*     m_aPerfMeasVal;                 /* performance measurement result buffer */
    EC_T_PERF_MEAS_INFO*    m_aPerfMeasInfo;                /* performance measurement info buffer */
    struct
    {
        EC_T_DWORD          dwInternalNumOf;
        EC_T_DWORD          dwAppNumOf;
    } m_aData[2]; /* 0: EC-Master / EC-Monitor, 1: EC-Simulator*/
#endif

    static EC_T_BOOL        s_bLogParmsArrayInitialized;
};

#ifdef INCLUDE_FRAME_SPY
/** \brief installs multiple frame loggers to a master instance */
class CFrameLogMultiplexer
{
public:
    virtual ~CFrameLogMultiplexer();

    static EC_T_DWORD CreateInstances(EC_T_VOID);
    static EC_T_DWORD DeleteInstances(EC_T_VOID);
    static CFrameLogMultiplexer* GetInstance(EC_T_DWORD dwInstanceId);

    /** \brief add logger to list */
    static EC_T_VOID AddFrameLogger(EC_T_DWORD dwInstanceId, EC_T_VOID* pvContext, EC_T_PFLOGFRAME_CB pvLogFrameCallBack);
    /** \brief search logger in list and remove */
    static EC_T_VOID RemoveFrameLogger(EC_T_DWORD dwInstanceId, EC_T_VOID* pvContext, EC_T_PFLOGFRAME_CB pvLogFrameCallBack);

protected:
    virtual EC_T_VOID AddFrameLogger(EC_T_VOID* pvContext, EC_T_PFLOGFRAME_CB pvLogFrameCallBack)
    {
        EC_T_FRAME_LOGGER_DESC LoggerDesc;
        LoggerDesc.pvContext = pvContext;
        LoggerDesc.pvLogFrameCallBack = pvLogFrameCallBack;
        OsLock(m_pvLoggerListLock);
        m_LoggerList.AddTail(LoggerDesc);
#if (defined INCLUDE_EC_SIMULATOR)
        esLogFrameEnable(m_dwInstanceId, StaticFrameHandler, this);
#elif (defined INCLUDE_EC_MASTER)
        emLogFrameEnable(m_dwInstanceId, StaticFrameHandler, this);
#endif
        OsUnlock(m_pvLoggerListLock);
    }

    EC_T_VOID RemoveFrameLogger(EC_T_VOID* pvContext, EC_T_PFLOGFRAME_CB pvLogFrameCallBack)
    {
        OsLock(m_pvLoggerListLock);
        for (CLoggerDescList::CNode* pNode = m_LoggerList.GetFirstNode(); EC_NULL != pNode; m_LoggerList.GetNext(pNode))
        {
            if ((pNode->data.pvLogFrameCallBack == pvLogFrameCallBack) &&
                (pNode->data.pvContext == pvContext))
            {
                m_LoggerList.RemoveAt(pNode);
                /* don't call LogFrameDisable to prevent crash due to race condition with JobTask */
                break;
            }
        }
        OsUnlock(m_pvLoggerListLock);
    }

    CFrameLogMultiplexer();

    /* logger list type */
    typedef struct
    {
        EC_T_VOID*         pvContext;
        EC_T_PFLOGFRAME_CB pvLogFrameCallBack;
    } EC_T_FRAME_LOGGER_DESC;
    typedef CList<EC_T_FRAME_LOGGER_DESC, EC_T_FRAME_LOGGER_DESC> CLoggerDescList;

    /* logger list instance */
    EC_T_VOID*      m_pvLoggerListLock;
    CLoggerDescList m_LoggerList;

    /** \brief parses EtherCAT datagrams. dwLogFlags: see EC_LOG_FRAME_FLAG_... */
    void FrameHandler(EC_T_DWORD dwLogFrameFlags, EC_T_DWORD dwFrameSize, EC_T_BYTE* pbyFrame)
    {
        OsLock(m_pvLoggerListLock);
        for (CLoggerDescList::CNode* pNode = m_LoggerList.GetFirstNode(); EC_NULL != pNode; m_LoggerList.GetNext(pNode))
        {
            pNode->data.pvLogFrameCallBack(pNode->data.pvContext, dwLogFrameFlags, dwFrameSize, pbyFrame);
        }
        OsUnlock(m_pvLoggerListLock);
    }

public:
    /** \brief static proxy */
    static EC_T_VOID StaticFrameHandler(EC_T_VOID* pvContext, EC_T_DWORD dwLogFlags, EC_T_DWORD dwFrameSize, EC_T_BYTE* pbyFrame)
    {
        ((CFrameLogMultiplexer*)pvContext)->FrameHandler(dwLogFlags, dwFrameSize, pbyFrame);
    }
    static CFrameLogMultiplexer* G_aLogMultiplexer;
    static EC_T_BOOL G_bAutoDispose;

    EC_T_DWORD m_dwInstanceId;
};

/** \brief parses EtherCAT datagrams from EC-Master frames and calls virtual handler */
class CFrameSpy
{
public:
    /** \brief Reset FrameSpy. Set frame handler at master if dwInstanceId != 0xffff */
    virtual void Reset(EC_T_DWORD dwInstanceId)
    {
        if (0xffff != dwInstanceId)
        {
            Install(dwInstanceId);
        }
        m_dwInstanceId = dwInstanceId;
    }

    CFrameSpy(EC_T_DWORD dwInstanceId = 0xffff) : m_dwInstanceId(0xffff)
    {
        Reset(dwInstanceId);
    }
    virtual ~CFrameSpy();

    EC_T_VOID Install(EC_T_DWORD dwInstanceId);
    EC_T_VOID Uninstall(EC_T_VOID);

    /** \brief static proxy */
    static EC_T_VOID StaticFrameHandler(EC_T_VOID* pvContext, EC_T_DWORD dwLogFlags, EC_T_DWORD dwFrameSize, EC_T_BYTE* pbyData)
    {
        ((CFrameSpy*)pvContext)->FrameHandler(dwLogFlags, dwFrameSize, pbyData);
    }
    /** \brief parses EtherCAT datagrams. dwLogFlags: see EC_LOG_FRAME_FLAG_... */
    virtual void FrameHandler(EC_T_DWORD dwLogFrameFlags, EC_T_DWORD dwFrameSize, EC_T_BYTE* pbyFrame);

    /** \brief DatagramHandler called by CFrameSpy::FrameHandler */
    virtual void DatagramHandler(EC_T_DWORD dwLogFrameFlags, EC_T_DWORD dwFrameSize, EC_T_BYTE* pFrame, struct TETYPE_EC_CMD_HEADER* pEcCmdHdr, EC_T_WORD wCmdLen)
    {
        EC_UNREFPARM(dwLogFrameFlags); EC_UNREFPARM(dwFrameSize); EC_UNREFPARM(pFrame); EC_UNREFPARM(pEcCmdHdr); EC_UNREFPARM(wCmdLen);
    }
protected:
    EC_T_DWORD m_dwInstanceId;
};
#endif

#if (defined INCLUDE_PCAP_RECORDER) || (defined INCLUDE_PCAP_READER)
#include EC_PACKED_INCLUDESTART(1)
struct pcap_file_header {
    EC_T_DWORD magic;
    EC_T_WORD version_major;
    EC_T_WORD version_minor;
    EC_T_INT thiszone;   /* gmt to local correction */
    EC_T_DWORD sigfigs;  /* accuracy of timestamps */
    EC_T_DWORD snaplen;  /* max length saved portion of each pkt */
    EC_T_DWORD linktype; /* data link type (LINKTYPE_*) */
} EC_PACKED(1);
struct pcap_pkthdr {
    struct {
        EC_T_DWORD dwSec; /* struct timeval */
        EC_T_DWORD dwUsec; /* struct timeval */
    } EC_PACKED(1) TimeStamp;
    EC_T_DWORD caplen;       /* length of portion present */
    EC_T_DWORD len;          /* length this packet (off wire) */
} EC_PACKED(1);
#include EC_PACKED_INCLUDESTOP

class CPcapFileProcessor
{
public:
    virtual EC_T_VOID Close(EC_T_VOID)
    {
        if (EC_NULL != m_pfHandle)
        {
            OsFclose(m_pfHandle);
            m_pfHandle = EC_NULL;
        }
    }
    CPcapFileProcessor()
    {
        m_pfHandle = EC_NULL;
        OsMemset(&m_FileHeader, 0, sizeof(struct pcap_file_header));
        OsMemset(&m_FrameHeader, 0, sizeof(struct pcap_pkthdr));
    }
    virtual ~CPcapFileProcessor()
    {
        Close();
    }

protected:
    FILE*                   m_pfHandle;
    struct pcap_file_header m_FileHeader;
    struct pcap_pkthdr      m_FrameHeader;
};
#endif /* INCLUDE_PCAP_RECORDER || defined INCLUDE_PCAP_READER */

#if (defined INCLUDE_PCAP_READER)
class CPcapFileReader : public CPcapFileProcessor
{
public:
    CPcapFileReader();
    EC_T_DWORD Open(const EC_T_CHAR* szFilename, int nSkipFrames = 0);
    EC_T_UINT64 GetNextFrame(EC_T_LINK_FRAMEDESC* pFrame);
    EC_T_VOID EnableAllFrames();
    EC_T_VOID SetMaxFrames(int nMaxFrames);
    EC_T_VOID SkipFrames(int nSkipFrames);

private:
    long        m_lNextFrame;
    EC_T_BOOL   m_bLimitedFrameCnt;
    EC_T_UINT64 m_qwFrameNumber;
    int         m_nMaxFrames;
    EC_T_BYTE   m_abyRecvFrameData[ETHERNET_MAX_FRAMEBUF_LEN];
};
#endif /* INCLUDE_PCAP_READER */

#if (defined INCLUDE_PCAP_RECORDER)
class CPcapFileWriter : public CPcapFileProcessor
{
public:
    CPcapFileWriter()
    {
        m_RotationDesc.dwFileCnt = 0;
        m_RotationDesc.dwMaxFileCnt = 0;
        m_RotationDesc.dwFrameCnt = 0;
        m_RotationDesc.dwMaxFrameCnt = 0;
        m_RotationDesc.qwFileSize = 0;
        m_RotationDesc.qwMaxFileSize = 0;
        m_szFileName = EC_NULL;
    }
    virtual ~CPcapFileWriter() { SafeOsFree(m_szFileName); }

private:
    EC_T_DWORD OpenPrivate(const EC_T_CHAR* szFilename)
    {
        if ((EC_NULL == szFilename) || (OsStrlen(szFilename) == 0))
        {
            return EC_E_INVALIDPARM;
        }
        if (EC_NULL != m_pfHandle)
        {
            return EC_E_INVALIDSTATE;
        }
        m_FileHeader.linktype = 1;
        m_FileHeader.magic = 0xa1b2c3d4;
        m_FileHeader.version_major = 2;
        m_FileHeader.version_minor = 4;
        m_FileHeader.snaplen = 65535;
        m_FileHeader.sigfigs = 0;
        m_FileHeader.thiszone = 0;

        m_pfHandle = OsFopen(szFilename, "wb+");
        if (EC_NULL == m_pfHandle)
        {
            return EC_E_INVALIDPARM;
        }

        OsFwrite(&m_FileHeader, sizeof(struct pcap_file_header), 1, m_pfHandle);
        return EC_E_NOERROR;
    }

public:
    EC_T_DWORD Open(const EC_T_CHAR* szFilename)
    {
        EC_T_DWORD dwRes = EC_E_ERROR;
        EC_T_DWORD dwRetVal = EC_E_ERROR;

        EC_T_DWORD dwFileNameLen = (EC_T_DWORD)OsStrlen(szFilename);

        m_szFileName = (EC_T_CHAR*)OsMalloc(dwFileNameLen + 1 + OsStrlen(".00000.pcap"));
        if (EC_NULL == m_szFileName)
        {
            dwRetVal = EC_E_NOMEMORY;
            goto Exit;
        }
        OsStrcpy(m_szFileName, szFilename);
        OsSnprintf(&m_szFileName[dwFileNameLen], (EC_T_DWORD)OsStrlen(".00000.pcap") + 1, ".%05d.pcap", m_RotationDesc.dwFileCnt);

        dwRes = OpenPrivate(m_szFileName);
        m_szFileName[dwFileNameLen] = '\0';
        if (EC_E_NOERROR != dwRes)
        {
            dwRetVal = dwRes;
            goto Exit;
        }

        dwRetVal = EC_E_NOERROR;
    Exit:
        return dwRetVal;
    }

    EC_INLINESTART EC_T_BOOL IsRotationNeeded(struct pcap_pkthdr* pFrameHeader)
    {
        return m_RotationDesc.oTimer.IsElapsed()
            || ((m_RotationDesc.dwMaxFrameCnt > 0) && (m_RotationDesc.dwFrameCnt >= m_RotationDesc.dwMaxFrameCnt))
            || ((m_RotationDesc.qwMaxFileSize > 0) && (m_RotationDesc.qwFileSize + sizeof(struct pcap_pkthdr) + pFrameHeader->len > m_RotationDesc.qwMaxFileSize));
    } EC_INLINESTOP

    EC_T_VOID Rotate()
    {
        EC_T_DWORD dwFileNameLen = (EC_T_DWORD)OsStrlen(m_szFileName);
        if (m_RotationDesc.dwFileCnt > m_RotationDesc.dwMaxFileCnt)
        {
            m_RotationDesc.dwFileCnt = 0;
        }
        OsSnprintf(&m_szFileName[dwFileNameLen], (EC_T_DWORD)OsStrlen(".00000.pcap") + 1, ".%05d.pcap", m_RotationDesc.dwFileCnt);
        CPcapFileProcessor::Close();
        OpenPrivate(m_szFileName);
        m_szFileName[dwFileNameLen] = '\0';

        m_RotationDesc.dwFileCnt++;
        m_RotationDesc.dwFrameCnt = 0;
        m_RotationDesc.qwFileSize = sizeof(struct pcap_file_header);
        if (m_RotationDesc.oTimer.IsStarted()) m_RotationDesc.oTimer.Restart();
    }

    EC_T_BOOL WriteFrame(struct pcap_pkthdr* pFrameHeader, EC_T_BYTE* pbyFrame)
    {
        if (EC_NULL == m_pfHandle)
            return EC_FALSE;

        if (IsRotationNeeded(pFrameHeader))
            Rotate();

        if (1 != OsFwrite(pFrameHeader, sizeof(struct pcap_pkthdr), 1, m_pfHandle))
        {
            return EC_FALSE;
        }
        m_RotationDesc.qwFileSize = m_RotationDesc.qwFileSize + sizeof(struct pcap_pkthdr);

        if (pFrameHeader->len != OsFwrite(pbyFrame, 1, pFrameHeader->len, m_pfHandle))
        {
            return EC_FALSE;
        }
        m_RotationDesc.qwFileSize = m_RotationDesc.qwFileSize + pFrameHeader->len;

        m_RotationDesc.dwFrameCnt++;
        return EC_TRUE;
    }

    EC_T_VOID SetLogRotation(EC_T_DWORD dwMaxFileCnt,
        EC_T_DWORD dwMaxFrameCnt, EC_T_UINT64 qwMaxFileSize, EC_T_DWORD dwInterval)
    {
        m_RotationDesc.dwMaxFileCnt = dwMaxFileCnt;
        m_RotationDesc.dwMaxFrameCnt = dwMaxFrameCnt;
        m_RotationDesc.qwMaxFileSize = qwMaxFileSize;
        if (dwInterval > 0)
        {
            m_RotationDesc.oTimer.Start(dwInterval);
        }
        Rotate();
    }

protected:
    struct
    {
        EC_T_DWORD   dwFileCnt;
        EC_T_DWORD   dwMaxFileCnt;
        EC_T_DWORD   dwFrameCnt;
        EC_T_DWORD   dwMaxFrameCnt;
        EC_T_UINT64  qwFileSize;
        EC_T_UINT64  qwMaxFileSize;
        CEcTimer     oTimer;
    } m_RotationDesc;

    EC_T_CHAR*       m_szFileName;
};

typedef struct
{
    EC_T_DWORD  dwSize;      /*< size of the frame buffer */
    EC_T_UINT64 qwTimestamp; /*< Send or receive time point */
} EC_T_LINK_FRAME_BUF_ENTRY_FRAME_DESC;
typedef struct
{
    EC_T_LINK_FRAME_BUF_ENTRY_FRAME_DESC Desc;
    EC_T_BYTE abyData[ETHERNET_MAX_FRAMEBUF_LEN - sizeof(EC_T_LINK_FRAME_BUF_ENTRY_FRAME_DESC)];
} EC_T_LINK_FRAME_BUF_ENTRY;

class CPcapFileBufferedWriter : public CPcapFileWriter, public CEcThread
{
public:
    CPcapFileBufferedWriter(EC_T_DWORD dwBufCnt, EC_T_CPUSET cpuAffinity /* EC_CPUSET_ZERO */, EC_T_DWORD dwPrio /* LOG_THREAD_PRIO */, EC_T_DWORD dwStackSize /* DEFAULT_LOG_STACK_SIZE */)
        : m_FrameBuffer(0 /* not used */, EC_NULL, (EC_T_CHAR*)"CPcapFileBufferedWriter"), m_cpuAffinity(cpuAffinity), m_dwPrio(dwPrio), m_dwStackSize(dwStackSize)
    {
        EC_UNREFPARM(dwBufCnt);
#if (defined INCLUDE_PCAP_RECORDER_OS_PERF_MEAS)
        m_qwStartTimeCounterTicks = 0;

        if (0 == OsMeasGet100kHzFrequency())
        {
            OsMeasCalibrate(0);
        }
#endif
    }
    virtual EC_T_DWORD InitInstance(EC_T_DWORD dwBufCnt)
    {
        return m_FrameBuffer.InitInstance(dwBufCnt);
    }
    EC_T_VOID StartThread()
    {
        Start(GetLogParms(), ThreadStep, (void*)this, "CLogThread", m_cpuAffinity, m_dwPrio, m_dwStackSize, 15000);
    }
    virtual ~CPcapFileBufferedWriter()
    {
        Close();
    }
    EC_T_VOID SetFrameTimestamp(EC_T_UINT64* pqwTimestamp);

    EC_T_VOID AddFrame(EC_T_DWORD dwFrameSize, EC_T_BYTE* pbyFrame);

    virtual EC_T_VOID Close(EC_T_VOID)
    {
        Stop(15000);
        FlushBuffer();
        CPcapFileProcessor::Close();
    }

    static EC_T_VOID ThreadStep(EC_T_PVOID pvParams)
    {
        ((CPcapFileBufferedWriter*)pvParams)->FlushBuffer();
        OsSleep(1);
    }
    EC_T_VOID FlushBuffer(EC_T_VOID);

protected:
    CFiFoListDyn<EC_T_LINK_FRAME_BUF_ENTRY> m_FrameBuffer;
    EC_T_CPUSET m_cpuAffinity;
    EC_T_DWORD  m_dwPrio;
    EC_T_DWORD  m_dwStackSize;

#if (defined INCLUDE_PCAP_RECORDER_OS_PERF_MEAS)
    EC_T_UINT64 m_qwStartTimeCounterTicks;
#endif
};

/** \brief writes frames to .pcap files */
class CPcapRecorder : public CPcapFileBufferedWriter
{
public:
#if (defined LOG_THREAD_PRIO) && (defined DEFAULT_LOG_STACK_SIZE)
   CPcapRecorder();
   CPcapRecorder(EC_T_CPUSET cpuAffinity, EC_T_DWORD dwPrio, EC_T_DWORD dwStackSize);
#endif

    /* legacy */
    CPcapRecorder(EC_T_DWORD dwBufCnt, EC_T_DWORD dwPrio, EC_T_DWORD dwInstanceId = 0xffff, const EC_T_CHAR* szFileName = EC_NULL);

    virtual ~CPcapRecorder();

    EC_T_DWORD InitInstance(EC_T_DWORD dwInstanceId, EC_T_DWORD dwBufCnt, const EC_T_CHAR* szFileName = EC_NULL);

    EC_T_VOID Install(EC_T_DWORD dwInstanceId);
    EC_T_VOID Uninstall();
    EC_T_VOID LogFrame(EC_T_DWORD dwLogFlags, EC_T_DWORD dwFrameSize, EC_T_BYTE* pbyFrame);
    static EC_T_VOID LogFrameStatic(EC_T_VOID* pvContext, EC_T_DWORD dwLogFlags, EC_T_DWORD dwFrameSize, EC_T_BYTE* pbyFrame)
    {
        ((CPcapRecorder*)pvContext)->LogFrame(dwLogFlags, dwFrameSize, pbyFrame);
    }

    EC_INLINESTART EC_T_DWORD      GetMasterId() { return m_dwInstanceId; } EC_INLINESTOP
    EC_INLINESTART EC_T_LOG_PARMS* GetLogParms()
    {
        if (GetMasterId() < MAX_NUMOF_MASTER_INSTANCES) return &G_aLogParms[GetMasterId()];
        return &G_aLogParms[0];
    } EC_INLINESTOP

    EC_T_INT                  m_nLoggedFrameCount;

protected:
    EC_T_DWORD                m_dwInstanceId;
};
#endif /* INCLUDE_PCAP_RECORDER */

#endif /* INC_LOGGING */

/*-END OF SOURCE FILE--------------------------------------------------------*/
