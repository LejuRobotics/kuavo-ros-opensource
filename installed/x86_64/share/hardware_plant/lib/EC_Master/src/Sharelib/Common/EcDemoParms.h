/*-----------------------------------------------------------------------------
 * EcDemoParms.h
 * Copyright                acontis technologies GmbH, Ravensburg, Germany
 * Response                 Holger Oelhaf
 * Description              Application specific settings for EC demo
 *---------------------------------------------------------------------------*/

#include <ctype.h>

#ifndef INC_ECDEMOPARMS_H
#define INC_ECDEMOPARMS_H 1

#if (!defined INCLUDE_EC_LOGGING) && (!defined EXCLUDE_EC_LOGGING)
#define INCLUDE_EC_LOGGING
#endif

/*-DEFINES-------------------------------------------------------------------*/
#if !(defined EC_DEMO_TINY)
#ifndef MASTER_CFG_ECAT_MAX_BUS_SLAVES
#define MASTER_CFG_ECAT_MAX_BUS_SLAVES       256    /* max number of pre-allocated bus slave objects */
#endif
#define MASTER_CFG_MAX_ACYC_FRAMES_QUEUED     32    /* max number of acyc frames queued, 127 = the absolute maximum number */
#define MASTER_CFG_MAX_ACYC_BYTES_PER_CYC   4096    /* max number of bytes sent during eUsrJob_SendAcycFrames within one cycle */
#else
#ifndef MASTER_CFG_ECAT_MAX_BUS_SLAVES
#define MASTER_CFG_ECAT_MAX_BUS_SLAVES         8    /* max number of pre-allocated bus slave objects */
#endif
#define MASTER_CFG_MAX_ACYC_FRAMES_QUEUED     32    /* max number of acyc frames queued, 127 = the absolute maximum number */
#define MASTER_CFG_MAX_ACYC_BYTES_PER_CYC    512    /* max number of bytes sent during eUsrJob_SendAcycFrames within one cycle */
#endif /* EC_DEMO_TINY */

#define MASTER_CFG_MAX_ACYC_CMD_RETRIES        3

#define ETHERCAT_STATE_CHANGE_TIMEOUT      15000    /* master state change timeout in ms */
#define ETHERCAT_SCANBUS_TIMEOUT           10000    /* scanbus timeout in ms, see also EC_SB_DEFAULTTIMEOUT */

#define COMMAND_LINE_BUFFER_LENGTH 512
#define MAX_LINKLAYER   5

#if (defined EC_SIMULATOR_DS402)
#define DEMO_MAX_NUM_OF_AXIS                   4    /* max number of of axis that can be used in DS402 demos */
#endif

#ifndef UINTMAX_MAX
#if (defined EC_VERSION_TKERNEL) || (defined EC_VERSION_VXWORKS)
#define UINTMAX_MAX      0xffffffffffffffffULL
#else
#define UINTMAX_MAX      ((EC_T_UINT64)0xffffffffffffffff)
#endif
#endif

/*-TYPEDEFS------------------------------------------------------------------*/
/* demo application parameters */
typedef struct _T_EC_DEMO_APP_PARMS
{
    EC_T_OS_PARMS       Os;                             /* operating system parameters */
    EC_T_DWORD          dwCpuIndex;                     /* CPU index */
    EC_T_CPUSET         CpuSet;                         /* CPU-set for SMP systems */
    /* link layer */
    EC_T_LINK_PARMS*    apLinkParms[MAX_LINKLAYER];     /* link layer parameters array */
    EC_T_DWORD          dwNumLinkLayer;                 /* link layer count in apLinkParms */
    EC_T_LINK_TTS       TtsParms;                       /* Time Triggered Send (TTS) parameters */
    /* configuration */
    EC_T_CNF_TYPE       eCnfType;                       /* configuration data type */
    EC_T_BYTE*          pbyCnfData;                     /* configuration data filename / buffer */
    EC_T_DWORD          dwCnfDataLen;                   /* configuration data length in bytes */
    EC_T_CHAR           szENIFilename[256];             /* ENI filename string */
    EC_T_CHAR           szLicenseKey[64];               /* license key string */
    EC_T_UINT64         qwOemKey;                       /* OEM key */
    /* timing */
    EC_T_BOOL           bUseAuxClock;                   /* enable use of auxiliary clock  */
    EC_T_DWORD          dwBusCycleTimeUsec;             /* bus cycle time in usec */
    EC_T_DWORD          dwDemoDuration;                 /* demo duration in msec */
    /* logging */
    EC_T_INT            nVerbose;                       /* verbosity level */
    EC_T_DWORD          dwAppLogLevel;                  /* demo application log level (derived from verbosity level) */
    EC_T_DWORD          dwMasterLogLevel;               /* Master / Monitor / Simulator stack log level (derived from verbosity level) */
    EC_T_CHAR           szLogFileprefix[64];            /* log file prefix string */
    EC_T_DWORD          dwLogBufferMaxMsgCnt;           /* max number of buffered messages (DEFAULT_LOG_MSG_BUFFER_SIZE)  */
    EC_T_BOOL           bPcapRecorder;                  /* EtherCAT packet capture in pcap format (wireshark) enabled */
    EC_T_CHAR           szPcapRecorderFileprefix[64];   /* log file prefix string */
    EC_T_DWORD          dwPcapRecorderBufferFrameCnt;   /* max number of buffered frames */
    /* RAS */
    EC_T_BOOL           bStartRasServer;
    EC_T_BYTE           abyRasServerIpAddress[4];       /* Remote Access Server (RAS) listen IP address */
    EC_T_WORD           wRasServerPort;                 /* Remote Access Server (RAS) listen port */
    EC_T_BYTE           abyRasClientIpAddress[4];       /* Remote Access Server (RAS) connect IP address */
    EC_T_WORD           wRasClientPort;                 /* Remote Access Server (RAS) connect port */
    EC_T_BOOL           bRasAccessControlEnabled;       /* Remote Access Server (RAS) access control enabled */
    EC_T_DWORD          dwRasAccessLevel;               /* Remote Access Server (RAS) access control level */
    /* mailbox gateway server */
    EC_T_WORD           wMbxGatewayServerPort;          /* Mailbox Gateway server port */
    /* DCM */
    EC_T_BOOL           bDcmConfigure;                  /* DCM configuration enabled */
    EC_T_DCM_MODE       eDcmMode;                       /* DCM mode */
    EC_T_BOOL           bDcmControlLoopDisabled;        /* DCM control loop disabled */
    EC_T_BOOL           bDcmLogEnabled;                 /* DCM logging enabled */
    /* master redundancy */
    EC_T_BOOL           bMasterRedPermanentStandby;     /* Master redundancy instance in permanent standby */
    /* additional parameters for the different demos */
    EC_T_DWORD          dwMasterInstanceId;             /* Master instance id */
    EC_T_DWORD          dwPerfMeasLevel;                /* performance measurement level */
    EC_T_BOOL           bPerfMeasShowCyclic;            /* show performance values cyclically  */
    EC_T_WORD           bFlash;                         /* flashing process data (Master: OUTPUTs / Simulator: INPUTs) */
    EC_T_WORD           wFlashSlaveAddr;                /* flashing process data (Master: OUTPUTs / Simulator: INPUTs) slave station address */
    EC_T_BOOL           bReadMasterOd;                  /* read Master / Simulator object directory example enabled */
    struct {
        EC_T_DWORD dwOffset;                            /* process data bit offset */
        EC_T_DWORD dwSize;                              /* process data bit size */
        EC_T_DWORD dwValue;                             /* process data to set */
        EC_T_DWORD dwDuration;                          /* duration of how long process data should be set in msec. 0 == forever */
    }                   SetProcessDataBits;             /* process data access helper struct */
    EC_T_DWORD          dwNotifyCode;                   /* notification code for ecatNotifyApp */
    EC_T_NOTIFYPARMS    NotifyParms;                    /* notification parameter for ecatNotifyApp */
    /* EC-Daq */
    EC_T_BOOL           bDaqRecorder;                   /* DAQ recorder config file enabled */
    EC_T_CHAR           szDaqRecorder[256];             /* DAQ recorder config filename string */
    /* EC-Simulator */
    EC_T_DWORD          dwSimulatorInstanceId;          /* simulator instance id */
    EC_T_BOOL           bDisableProcessDataImage;       /* don't allocate Process Data Image at EC-Simulator (Master ENI / Simulator ENI mismatch support) */
    EC_T_BOOL           bConnectHcGroups;               /* auto-connect floating Hot Connect groups / disconnect all Hot Connect groups */
    EC_T_DWORD          dwCfgDeviceConnectionCount;
    EC_T_SIMULATOR_DEVICE_CONNECTION_DESC aoDeviceConnection[MAX_LINKLAYER]; /* see EC_SIMULATOR_DEVICE_CONNECTION_TYPE_... */
    /* DS402 */
#if (defined EC_SIMULATOR_DS402)
    EC_T_DWORD          dwDS402NumSlaves;               /* DS402 simulated slaves count */
    EC_T_WORD           awDS402SlaveAddr[DEMO_MAX_NUM_OF_AXIS]; /* station fixed adresses of DS402 simulated slaves */
#endif
#if (defined INCLUDE_EC_EAP)
    /* EAP */
    EC_T_BYTE           abyIpAddress[4];                /* IP address */
#endif
#if (defined INCLUDE_EC_MONITOR)
    EC_T_BOOL           bPcapProcessing;                /* packet capture file processing enabled */
    EC_T_CHAR           szPcapFilename[256];            /* packet capture file name */
#endif
} T_EC_DEMO_APP_PARMS;

/* demo application context */
typedef struct _T_EC_DEMO_APP_CONTEXT
{
    T_EC_DEMO_APP_PARMS       AppParms;                 /* demo application parameters */
    EC_T_LOG_PARMS            LogParms;                 /* log parameters */
    EC_T_DWORD                dwInstanceId;             /* instance id */
    EC_T_VOID*                pvJobTaskEvent;           /* job task event */
    EC_T_BOOL                 bJobTaskRunning;          /* job task running flag */
    EC_T_BOOL                 bJobTaskShutdown;         /* job task shutdown request flag */
#if (defined __cplusplus)
    class CEmNotification*    pNotificationHandler;     /* notification handler */
#else
    struct _T_CEmNotification* pNotificationHandler;    /* notification handler */
#endif
    EC_T_VOID*                pvCycFrameReceivedEvent;  /* cyclic frame received event */
    EC_T_DWORD                dwPerfMeasLevel;          /* performance measurement level */
    EC_T_VOID*                pvPerfMeas;               /* performance measurement object */
    struct _T_MY_APP_DESC*    pMyAppDesc;               /* my app descriptor */
    struct _T_MASTER_RED_DEMO_PARMS* pMasterRedParms;   /* Master redundancy parameters */
    struct _T_EC_MONITOR_DEMO_PARMS* pMonitorParms;     /* EC-Monitor parameters */
} T_EC_DEMO_APP_CONTEXT;

/*-GLOBAL VARIABLES-----------------------------------------------------------*/
extern volatile EC_T_BOOL  bRun;                        /* global demo run flag */
extern volatile EC_T_BOOL  th_run;                      /* global run flag */

/*-FUNCTION DECLARATION------------------------------------------------------*/
EC_T_VOID  ResetAppParms(T_EC_DEMO_APP_CONTEXT* pAppContext, T_EC_DEMO_APP_PARMS* pAppParms);
EC_T_VOID  FreeAppParms(T_EC_DEMO_APP_CONTEXT* pAppContext, T_EC_DEMO_APP_PARMS* pAppParms);
EC_T_DWORD SetAppParmsFromCommandLine(T_EC_DEMO_APP_CONTEXT* pAppContext, const EC_T_CHAR* szCommandLine, T_EC_DEMO_APP_PARMS* pAppParms);
EC_T_VOID  ShowSyntaxCommon(T_EC_DEMO_APP_CONTEXT* pAppContext);

/**
 * \brief convert a string to an unsigned long long
 *
 * \return unsigned long long
 */
static EC_INLINESTART EC_T_UINT64 EcStrtoull(const EC_T_CHAR* szNptr, EC_T_CHAR** pszEndptr, EC_T_INT nBase)
{
    const EC_T_CHAR* szLocalString = EC_NULL;
    EC_T_INT64 nAcc = 0;
    EC_T_INT64 nCutoff = 0;
    EC_T_INT64 nCutlim = 0;
    EC_T_INT nCharacter = 0;
    EC_T_INT nNeg = 0;
    EC_T_INT nAny = 0;

    /* see strtoq for comments to logic below */
    szLocalString = szNptr;
    do
    {
        nCharacter = (EC_T_BYTE)*szLocalString++;
    } while (isspace(nCharacter));
    if (nCharacter == '-')
    {
        nNeg = 1;
        nCharacter = *szLocalString++;
    }
    else
    {
        nNeg = 0;
        if (nCharacter == '+')
        {
            nCharacter = *szLocalString++;
        }
    }
    if (((nBase == 0) || (nBase == 16)) && (nCharacter == '0') && ((*szLocalString == 'x') || (*szLocalString == 'X')))
    {
        nCharacter = szLocalString[1];
        szLocalString += 2;
        nBase = 16;
    }
    if (nBase == 0)
    {
        nBase = nCharacter == '0' ? 8 : 10;
    }
    /* BIONIC: avoid division and modulo for common cases */
#define  CASE_BASE(x) \
            case x: nCutoff = UINTMAX_MAX / x; \
                nCutlim = UINTMAX_MAX % x; \
            break

    switch (nBase)
    {
        CASE_BASE(8);
        CASE_BASE(10);
        CASE_BASE(16);
    default:
        nCutoff = UINTMAX_MAX / nBase;
        nCutlim = UINTMAX_MAX % nBase;
    }

    for (nAcc = 0, nAny = 0;; nCharacter = (EC_T_BYTE)*szLocalString++)
    {
        if (isdigit(nCharacter))
        {
            nCharacter -= '0';
        }
        else if (isalpha(nCharacter))
        {
            nCharacter -= isupper(nCharacter) ? 'A' - 10 : 'a' - 10;
        }
        else
        {
            break;
        }
        if (nCharacter >= nBase)
        {
            break;
        }
        if (nAny < 0)
        {
            continue;
        }
        if ((nAcc > nCutoff) || ((nAcc == nCutoff) && (nCharacter > nCutlim)))
        {
            nAny = -1;
            nAcc = UINTMAX_MAX;
        }
        else
        {
            nAny = 1;
            nAcc *= (EC_T_UINT64)nBase;
            nAcc += nCharacter;
        }
    }
    if (nNeg && (nAny > 0))
    {
        nAcc = -nAcc;
    }
    if (pszEndptr != 0)
    {
        *pszEndptr = (EC_T_CHAR*)(nAny ? szLocalString - 1 : szNptr);
    }
    return (nAcc);
} EC_INLINESTOP

#endif /* INC_ECDEMOPARMS_H */

/*-END OF SOURCE FILE--------------------------------------------------------*/
