/** ---------------------------------------------------------------------------
 * \file
 * \brief       EC-Master OS-Layer header
 * \author      Stefan Zintgraf
 * \copyright   acontis technologies GmbH, Weingarten, Germany
 *---------------------------------------------------------------------------*/

#ifndef INC_ECOS
#define INC_ECOS

#ifndef INC_ECTYPE
#include "EcType.h"
#endif

/* API decoration */
#ifndef EC_FNNAME_DEFAULT
#ifdef __cplusplus
#define EC_FNNAME_DEFAULT extern "C"
#else
#define EC_FNNAME_DEFAULT
#endif
#endif /* EC_FNNAME_DEFAULT */

#ifndef EC_API_DEFAULT
#define EC_API_DEFAULT EC_FNNAME_DEFAULT
#endif

/*****************************************************************************
 * OsPlatform... declarations
 * TODO Fix #defines of OsMalloc in EcOsPlatform.h
 *****************************************************************************/
#ifndef INC_ECOSPLATFORM
#include "EcOsPlatform.h"
#endif

#ifndef EC_FNNAME
#define EC_FNNAME EC_FNNAME_DEFAULT
#endif
#ifndef EC_FNCALL
#define EC_FNCALL /* as default, no explicit calling convention is given */
#endif
#ifndef EC_API
#define EC_API EC_API_DEFAULT
#endif
#ifndef EC_API_FNCALL
#define EC_API_FNCALL EC_FNCALL
#endif

#if (defined EC_OS_NAMESPACE)
/* set EC_OS_... */
#undef  EC_OS_API
#define EC_OS_API
#undef  EC_OS_API_FNCALL
#define EC_OS_API_FNCALL
#undef  EC_OS_FNNAME
#define EC_OS_FNNAME
#undef  EC_OS_FNCALL
#define EC_OS_FNCALL
#endif /* EC_OS_NAMESPACE */

#ifndef EC_OS_API
#define EC_OS_API EC_API
#endif
#ifndef EC_OS_API_FNCALL
#define EC_OS_API_FNCALL EC_API_FNCALL
#endif
#ifndef EC_OS_FNNAME
#define EC_OS_FNNAME EC_FNNAME_DEFAULT
#endif
#ifndef EC_OS_FNCALL
#define EC_OS_FNCALL
#endif

#ifdef  EC_DLL
#ifndef EC_APIENTRY
#define EC_APIENTRY WINAPI
#endif
#ifndef EcDllMain
#define EcDllMain DllMain
#define EC_DLL_PROCESS_ATTACH DLL_PROCESS_ATTACH
#define EC_DLL_THREAD_ATTACH  DLL_THREAD_ATTACH
#define EC_DLL_THREAD_DETACH  DLL_THREAD_DETACH
#define EC_DLL_PROCESS_DETACH DLL_PROCESS_DETACH
#endif
#endif
#ifndef EC_ARCH
#define EC_ARCH EC_ARCH_UNDEFINED
#endif

#ifndef INC_ECFEATURES
#include "EcFeatures.h"
#endif

#ifndef INC_ECERROR
#include "EcError.h"
#endif

/* legacy. TODO: remove */
#ifndef ATECAT_API
#define ATECAT_API EC_API
#endif

/* TODO: remove */
#if 1
#if (defined OsSleep && !defined OsPlatformImplSleep && !defined EXCLUDE_OSSLEEP_PROTOTYPE)
#define EXCLUDE_OSSLEEP_PROTOTYPE
#endif

#ifndef OsPlatformMalloc
#define OsPlatformMalloc(nSize)                 malloc((size_t)(nSize))
#endif

#ifndef OsPlatformFree
#define OsPlatformFree(pvMem)                   free((void*)(pvMem))
#endif

#ifndef OsPlatformRealloc
#define OsPlatformRealloc(pMem,nSize)           realloc((void*)(pMem),(size_t)(nSize))
#endif

#ifndef OsPlatformMemoryBarrier
#define OsPlatformMemoryBarrier()               { EC_T_DWORD dwDummy = 0; __asm xchg dwDummy, eax }
#endif

#ifndef OsPlatformSetEvent
#define OsPlatformSetEvent(hEvent)              SetEvent((HANDLE)(hEvent))
#endif

#ifndef OsPlatformResetEvent
#define OsPlatformResetEvent(hEvent)            ResetEvent(hEvent)
#endif

#endif /* *** legacy code handling OS-Layer change in V3.0.0.13 *** */

/*****************************************************************************
 * OsPlatform instrumentation
 *****************************************************************************/
#if (defined INSTRUMENT_OS)

/* TODO: remove */
#ifdef __cplusplus
extern "C"
{
#endif

#include "EcOsInstr.h"

/* TODO: remove */
#ifdef __cplusplus
}
#endif

#else /* !INSTRUMENT_OS */
 /* API function naming */
#ifdef  OsPlatformImplMalloc
#undef  OsPlatformImplMalloc
#define OsMalloc                                OsMalloc
#endif
#define OsPlatformImplMalloc                    OsMalloc

#ifdef  OsPlatformImplFree
#undef  OsPlatformImplFree
#define OsFree                                  OsFree
#endif
#define OsPlatformImplFree                      OsFree

#ifdef  OsPlatformImplRealloc
#undef  OsPlatformImplRealloc
#define OsRealloc                               OsRealloc
#endif
#define OsPlatformImplRealloc                   OsRealloc

#ifdef  OsPlatformImplCreateThread
#undef  OsPlatformImplCreateThread
#ifndef OsCreateThread
#define OsCreateThread                          OsCreateThread
#endif
#endif
#define OsPlatformImplCreateThread              OsCreateThread

#ifdef  OsPlatformImplDeleteThreadHandle
#undef  OsPlatformImplDeleteThreadHandle
#define OsDeleteThreadHandle                    OsDeleteThreadHandle
#endif
#define OsPlatformImplDeleteThreadHandle        OsDeleteThreadHandle

#ifdef  OsPlatformImplGetLinkLayerRegFunc
#undef  OsPlatformImplGetLinkLayerRegFunc
#define OsGetLinkLayerRegFunc                   OsGetLinkLayerRegFunc
#endif
#define OsPlatformImplGetLinkLayerRegFunc       OsGetLinkLayerRegFunc


#ifdef  OsPlatformImplReleaseLinkLayerRegFunc
#undef  OsPlatformImplReleaseLinkLayerRegFunc
#define OsReleaseLinkLayerRegFunc               OsReleaseLinkLayerRegFunc
#endif
#define OsPlatformImplReleaseLinkLayerRegFunc   OsReleaseLinkLayerRegFunc


#ifdef  OsPlatformImplReplaceGetLinkLayerRegFunc
#undef  OsPlatformImplReplaceGetLinkLayerRegFunc
#define OsReplaceGetLinkLayerRegFunc            OsReplaceGetLinkLayerRegFunc
#endif
#define OsPlatformImplReplaceGetLinkLayerRegFunc OsReplaceGetLinkLayerRegFunc

#ifdef  OsPlatformImplSleep
#undef  OsPlatformImplSleep
#define OsSleep                                 OsSleep
#endif
#define OsPlatformImplSleep                     OsSleep
#endif /* !INSTRUMENT_OS */

/* No OS specific includes files allowed */
#if (defined __cplusplus) && !(defined EC_NEW)
#include <new>
#endif
#include <limits.h>

/* TODO: remove */
#if (defined __cplusplus) && (!defined EC_OS_NAMESPACE)
extern "C"
{
#endif
#ifndef INC_EC_COMMON
#include "OsCommon.h"
#endif

/*-DEFINES-------------------------------------------------------------------*/

/* -PACKED STRUCTURES------------------------------------------------------- */
#ifndef EC_T_OS_PARMS
#include EC_PACKED_API_INCLUDESTART
#if __GNUC__ >= 8
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpacked-not-aligned"
#endif
typedef struct _EC_T_OS_PARMS
{
#define EC_OS_PARMS_SIGNATURE_PATTERN          (EC_T_DWORD)0x5BA00000      /* Mask 0xFFF00000 */
#define EC_OS_PARMS_SIGNATURE_VERSION          (EC_T_DWORD)0x00010000      /* Version 1, mask 0x000F0000 */
#ifndef EC_T_OS_PLATFORM_PARMS
#define EC_OS_PLATFORM_PARMS_SIGNATURE_PATTERN (EC_T_DWORD)0x00000000      /* Mask 0x0000FFF0 */
#define EC_OS_PLATFORM_PARMS_SIGNATURE_VERSION (EC_T_DWORD)0x00000000      /* Version 0, mask 0x0000000F */
#endif
#define EC_OS_PARMS_SIGNATURE (EC_T_DWORD)(EC_OS_PARMS_SIGNATURE_PATTERN|EC_OS_PARMS_SIGNATURE_VERSION|EC_OS_PLATFORM_PARMS_SIGNATURE_PATTERN|EC_OS_PLATFORM_PARMS_SIGNATURE_VERSION)
    EC_T_DWORD                          dwSignature;                  /**< [in] Set to EC_OS_PARMS_SIGNATURE */
    EC_T_DWORD                          dwSize;                       /**< [in] Set to sizeof(EC_T_OS_PARMS) */
    struct _EC_T_LOG_PARMS*             pLogParms;                    /**< [in] Pointer to logging parameters  */
    EC_PF_SYSTIME                       pfSystemTimeGet;              /**< [in] Function to get host time in nanoseconds since 1st January 2000. Used as time base for DC Initialization. */
    EC_T_DWORD                          dwSupportedFeatures;          /**< [in/out] Reserved */
    EC_PF_QUERY_MSEC_COUNT              pfSystemQueryMsecCount;       /**< [in] Function to get system's ms count */
#if (defined EC_T_OS_PLATFORM_PARMS)
    EC_T_OS_PLATFORM_PARMS              PlatformParms;                /**< [in] Platform specific parameters */
#endif
} EC_PACKED_API EC_T_OS_PARMS;
#if __GNUC__ >= 8
#pragma GCC diagnostic pop
#endif
#include EC_PACKED_INCLUDESTOP
#define EC_T_OS_PARMS EC_T_OS_PARMS
#endif

/*-MACROS--------------------------------------------------------------------*/
#ifndef ATECAT_PLATFORMSTR
#define ATECAT_PLATFORMSTR ""
#endif

#define     OsSafeFree(p)        {if(EC_NULL!=(p)){OsFree(p);(p)=EC_NULL;}}

#define     OsSafeDeleteEvent(p) {if(EC_NULL!=(p)){OsDeleteEvent(p);(p)=EC_NULL;}}

#define     OsSafeDeleteLock(p)  {if(EC_NULL!=(p)){OsDeleteLock(p);(p)=EC_NULL;}}

/*-FUNCTION DECLARATIONS-----------------------------------------------------*/
#if (defined EC_OS_NAMESPACE) && (defined __cplusplus)
namespace EC_OS_NAMESPACE
{
#endif

EC_OS_API EC_T_DWORD EC_OS_API_FNCALL OsInit(EC_T_OS_PARMS* pOsParms);
EC_OS_API EC_T_DWORD EC_OS_API_FNCALL OsDeinit(EC_T_VOID);
EC_OS_API EC_T_VOID  EC_OS_API_FNCALL OsSetLogParms(const struct _EC_T_LOG_PARMS* pLogParms);

#ifndef OsMemset
#define OsMemset(pDest,nVal,nSize)              memset((EC_T_VOID*)(pDest), nVal, nSize)
#endif

#ifndef OsMemsetPdIn
#define OsMemsetPdIn                            OsMemset
#endif

#ifndef OsMemsetPdOut
#define OsMemsetPdOut                           OsMemset
#endif

#ifndef OsMemcpy
#define OsMemcpy(pDest,pSrc,nSize)              memcpy(pDest, pSrc, nSize)
#endif

#ifndef OsMemcpyPdIn
#define OsMemcpyPdIn                            OsMemcpy
#endif

#ifndef OsMemcpyPdOut
#define OsMemcpyPdOut                           OsMemcpy
#endif

#ifndef OsMemcpyMasterRedPdIn
#define OsMemcpyMasterRedPdIn                   OsMemcpyPdIn
#endif

#ifndef OsMemcpyMasterRedPdOut
#define OsMemcpyMasterRedPdOut                  OsMemcpyPdOut
#endif

#ifndef OsMemcmp
#define OsMemcmp(pBuf1,pBuf2,nSize)             memcmp(pBuf1, pBuf2, nSize)
#endif

#ifndef OsMemmove
#define OsMemmove(pDest,pSrc,nSize)             memmove(pDest, pSrc, nSize)
#endif

#ifndef OsStrlen
#define OsStrlen(szString)                      strlen((const char*)(szString))
#endif

#ifndef OsStrcmp
#define OsStrcmp(szStr1,szStr2)                 strcmp((const char*)(szStr1),(const char*)(szStr2))
#endif

#ifndef OsStrncmp
#define OsStrncmp(szStr1,szStr2, nSize)         strncmp((const char*)(szStr1),(const char*)(szStr2), (size_t)(nSize))
#endif

#ifndef OsStricmp
#define OsStricmp(szStr1,szStr2)                stricmp((const char*)(szStr1),(const char*)(szStr2))
#endif

#ifndef OsStrtok
#define OsStrtok(szToken,szDelimit)             strtok((char*)(szToken), (const char*)(szDelimit))
#endif

#ifndef OsStrtol
#define OsStrtol(szToken,ppEnd,nRadix)          strtol((const char*)(szToken), (ppEnd), (nRadix))
#endif

#ifndef OsAtoi
#define OsAtoi(szString)                        atoi((const char*)(szString))
#endif

#ifndef OsStrtoul
#define OsStrtoul(szString,ptr,base)            strtoul((const char*)(szString), (ptr), (base))
#endif

#ifndef OsPrintf
#define OsPrintf                                printf
#endif

#ifndef OsVprintf
#define OsVprintf                               vprintf
#endif

#ifndef OsVsnprintf
#define OsVsnprintf                             EcVsnprintf
#endif

#ifndef OsSnprintf
/** \def OsSnprintf(EC_T_CHAR* szDest, EC_T_DWORD dwSize, const EC_T_CHAR* szFormat, ...)
    \brief An macro for platform-abstracted snprintf().
    \param szDest buffer to write to
    \param dwSize max bytes to print
    \param szFormat format text to print
    \param ... variable args to be formatted printed.
    \return Length of string (without terminating zero character).
    Securely prints given parameters formatted to buffer.
*/
#define OsSnprintf                              EcSnprintf
#endif

#ifndef OsFopen
#define OsFopen                                 fopen
#endif

#ifndef OsFclose
#define OsFclose(pFile)                         fclose(((FILE*)pFile))
#endif

#ifndef OsFwrite
#define OsFwrite                                fwrite
#endif

#ifndef OsFread
#define OsFread(pDstBuf,dwElemSize,dwCnt,hFile) fread((pDstBuf),(dwElemSize),(dwCnt),((FILE*)hFile))
#endif

#ifndef OsFflush
#define OsFflush                                fflush
#endif

/* this function currently is only used in the MotionDemo application, see also AtXmlParser.cpp */
/*
#ifndef OsGetFileSize
EC_T_INLINE(EC_T_DWORD OsGetFileSize(FILE* fp)) {long l;fseek((fp),0,SEEK_END);l=ftell((fp));fseek((fp),0,SEEK_SET);return l;}
#endif
*/

#ifndef OsMkdir
#if (defined WIN32)
#define OsMkdir(szDir) _mkdir((szDir))
#else
#define OsMkdir(szDir) mkdir((szDir),S_IRWXU|S_IRWXG|S_IROTH|S_IXOTH)
#endif
#endif

/********************************************************************************/
/** \brief  OsDbgAssertFunc(): callback for checked code
 *
 * \return  N/A.
 */
EC_OS_FNNAME EC_T_VOID EC_OS_FNCALL OsDbgAssertFunc(EC_T_BOOL bAssertCondition, const EC_T_CHAR* szFile, EC_T_DWORD dwLine);
EC_OS_FNNAME EC_T_VOID EC_OS_FNCALL OsDbgAssertFail(const EC_T_CHAR* szFile, EC_T_DWORD dwLine);

#if (!defined OsDbgAssert)
#if (defined INCLUDE_ASSERT_FUNC)
#define OsDbgAssert(bAssertCondition) OsDbgAssertFunc((bAssertCondition),(__FILE__), (__LINE__))
#elif (defined DEBUG)
#define OsDbgAssert assert
#else
#define OsDbgAssert(x)
#endif /* !INCLUDE_ASSERT_FUNC */
#endif /* OsDbgAssert() */

#ifndef OsSetLastError
#define OsSetLastError(dwError)                 dwError
#endif

#ifndef OsReleaseLinkLayerRegFunc
#define OsReleaseLinkLayerRegFunc(szDriverIdent)
#endif

#ifdef __cplusplus
#ifndef EC_NEW
#define EC_NEW(x)    new (std::nothrow) x
#endif
#endif /* __cplusplus */

EC_OS_FNNAME EC_T_VOID* EC_OS_FNCALL OsCfgFileOpen(const EC_T_CHAR* szCfgFileName);
EC_OS_FNNAME EC_T_INT   EC_OS_FNCALL OsCfgFileClose(EC_T_VOID* pvCfgFile);
EC_OS_FNNAME EC_T_INT   EC_OS_FNCALL OsCfgFileRead(EC_T_VOID* pvCfgFile, EC_T_VOID* pvDst, EC_T_INT nLen);
EC_OS_FNNAME EC_T_INT   EC_OS_FNCALL OsCfgFileError(EC_T_VOID* pvCfgFile);
EC_OS_FNNAME EC_T_INT   EC_OS_FNCALL OsCfgFileEof(EC_T_VOID* pvCfgFile);

/* these functions are actually part of the master core */
EC_OS_API EC_T_INT   EC_OS_API_FNCALL EcVsnprintf(EC_T_CHAR* szDest, EC_T_INT nMaxSize, const EC_T_CHAR* szFormat, EC_T_VALIST vaList);
EC_OS_API EC_T_INT   EC_OS_API_FNCALL EcSnprintf(EC_T_CHAR* szDest, EC_T_INT nMaxSize, const EC_T_CHAR* szFormat, ...);

#ifndef OsQueryMsecCount
EC_OS_API EC_T_DWORD EC_OS_API_FNCALL OsQueryMsecCount(EC_T_VOID);
#endif

#ifndef OsReplaceQueryMsecCount
EC_OS_API EC_T_DWORD EC_OS_API_FNCALL OsReplaceQueryMsecCount(EC_PF_QUERY_MSEC_COUNT pfQueryMsecCount);
#endif

#ifndef EXCLUDE_OSSLEEP_PROTOTYPE
EC_OS_API EC_T_VOID  EC_OS_API_FNCALL OsPlatformImplSleep(EC_T_DWORD dwMsec);
#endif

/***************************/
/* performance measurement */
/***************************/
/* timestamp counter interface for performance measurements (main functions are part of the master core) */
#ifndef INCLUDE_OS_PLATFORM_TSC_SUPPORT

/* if platform does not have specific TSC support, check for defaults if it is yet available */
#if (defined UNDER_CE && defined _M_IX86) || (EC_OS == EC_OS_RTOS32) || ((EC_OS == EC_OS_VXWORKS) && (defined CPU) && ((CPU == PENTIUM) || (CPU == PENTIUM4))) || (EC_OS == EC_OS_QNX) || ((defined WIN32) && !(defined UNDER_CE)) || ((EC_OS == EC_OS_LINUX) && (defined __i386__)) || (EC_OS == EC_OS_INTIME)
#define INCLUDE_OS_PLATFORM_TSC_SUPPORT
#define INCLUDE_PENTIUM_TSC
#endif
#if (defined __RCX__)
#define INCLUDE_ARM_TSC
#endif

#endif

/* TODO: EC_T_TSC_FNMESSAGE */
typedef EC_T_VOID (EC_FNCALL *EC_T_FNMESSAGE) (EC_T_CHAR* szMsg);

/* performance measurement descriptors */
#include EC_PACKED_API_INCLUDESTART
typedef struct _EC_T_TSC_TIME
{
    EC_T_UINT64     qwStart;            /**< start time */
    EC_T_UINT64     qwEnd;              /**< end time */
    EC_T_DWORD      dwCurr;             /**< [1/10   us] */
    EC_T_DWORD      dwMin;              /**< [1/10   us] */
    EC_T_DWORD      dwMax;              /**< [1/10   us] */
    EC_T_DWORD      dwAvg;              /**< [1/1600 us] */
    EC_T_BOOL       bMeasReset;         /**< EC_TRUE if measurement values shall be reset */
    EC_T_INT        nIntLevel;          /**< for interrupt lockout handling */
} EC_PACKED_API EC_T_TSC_TIME;
#include EC_PACKED_INCLUDESTOP

#include EC_PACKED_API_INCLUDESTART
typedef struct _EC_T_TSC_MEAS_DESC
{
    EC_T_TSC_TIME*  aTscTime;           /* timestamp info array */
    EC_T_DWORD      dwNumMeas;          /* number of elements in aTscTime */
    EC_T_BOOL       bMeasEnabled;       /* EC_TRUE if measurement is enabled */
    EC_T_VOID*      pPrivateData;       /* OS layer private data */
    EC_T_FNMESSAGE  pfnMessage;         /* Function pointer for message drop */
} EC_PACKED_API EC_T_TSC_MEAS_DESC;
#include EC_PACKED_INCLUDESTOP

#ifndef OsTscMeasDisableIrq
#define OsTscMeasDisableIrq(pTscMeasDesc,dwIndex)
#endif
#ifndef OsTscMeasEnableIrq
#define OsTscMeasEnableIrq(pTscMeasDesc,dwIndex)
#endif

EC_OS_API EC_T_VOID   EC_OS_API_FNCALL OsMeasCalibrate(EC_T_UINT64 dwlFreqSet);
EC_OS_API EC_T_DWORD  EC_OS_API_FNCALL OsMeasGet100kHzFrequency(EC_T_VOID);
EC_OS_API EC_T_UINT64 EC_OS_API_FNCALL OsMeasGetCounterTicks(EC_T_VOID);

/* optional: redirect trace messages into OS specific function, e.g. to store trace log into file
 * default: print trace log as debug message
 */
#ifndef OsTrcMsg
#define     OsTrcMsg    EC_DBGMSG
#endif

EC_OS_API EC_T_VOID*  EC_OS_API_FNCALL OsCreateLock(EC_T_VOID);
EC_OS_API EC_T_VOID*  EC_OS_API_FNCALL OsCreateLockTyped(EC_T_OS_LOCK_TYPE eLockType);
EC_OS_API EC_T_VOID   EC_OS_API_FNCALL OsDeleteLock(EC_T_VOID* pvLock);
EC_OS_API EC_T_VOID   EC_OS_API_FNCALL OsLock(EC_T_VOID* pvLock);
EC_OS_API EC_T_VOID   EC_OS_API_FNCALL OsUnlock(EC_T_VOID* pvLock);

EC_OS_API EC_T_VOID*  EC_OS_API_FNCALL OsCreateEvent(EC_T_VOID);
EC_OS_API EC_T_VOID   EC_OS_API_FNCALL OsDeleteEvent(EC_T_VOID* pvEvent);
#ifndef OsWaitForEvent
EC_OS_API EC_T_DWORD  EC_OS_API_FNCALL OsWaitForEvent(EC_T_VOID* pvEvent, EC_T_DWORD dwTimeout);
#endif
EC_OS_API EC_T_VOID   EC_OS_API_FNCALL OsPlatformImplSetEvent(EC_T_VOID* pvEvent);
EC_OS_API EC_T_VOID   EC_OS_API_FNCALL OsPlatformImplResetEvent(EC_T_VOID* pvEvent);

EC_OS_API EC_T_VOID* EC_OS_API_FNCALL OsPlatformImplCreateThread(const EC_T_CHAR* szThreadName, EC_PF_THREADENTRY pfThreadEntry, EC_T_CPUSET cpuAffinityMask, EC_T_DWORD dwPrio, EC_T_DWORD dwStackSize, EC_T_VOID* pvParams);
EC_OS_API EC_T_DWORD  EC_OS_API_FNCALL OsPlatformImplDeleteThreadHandle(EC_T_VOID* pvThreadObject);

#ifndef OsSetThreadPriority
EC_OS_API EC_T_DWORD  EC_OS_API_FNCALL OsSetThreadPriority(EC_T_VOID* pvThreadObject, EC_T_DWORD dwPrio);
#endif
#ifndef OsSetThreadAffinity
EC_OS_API EC_T_DWORD  EC_OS_API_FNCALL OsSetThreadAffinity(EC_T_VOID* pvThreadObject, EC_T_CPUSET CpuSet);
#endif
#ifndef OsGetThreadAffinity
EC_OS_API EC_T_DWORD  EC_OS_API_FNCALL OsGetThreadAffinity(EC_T_VOID* pvThreadObject, EC_T_CPUSET* pCpuSet);
#endif

typedef EC_PF_LLREGISTER(EC_FNCALL *EC_PF_GETLINKLAYERREGFUNC)(const EC_T_CHAR* szDriverIdent, const EC_T_CHAR* szLoadPath);
EC_OS_API EC_PF_LLREGISTER EC_OS_API_FNCALL OsPlatformImplGetLinkLayerRegFunc(const EC_T_CHAR* szDriverIdent, const EC_T_CHAR* szLoadPath);

EC_OS_API EC_T_VOID OsPlatformImplReleaseLinkLayerRegFunc(const EC_T_CHAR* szDriverIdent);

EC_OS_API EC_T_VOID OsPlatformImplReplaceGetLinkLayerRegFunc(EC_PF_GETLINKLAYERREGFUNC pfLinkLayerRegFunc);

EC_OS_API EC_T_DWORD EC_OS_API_FNCALL OsSystemTimeGet(EC_T_UINT64* pqwSystemTime);


#if (defined EC_SOCKET_IP_SUPPORTED)

#ifndef EC_FD_SET
#define EC_FD_SET FD_SET
#endif

#ifndef EC_FD_ZERO
#define EC_FD_ZERO FD_ZERO
#endif

#ifndef EC_FD_ISSET
#define EC_FD_ISSET FD_ISSET
#endif

#ifndef EC_POLL_FD
#define EC_POLL_FD pollfd
#endif

#ifndef EC_EINPROGRESS
#define EC_EINPROGRESS EINPROGRESS
#endif

#ifndef OsSocketInit
EC_OS_API EC_T_DWORD EC_OS_API_FNCALL OsSocketInit(EC_T_VOID);
#endif
#ifndef OsSocketDeInit
EC_OS_API EC_T_DWORD EC_OS_API_FNCALL OsSocketDeInit(EC_T_VOID);
#endif

#ifndef OsSocket
#define OsSocket(nAddrFamily, nSockType, nProtocol) socket(nAddrFamily, nSockType, nProtocol)
#endif

#ifndef OsSocketBind
#define OsSocketBind(hSockHandle, oSockAddr, nSockAddrLen) bind(hSockHandle, oSockAddr, nSockAddrLen)
#endif

#ifndef OsSocketListen
#define OsSocketListen(hSockHandle, nBacklog) listen(hSockHandle, nBacklog)
#endif

#ifndef EC_SOCKET_IP_POLL_NOTSUPPORTED
#ifndef OsSocketPoll
#define OsSocketPoll(pFdArray, fds, timeout) poll(pFdArray, fds, timeout)
#endif
#endif

#ifndef OsSocketSelect
#define OsSocketSelect(nNfds, poReadFds, poWriteFds, poExceptFds, poTimeout) select(nNfds, poReadFds, poWriteFds, poExceptFds, poTimeout)
#endif

#ifndef OsSocketFcntl
#define OsSocketFcntl fcntl
#endif

#ifndef OsSocketIoctlsocket
#define OsSocketIoctlsocket ioctlsocket
#endif

#ifndef OsSocketAccept
#define OsSocketAccept(hSockHandle, oSockAddr, nSockAddrLen) accept(hSockHandle, oSockAddr, nSockAddrLen)
#endif

#ifndef OsSocketConnect
#define OsSocketConnect(hSockHandle, oSockAddr, nSockAddrLen) connect(hSockHandle, oSockAddr, nSockAddrLen)
#endif

#ifndef OsSocketShutdown
#define OsSocketShutdown(hSockHandle, nFlags) shutdown(hSockHandle, nFlags)
#endif

#ifndef OsSocketSend
#define OsSocketSend(hSockHandle, pbyBuffer, dwBufferLen, dwFlags) send(hSockHandle, pbyBuffer, dwBufferLen, dwFlags)
#endif

#ifndef OsSocketSendTo
#define OsSocketSendTo(hSockHandle, pbyBuffer, dwBufferLen, dwFlags, oDstAddr, dwDstAddrLen) sendto(hSockHandle, pbyBuffer, dwBufferLen, dwFlags, oDstAddr, dwDstAddrLen)
#endif

#ifndef OsSocketRecv
#define OsSocketRecv(hSockHandle, pbyBuffer, dwBufferLen, dwFlags) recv(hSockHandle, pbyBuffer, dwBufferLen, dwFlags)
#endif

#ifndef OsSocketRecvFrom
#define OsSocketRecvFrom(hSockHandle, pbyBuffer, dwBufferLen, dwFlags, oSrcAddr, dwSrcAddrLen) recvfrom(hSockHandle, pbyBuffer, dwBufferLen, dwFlags, oSrcAddr, dwSrcAddrLen)
#endif

#ifndef OsSocketCloseSocket
#define OsSocketCloseSocket(hSockHandle) close(hSockHandle)
#endif

#ifndef OsSocketGetLastError
#error "OsSocketGetLastError has to be defined in EcOsPlatform.h"
#endif

#ifndef OsSocketSetSockOpt
#define OsSocketSetSockOpt(hSockHandle, nLevel, nOptName, pOptValue, nOptLen ) setsockopt(hSockHandle, nLevel, nOptName, pOptValue, nOptLen)
#endif

#ifndef OsSocketGetSockOpt
#define OsSocketGetSockOpt(hSockHandle, nLevel, nOptName, pOptValue, pnOptLen ) getsockopt(hSockHandle, nLevel, nOptName, pOptValue, pnOptLen)
#endif

#ifndef OsHTONS
#define OsHTONS(wHostVal) htons(wHostVal)
#endif

#endif /* EC_SOCKET_IP_SUPPORTED */

#ifndef OsTerminateAppRequest
#ifdef  OsPlatformTerminateAppRequest
#define OsTerminateAppRequest OsPlatformTerminateAppRequest
#else
#define OsTerminateAppRequest() EC_FALSE
#endif
#endif
#ifndef OsMalloc
#ifdef  OsPlatformMalloc
#define OsMalloc OsPlatformMalloc
#endif
#endif
#ifndef OsRealloc
#ifdef  OsPlatformRealloc
#define OsRealloc OsPlatformRealloc
#endif
#endif
#ifndef OsFree
#ifdef  OsPlatformFree
#define OsFree OsPlatformFree
#endif
#endif
#ifndef OsSleep
#ifdef  OsPlatformSleep
#define OsSleep OsPlatformSleep
#endif
#endif
#ifndef OsMemoryBarrier
#ifdef  OsPlatformMemoryBarrier
#define OsMemoryBarrier OsPlatformMemoryBarrier
#endif
#endif
#ifndef OsSetEvent
#ifdef  OsPlatformSetEvent
#define OsSetEvent OsPlatformSetEvent
#endif
#endif
#ifndef OsResetEvent
#ifdef  OsPlatformResetEvent
#define OsResetEvent OsPlatformResetEvent
#endif
#endif
#ifndef OsDeleteThreadHandle
#ifdef  OsPlatformDeleteThreadHandle
#define OsDeleteThreadHandle OsPlatformDeleteThreadHandle
#endif
#endif
#ifndef OsGetLinkLayerRegFunc
#ifdef  OsPlatformGetLinkLayerRegFunc
#define OsGetLinkLayerRegFunc OsPlatformGetLinkLayerRegFunc
#endif
#endif

/*****************************************************************************/
/**
 * \brief Copy zero terminated string from szSrc to szDest. 
 *
 * In contrast to strncpy() append a guaranteed '\0' at szDest[nMaxSize - 1].
 *
 * In contrast to OsSafeStrncpy2() read at most nLen = nMaxSize - 1 chars from szSrc.
 *
 * \return N/A
 */
static EC_INLINESTART EC_T_VOID OsSafeStrncpy(EC_T_CHAR* szDest, const EC_T_CHAR* szSrc, size_t nMaxSize)
{
    if (nMaxSize > 0)
    {
        strncpy(szDest, szSrc, nMaxSize - 1);
        szDest[nMaxSize - 1] = '\0';
    }
} EC_INLINESTOP

/*****************************************************************************/
/**
 * \brief Copy zero terminated string from szSrc to szDest. 
 *
 * In contrast to strncpy() append a guaranteed '\0' at szDest[nMaxSize - 1].
 * 
 * In contrast to OsSafeStrncpy() read at most nLen = nMaxSize chars from szSrc and overwrite the last character with the guaranteed '\0' in case of buffer overflow.
 *
 * Needed for OsSafeStrcpy2().
 *
 * \return N/A
 */
static EC_INLINESTART EC_T_VOID OsSafeStrncpy2(EC_T_CHAR* szDest, const EC_T_CHAR* szSrc, size_t nMaxSize)
{
    if (nMaxSize > 0)
    {
        strncpy(szDest, szSrc, nMaxSize);
        szDest[nMaxSize - 1] = '\0';
    }
} EC_INLINESTOP

/*****************************************************************************/
/**
 * \brief Search for szSubStr in szStr.
 *
 * \return First match of szSubStr within szStr.
 */
static EC_INLINESTART EC_T_CHAR* OsStrstr(EC_T_CHAR* szStr, const EC_T_CHAR* szSubStr)
{
    EC_T_INT i;
    EC_T_INT nStrLen = (EC_T_INT)OsStrlen(szStr);
    EC_T_INT nSubStrLen = (EC_T_INT)OsStrlen(szSubStr);

    if ((0 == nStrLen) || (0 == nSubStrLen))
        return EC_NULL;

    for (i = 0; i <= (nStrLen - nSubStrLen); i++)
    {
        if (0 == OsStrncmp(szStr + i, szSubStr, nSubStrLen))
            return szStr + i;
    }
    return EC_NULL;
} EC_INLINESTOP
#define EC_STRSTR OsStrstr /* deprecated */

#if (defined __cplusplus) && (defined EC_OS_NAMESPACE)
} /* namespace EC_OS_NAMESPACE */
using namespace EC_OS_NAMESPACE;
#endif

/* TODO: remove */
#if (defined __cplusplus) && (!defined EC_OS_NAMESPACE)
} /* extern "C"*/
#endif

#if (defined __cplusplus) && ((EC_OS == EC_OS_WINDOWS) /* || __cplusplus >= 201103 */)
#include <type_traits>
template <typename T, typename R>
static EC_INLINESTART EC_T_VOID OsSafeStrcpy2(T& szDest, R& szSrc) {
    static_assert(std::is_array<T>::value, "OsSafeStrcpy2(): szDest must be an array, please use OsSafeStrncpy() instead!");
    static_assert(std::is_array<R>::value, "OsSafeStrcpy2(): szSrc must be an array or EC_T_CHAR*!");
    static_assert(sizeof(szDest) == sizeof(szSrc), "OsSafeStrcpy2: sizeof(szDest) != sizeof(szSrc)");
    OsSafeStrncpy2(szDest, szSrc, sizeof(szDest));
} EC_INLINESTOP

template <typename T>
static EC_INLINESTART EC_T_VOID OsSafeStrcpy2(T& szDest, const EC_T_CHAR* szSrc) {
    static_assert(std::is_array<T>::value, "OsSafeStrcpy2(): szDest must be an array, please use OsSafeStrncpy() instead!");
    OsSafeStrncpy2(szDest, szSrc, sizeof(szDest));
} EC_INLINESTOP

template <typename T>
static EC_INLINESTART EC_T_VOID OsSafeStrcpy2(T& szDest, EC_T_CHAR* szSrc) {
    OsSafeStrcpy2(szDest, (const EC_T_CHAR*)szSrc);
} EC_INLINESTOP

template <typename T>
static EC_INLINESTART EC_T_VOID OsSafeStrcpy(T& szDest, const EC_T_CHAR* szSrc) {
    OsSafeStrcpy2(szDest, szSrc);
} EC_INLINESTOP
template<typename T>
EC_T_VOID OsMemcpyTyped(T* pDes, const T* pSrc)
{
   OsMemcpy(pDes, pSrc, sizeof(T));
}
template<typename T>
EC_T_VOID OsMemcpyTyped(T& rDes, const T& rSrc)
{
    OsMemcpyTyped(&rDes, &rSrc);
}
#else
#define OsSafeStrcpy(szDest, szSrc)  OsSafeStrncpy(szDest, szSrc, sizeof(szDest))
#define OsSafeStrcpy2(szDest, szSrc) OsSafeStrncpy2(szDest, szSrc, sizeof(szDest))
#define OsMemcpyTyped(rDes, rSrc) OsMemcpy(&rDes, &rSrc, EC_MIN(sizeof(rDes), sizeof(rSrc)))
#endif

#endif /* INC_ECOS */

/*-END OF SOURCE FILE--------------------------------------------------------*/
