/*-----------------------------------------------------------------------------
 * EcOs.h
 * Copyright                acontis technologies GmbH, Ravensburg, Germany
 * Response                 Stefan Zintgraf
 * Description              EC-Master OS-Layer header
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

#ifdef  OsPlatformImplReplaceGetLinkLayerRegFunc
#undef  OsPlatformImplReplaceGetLinkLayerRegFunc
#define OsReplaceGetLinkLayerRegFunc            OsReplaceGetLinkLayerRegFunc
#endif
#define OsPlatformImplReplaceGetLinkLayerRegFunc  OsReplaceGetLinkLayerRegFunc

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
#if 1
#ifdef __cplusplus
extern "C"
{
#endif
#endif

#ifndef INC_EC_COMMON
#include "OsCommon.h"
#endif

/*-DEFINES-------------------------------------------------------------------*/

/* -PACKED STRUCTURES------------------------------------------------------- */
#ifndef EC_T_OS_PARMS
#include EC_PACKED_API_INCLUDESTART
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
    EC_T_DWORD                          dwSupportedFeatures;          /**< [in/out] reserved */
    EC_PF_QUERY_MSEC_COUNT              pfSystemQueryMsecCount;       /**< [in] Function to get system's msec count */
    EC_PF_HW_TIMER_GET_INPUT_FREQUENCY  pfHwTimerGetInputFrequency;   /**< [in] Function to get input frequency of HW timer. This function is needed by some DCM modes described in the Class A manual */
    EC_PF_HW_TIMER_MODIFY_INITIAL_COUNT pfHwTimerModifyInitialCount;  /**< [in] Function to modify initial count of HW timer.  This function is needed by some DCM modes described in the Class A manual */
    EC_PF_HW_TIMER_GET_CURRENT_COUNT    pfHwTimerGetCurrentCount;     /**< [in] Function to get current count of HW timer.  This function is needed by some DCM modes described in the Class A manual */
#if (defined EC_T_OS_PLATFORM_PARMS)
    EC_T_OS_PLATFORM_PARMS              PlatformParms;                /**< [in] Platform specific parameters */
#endif
} EC_PACKED_API EC_T_OS_PARMS;
#include EC_PACKED_INCLUDESTOP
#define EC_T_OS_PARMS EC_T_OS_PARMS
#endif

/*-MACROS--------------------------------------------------------------------*/
#ifndef ATECAT_PLATFORMSTR
#define ATECAT_PLATFORMSTR ""
#endif

#define     SafeOsFree(p)        {if(EC_NULL!=(p)){OsFree(p);(p)=EC_NULL;}}

#define     SafeOsDeleteEvent(p) {if(EC_NULL!=(p)){OsDeleteEvent(p);(p)=EC_NULL;}}

#define     SafeOsDeleteLock(p)  {if(EC_NULL!=(p)){OsDeleteLock(p);(p)=EC_NULL;}}

/*-FUNCTION DECLARATIONS-----------------------------------------------------*/
EC_API EC_T_DWORD EC_API_FNCALL OsInit(EC_T_OS_PARMS* pOsParms);
EC_API EC_T_DWORD EC_API_FNCALL OsDeinit(EC_T_VOID);

#ifndef OsMemset
#define OsMemset(pDest,nVal,nSize)              memset((void*)(pDest),(int)(nVal),(size_t)(nSize))
#endif

#ifndef OsMemsetPdIn
#define OsMemsetPdIn                            OsMemset
#endif

#ifndef OsMemsetPdOut
#define OsMemsetPdOut                           OsMemset
#endif

#ifndef OsMemcpy
#define OsMemcpy(pDest,pSrc,nSize)              memcpy((void*)(pDest),(const void*)(pSrc),(size_t)(nSize))
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
#define OsMemcmp(pBuf1,pBuf2,nSize)             memcmp((void*)(pBuf1),(const void*)(pBuf2),(size_t)(nSize))
#endif

#ifndef OsMemmove
#define OsMemmove(pDest,pSrc,nSize)             memmove((void*)(pDest),(const void*)(pSrc),(size_t)(nSize))
#endif

#ifndef OsStrlen
#define OsStrlen(szString)                      strlen((const char*)(szString))
#endif

#ifndef OsStrcpy
#define OsStrcpy(szDest,szSrc)                  strcpy((char*)(szDest),(const char*)(szSrc))
#endif

#ifndef OsStrncpy
#define OsStrncpy(szDest,szSrc,nSize)           strncpy((char*)(szDest),(const char*)(szSrc),(size_t)(nSize))
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

#ifndef OsDbgAssert
#ifdef INCLUDE_ASSERT_FUNC
EC_FNNAME EC_T_VOID EC_FNCALL         OsDbgAssertFunc(EC_T_BOOL bAssertCondition, const EC_T_CHAR* szFile, EC_T_DWORD dwLine);
#define OsDbgAssert(bAssertCondition) OsDbgAssertFunc((bAssertCondition),(__FILE__), (__LINE__))
#else /* !INCLUDE_ASSERT_FUNC */
#ifdef DEBUG
#define OsDbgAssert                             assert
#else /* !DEBUG */
#define OsDbgAssert(x)
#endif /* !DEBUG */
#endif /* !INCLUDE_ASSERT_FUNC */
#endif /* OsDbgAssert */

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

EC_FNNAME EC_T_VOID* EC_FNCALL OsCfgFileOpen(const EC_T_CHAR* szCfgFileName);
EC_FNNAME EC_T_INT   EC_FNCALL OsCfgFileClose(EC_T_VOID* pvCfgFile);
EC_FNNAME EC_T_INT   EC_FNCALL OsCfgFileRead(EC_T_VOID* pvCfgFile, EC_T_VOID* pvDst, EC_T_INT nLen);
EC_FNNAME EC_T_INT   EC_FNCALL OsCfgFileError(EC_T_VOID* pvCfgFile);
EC_FNNAME EC_T_INT   EC_FNCALL OsCfgFileEof(EC_T_VOID* pvCfgFile);

/* these functions are actually part of the master core */
EC_API  EC_T_INT   EC_API_FNCALL EcVsnprintf(EC_T_CHAR* szDest, EC_T_INT nMaxSize, const EC_T_CHAR* szFormat, EC_T_VALIST vaList);
EC_API  EC_T_INT   EC_API_FNCALL EcSnprintf(EC_T_CHAR* szDest, EC_T_INT nMaxSize, const EC_T_CHAR* szFormat, ...);

#ifndef OsQueryMsecCount
EC_API  EC_T_DWORD EC_API_FNCALL OsQueryMsecCount(EC_T_VOID);
#endif

#ifndef OsReplaceQueryMsecCount
EC_API  EC_T_DWORD EC_API_FNCALL OsReplaceQueryMsecCount(EC_PF_QUERY_MSEC_COUNT pfQueryMsecCount);
#endif

#ifndef EXCLUDE_OSSLEEP_PROTOTYPE
EC_API  EC_T_VOID  EC_API_FNCALL OsPlatformImplSleep(EC_T_DWORD dwMsec);
#endif

/***************************/
/* performance measurement */
/***************************/
/* timestamp counter interface for performance measurements (main functions are part of the master core) */
#ifndef INCLUDE_OS_PLATFORM_TSC_SUPPORT

/* if platform does not have specific TSC support, check for defaults if it is yet available */
#if (defined UNDER_CE && defined _M_IX86) || (defined RTOS_32) || (defined VXWORKS && defined CPU && ((CPU==PENTIUM) || (CPU==PENTIUM4))) || (defined EC_VERSION_QNX) || ((defined WIN32) && !(defined UNDER_CE)) || ((defined EC_VERSION_LINUX) && (defined __i386__)) || defined __INTIME__
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
    EC_T_DWORD      dwCurr;             /**< [1/10   usec] */
    EC_T_DWORD      dwMin;              /**< [1/10   usec] */
    EC_T_DWORD      dwMax;              /**< [1/10   usec] */
    EC_T_DWORD      dwAvg;              /**< [1/1600 usec] */
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

EC_API EC_T_VOID   EC_API_FNCALL OsMeasCalibrate(EC_T_UINT64 dwlFreqSet);
EC_API EC_T_DWORD  EC_API_FNCALL OsMeasGet100kHzFrequency(EC_T_VOID);
EC_API EC_T_UINT64 EC_API_FNCALL OsMeasGetCounterTicks(EC_T_VOID);

/* optional: redirect trace messages into OS specific function, e.g. to store trace log into file
 * default: print trace log as debug message
 */
#ifndef OsTrcMsg
#define     OsTrcMsg    EC_DBGMSG
#endif

EC_API EC_T_VOID*  EC_API_FNCALL OsCreateLock(EC_T_VOID);
EC_API EC_T_VOID*  EC_API_FNCALL OsCreateLockTyped(EC_T_OS_LOCK_TYPE   eLockType);
EC_API EC_T_VOID   EC_API_FNCALL OsDeleteLock(EC_T_VOID* pvLock);
EC_API EC_T_VOID   EC_API_FNCALL OsLock(EC_T_VOID* pvLock);
EC_API EC_T_VOID   EC_API_FNCALL OsUnlock(EC_T_VOID* pvLock);

EC_API EC_T_VOID*  EC_API_FNCALL OsCreateEvent(EC_T_VOID);
EC_API EC_T_VOID   EC_API_FNCALL OsDeleteEvent(EC_T_VOID* pvEvent);
#ifndef OsWaitForEvent
EC_API EC_T_DWORD  EC_API_FNCALL OsWaitForEvent(EC_T_VOID* pvEvent, EC_T_DWORD dwTimeout);
#endif
EC_API EC_T_VOID   EC_API_FNCALL OsPlatformImplSetEvent(EC_T_VOID* pvEvent);
EC_API EC_T_VOID   EC_API_FNCALL OsPlatformImplResetEvent(EC_T_VOID* pvEvent);

EC_API EC_T_VOID*  EC_API_FNCALL OsPlatformImplCreateThread(const EC_T_CHAR* szThreadName, EC_PF_THREADENTRY pfThreadEntry, EC_T_CPUSET cpuAffinityMask, EC_T_DWORD dwPrio, EC_T_DWORD dwStackSize, EC_T_VOID* pvParams);
EC_API EC_T_DWORD  EC_API_FNCALL OsPlatformImplDeleteThreadHandle(EC_T_VOID* pvThreadObject);

#ifndef OsSetThreadPriority
EC_API EC_T_DWORD  EC_API_FNCALL OsSetThreadPriority(EC_T_VOID* pvThreadObject, EC_T_DWORD dwPrio);
#endif
#ifndef OsSetThreadAffinity
EC_API EC_T_DWORD  EC_API_FNCALL OsSetThreadAffinity(EC_T_VOID* pvThreadObject, EC_T_CPUSET CpuSet);
#endif
#ifndef OsGetThreadAffinity
EC_API EC_T_DWORD  EC_API_FNCALL OsGetThreadAffinity(EC_T_VOID* pvThreadObject, EC_T_CPUSET* pCpuSet);
#endif

EC_API EC_PF_LLREGISTER EC_API_FNCALL OsPlatformImplGetLinkLayerRegFunc(EC_T_CHAR* szDriverIdent);
typedef EC_PF_LLREGISTER(*EC_PF_GETLINKLAYERREGFUNC)(EC_T_CHAR* szDriverIdent);
EC_API EC_T_VOID OsPlatformImplReplaceGetLinkLayerRegFunc(EC_PF_GETLINKLAYERREGFUNC pfLinkLayerRegFunc);

EC_API EC_T_DWORD EC_API_FNCALL OsSystemTimeGet(EC_T_UINT64* pqwSystemTime);

#ifndef OsForceThreadPrioNormal
EC_API EC_T_DWORD EC_API_FNCALL OsForceThreadPrioNormal(EC_T_VOID);
#endif


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

#ifndef OsSocketInit
EC_API EC_T_DWORD EC_API_FNCALL OsSocketInit(EC_T_VOID);
#endif
#ifndef OsSocketDeInit
EC_API EC_T_DWORD EC_API_FNCALL OsSocketDeInit(EC_T_VOID);
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

#ifndef OsSocketSelect
#define OsSocketSelect(nNfds, poReadFds, poWriteFds, poExceptFds, poTimeout) select(nNfds, poReadFds, poWriteFds, poExceptFds, poTimeout)
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

#ifndef OsHTONS
#define OsHTONS(wHostVal) htons(wHostVal)
#endif

#endif /* EC_SOCKET_IP_SUPPORTED */

#ifndef OsAuxClkInit
#define OsAuxClkInit(dwCpuIndex, dwFrequencyHz, pvOsEvent) EC_FALSE
#endif

#ifndef OsAuxClkDeinit
#define OsAuxClkDeinit()
#endif

#ifndef OsAuxClkInitialCountGet
#define OsAuxClkInitialCountGet(pdwInitialCount) EC_E_NOTSUPPORTED
#endif

#ifndef OsAuxClkInputFrequencyGet
#define OsAuxClkInputFrequencyGet(pdwFrequencyHz) EC_E_NOTSUPPORTED
#endif

#ifndef OsAuxClkOutputFrequencyGet
#define OsAuxClkOutputFrequencyGet(pdwFrequencyHz) EC_E_NOTSUPPORTED
#endif

#ifndef OsAuxClkCorrectionSet
#define OsAuxClkCorrectionSet(dwInitialCountNew, nIncrementDif, dwIntCountSet) EC_E_NOTSUPPORTED
#endif

#ifndef OsAuxClkTickSinceInterrupt
#define OsAuxClkTickSinceInterrupt(pqwTicksSinceInterrupt) EC_E_NOTSUPPORTED
#endif

#ifndef OsHwTimerGetInputFrequency
#define OsHwTimerGetInputFrequency(pdwFrequencyHz) EC_E_NOTSUPPORTED
#endif

#ifndef OsHwTimerModifyInitialCount
#define OsHwTimerModifyInitialCount(nAdjustPermil) EC_E_NOTSUPPORTED
#endif

#ifndef OsHwTimerGetCurrentCount
static EC_INLINESTART EC_T_DWORD OsHwTimerGetCurrentCountInline(EC_T_INT* pnCountPermil) { EC_UNREFPARM(pnCountPermil); return EC_E_NOTSUPPORTED; } EC_INLINESTOP
#define OsHwTimerGetCurrentCount OsHwTimerGetCurrentCountInline
#endif

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

static EC_INLINESTART EC_T_VOID OsSafeStrcpy(EC_T_CHAR* szTargetString, const EC_T_CHAR* szSourceString, EC_T_INT nMaxSize)
{
    if (nMaxSize != 0)
    {
        OsMemset(szTargetString, 0, nMaxSize);
        OsStrncpy(szTargetString, szSourceString, nMaxSize - 1);
    }
    else
    {
        szTargetString[0] = '\0';
    }
} EC_INLINESTOP
#define SAFE_STRCPY OsSafeStrcpy /* deprecated */

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

/* TODO: remove */
#if 1
#ifdef __cplusplus
} /* extern "C"*/
#endif
#endif

#endif /* INC_ECOS */

/*-END OF SOURCE FILE--------------------------------------------------------*/
