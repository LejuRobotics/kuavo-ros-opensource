/*-----------------------------------------------------------------------------
 * EcOsPlatform.h
 * Copyright                acontis technologies GmbH, Ravensburg, Germany
 * Response                 Paul Bussmann
 * Description              EC-Master OS-Layer header file for Linux
 *----------------------------------------------------------------------------*/

#ifndef INC_ECOSPLATFORM
#define INC_ECOSPLATFORM

/*-SANITY-CHECK---------------------------------------------------------------*/
#ifndef INC_ECOS
#warning EcOsPlatform.h should not be included directly. Include EcOs.h instead
#endif
#ifdef INC_LINK_OS_PLATFORM
#warning EcOsPlatform.h should not be included together with LinkOsPlatform.h.
#endif

/*-SUPPORT-SELECTION----------------------------------------------------------*/
#ifndef EC_SOCKET_IP_SUPPORTED
#define EC_SOCKET_IP_SUPPORTED
#endif

/* LxWin always includes rtoslib sockets */
#if (defined LXWIN) && (!defined EC_SOCKET_RTOSLIB_SUPPORTED)
#define EC_SOCKET_RTOSLIB_SUPPORTED
#endif

/*-SYSTEM-INCLUDES------------------------------------------------------------*/
#include <assert.h>
#include <errno.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <stdarg.h>

#include <pthread.h>
#include <unistd.h>
#include <semaphore.h>

#ifdef EC_SOCKET_IP_SUPPORTED
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#endif /* EC_SOCKET_IP_SUPPORTED */

#if (defined LXWIN)
#include "rtosLib.h"
#endif

/*-OS-VERSION-IDENTIFICATION--------------------------------------------------*/
#if (defined LXWIN)
#ifndef ECWIN
#define ECWIN
#endif
#ifndef ECWIN_LX
#define ECWIN_LX
#endif
#endif

#if (defined ECWIN_LX) && (!defined ATECAT_OSSTR)
#define ATECAT_OSSTR "EC-WinRT-Linux"
#endif

#ifndef ATECAT_OSSTR
#define ATECAT_OSSTR "Linux"
#endif

#ifndef EC_VERSION_LINUX
#define EC_VERSION_LINUX
#endif

/* deprecated, replaced by EC_VERSION_LINUX, kept for legacy code support */
#ifndef LINUX
#define LINUX
#endif

/*-PLATFORM-IDENTIFICATION----------------------------------------------------*/
/* armv4t-eabi */
#if (defined __ARM_ARCH_4T__)
#if (__ARM_ARCH == 4) && (__ARM_ARCH_4T__ == 1)
#ifndef ATECAT_ARCHSTR
#define ATECAT_ARCHSTR "armv4t-eabi"
#endif
#ifndef EC_ARCH
#define EC_ARCH EC_ARCH_ARM
#endif
#endif
#endif /* __ARM_ARCH_4T__ */

/* armv6-vfp-eabihf */
#if (defined __ARM_PCS_VFP)
#if (__ARM_ARCH == 6) && (__ARM_PCS_VFP == 1)
#ifndef ATECAT_ARCHSTR
#define ATECAT_ARCHSTR "armv6-vfp-eabihf"
#endif
#ifndef EC_ARCH
#define EC_ARCH EC_ARCH_ARM
#endif
#endif
#endif /* __ARM_PCS_VFP */

/* arm64 */
#if (defined __aarch64__)
#ifndef ATECAT_ARCHSTR
#define ATECAT_ARCHSTR "aarch64"
#endif
#ifndef EC_ARCH
#define EC_ARCH EC_ARCH_ARM64
#endif
#endif /* __aarch64__ */

/* x64 */
#if (defined __x86_64)
#if (__x86_64 == 1)
#ifndef ATECAT_ARCHSTR
#define ATECAT_ARCHSTR "x64"
#endif
#ifndef EC_ARCH
#define EC_ARCH EC_ARCH_X64
#endif
#endif
#endif /* __x86_64 */

/* x86 (i386) */
#if (defined __i386)
#if (__i386 == 1)
#ifndef ATECAT_ARCHSTR
#define ATECAT_ARCHSTR "x86"
#endif
#ifndef EC_ARCH
#define EC_ARCH EC_ARCH_X86
#endif
#endif
#endif /* __i386 */

/* x86 (i686) */
#if (defined __i686)
#if (__i686 == 1)
#ifndef ATECAT_ARCHSTR
#define ATECAT_ARCHSTR  "x86"
#endif
#ifndef EC_ARCH
#define EC_ARCH EC_ARCH_X86
#endif
#endif
#endif /* __i686 */

/* PPC */
#if (defined __PPC__)
#if (__PPC__ == 1)
#ifndef ATECAT_ARCHSTR
#define ATECAT_ARCHSTR  "PPC"
#endif
#ifndef EC_ARCH
#define EC_ARCH EC_ARCH_PPC
#endif
#endif
#endif /* PPC */

/* undefined architecture */
#ifndef EC_ARCH
#define EC_ARCH EC_ARCH_UNDEFINED
#endif

#ifndef ATECAT_PLATFORMSTR
#ifdef ATECAT_ARCHSTR
#define ATECAT_PLATFORMSTR ATECAT_OSSTR "_" ATECAT_ARCHSTR
#else
#define ATECAT_PLATFORMSTR ATECAT_OSSTR
#endif
#endif /* !ATECAT_PLATFORMSTR */

/*-DEFINES--------------------------------------------------------------------*/
/* Structure pack */
#define EC_PACKED_INCLUDESTART(Bytes)   "EcOsPlatform.h"
#define EC_PACKED_INCLUDESTOP           "EcOsPlatform.h"
#define EC_PACKED(Bytes)                __attribute__((aligned(Bytes),packed))

/* Needed for type / function declarations in EcOsPlatform.h */
#ifndef EC_INLINESTART
#define EC_INLINESTART inline
#endif
#ifndef EC_INLINESTOP
#define EC_INLINESTOP
#endif

#ifndef EC_FNNAME
#define EC_FNNAME EC_FNNAME_DEFAULT
#endif
#ifndef EC_API
#define EC_API EC_FNNAME
#endif

/* API calling convention */
#ifndef EC_FNCALL
#if (EC_ARCH == EC_ARCH_X86)
#define EC_FNCALL __attribute__((__cdecl__))
#else
/* __cdecl__ is not availble for ARM, PPC, x64 */
#define EC_FNCALL
#endif
#endif /* EC_FNCALL */

#ifndef EC_API_FNCALL
#define EC_API_FNCALL EC_FNCALL
#endif

/* on aarch64 unaligned accesses are handled by CPU */
#if (!defined __aarch64__)
#if (defined __arm__)
 /* ARMv6 handles unaligned WORD and DWORD access, but not QWORD access */
 #if (__ARM_ARCH >= 6)
  #if ((defined __ARM_FEATURE_UNALIGNED) && (__ARM_FEATURE_UNALIGNED == 1))
   #define QWORD_WITHALIGNMENT
  #else
   /* __ARM_FEATURE_UNALIGNED should be set for __ARM_ARCH >= 6 */
   #ifndef WITHALIGNMENT
   #warning  Unaligned memory access is turned off!
   #define WITHALIGNMENT
   #endif
  #endif
 #else
  #ifndef WITHALIGNMENT
  #define WITHALIGNMENT   /* --> for ARM up to ARMv5 */
  #endif
 #endif
#endif
#endif

#ifdef _ARCH_PPC
#define EC_BIG_ENDIAN 1
#endif

#if (defined EC_SOCKET_IP_SUPPORTED) && (!defined SD_BOTH)
#define SD_BOTH ((EC_T_INT)2)
#endif

/*-TYPEDEFS-------------------------------------------------------------------*/
typedef struct _OS_LOCK_DESC
{
    EC_T_OS_LOCK_TYPE   eLockType;            /* type of lock e. g. Default, SpinLock, Semaphore */
    pthread_mutex_t     Mutex;
    int                 nLockCnt;
    pthread_t           pThread;
} OS_LOCK_DESC;

typedef va_list             EC_T_VALIST;

typedef enum
{
    eSLEEP_USLEEP,
    eSLEEP_NANOSLEEP,
    eSLEEP_CLOCK_NANOSLEEP
} E_SLEEP;

/**
\defgroup EC_TYPES Fundamental data types
@{
*/
typedef unsigned long long  EC_T_UINT64;      /**< Unsigned integer 64 bits */
typedef signed long long    EC_T_INT64;       /**< Signed integer 64 bits  */
/**@}*/

#define EC_CPUSET_DEFINED
typedef int EC_T_CPUSET;                                          /* CPU-set for SMP systems */

#include EC_PACKED_INCLUDESTART(8)
typedef struct _EC_T_OS_PLATFORM_PARMS
{
#define EC_OS_PLATFORM_PARMS_SIGNATURE_PATTERN (EC_T_DWORD)0x0000CFE0      /* Mask 0x0000FFF0 */
#define EC_OS_PLATFORM_PARMS_SIGNATURE_VERSION (EC_T_DWORD)0x00000001      /* Version 1, mask 0x0000000F */

    EC_T_BOOL bConfigMutex;
    EC_T_INT  nMutexType;
    EC_T_INT  nMutexProtocol;
} EC_PACKED(8) EC_T_OS_PLATFORM_PARMS;
#include EC_PACKED_INCLUDESTOP
#define EC_T_OS_PLATFORM_PARMS EC_T_OS_PLATFORM_PARMS

/*-MACROS---------------------------------------------------------------------*/
#define EC_CPUSET_ZERO(CpuSet)          (CpuSet)=0                /* clear all CPU indexes in the CPU set */
#define EC_CPUSET_IS_ZERO(CpuSet)       (0==(CpuSet))             /* check if any CPU index in the CPU is set */
#define EC_CPUSET_SET(CpuSet,nCpuIndex) (CpuSet)=(1<<(nCpuIndex)) /* set CPU index nCpuIndex (0..x) in the CPU set */
#define EC_CPUSET_SETALL(CpuSet)        (CpuSet)=0xFFFF           /* set all CPU indexes in the CPU set */

#define EC_T_FD_SET  fd_set
#define EC_VASTART   va_start
#define EC_VAEND     va_end
#define EC_VAARG     va_arg

#if (defined DEBUG) && (!defined EXCLUDE_ASSERT_FUNC) && (!defined INCLUDE_ASSERT_FUNC)
#define INCLUDE_ASSERT_FUNC
#endif

/* use macros for the most important OS layer routines */
#define OsSetLastError(dwError)        dwError
#ifndef OsPlatformSleep
#define OsPlatformImplSleep OsPlatformImplSleep
#define OsPlatformSleep     OsPlatformImplSleep
#endif

#ifndef OsPlatformCreateThread
#define OsPlatformImplCreateThread OsPlatformImplCreateThread
#define OsPlatformCreateThread     OsPlatformImplCreateThread
#endif

#ifndef OsPlatformDeleteThreadHandle
#define OsPlatformImplDeleteThreadHandle OsPlatformImplDeleteThreadHandle
#define OsPlatformDeleteThreadHandle     OsPlatformImplDeleteThreadHandle
#endif

#ifndef OsPlatformGetLinkLayerRegFunc
#define OsPlatformImplGetLinkLayerRegFunc OsPlatformImplGetLinkLayerRegFunc
#define OsPlatformGetLinkLayerRegFunc     OsPlatformImplGetLinkLayerRegFunc
#endif

#ifndef OsPlatformReplaceGetLinkLayerRegFunc
#define OsPlatformImplReplaceGetLinkLayerRegFunc OsPlatformImplReplaceGetLinkLayerRegFunc
#define OsPlatformReplaceGetLinkLayerRegFunc     OsPlatformImplReplaceGetLinkLayerRegFunc
#endif

#ifndef OsPlatformSetEvent
#define OsPlatformImplSetEvent OsPlatformImplSetEvent
#define OsPlatformSetEvent     OsPlatformImplSetEvent
#endif

#define OsPlatformResetEvent(pvEvent)  sem_trywait((sem_t*)(pvEvent))

#ifndef OsPlatformMemoryBarrier
#if defined(__GNUC__) && defined(__i386__)
#define OsPlatformMemoryBarrier()     {__asm__ __volatile__ ("lock; addl $0,0(%%esp)" : : : "memory");}
#elif defined(__GNUC__) && defined(__PPC__)
  #define OsPlatformMemoryBarrier()   {__asm__ __volatile__ ("eieio" : : : "memory");}
#elif defined(__GNUC__) && defined(__arm__)
  #if (__ARM_ARCH >= 7)
    #define OsPlatformMemoryBarrier()   {__asm__ __volatile__ ("dsb" : : : "memory");}
  #else
    #define OsPlatformMemoryBarrier()   {__asm__ __volatile__ ("mcr   p15,0,%[t],c7,c10,4\n" :: [t] "r" (0) : "memory");}
  #endif
#else
  #define OsPlatformMemoryBarrier()
#endif
#endif /* OsPlatformMemoryBarrier */

#define OsStricmp strcasecmp

#ifdef EC_SOCKET_IP_SUPPORTED
#  define INVALID_SOCKET (-1)
#  define SOCKET_ERROR (-1)
#  define OsSocketAccept(hSockHandle, oSockAddr, nSockAddrLen) \
     accept(hSockHandle, oSockAddr, (socklen_t *)(nSockAddrLen))

static EC_INLINESTART ssize_t OsSocketSend(int sockfd, const void *buf, size_t len, int flags)
{
    return send(sockfd, buf, len, flags | MSG_NOSIGNAL);
} EC_INLINESTOP
#define OsSocketSend OsSocketSend

#  define OsSocketRecvFrom(hSockHandle, pbyBuffer, dwBufferLen, dwFlags, oSrcAddr, dwSrcAddrLen) \
      recvfrom(hSockHandle, pbyBuffer, dwBufferLen, dwFlags, oSrcAddr, (socklen_t *) (dwSrcAddrLen))

#define OsSocketGetLastError() errno
#endif /* EC_SOCKET_IP_SUPPORTED */

/*-PROTOTYPES-----------------------------------------------------------------*/

EC_API EC_T_VOID EC_API_FNCALL OsSleepSetType(E_SLEEP ESleep);

EC_API EC_T_DWORD EC_API_FNCALL OsAuxClkInit(EC_T_DWORD dwCpuIndex, EC_T_DWORD dwFrequencyHz, EC_T_VOID* pvOsEvent);
#define OsAuxClkInit OsAuxClkInit

EC_API EC_T_DWORD EC_API_FNCALL OsAuxClkDeinit(EC_T_VOID);
#define OsAuxClkDeinit OsAuxClkDeinit

EC_API EC_T_DWORD EC_API_FNCALL OsHwTimerGetInputFrequency(EC_T_DWORD *pdwFrequencyHz);
#define OsHwTimerGetInputFrequency OsHwTimerGetInputFrequency

EC_API EC_T_DWORD EC_API_FNCALL OsHwTimerModifyInitialCount(EC_T_INT nAdjustPermil);
#define OsHwTimerModifyInitialCount OsHwTimerModifyInitialCount


#endif /* INC_ECOSPLATFORM */

/*-END OF SOURCE FILE---------------------------------------------------------*/

