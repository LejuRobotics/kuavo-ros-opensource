/** ---------------------------------------------------------------------------
 * \file
 * \brief       Common declarations for EcOs.h / LinkOsLayer.h
 * \author      Paul Bussmann
 * \copyright   acontis technologies GmbH, Weingarten, Germany
 *---------------------------------------------------------------------------*/

#ifndef INC_OS_COMMON
#define INC_OS_COMMON

/*-INCLUDES------------------------------------------------------------------*/
#include <stddef.h>
#if (defined WIN32) && (EC_OS != EC_OS_WINCE)
#include <direct.h>
#endif

/*-DEFINES-------------------------------------------------------------------*/
/* *** EC_PACKED_... *** */
#ifndef EC_PACKED_API_INCLUDESTART
#define EC_PACKED_API_INCLUDESTART  EC_PACKED_INCLUDESTART(8)
#endif
#ifndef EC_PACKED_API_MEMBER
#define EC_PACKED_API_MEMBER        EC_PACKED(8)
#endif
#ifndef EC_PACKED_API
#define EC_PACKED_API               EC_PACKED(8)
#endif

/* *** EC_INT32_MAX, EC_INT64_MAX *** */
#if (!defined EC_INT32_MAX) && (defined LONG_MAX)
#define EC_INT32_MAX    LONG_MAX
#endif
#if (!defined EC_INT64_MAX) && (defined LLONG_MAX)
#define EC_INT64_MAX    LLONG_MAX
#endif
#if (!defined EC_INT64_MAX) && (defined LONG_LONG_MAX)
#define EC_INT64_MAX    LONG_LONG_MAX
#endif
#if (!defined EC_INT32_MAX) && (defined _I32_MAX)
#define EC_INT32_MAX    _I32_MAX
#endif
#if (!defined EC_INT64_MAX) && (defined _I64_MAX)
#define EC_INT64_MAX    _I64_MAX
#endif

/* *** Socket support *** */
#if (defined EC_SOCKET_IP_SUPPORTED) || (defined EC_SOCKET_MSGQUEUE_WIN32_SUPPORTED) || (defined EC_SOCKET_MSGQUEUE_RTOSSHM_SUPPORTED) || (defined EC_SOCKET_RTOSLIB_SUPPORTED)
#define EC_SOCKET_SUPPORTED
#endif

#if (defined EC_SOCKET_SUPPORTED)

/* Socket layer internal error code selection */
#ifndef EC_SOCKET_NOERROR
#define EC_SOCKET_NOERROR 0
#endif

#if !(defined EC_E_BSD_ENOTCONN) && (defined ENOTCONN)
#define EC_E_BSD_ENOTCONN        ENOTCONN
#endif
#if !(defined EC_E_BSD_ENOTSOCK) && (defined ENOTSOCK)
#define EC_E_BSD_ENOTSOCK        ENOTSOCK
#endif
#if !(defined EC_E_BSD_ESHUTDOWN) && (defined ESHUTDOWN)
#define EC_E_BSD_ESHUTDOWN       ESHUTDOWN
#endif
#if !(defined EC_E_BSD_EHOSTUNREACH) && (defined EHOSTUNREACH)
#define EC_E_BSD_EHOSTUNREACH    EHOSTUNREACH
#endif
#if !(defined EC_E_BSD_EINVAL) && (defined EINVAL)
#define EC_E_BSD_EINVAL          EINVAL
#endif
#if !(defined EC_E_BSD_EMSGSIZE) && (defined EMSGSIZE)
#define EC_E_BSD_EMSGSIZE        EMSGSIZE
#endif
#if !(defined EC_E_BSD_ECONNABORTED) && (defined ECONNABORTED)
#define EC_E_BSD_ECONNABORTED    ECONNABORTED
#endif
#if !(defined EC_E_BSD_ETIMEDOUT) && (defined ETIMEDOUT)
#define EC_E_BSD_ETIMEDOUT       ETIMEDOUT
#endif
#if !(defined EC_E_BSD_ECONNRESET) && (defined ECONNRESET)
#define EC_E_BSD_ECONNRESET      ECONNRESET
#endif
#if !(defined EC_E_BSD_ECONNREFUSED) && (defined ECONNREFUSED)
#define EC_E_BSD_ECONNREFUSED    ECONNREFUSED
#endif
#if !(defined EC_E_BSD_EPIPE) && (defined EPIPE)
#define EC_E_BSD_EPIPE           EPIPE
#endif
#if !(defined EC_E_BSD_EINPROGRESS) && (defined EINPROGRESS)
#define EC_E_BSD_EINPROGRESS     EINPROGRESS
#endif

#ifndef EC_INVALID_SOCKET
#define EC_INVALID_SOCKET INVALID_SOCKET
#endif

#ifndef EC_SOCKET_ERROR
#define EC_SOCKET_ERROR SOCKET_ERROR
#endif

#ifndef EC_T_SOCKET
#define EC_T_SOCKET EC_T_INT
#endif

#ifndef EC_SOMAXCONN
#define EC_SOMAXCONN SOMAXCONN
#endif

#ifndef OsInetAddr
#define OsInetAddr inet_addr
#endif
#endif /* EC_SOCKET_SUPPORTED */

/*-TYPEDEFS------------------------------------------------------------------*/
typedef EC_T_VOID (EC_FNCALL *EC_PF_OSTIMER)(EC_T_VOID);
typedef EC_T_BOOL (EC_FNCALL *EC_PF_OSDBGMSGHK)(const EC_T_CHAR* szFormat, EC_T_VALIST vaArgs);
typedef EC_T_VOID (EC_FNCALL *EC_PF_THREADENTRY)(EC_T_VOID* pvParams);
typedef EC_T_VOID (EC_FNCALL *EC_PF_EXCEPTION_CALLBACK)(const EC_T_CHAR* szThreadName, EC_T_VOID* pvCbParams);
struct _EC_T_LOG_CONTEXT;
/**
\typedef EC_PF_LOGMSGHK
\param pContext          [in] Context pointer. This pointer is used as parameter when the callback function is called.
\param dwLogMsgSeverity  [in] Log message severity, EC_LOG_LEVEL_...
\param szFormat          [in] String that contains the text to be written. It can optionally contain embedded format specifiers that are replaced by the values specified in subsequent additional arguments and formatted as requested.
\return #EC_E_NOERROR or error code
*/
typedef EC_T_DWORD (EC_FNCALL *EC_PF_LOGMSGHK)(struct _EC_T_LOG_CONTEXT* pContext, EC_T_DWORD dwLogMsgSeverity, const EC_T_CHAR* szFormat, ...);

struct _EC_T_LINK_DRV_DESC;
typedef EC_T_DWORD (EC_FNCALL *EC_PF_LLREGISTER)(struct _EC_T_LINK_DRV_DESC* pLinkDrvDesc, EC_T_DWORD dwLinkDrvDescSize);

/* CPU-set for SMP systems. Widened to 64 bit (was 32 bit / unsigned long) so systems with
   more than 32 logical processors can be addressed via EC_CPUSET_SET() etc. This is the
   single shared definition for every platform that doesn't need its own native CPU-set type
   (see e.g. VxWorks, which delegates to the native cpuset_t instead of overriding this). */
#ifndef EC_CPUSET_DEFINED
#define EC_CPUSET_DEFINED 1
typedef EC_T_UINT64 EC_T_CPUSET;
#endif

typedef EC_T_DWORD (EC_FNCALL *EC_PF_SYSTIME)(EC_T_UINT64* pqwSystemTime);
typedef EC_T_DWORD (EC_FNCALL *EC_PF_QUERY_MSEC_COUNT)(EC_T_VOID);
typedef EC_T_VOID  (EC_FNCALL *EC_PF_SLEEP)(EC_T_DWORD dwMsec);

#if (defined EC_SOCKET_SUPPORTED)
#ifndef EC_T_FD_SET
typedef struct fd_set EC_T_FD_SET;
#endif

#ifndef EC_T_TIMEVAL
typedef struct timeval EC_T_TIMEVAL;
#endif

#ifndef EC_T_SOCKADDR
typedef struct sockaddr EC_T_SOCKADDR;
#endif

#ifndef EC_T_SOCKADDR_IN
typedef struct sockaddr_in EC_T_SOCKADDR_IN;
#endif

#ifndef EC_T_SOCKLEN
#if defined(__BORLANDC__)
typedef int EC_T_SOCKLEN;
#else
typedef socklen_t EC_T_SOCKLEN;
#endif
#endif
#endif /* EC_SOCKET_SUPPORTED */


/* *** EC_INLINE *** */
#ifndef EC_INLINEKEYWORD
#define EC_INLINEKEYWORD __inline
#endif
#ifndef EC_INLINEATTRIBUTE
#define EC_INLINEATTRIBUTE
#endif

#ifndef EC_INLINESTART
#define EC_INLINESTART __inline
#endif
#ifndef EC_INLINESTOP
#define EC_INLINESTOP
#endif

#ifndef EC_OVERRIDE
#define EC_OVERRIDE
#endif

#ifndef EC_EXPLICIT
#if defined(_MSC_VER) && (_MSC_VER >= 1200 )
#define EC_EXPLICIT explicit
#elif defined(__GNUC__) && (__GNUC__ >= 3 )
#define EC_EXPLICIT explicit
#else
#define EC_EXPLICIT
#endif
#endif

#if (defined INSTRUMENT_OS) || (defined INSTRUMENT_MASTER) || (defined INSTRUMENT_LL || (defined INSTRUMENT_SIMULATOR))
#define EC_INSTRUMENT_MOCKABLE_INLINESTART virtual
#define EC_INSTRUMENT_MOCKABLE_INLINESTOP
#define EC_INSTRUMENT_MOCKABLE_FUNC virtual
#define EC_INSTRUMENT_MOCKABLE_VAR public:
#else
#define EC_INSTRUMENT_MOCKABLE_INLINESTART EC_INLINESTART
#define EC_INSTRUMENT_MOCKABLE_INLINESTOP
#define EC_INSTRUMENT_MOCKABLE_FUNC
#define EC_INSTRUMENT_MOCKABLE_VAR
#endif

#define     EC_OFFSETOF(s,m)    ((size_t)&(((s *)0)->m))

#define     EC_INTSIZEOF(n)     ( (sizeof(n) + sizeof(int) - 1) & ~(sizeof(int) - 1) )
#ifndef EC_VASTART
#define     EC_VASTART(ap,v)    ( ap = (EC_T_VALIST)&v + EC_INTSIZEOF(v) )
#endif
#ifndef EC_VAARG
#define     EC_VAARG(ap,t)      ( *(t *)((ap += EC_INTSIZEOF(t)) - EC_INTSIZEOF(t)) )
#endif
#ifndef EC_VAEND
#define     EC_VAEND(ap)        ( ap = (EC_T_VALIST)0 )
#endif

#ifndef EC_UNREFPARM
#define     EC_UNREFPARM(p)     {(EC_T_VOID)(p);}
#endif
#define     EC_MAX(a,b)         (((a) > (b)) ? (a) : (b))
#define     EC_MIN(a,b)         (((a) < (b)) ? (a) : (b))
#define     EC_AT_LEAST         EC_MAX
#define     EC_AT_MOST          EC_MIN
#define     EC_ENDOF(p)         ((p)+1)
#define     EC_NUMOFELEMENTS(table) (sizeof(table)/sizeof(table[0]))
#define     EC_IS_RANGE_OUT_OF_BOUNDS(loRange, hiRange, loBound, hiBound) (((hiRange) <= (loBound)) || ((loRange) >= (hiBound)))
#define     EC_VARSIZE_STRUCT_SIZE(pParms) ((pParms)->dwSize)
#define     EC_VARSIZE_STRUCT_MEMBER_OFFSET(pParms, MEMBER) ((EC_T_DWORD)((EC_T_BYTE*)(&((pParms)->MEMBER))-(EC_T_BYTE*)(pParms)))
#define     EC_VARSIZE_STRUCT_CONTAINS_MEMBER(pParms, MEMBER) (EC_VARSIZE_STRUCT_SIZE((pParms)) > EC_VARSIZE_STRUCT_MEMBER_OFFSET((pParms), MEMBER))

#ifndef SafeDelete
#define     SafeDelete(p)       {if (EC_NULL!=(p)) {delete    (p); (p) = EC_NULL;}}
#endif
#ifndef SafeDeleteArray
#define     SafeDeleteArray(p)  {if (EC_NULL!=(p)) {delete [] (p); (p) = EC_NULL;}}
#endif

/* SMP support */
/* This macros will be overloaded by the specific EcOsPlatform.h */
#ifndef EC_CPUSET_ZERO
#define EC_CPUSET_ZERO(CpuSet)          (CpuSet)=0                /* clear all CPU indexes in the CPU set */
#endif
#ifndef EC_CPUSET_IS_ZERO
#define EC_CPUSET_IS_ZERO(CpuSet)       (0==(CpuSet))             /* check if any CPU index in the CPU is set */
#endif
#ifndef EC_CPUSET_SET
#define EC_CPUSET_SET(CpuSet,nCpuIndex) (CpuSet)=((EC_T_CPUSET)1<<(nCpuIndex)) /* set CPU index nCpuIndex (0..x) in the CPU set */
#endif
#ifndef EC_CPUSET_SETALL
#define EC_CPUSET_SETALL(CpuSet)        (CpuSet)=(EC_T_CPUSET)-1  /* set all CPU indexes in the CPU set */
#endif

#ifndef BIT2BYTE
#define BIT2BYTE(x) (((x)+7)>>3)
#endif

/* EC_BYTE_ALIGN_DOWN, EC_BYTE_ALIGN_UP, EC_WORD_ALIGN_UP */
static EC_INLINESTART EC_T_DWORD EC_BYTE_ALIGN_DOWN(EC_T_DWORD val) { return val & ~7; } EC_INLINESTOP
static EC_INLINESTART EC_T_DWORD EC_BYTE_ALIGN_UP(EC_T_DWORD val) { return (val + 7) & ~7; } EC_INLINESTOP
static EC_INLINESTART EC_T_DWORD EC_WORD_ALIGN_UP(EC_T_DWORD val) { return (val + 15) & ~15; } EC_INLINESTOP

#define EC_BIT_MASK(Bitsize)                            ((1<<(Bitsize))-1)
#define EC_BITFIELD_MASK(Bitpos,Bitsize)                ((EC_BIT_MASK((Bitsize)))<<(Bitpos))
#define EC_RESET_WORD_IN_BITFIELD(wVal,Bitpos,Bitsize)  ((wVal)&(~EC_BITFIELD_MASK((Bitpos),(Bitsize))))
#define EC_SET_WORD_IN_BITFIELD(Bitpos,Bitsize,wVal)    (((wVal)&(EC_BIT_MASK((Bitsize))))<<(Bitpos))
#define EC_GET_WORD_IN_BITFIELD(Bitpos,Bitsize,wVal)    (((wVal)>>(Bitpos))&EC_BIT_MASK((Bitsize)))

#define EC_BYTE0(x)     (EC_T_BYTE)(((x) >>  0) & 0xFF)
#define EC_BYTE1(x)     (EC_T_BYTE)(((x) >>  8) & 0xFF)
#define EC_BYTE2(x)     (EC_T_BYTE)(((x) >> 16) & 0xFF)
#define EC_BYTE3(x)     (EC_T_BYTE)(((x) >> 24) & 0xFF)
#define EC_BYTE4(x)     (EC_T_BYTE)(((x) >> 32) & 0xFF)
#define EC_BYTE5(x)     (EC_T_BYTE)(((x) >> 40) & 0xFF)
#define EC_BYTE6(x)     (EC_T_BYTE)(((x) >> 48) & 0xFF)
#define EC_BYTE7(x)     (EC_T_BYTE)(((x) >> 56) & 0xFF)

#define EC_BYTEN(ptr, n) \
    ((EC_T_BYTE*)(ptr))[(n)]

#define EC_WORD0(x)     (((x) >>  0) & 0xFFFF)
#define EC_WORD1(x)     (((x) >> 16) & 0xFFFF)

#if (!defined EC_WORDSWAP)
static EC_INLINESTART EC_T_WORD EC_WORDSWAP_IMPL(EC_T_WORD wSrc)
{
    return (EC_T_WORD)((((EC_T_WORD)EC_BYTE0(wSrc)) << 8) |
                       (((EC_T_WORD)EC_BYTE1(wSrc))));
} EC_INLINESTOP
#define EC_WORDSWAP(x) EC_WORDSWAP_IMPL(x)
#endif

#if (!defined EC_DWORDSWAP)
static EC_INLINESTART EC_T_DWORD EC_DWORDSWAP_IMPL(EC_T_DWORD dwSrc)
{
    return (EC_T_DWORD)((((EC_T_DWORD)EC_BYTE0(dwSrc)) << 24) |
                        (((EC_T_DWORD)EC_BYTE1(dwSrc)) << 16) |
                        (((EC_T_DWORD)EC_BYTE2(dwSrc)) <<  8) |
                        (((EC_T_DWORD)EC_BYTE3(dwSrc))));
} EC_INLINESTOP
#define EC_DWORDSWAP(x) EC_DWORDSWAP_IMPL(x)
#endif

#if (!defined EC_DWORDSWAPHBLB)
static EC_INLINESTART EC_T_DWORD EC_DWORDSWAPHBLB_IMPL(EC_T_DWORD dwSrc)
{
    return (EC_T_DWORD)((((EC_T_DWORD)EC_BYTE0(dwSrc)) <<  8) |
                        (((EC_T_DWORD)EC_BYTE1(dwSrc)))       |
                        (((EC_T_DWORD)EC_BYTE2(dwSrc)) << 24) |
                        (((EC_T_DWORD)EC_BYTE3(dwSrc)) << 16));
} EC_INLINESTOP
#define EC_DWORDSWAPHBLB(x) EC_DWORDSWAPHBLB_IMPL(x)
#endif

#if (!defined EC_QWORDSWAP)
static EC_INLINESTART EC_T_UINT64 EC_QWORDSWAP_IMPL(EC_T_UINT64 qwSrc)
{
    return (EC_T_UINT64)((((EC_T_UINT64)EC_BYTE0(qwSrc)) << 56) |
                         (((EC_T_UINT64)EC_BYTE1(qwSrc)) << 48) |
                         (((EC_T_UINT64)EC_BYTE2(qwSrc)) << 40) |
                         (((EC_T_UINT64)EC_BYTE3(qwSrc)) << 32) |
                         (((EC_T_UINT64)EC_BYTE4(qwSrc)) << 24) |
                         (((EC_T_UINT64)EC_BYTE5(qwSrc)) << 16) |
                         (((EC_T_UINT64)EC_BYTE6(qwSrc)) <<  8) |
                         (((EC_T_UINT64)EC_BYTE7(qwSrc))));
} EC_INLINESTOP
#define EC_QWORDSWAP(x) EC_QWORDSWAP_IMPL(x)
#endif

#ifndef EC_REALSWAP
static EC_INLINESTART EC_T_REAL EC_REALSWAP_INLINE(EC_T_REAL fSrc)
{
        EC_T_VOID* pvSrc = (EC_T_VOID*)&fSrc;
        EC_T_DWORD dwHelper = EC_DWORDSWAP(*((EC_T_DWORD*)pvSrc));
        EC_T_VOID* pvDest = (EC_T_VOID*)&dwHelper;
        return *((EC_T_REAL*)pvDest);
} EC_INLINESTOP
#define EC_REALSWAP(fSrc) EC_REALSWAP_INLINE((fSrc))
#endif /* EC_REALSWAP */

#ifndef EC_LREALSWAP
static EC_INLINESTART EC_T_LREAL EC_LREALSWAP_INLINE(EC_T_LREAL fSrc)
{
        EC_T_VOID* pvSrc = (EC_T_VOID*)&fSrc;
        EC_T_UINT64 qwHelper = EC_QWORDSWAP(*((EC_T_UINT64*)pvSrc));
        EC_T_VOID* pvDest = (EC_T_VOID*)&qwHelper;
        return *((EC_T_LREAL*)pvDest);
} EC_INLINESTOP
#define EC_LREALSWAP(fSrc) EC_LREALSWAP_INLINE((fSrc))
#endif /* EC_LREALSWAP */

#ifndef EC_MAKE_SIGNATURE_VERSION
#define EC_MAKE_SIGNATURE_VERSION(a,b,c,d) (((a)<<16)+((b)<<12)+((c)<<8)+d)
#endif
#ifndef EC_MAKEWORD
#define EC_MAKEWORD(hi, lo)     ((EC_T_WORD )(((EC_T_BYTE)(lo)) | ((EC_T_WORD )((EC_T_BYTE)(hi))) <<  8))
#endif
#ifndef EC_MAKEDWORD
#define EC_MAKEDWORD(hi, lo)    ((EC_T_DWORD)(((EC_T_WORD)(lo)) | ((EC_T_DWORD)((EC_T_WORD)(hi))) << 16))
#endif
#ifndef EC_MAKEQWORD
#define EC_MAKEQWORD(hi, lo)    ((EC_T_UINT64)(((EC_T_DWORD)(lo)) | ((EC_T_UINT64)((EC_T_DWORD)(hi))) << 32))
#endif
#ifndef EC_LODWORD
#define EC_LODWORD(qw)          ((EC_T_DWORD)((EC_T_UINT64)(qw) & 0xFFFFFFFF))
#endif
#ifndef EC_HIDWORD
#define EC_HIDWORD(qw)          ((EC_T_DWORD)(((EC_T_UINT64)(qw) >> 32) & 0xFFFFFFFF))
#endif
#ifndef EC_LOWORD
#define EC_LOWORD(dw)           ((EC_T_WORD)((dw) & 0xFFFF))
#endif
#ifndef EC_HIWORD
#define EC_HIWORD(dw)           ((EC_T_WORD)(((EC_T_DWORD)(dw) >> 16) & 0xFFFF))
#endif
#ifndef EC_LOBYTE
#define EC_LOBYTE(w)            ((EC_T_BYTE)((w) & 0xFF))
#endif
#ifndef EC_HIBYTE
#define EC_HIBYTE(w)            ((EC_T_BYTE)(((EC_T_WORD)(w)   >>  8) &   0xFF))
#endif


/* memory access */
#if (defined WITHALIGNMENT)
    #ifndef EC_SETWORD
    #define EC_SETWORD(pvAddress, wVal) EC_SETWORD_IMPL((EC_T_VOID*)(pvAddress),(EC_T_WORD)(wVal))
    static EC_INLINESTART EC_T_VOID EC_SETWORD_IMPL(EC_T_VOID* pvAddress, EC_T_WORD wVal)
    {
        EC_BYTEN((pvAddress), 0) = EC_BYTE0(((EC_T_WORD)wVal));
        EC_BYTEN((pvAddress), 1) = EC_BYTE1(((EC_T_WORD)wVal));
    } EC_INLINESTOP
    #endif
    #ifndef EC_SETDWORD
    #define EC_SETDWORD(pvAddress, dwVal) EC_SETDWORD_IMPL((EC_T_VOID*)(pvAddress),(EC_T_DWORD)(dwVal))
    static EC_INLINESTART EC_T_VOID EC_SETDWORD_IMPL(EC_T_VOID* pvAddress, EC_T_DWORD dwVal)
    {
        EC_BYTEN((pvAddress), 0) = EC_BYTE0(((EC_T_DWORD)dwVal));
        EC_BYTEN((pvAddress), 1) = EC_BYTE1(((EC_T_DWORD)dwVal));
        EC_BYTEN((pvAddress), 2) = EC_BYTE2(((EC_T_DWORD)dwVal));
        EC_BYTEN((pvAddress), 3) = EC_BYTE3(((EC_T_DWORD)dwVal));
    } EC_INLINESTOP
    #endif
    #ifndef EC_SETINT
    #define EC_SETINT(pvAddress, nVal) EC_SETINT_IMPL((EC_T_VOID*)(pvAddress),(EC_T_INT)(nVal))
    static EC_INLINESTART EC_T_VOID EC_SETINT_IMPL(EC_T_VOID* pvAddress, EC_T_INT nVal)
    {
        EC_BYTEN((pvAddress), 0) = EC_BYTE0(((EC_T_INT)nVal));
        EC_BYTEN((pvAddress), 1) = EC_BYTE1(((EC_T_INT)nVal));
        EC_BYTEN((pvAddress), 2) = EC_BYTE2(((EC_T_INT)nVal));
        EC_BYTEN((pvAddress), 3) = EC_BYTE3(((EC_T_INT)nVal));
    } EC_INLINESTOP
    #endif

    #ifndef EC_GETWORD
    #define EC_GETWORD(pvAddress) EC_GETWORD_IMPL((EC_T_VOID*)(pvAddress))
    static EC_INLINESTART EC_T_WORD EC_GETWORD_IMPL(EC_T_VOID* pvAddress)
    {
        return ((((EC_T_WORD)(EC_BYTEN((pvAddress), 0))) << 0) |
                (((EC_T_WORD)(EC_BYTEN((pvAddress), 1))) << 8));
    } EC_INLINESTOP
    #endif
    #ifndef EC_GETDWORD
    #define EC_GETDWORD(pvAddress) EC_GETDWORD_IMPL((EC_T_VOID*)(pvAddress))
    static EC_INLINESTART EC_T_DWORD EC_GETDWORD_IMPL(EC_T_VOID* pvAddress)
    {
        return ((((EC_T_DWORD)(EC_BYTEN((pvAddress), 0))) << 0)  |
                (((EC_T_DWORD)(EC_BYTEN((pvAddress), 1))) << 8)  |
                (((EC_T_DWORD)(EC_BYTEN((pvAddress), 2))) << 16) |
                (((EC_T_DWORD)(EC_BYTEN((pvAddress), 3))) << 24));
    } EC_INLINESTOP
    #endif
    #ifndef EC_GETINT
    #define EC_GETINT(pvAddress) EC_GETINT_IMPL((EC_T_VOID*)(pvAddress))
    static EC_INLINESTART EC_T_INT EC_GETINT_IMPL(EC_T_VOID* pvAddress)
    {
        return ((((EC_T_INT)(EC_BYTEN((pvAddress), 0))) << 0) |
                (((EC_T_INT)(EC_BYTEN((pvAddress), 1))) << 8) |
                (((EC_T_INT)(EC_BYTEN((pvAddress), 2))) << 16) |
                (((EC_T_INT)(EC_BYTEN((pvAddress), 3))) << 24));
    } EC_INLINESTOP
    #endif
#else
    #ifndef EC_SETWORD
    #define EC_SETWORD(pvAddress, wVal) EC_SETWORD_IMPL((EC_T_VOID*)(pvAddress),(EC_T_WORD)(wVal))
    static EC_INLINESTART EC_T_VOID EC_SETWORD_IMPL(EC_T_VOID* pvAddress, EC_T_WORD wVal)
    {
        /* alignment does not need to be considered on x86 */
        *(EC_T_WORD*)pvAddress = wVal;
    } EC_INLINESTOP
    #endif
    #ifndef EC_SETDWORD
    #define EC_SETDWORD(pvAddress, dwVal) EC_SETDWORD_IMPL((EC_T_VOID*)(pvAddress),(EC_T_DWORD)(dwVal))
    static EC_INLINESTART EC_T_VOID EC_SETDWORD_IMPL(EC_T_VOID* pvAddress, EC_T_DWORD dwVal)
    {
        /* alignment does not need to be considered on x86 */
        *(EC_T_DWORD*)pvAddress = dwVal;
    } EC_INLINESTOP
    #endif
    #ifndef EC_SETINT
    #define EC_SETINT(pvAddress, nVal) EC_SETINT_IMPL((EC_T_VOID*)(pvAddress),(EC_T_INT)(nVal))
    static EC_INLINESTART EC_T_VOID EC_SETINT_IMPL(EC_T_VOID* pvAddress, EC_T_INT nVal)
    {
        /* alignment does not need to be considered on x86 */
        *(EC_T_INT*)pvAddress = nVal;
    } EC_INLINESTOP
    #endif
    #ifndef EC_GETWORD
    #define EC_GETWORD(pvAddress) EC_GETWORD_IMPL((EC_T_VOID*)(pvAddress))
    static EC_INLINESTART EC_T_WORD EC_GETWORD_IMPL(EC_T_VOID* pvAddress)
    {
        /* alignment does not need to be considered on x86 */
        return *(EC_T_WORD*)pvAddress;
    } EC_INLINESTOP
    #endif
    #ifndef EC_GETDWORD
    #define EC_GETDWORD(pvAddress) EC_GETDWORD_IMPL((EC_T_VOID*)(pvAddress))
    static EC_INLINESTART EC_T_DWORD EC_GETDWORD_IMPL(EC_T_VOID* pvAddress)
    {
        /* alignment does not need to be considered on x86 */
        return *(EC_T_DWORD*)pvAddress;
    } EC_INLINESTOP
    #endif
    #ifndef EC_GETINT
    #define EC_GETINT(pvAddress) EC_GETINT_IMPL((EC_T_VOID*)(pvAddress))
    static EC_INLINESTART EC_T_INT EC_GETINT_IMPL(EC_T_VOID* pvAddress)
    {
        /* alignment does not need to be considered on x86 */
        return *(EC_T_INT*)pvAddress;
    } EC_INLINESTOP
    #endif
#endif

#if (defined WITHALIGNMENT) || (defined QWORD_WITHALIGNMENT)
    #ifndef EC_SETQWORD
    #define EC_SETQWORD(pvAddress, qwVal) EC_SETQWORD_IMPL((EC_T_VOID*)(pvAddress),(EC_T_UINT64)(qwVal))
    static EC_INLINESTART EC_T_VOID EC_SETQWORD_IMPL(EC_T_VOID* pvAddress, EC_T_UINT64 qwVal)
    {
        EC_BYTEN((pvAddress), 0) = EC_BYTE0(((EC_T_UINT64)qwVal));
        EC_BYTEN((pvAddress), 1) = EC_BYTE1(((EC_T_UINT64)qwVal));
        EC_BYTEN((pvAddress), 2) = EC_BYTE2(((EC_T_UINT64)qwVal));
        EC_BYTEN((pvAddress), 3) = EC_BYTE3(((EC_T_UINT64)qwVal));
        EC_BYTEN((pvAddress), 4) = EC_BYTE4(((EC_T_UINT64)qwVal));
        EC_BYTEN((pvAddress), 5) = EC_BYTE5(((EC_T_UINT64)qwVal));
        EC_BYTEN((pvAddress), 6) = EC_BYTE6(((EC_T_UINT64)qwVal));
        EC_BYTEN((pvAddress), 7) = EC_BYTE7(((EC_T_UINT64)qwVal));
    } EC_INLINESTOP
    #endif

    #ifndef EC_GETQWORD
    #define EC_GETQWORD(pvAddress) EC_GETQWORD_IMPL((EC_T_VOID*)(pvAddress))
    static EC_INLINESTART EC_T_UINT64 EC_GETQWORD_IMPL(EC_T_VOID* pvAddress)
    {
        return ((((EC_T_UINT64)(EC_BYTEN((pvAddress), 0))) << 0)   |
                (((EC_T_UINT64)(EC_BYTEN((pvAddress), 1))) << 8)  |
                (((EC_T_UINT64)(EC_BYTEN((pvAddress), 2))) << 16) |
                (((EC_T_UINT64)(EC_BYTEN((pvAddress), 3))) << 24) |
                (((EC_T_UINT64)(EC_BYTEN((pvAddress), 4))) << 32) |
                (((EC_T_UINT64)(EC_BYTEN((pvAddress), 5))) << 40) |
                (((EC_T_UINT64)(EC_BYTEN((pvAddress), 6))) << 48) |
                (((EC_T_UINT64)(EC_BYTEN((pvAddress), 7))) << 56));
    } EC_INLINESTOP
    #endif
#else
    #ifndef EC_SETQWORD
    #define EC_SETQWORD(pvAddress, qwVal) EC_SETQWORD_IMPL((EC_T_VOID*)(pvAddress),(EC_T_UINT64)(qwVal))
    static EC_INLINESTART EC_T_VOID EC_SETQWORD_IMPL(EC_T_VOID* pvAddress, EC_T_UINT64 qwVal)
    {
        /* alignment does not need to be considered on x86 */
        *(EC_T_UINT64*)pvAddress = qwVal;
    } EC_INLINESTOP
    #endif

    #ifndef EC_GETQWORD
    #define EC_GETQWORD(pvAddress) EC_GETQWORD_IMPL((EC_T_VOID*)(pvAddress))
    static EC_INLINESTART EC_T_UINT64 EC_GETQWORD_IMPL(EC_T_VOID* pvAddress)
    {
        /* alignment does not need to be considered on x86 */
        return *(EC_T_UINT64*)pvAddress;
    } EC_INLINESTOP
    #endif
#endif

#ifndef EC_SETBOOL
#define EC_SETBOOL(ptr, val)    EC_SETDWORD((ptr),(EC_T_DWORD)(val))
#endif

#ifndef EC_GETBOOL
#define EC_GETBOOL(ptr)         ((EC_T_BOOL)(EC_GETDWORD((ptr)))!=EC_FALSE)
#endif

#if __GNUC__ >= 9
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Warray-bounds="
#endif
static EC_INLINESTART EC_T_VOID EC_COPYBITS_INLINE(EC_T_BYTE* pbyDst, EC_T_INT nDstBitOffs, const EC_T_BYTE* pbySrc, EC_T_INT nSrcBitOffs, EC_T_INT nBitSize)
{
    EC_T_INT    nRemLen = nBitSize;            /* remaining length */
    EC_T_BYTE   byMask = 0;
    EC_T_INT    nNumBits = 0;
    EC_T_WORD   wSrcWork = 0;
    const EC_T_BYTE* pSrcHelp = EC_NULL;

    if (nBitSize == 0)
    {
        return;
    }
    /* next byte bit count in destination buffer */
    nNumBits = EC_AT_MOST((8 - (nDstBitOffs & 7)), nRemLen);

    byMask   = (EC_T_BYTE)((1 << nNumBits) - 1);
    byMask   = (EC_T_BYTE)(byMask<<(nDstBitOffs & 7));

    /* get first byte from source buffer */
    pSrcHelp = &pbySrc[nSrcBitOffs / 8];
    if ((nSrcBitOffs & 7) + nBitSize <= 8)
    {
        wSrcWork = (EC_T_WORD)pSrcHelp[0];
    }
    else
    {
        wSrcWork = (EC_T_WORD)((EC_T_WORD)pSrcHelp[0] | (((EC_T_WORD)pSrcHelp[1]) << 8));   /* xxxx xxxx xx-- ----  e. g. nSrcBitOffs=6, nDstBitOffs=2 */
    }
    wSrcWork = (EC_T_WORD)(wSrcWork >> (nSrcBitOffs & 7));                                  /* 0000 00xx xxxx xxxx */
    wSrcWork = (EC_T_WORD)(wSrcWork & ((1 << nNumBits) - 1));

    /* set first byte in destination buffer */
    pbyDst   = &pbyDst[nDstBitOffs/8];
    *pbyDst  = (EC_T_BYTE)((*pbyDst & ~byMask) | (wSrcWork<<(nDstBitOffs & 7)));
    pbyDst++;

    nSrcBitOffs = nSrcBitOffs + nNumBits;
    nRemLen -= nNumBits;

    while (nRemLen > 0)
    {
        nNumBits  = EC_AT_MOST(8, nRemLen);

        byMask    = (EC_T_BYTE)((1 << nNumBits) - 1);

        pSrcHelp = &pbySrc[nSrcBitOffs / 8];

        if ((nSrcBitOffs & 7) + nRemLen <= 8)
        {
            wSrcWork = (EC_T_WORD)pSrcHelp[0];
        }
        else
        {
            /* remaining src data overlapping the one-byte boundary */
            wSrcWork = (EC_T_WORD)(pSrcHelp[0] | (pSrcHelp[1] << 8));
        }
        wSrcWork = (EC_T_WORD)(wSrcWork >> (nSrcBitOffs & 7));
        wSrcWork = (EC_T_WORD)(wSrcWork & (EC_T_WORD)byMask);

        *pbyDst = (EC_T_BYTE)((*pbyDst & ~byMask) | (wSrcWork));
        pbyDst++;

        nSrcBitOffs = nSrcBitOffs + nNumBits;
        nRemLen -= nNumBits;
    }
} EC_INLINESTOP
#if __GNUC__ >= 9
#pragma GCC diagnostic pop
#endif

#ifndef EC_COPYBITS
/** \brief Copies a block of bits from a source buffer to a destination buffer.
\note The memory buffers must be allocated before. The buffers must be big enough to hold the block starting at the given offsets! The buffers are not checked for overrun.
\param pbyDst       [out] Destination buffer
\param nDstBitOffs  [in]  Bit offset within destination buffer
\param pbySrc       [in]  Source buffer
\param nSrcBitOffs  [in]  Bit offset within source buffer
\param nBitSize     [in]  Block size in bits
*/
#define EC_COPYBITS(pbyDst, nDstBitOffs, pbySrc, nSrcBitOffs, nBitSize) EC_COPYBITS_INLINE((pbyDst),(nDstBitOffs),(pbySrc),(nSrcBitOffs),(nBitSize))
#endif

#ifndef OsCopyBitsPdIn
#define OsCopyBitsPdIn          EC_COPYBITS
#endif

#ifndef OsCopyBitsPdOut
#define OsCopyBitsPdOut         EC_COPYBITS
#endif

#ifndef EC_SETBIT
/** \brief Set a single bit in the buffer.
 \param pbyBuf Destination buffer.
 \param nBitOffs Bit offset to set.
*/
#define EC_SETBIT(pbyBuf, nBitOffs) (pbyBuf)[(nBitOffs)/8] |= (1<<((nBitOffs)%8))
#endif
#ifndef EC_CLRBIT
/** \brief Clear a single bit in the buffer.
 \param pbyBuf Destination buffer.
 \param nBitOffs Bit offset to clear.
 */
#define EC_CLRBIT(pbyBuf, nBitOffs) (pbyBuf)[(nBitOffs)/8] &= ~(1<<((nBitOffs)%8))
#endif
#ifndef EC_COPYBIT
/** \brief Copy a boolean bit value into the buffer.
 \param pbyBuf Destination buffer.
 \param nBitOffs Bit offset to write.
 \param bVal Boolean bit value.
 */
#define EC_COPYBIT(pbyBuf, nBitOffs, bVal) { if (bVal) { EC_SETBIT((pbyBuf), (nBitOffs)); } else { EC_CLRBIT((pbyBuf), (nBitOffs)); }}
#endif
#ifndef EC_TESTBIT
/** \brief Test whether a bit in the buffer is set.
 \param pbyBuf Source buffer.
 \param nBitOffs Bit offset to test.
 \return EC_TRUE if bit is set, otherwise EC_FALSE.
 */
#define EC_TESTBIT(pbyBuf, nBitOffs) (((pbyBuf)[(nBitOffs)/8] & (1<<((nBitOffs)%8))) ? EC_TRUE : EC_FALSE)
#endif
#ifndef EC_SETBITS
/** \brief Writes a given number of bits from source data starting at first bit to destination buffer at given bit offset
\note This function should only be used to set bit-aligned data. For byte-aligned data the corresponding functions should be used.
\param pbyDstBuf   [out] Destination buffer where data is copied to
\param pbySrcData  [in]  Source buffer to be copied, starting with first bit
\param nDstBitOffs [in]  Destination bit offset where data is copied to
\param nBitSize    [in]  Bit count to be copied
*/
#define EC_SETBITS(pbyDstBuf, pbySrcData, nDstBitOffs, nBitSize) \
        EC_COPYBITS((pbyDstBuf),(nDstBitOffs),(pbySrcData),0,(nBitSize))
#endif
#ifndef EC_GETBITS
/** \brief Reads a given number of bits from source buffer starting at given bit offset to destination buffer
\note This function should only be used to get bit-aligned data. For byte-aligned data the corresponding functions should be used.
\param pbySrcBuf   [in]  Source buffer to be copied
\param pbyDstData  [out] Destination buffer where data is copied to
\param nSrcBitOffs [in]  Source bit offset where data is copied from
\param nBitSize    [in]  Bit count to be copied
*/
#define EC_GETBITS(pbySrcBuf, pbyDstData, nSrcBitOffs, nBitSize) \
        EC_COPYBITS((pbyDstData),0,(pbySrcBuf),(nSrcBitOffs),(nBitSize))
#endif

static EC_INLINESTART EC_T_INT EC_COMPAREBITS_INLINE(const EC_T_BYTE* pbyBuf1, EC_T_INT nBitOffs1, const EC_T_BYTE* pbyBuf2, EC_T_INT nBitOffs2, EC_T_INT nBitSize)
{
    EC_T_INT         nRemLen = nBitSize;            /* remaining length */
    EC_T_BYTE        byMask;
    EC_T_INT         nNumBits = 0;
    const EC_T_BYTE* pSrcHelp = EC_NULL;
    EC_T_WORD        wSrcWork;
    EC_T_BYTE        byCheckVal = 0;

    if (nBitSize == 0)
    {
        return 0;
    }
    /* next byte bit count in buffer 1 */
    nNumBits = EC_AT_MOST((8 - (nBitOffs1 & 7)), nRemLen);

    byMask   = (EC_T_BYTE)((1 << nNumBits) - 1);
    byMask   = (EC_T_BYTE)(byMask << (nBitOffs1 & 7));

    /* get first byte from buffer 2 */
    pSrcHelp = &pbyBuf2[nBitOffs2 / 8];
    if ((nBitOffs2 & 7) + nBitSize <= 8)
    {
        wSrcWork = (EC_T_WORD)pSrcHelp[0];
    }
    else
    {
        wSrcWork = (EC_T_WORD)((EC_T_WORD)pSrcHelp[0] | (((EC_T_WORD)pSrcHelp[1]) << 8));   /* xxxx xxxx xx-- ----  e. g. nBitOffs2=6, nBitOffs1=2 */
    }
    wSrcWork = (EC_T_WORD)(wSrcWork >> (nBitOffs2 & 7));                                  /* 0000 00xx xxxx xxxx */
    wSrcWork = (EC_T_WORD)(wSrcWork & ((1 << nNumBits) - 1));

    /* set first byte in buffer 1 buffer */
    pbyBuf1 = &pbyBuf1[nBitOffs1 / 8];

    byCheckVal = (EC_T_BYTE)((*pbyBuf1 & ~byMask) | (wSrcWork << (nBitOffs1 & 7)));
    if (*pbyBuf1 != byCheckVal)
    {
        return *pbyBuf1 - byCheckVal;
    }
    pbyBuf1++;

    nBitOffs2 = nBitOffs2 + nNumBits;
    nRemLen -= nNumBits;

    while (nRemLen > 0)
    {
        nNumBits = EC_AT_MOST(8, nRemLen);

        byMask   = (EC_T_BYTE)((1 << nNumBits) - 1);

        pSrcHelp = &pbyBuf2[nBitOffs2 / 8];

        if ((nBitOffs2 & 7) + nRemLen <= 8)
        {
            wSrcWork = (EC_T_WORD)pSrcHelp[0];
        }
        else
        {
            /* remaining src data overlapping the one-byte boundary */
            wSrcWork = (EC_T_WORD)(pSrcHelp[0] | (pSrcHelp[1] << 8));
        }
        wSrcWork = (EC_T_WORD)(wSrcWork >> (nBitOffs2 & 7));
        wSrcWork = (EC_T_WORD)(wSrcWork & (EC_T_WORD)byMask);

        byCheckVal = (EC_T_BYTE)((*pbyBuf1 & ~byMask) | (wSrcWork));
        if (*pbyBuf1 != byCheckVal)
        {
            return *pbyBuf1 - byCheckVal;
        }
        pbyBuf1++;

        nBitOffs2 = nBitOffs2 + nNumBits;
        nRemLen -= nNumBits;
    }
    return 0;
} EC_INLINESTOP

#ifndef EC_COMPAREBITS
/** \brief Compares a block of bits of two buffers.
\note The buffers are compared bitwise with the same semantic as memcmp(). The buffers are not checked for overrun.
\param pbyBuf1       [in]  Buffer 1
\param nBitOffs1     [in]  Bit offset within buffer 1
\param pbyBuf2       [in]  Buffer 2
\param nBitOffs2     [in]  Bit offset within buffer 2
\param nBitSize      [in]  Bit size to compare
\return Integral value indicating the relationship between the content of the memory blocks:
      - 0:  The blocks of bits within the two buffers are equal
      - <0: Binary value from block of bits in buffer 1 has a lower value than binary value from block of bits in buffer 2
      - >0: Binary value from block of bits in buffer 1 has a greater value than binary value from block of bits in buffer 2
*/
#define EC_COMPAREBITS(pbyBuf1, nBitOffs1, pbyBuf2, nBitOffs2, nBitSize) EC_COMPAREBITS_INLINE((pbyBuf1), (nBitOffs1), (pbyBuf2), (nBitOffs2), (nBitSize))
#endif

static EC_INLINESTART EC_T_BYTE   EcBoolToByte(EC_T_BOOL bValue) { return (EC_T_BYTE)(bValue ? 1 : 0);} EC_INLINESTOP
static EC_INLINESTART EC_T_BOOL   EcByteToBool(EC_T_BYTE bValue) { return bValue == 1;} EC_INLINESTOP

static EC_INLINESTART EC_T_WORD   EcPtrToWord (EC_T_VOID* pvVal) { return (EC_T_WORD)  ((EC_T_BYTE*)pvVal - (EC_T_BYTE*)EC_NULL);} EC_INLINESTOP
static EC_INLINESTART EC_T_DWORD  EcPtrToDword(EC_T_VOID* pvVal) { return (EC_T_DWORD) ((EC_T_BYTE*)pvVal - (EC_T_BYTE*)EC_NULL);} EC_INLINESTOP
static EC_INLINESTART EC_T_UINT64 EcPtrToQword(EC_T_VOID* pvVal) { return (EC_T_UINT64)((EC_T_BYTE*)pvVal - (EC_T_BYTE*)EC_NULL);} EC_INLINESTOP

#if (defined EC_BIG_ENDIAN)
#define EC_NO_BITFIELDS /* big endian: do not use bit fields! */
#define EC_GET_FRM_BOOL(ptr)        EC_DWORDSWAP(EC_GETDWORD((ptr)))
#define EC_GET_FRM_WORD(ptr)        EC_WORDSWAP(EC_GETWORD((ptr)))
#define EC_GET_FRM_DWORD(ptr)       EC_DWORDSWAP(EC_GETDWORD((ptr)))
#define EC_GET_FRM_QWORD(ptr)       EC_QWORDSWAP(EC_GETQWORD((ptr)))

#define EC_GET_FRM_INT(ptr)         EC_DWORDSWAP(EC_GETDWORD((ptr)))

#define EC_SET_FRM_BOOL(ptr, dw)    EC_SETDWORD((ptr),EC_DWORDSWAP((dw)))
#define EC_SET_FRM_WORD(ptr, w)     EC_SETWORD((ptr),EC_WORDSWAP((EC_T_WORD)(w)))
#define EC_SET_FRM_DWORD(ptr, dw)   EC_SETDWORD((ptr),EC_DWORDSWAP((EC_T_DWORD)(dw)))
#define EC_SET_FRM_QWORD(ptr, qw)   EC_SETQWORD((ptr),EC_QWORDSWAP((EC_T_UINT64)(qw)))

#define EC_SET_FRM_INT(ptr, n)      EC_SETDWORD((ptr),EC_DWORDSWAP((EC_T_DWORD)(n)))

#define EC_GET_FRM_WORD_BITFIELD(Bitpos,Bitsize,wVal)           EC_GET_WORD_IN_BITFIELD((Bitpos),(Bitsize),EC_WORDSWAP((wVal)))
#define EC_SET_FRM_WORD_BITFIELD(wVal,wNewVal,Bitpos,Bitsize)   {(wVal) = ((wVal) & ~EC_WORDSWAP(EC_BITFIELD_MASK((Bitpos),(Bitsize)))) | (EC_WORDSWAP((wNewVal)<<(Bitpos)));}
#else /* !EC_BIG_ENDIAN */
#define EC_GET_FRM_BOOL(ptr)        EC_GETBOOL((ptr))

/** \brief Reads a value of type EC_T_WORD (16 bit) at given pointer. The value is swapped on big endian systems.
\param ptr [in] Source buffer
\return EC_T_WORD value (16 bit) from buffer
*/
#define EC_GET_FRM_WORD(ptr)        EC_GETWORD((ptr))

/** \brief Reads a value of type EC_T_DWORD (32 bit) at given pointer. The value is swapped on big endian systems.
\param ptr [in] Source buffer
\return EC_T_DWORD value (32 bit) from buffer
*/
#define EC_GET_FRM_DWORD(ptr)       EC_GETDWORD((ptr))

/** \brief Reads a value of type EC_T_QWORD (64 bit) at given pointer. The value is swapped on big endian systems.
\param ptr [in] Source buffer
\return EC_T_QWORD value (64 bit) from buffer
*/
#define EC_GET_FRM_QWORD(ptr)       EC_GETQWORD((ptr))

#define EC_SET_FRM_BOOL(ptr,b)      EC_SETBOOL((ptr),(b))

/** \brief Writes a value of type EC_T_WORD (16 bit) at given pointer. The value is swapped on big endian systems.
\param ptr [in] Destination buffer
\param w   [in] 16 bit value
*/
#define EC_SET_FRM_WORD(ptr,w)      EC_SETWORD((ptr),(w))

/** \brief Writes a value of type EC_T_DWORD (32 bit) at given pointer. The value is swapped on big endian systems.
\param ptr [in] Destination buffer
\param dw  [in] 32 bit value
*/
#define EC_SET_FRM_DWORD(ptr,dw)    EC_SETDWORD((ptr),(dw))

/** \brief Writes a value of type EC_T_QWORD (64 bit) at given pointer. The value is swapped on big endian systems.
\param ptr [in] Destination buffer
\param qw  [in] 64 bit value
*/
#define EC_SET_FRM_QWORD(ptr,qw)    EC_SETQWORD((ptr),(qw))

/** \brief Reads a value of type EC_T_INT (signed 32 bit) at given pointer. The value is swapped on big endian systems.
\param ptr [in] Source buffer
\return EC_T_INT value (32 bit) from buffer
*/
#define EC_GET_FRM_INT(ptr)       EC_GETINT((ptr))

#define EC_SET_FRM_INT(ptr, n)    EC_SETINT((ptr),(n))

static EC_INLINESTART EC_T_LREAL EC_GET_FRM_LREAL_INLINE(EC_T_VOID* ptr)
{
    EC_T_UINT64 qwVal = EC_GET_FRM_QWORD(ptr);
    EC_T_BYTE* pValPtr = (EC_T_BYTE*)&qwVal;
    return *(EC_T_LREAL*)pValPtr;
} EC_INLINESTOP
#define EC_GET_FRM_LREAL(ptr)       EC_GET_FRM_LREAL_INLINE((ptr))

static EC_INLINESTART EC_T_REAL EC_GET_FRM_REAL_INLINE(EC_T_VOID* ptr)
{
    EC_T_UINT dwVal = EC_GET_FRM_DWORD(ptr);
    EC_T_BYTE* pValPtr = (EC_T_BYTE*)&dwVal;
    return *(EC_T_REAL*)pValPtr;
} EC_INLINESTOP
#define EC_GET_FRM_REAL(ptr)       EC_GET_FRM_REAL_INLINE((ptr))

#define EC_GET_FRM_WORD_BITFIELD(Bitpos,Bitsize,wVal)           EC_GET_WORD_IN_BITFIELD((Bitpos),(Bitsize),(wVal))
#define EC_SET_FRM_WORD_BITFIELD(wVal,wNewVal,Bitpos,Bitsize)   {(wVal) = (EC_T_WORD)(((wVal) & ~EC_BITFIELD_MASK((Bitpos),(Bitsize))) | ((wNewVal)<<(Bitpos)));}
#endif /* !EC_BIG_ENDIAN */

/* *** EC_NTOHS, EC_HTONS *** */
/* convert values between host and network byte order
 * On the i386 the host byte order is Least Significant Byte first,
 * whereas the network byte order, as used on the Internet, is Most Significant Byte first.
 */
#if (defined EC_BIG_ENDIAN)
#define EC_HTONS(w)                 (w)
#define EC_NTOHS(w)                 (w)
#define EC_HTONL(dw)                (dw)
#define EC_NTOHL(dw)                (dw)
#define EC_HTONLL(qw)               (qw)
#define EC_NTOHLL(qw)               (qw)
#else
#define EC_HTONS(w)                 EC_WORDSWAP((w))
#define EC_NTOHS(w)                 EC_WORDSWAP((w))
#define EC_HTONL(dw)                EC_DWORDSWAP((dw))
#define EC_NTOHL(dw)                EC_DWORDSWAP((dw))
#define EC_HTONLL(qw)               EC_QWORDSWAP((qw))
#define EC_NTOHLL(qw)               EC_QWORDSWAP((qw))
#endif

static EC_INLINESTART EC_T_VOID* EcPtrAlignDown(EC_T_VOID* pvVal, EC_T_DWORD dwAlignment)
{
    return (EC_T_VOID*)((EC_T_BYTE*)pvVal - (((EC_T_BYTE*)pvVal - (EC_T_BYTE*)0) % dwAlignment));
} EC_INLINESTOP

static EC_INLINESTART EC_T_VOID* EcPtrAlignUp(EC_T_VOID* pvVal, EC_T_DWORD dwAlignment)
{
    return EcPtrAlignDown((EC_T_BYTE*)pvVal + dwAlignment - 1, dwAlignment);
} EC_INLINESTOP

static EC_INLINESTART EC_T_DWORD EcSizeAlignDown(EC_T_DWORD dwVal, EC_T_DWORD dwAlignment)
{
    return (dwVal & (~(dwAlignment - 1)));
} EC_INLINESTOP

static EC_INLINESTART EC_T_DWORD EcSizeAlignUp(EC_T_DWORD dwVal, EC_T_DWORD dwAlignment)
{
    return ((dwVal + (dwAlignment - 1)) & (~(dwAlignment - 1)));
} EC_INLINESTOP

/*-SANITY CHECKS-------------------------------------------------------------*/
#if (defined __GNUC__) || (defined __ICCARM__) || (defined __BORLANDC__)
#define EC_ASSERT_CONCAT_(a, b) a##b
#define EC_ASSERT_CONCAT(a, b) EC_ASSERT_CONCAT_(a, b)
#define EC_CHECK_STRUCTURE_SIZE_ASSERT(expr) enum { EC_ASSERT_CONCAT(assert_line_, __LINE__) = 1/(!!(expr)) }
#define EC_CHECK_STRUCTURE_SIZE(type, size) \
    EC_CHECK_STRUCTURE_SIZE_ASSERT(sizeof(type)==size);
#define EC_CHECK_VALUE_EQUAL(name, set, act) \
    EC_CHECK_STRUCTURE_SIZE_ASSERT(set==act);
#define EC_STRUCTURE_VAR_OFFSET(type,member) offsetof(type,member)
#else
#define EC_CHECK_VALUE_EQUAL(name, type, size) \
  typedef char EC_T_CHECK_UPPER_##name[##type-##size+1];\
  typedef char EC_T_CHECK_LOWER_##name[##size-##type+1];
#define EC_CHECK_STRUCTURE_SIZE(type, size) EC_CHECK_VALUE_EQUAL(##type,sizeof(##type),##size)
#define EC_STRUCTURE_VAR_OFFSET(type,member) ((EC_T_DWORD)(((char*)(&((type*)0)->member))-((char*)0)))
#endif

#endif /* INC_OS_COMMON */
