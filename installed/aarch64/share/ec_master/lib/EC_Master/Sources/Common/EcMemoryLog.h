/** ---------------------------------------------------------------------------
 * \file
 * \brief       Memory Logger implementation
 * \copyright   acontis technologies GmbH, Weingarten, Germany
 *---------------------------------------------------------------------------*/

#ifndef INC_ECMEMORYLOG
#define INC_ECMEMORYLOG

/*-INCLUDES------------------------------------------------------------------*/
#ifndef INC_ECOS
#include "EcOs.h"
#endif
#ifndef INC_ECCOMMON
#include "EcCommon.h"
#endif

/*-TYPEDEFS------------------------------------------------------------------*/
#if (defined INCLUDE_MEMORY_LOGGER_HISTORY)
typedef struct _EC_T_MEMORY_LOGGER_HISTORY_ENTRY 
{
    const EC_T_VOID* pvAddress;
    size_t           nSize;
    const EC_T_CHAR* szLoc;
} EC_T_MEMORY_LOGGER_HISTORY_ENTRY;
#endif

typedef struct _EC_T_MEMORY_LOGGER
{
    size_t       nCurrUsage;         /* current dynamic memory usage */
    size_t       nMaxUsage;          /* maximum dynamic memory usage */

#if (defined INCLUDE_MEMORY_LOGGER_HISTORY)
    EC_T_DWORD   dwHistoryEntryCnt;
    EC_T_MEMORY_LOGGER_HISTORY_ENTRY aHistoryEntries[65536];
#endif
} EC_T_MEMORY_LOGGER;

/* ------- mem logging ------------ */
#if (defined INCLUDE_MEMORY_LOGGER_HISTORY)
EC_INLINESTART EC_T_VOID EcMemLogHistoryAddEntry(EC_T_MEMORY_LOGGER* pMemoryLogger, const EC_T_VOID* const pvAddress, size_t nSize, const EC_T_CHAR* const szLoc)
{
    OsDbgAssert((pMemoryLogger != EC_NULL) && (pvAddress != EC_NULL) && (nSize != 0) && (szLoc != EC_NULL));

    EC_T_DWORD dwIdx = 0;
    for (dwIdx = 0; dwIdx < pMemoryLogger->dwHistoryEntryCnt; dwIdx++)
    {
        if (pvAddress == pMemoryLogger->aHistoryEntries[dwIdx].pvAddress)
        {
            fprintf(stderr, "MEMLOGGER ADD ADDRESS ALREADY EXISTS (logger '%p', context '%s'): address (%p) entry already exists! Already registred size %d. Size that shall be registered %d to same address.\n", pMemoryLogger, szLoc, pvAddress, (int)pMemoryLogger->aHistoryEntries[dwIdx].nSize, (int)nSize);
            OsDbgAssert(EC_FALSE);
            break;
        }
    }
    if (dwIdx == pMemoryLogger->dwHistoryEntryCnt)
    {
        OsDbgAssert(pMemoryLogger->dwHistoryEntryCnt < EC_NUMOFELEMENTS(pMemoryLogger->aHistoryEntries));
        if (pMemoryLogger->dwHistoryEntryCnt < EC_NUMOFELEMENTS(pMemoryLogger->aHistoryEntries))
        {
            pMemoryLogger->aHistoryEntries[dwIdx].pvAddress = pvAddress;
            pMemoryLogger->aHistoryEntries[dwIdx].nSize = nSize;
            pMemoryLogger->aHistoryEntries[dwIdx].szLoc = szLoc;
            pMemoryLogger->dwHistoryEntryCnt++;
        }
    }
} EC_INLINESTOP

EC_INLINESTART EC_T_VOID EcMemLogHistoryRemoveEntry(EC_T_MEMORY_LOGGER* pMemoryLogger, const EC_T_VOID* const pvAddress, size_t nSize, const EC_T_CHAR* const szLoc)
{
    OsDbgAssert((pMemoryLogger != EC_NULL) && (pvAddress != EC_NULL) && (nSize != 0) && (szLoc != EC_NULL));

    EC_T_DWORD dwIdx = 0;
    for (dwIdx = 0; dwIdx < pMemoryLogger->dwHistoryEntryCnt; dwIdx++)
    {
        if (pvAddress == pMemoryLogger->aHistoryEntries[dwIdx].pvAddress)
        {
            if (nSize != pMemoryLogger->aHistoryEntries[dwIdx].nSize)
            {
                fprintf(stderr, "MEMLOGGER SUB SIZE MISMATCH (logger '%p', context '%s'): entry for registered address %p was added with a size of %d, but is deregistered with a size of %d.\n", pMemoryLogger, szLoc, pvAddress, (int)pMemoryLogger->aHistoryEntries[dwIdx].nSize, (int)nSize);
                OsDbgAssert(EC_FALSE);
            }
            OsMemset(&pMemoryLogger->aHistoryEntries[dwIdx], 0, sizeof(pMemoryLogger->aHistoryEntries[dwIdx]));
            break;
        }
    }
    if ((dwIdx == pMemoryLogger->dwHistoryEntryCnt)) 
    {
        fprintf(stderr, "MEMLOGGER SUB ADDRESS NOT FOUND (logger '%p', context '%s'): address %p could not be found! Size to be deregistered is %d!\n", pMemoryLogger, szLoc, pvAddress, (int)nSize);
        OsDbgAssert(EC_FALSE);
    }
} EC_INLINESTOP
#endif

static EC_INLINESTART EC_T_VOID EC_TRACE_ADDMEM_LOG(EC_T_MEMORY_LOGGER* pMemoryLogger, size_t nSize)
{
#ifdef INCLUDE_MEMORY_LOGGER_HISTORY
    OsDbgAssert((EC_NULL != pMemoryLogger) && (nSize != 0));
#else
    if (EC_NULL != pMemoryLogger)
#endif
    {
        pMemoryLogger->nCurrUsage += nSize;
        if (pMemoryLogger->nCurrUsage > pMemoryLogger->nMaxUsage)
        {
            pMemoryLogger->nMaxUsage = pMemoryLogger->nCurrUsage;
        }
    }
} EC_INLINESTOP

static EC_INLINESTART EC_T_VOID EC_TRACE_ADDMEM_LOG(EC_T_DWORD, EC_T_MEMORY_LOGGER* pMemoryLogger, const EC_T_CHAR* const szLoc, const EC_T_VOID* const pvAddress, size_t nSize)
{
#ifdef INCLUDE_MEMORY_LOGGER_HISTORY
    OsDbgAssert((pMemoryLogger != EC_NULL) && (pvAddress != EC_NULL) && (nSize != 0) && (szLoc != EC_NULL));
#else
    if (EC_NULL != pMemoryLogger)
#endif
    {
        EC_TRACE_ADDMEM_LOG(pMemoryLogger, nSize);
#if (defined INCLUDE_MEMORY_LOGGER_HISTORY)
        EcMemLogHistoryAddEntry(pMemoryLogger, (EC_T_VOID*)pvAddress, nSize, szLoc);
#else
        EC_UNREFPARM(szLoc);
        EC_UNREFPARM(pvAddress);
#endif
    }
} EC_INLINESTOP

static EC_INLINESTART EC_T_VOID EC_TRACE_SUBMEM_LOG(EC_T_MEMORY_LOGGER* pMemoryLogger, size_t nSize)
{
#ifdef INCLUDE_MEMORY_LOGGER_HISTORY
    OsDbgAssert((EC_NULL != pMemoryLogger) && (nSize != 0));
#else
    if (EC_NULL != pMemoryLogger)
#endif
    {
        if (nSize < pMemoryLogger->nCurrUsage)
        {
            pMemoryLogger->nCurrUsage -= nSize;
        }
        else
        {
            pMemoryLogger->nCurrUsage = 0;
        }
    }
} EC_INLINESTOP

static EC_INLINESTART EC_T_VOID EC_TRACE_SUBMEM_LOG(EC_T_DWORD, EC_T_MEMORY_LOGGER* pMemoryLogger, const EC_T_CHAR* const szLoc, const EC_T_VOID* const pvAddress, size_t nSize)
{
    OsDbgAssert(EC_NULL != pvAddress);
#ifdef INCLUDE_MEMORY_LOGGER_HISTORY
    OsDbgAssert((pMemoryLogger != EC_NULL) && (pvAddress != EC_NULL) && (nSize != 0) && (szLoc != EC_NULL));
#else
    if (EC_NULL != pMemoryLogger)
#endif
    {
#if (defined INCLUDE_MEMORY_LOGGER_HISTORY)
        EcMemLogHistoryRemoveEntry(pMemoryLogger, (EC_T_VOID*)pvAddress, nSize, szLoc);
#else
        EC_UNREFPARM(szLoc);
        EC_UNREFPARM(pvAddress);
#endif
        EC_TRACE_SUBMEM_LOG(pMemoryLogger, nSize);
    }
} EC_INLINESTOP

#ifndef EC_TRACE_ADDMEM
#define EC_TRACE_ADDMEM(Mask, szLoc, pvAddress, dwSize) EC_TRACE_ADDMEM_LOG(Mask, GetMemLog(), szLoc, pvAddress, dwSize)
#endif

#ifndef EC_TRACE_SUBMEM
#define EC_TRACE_SUBMEM(Mask, szLoc, pvAddress, dwSize) EC_TRACE_SUBMEM_LOG(Mask, GetMemLog(), szLoc, pvAddress, dwSize);
#endif

#endif /* INC_ECMEMORYLOG */

/*-END OF SOURCE FILE--------------------------------------------------------*/
