/*-----------------------------------------------------------------------------
* EcLog.h
* Copyright                acontis technologies GmbH, Ravensburg, Germany
* Response                 Holger Oelhaf
* Description              Logging interface
*---------------------------------------------------------------------------*/

#ifndef INC_ECLOG
#define INC_ECLOG

#if (!defined INC_ECOS) && (!defined INC_LINK_OS_LAYER)
#error EcOs.h / LinkOsLayer.h include missing!
#endif

/** \defgroup EC_LOG_LEVELS
@{
*/
#define EC_LOG_LEVEL_SILENT       0
#define EC_LOG_LEVEL_ANY          1
#define EC_LOG_LEVEL_CRITICAL     2
#define EC_LOG_LEVEL_ERROR        3
#define EC_LOG_LEVEL_WARNING      4
#define EC_LOG_LEVEL_INFO         5
#define EC_LOG_LEVEL_INFO_API     6
#define EC_LOG_LEVEL_VERBOSE      7
#define EC_LOG_LEVEL_VERBOSE_ACYC EC_LOG_LEVEL_VERBOSE
#define EC_LOG_LEVEL_VERBOSE_CYC  8
#define EC_LOG_LEVEL_UNDEFINED    ((EC_T_DWORD)-1)
/**@}*/

#include EC_PACKED_API_INCLUDESTART
typedef struct _EC_T_LOG_PARMS
{
    EC_T_DWORD                  dwLogLevel;   /**< [in] Log level. See EC_LOG_LEVEL_... */
    EC_PF_LOGMSGHK              pfLogMsg;     /**< [in] Log callback function called on every message */
    struct _EC_T_LOG_CONTEXT*   pLogContext;  /**< [in] Log context to be passed to log callback */
} EC_PACKED_API EC_T_LOG_PARMS;
#include EC_PACKED_INCLUDESTOP

/* default definition for pEcLogParms */
#if (defined G_pEcLogParms)
#ifndef pEcLogParms
#define pEcLogParms (G_pEcLogParms)
#endif
#ifndef G_pEcLogContext
#define G_pEcLogContext (G_pEcLogParms->pLogContext)
#endif
#endif /* G_pEcLogParms */

/* default definitions for dwLogLevel, pfLogMsg, pLogContext */
#if (defined pEcLogParms)
#ifndef dwEcLogLevel
#define dwEcLogLevel    (pEcLogParms->dwLogLevel)
#endif
#ifndef pLogMsgCallback
#define pLogMsgCallback (pEcLogParms->pfLogMsg)
#endif
#ifndef pEcLogContext
#define pEcLogContext   (pEcLogParms->pLogContext)
#endif
#endif /* pEcLogParms */

#ifndef EcLogMsg
#if (defined INCLUDE_LOG_MESSAGES)
#define EcLogMsg(dwLogSeverity, pLogMsgCallbackParms) ((dwLogSeverity <= dwEcLogLevel)?(pLogMsgCallback pLogMsgCallbackParms):0)
#else
#define EcLogMsg(dwLogSeverity, pLogMsgCallbackParms)
#endif
#endif

/* global GetLogParms() for static functions (context not available) */
#if (!defined GetLogParms) && (defined G_pEcLogParms)
static EC_INLINESTART struct _EC_T_LOG_PARMS* GetLogParms()
{
    return G_pEcLogParms;
} EC_INLINESTOP
#endif /* global GetLogParms() */

#endif
