/** ---------------------------------------------------------------------------
 * \file
 * \author      Timo Nussbaumer
 * \copyright   acontis technologies GmbH, Weingarten, Germany
 *---------------------------------------------------------------------------*/

#ifndef INC_ECMASTERRASSERVER
#define INC_ECMASTERRASSERVER 1

/*-INCLUDES------------------------------------------------------------------*/
#ifndef INC_ECOS
#include "EcOs.h"
#endif
#ifndef INC_ECRASTYPE
#include "EcRasType.h"
#endif

/*-FUNCTION DECLARATION------------------------------------------------------*/
#ifdef __cplusplus
extern "C"
{
#endif

/* EC-Master RAS Server API */
EC_API EC_T_DWORD EC_API_FNCALL emRasSrvGetVersion(EC_T_VOID);
EC_API EC_T_DWORD EC_API_FNCALL emRasSrvStart(const EC_T_RAS_SERVER_PARMS* pParms, EC_T_PVOID* ppHandle);
EC_API EC_T_DWORD EC_API_FNCALL emRasSrvStop(EC_T_PVOID pvHandle, EC_T_DWORD dwTimeout);
EC_API const EC_T_CHAR* EC_API_FNCALL emRasErrorText(EC_T_DWORD dwError);
EC_API const EC_T_CHAR* EC_API_FNCALL emRasEventText(EC_T_DWORD dwEvent);

#if (defined INCLUDE_RAS_TRACESUPPORT)
EC_API EC_T_DWORD EC_API_FNCALL emRasSrvTraceEnable(EC_T_BOOL bEnable);
#endif

EC_API EC_T_DWORD EC_API_FNCALL emRasSrvSetAccessLevel(EC_T_PVOID pvHandle, EC_T_DWORD dwAccessLevel);
EC_API EC_T_DWORD EC_API_FNCALL emRasSrvGetAccessLevel(EC_T_PVOID pvHandle, EC_T_DWORD* pdwAccessLevel);
EC_API EC_T_DWORD EC_API_FNCALL emRasSrvConfigAccessLevel(EC_T_PVOID pvHandle, const EC_T_RAS_SPOCCFG* pCfgData, EC_T_DWORD dwCfgDataCnt);
EC_API EC_T_DWORD EC_API_FNCALL emRasSrvSetAccessControl(EC_T_PVOID pvHandle, EC_T_BOOL bActive);
EC_API EC_T_DWORD EC_API_FNCALL emRasSrvSetCallAccessLevel(
    EC_T_PVOID      pvHandle        /**< [in]   Handle to previously started Server */
    , EC_T_DWORD    dwOrdinal       /**< [in]   Function call ID */
    , EC_T_DWORD    dwIndex         /**< [in]   Function call index */
    , EC_T_DWORD    dwSubIndex      /**< [in]   Function call subindex */
    , EC_T_DWORD    dwAccessLevel   /**< [in]   New Access level */
    );

EC_API EC_T_DWORD EC_API_FNCALL emRasGetMemoryUsage(EC_T_PVOID pvHandle, EC_T_DWORD* pdwCurrentUsage, EC_T_DWORD* pdwMaxUsage);
EC_API EC_T_DWORD EC_API_FNCALL emRasSrvSetLogParms(EC_T_PVOID pvHandle, const EC_T_LOG_PARMS* pLogParms);

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* INC_ECMASTERRASSERVER */

/*-END OF SOURCE FILE--------------------------------------------------------*/
