/*-----------------------------------------------------------------------------
 * EcRasError.h
 * Copyright                acontis technologies GmbH, Ravensburg, Germany
 * Response                 Paul Bussmann
 * Description              RAS error structures
 *---------------------------------------------------------------------------*/

#ifndef INC_ECMASTERRASERROR
#define INC_ECMASTERRASERROR 1

/*-INCLUDES------------------------------------------------------------------*/
#ifndef INC_ECOS
#include "EcOs.h"
#endif
#ifndef INC_ECERROR
#include "EcError.h"
#endif

/* legacy */
#ifndef EC_API
#define EC_API ATECAT_API
#endif
#ifndef EC_API_FNCALL
#define EC_API_FNCALL
#endif

/*-TYPEDEFS------------------------------------------------------------------*/
#include EC_PACKED_INCLUDESTART(1)
typedef struct _ECMASTERRAS_T_CONNOTIFYDESC
{
    EC_T_DWORD      dwCause;    /**< [in] Cause of state connection state change */
    EC_T_DWORD      dwCookie;   /**< [in] Unique identification cookie of connection instance. */
} EC_PACKED(1) ECMASTERRAS_T_CONNOTIFYDESC, *ECMASTERRAS_PT_CONNOTIFYDESC;

typedef struct _ECMASTERRAS_T_REGNOTIFYDESC
{
    EC_T_DWORD      dwCookie;       /**< [in] Unique identification cookie of connection instance  */
    EC_T_DWORD      dwResult;       /**< [in] Result of registration request  */
    EC_T_DWORD      dwInstanceId;   /**< [in] Master Instance client registered to  */
    EC_T_DWORD      dwClientId;     /**< [in] Client ID of registered client */
} EC_PACKED(1) ECMASTERRAS_T_REGNOTIFYDESC, *ECMASTERRAS_PT_REGNOTIFYDESC;

typedef struct _ECMASTERRAS_T_MARSHALERRORDESC
{
    EC_T_DWORD      dwCookie;       /**< [in] Unique identification cookie of connection instance */
    EC_T_DWORD      dwCause;        /**< [in] Cause of the command marshalling error */
    EC_T_DWORD      dwLenStatCmd;   /**< [in] Length faulty command */
    EC_T_DWORD      dwCommandCode;  /**< [in] Command code of faulty command  */
} EC_PACKED(1) ECMASTERRAS_T_MARSHALERRORDESC, *ECMASTERRAS_PT_MARSHALERRORDESC;

typedef struct _ECMASTERRAS_T_NONOTIFYMEMORYDESC
{
    EC_T_DWORD      dwCookie;       /**< [in]   Cookie of faulting connection */
    EC_T_DWORD      dwCode;         /**< [in]   Fault causing notification code */
} EC_PACKED(1) ECMASTERRAS_T_NONOTIFYMEMORYDESC, *ECMASTERRAS_PT_NONOTIFYMEMORYDESC;
#include EC_PACKED_INCLUDESTOP

/*-FUNCTION DECLARATION------------------------------------------------------*/
EC_API const EC_T_CHAR* EC_API_FNCALL emRasErrorText(EC_T_DWORD dwError);
EC_API const EC_T_CHAR* EC_API_FNCALL emRasEventText(EC_T_DWORD dwEvent);

EC_API const EC_T_CHAR* EC_API_FNCALL esRasErrorText(EC_T_DWORD dwError);
EC_API const EC_T_CHAR* EC_API_FNCALL esRasEventText(EC_T_DWORD dwEvent);

#endif /* ECMASTERRASERROR_H */

/*-END OF SOURCE FILE--------------------------------------------------------*/
