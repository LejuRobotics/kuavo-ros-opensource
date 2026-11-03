/** ---------------------------------------------------------------------------
 * \file
 * \brief       RAS error structures
 * \author      Paul Bussmann
 * \copyright   acontis technologies GmbH, Weingarten, Germany
 *---------------------------------------------------------------------------*/

#ifndef INC_ECRASERROR
#define INC_ECRASERROR 1

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

/*-DEFINES-------------------------------------------------------------------*/
#define EC_RAS_EXCEPTION_THREAD_NAME_SIZE    128

/*-TYPEDEFS------------------------------------------------------------------*/
#include EC_PACKED_INCLUDESTART(1)
typedef struct _EC_T_RAS_CONNOTIFYDESC
{
    EC_T_DWORD      dwCause;    /**< [in] Cause of state connection state change */
    EC_T_DWORD      dwCookie;   /**< [in] Unique identification cookie of connection instance */
} EC_PACKED(1) EC_T_RAS_CONNOTIFYDESC, *EC_PT_RAS_CONNOTIFYDESC;

typedef struct _EC_T_RAS_REGNOTIFYDESC
{
    EC_T_DWORD      dwCookie;       /**< [in] Unique identification cookie of connection instance  */
    EC_T_DWORD      dwResult;       /**< [in] Result of registration request  */
    EC_T_DWORD      dwInstanceId;   /**< [in] Master Instance client registered to  */
    EC_T_DWORD      dwClientId;     /**< [in] Client ID of registered client */
} EC_PACKED(1) EC_T_RAS_REGNOTIFYDESC, *EC_PT_RAS_REGNOTIFYDESC;

typedef struct _EC_T_RAS_MARSHALERRORDESC
{
    EC_T_DWORD      dwCookie;       /**< [in] Unique identification cookie of connection instance */
    EC_T_DWORD      dwCause;        /**< [in] Cause of the command marshalling error */
    EC_T_DWORD      dwLenStatCmd;   /**< [in] Length of the faulty command */
    EC_T_DWORD      dwCommandCode;  /**< [in] Command code of the faulty command */
} EC_PACKED(1) EC_T_RAS_MARSHALERRORDESC, *EC_PT_RAS_MARSHALERRORDESC;

typedef struct _EC_T_RAS_NONOTIFYMEMORYDESC
{
    EC_T_DWORD      dwCookie;       /**< [in]   Cookie of faulting connection */
    EC_T_DWORD      dwCode;         /**< [in]   Fault causing notification code */
} EC_PACKED(1) EC_T_RAS_NONOTIFYMEMORYDESC, *EC_PT_RAS_NONOTIFYMEMORYDESC;

typedef struct _EC_T_RAS_EXCEPTIONDESC
{
    EC_T_DWORD      dwCookie;                                          /**< [in] Unique identification cookie of connection instance */
    EC_T_CHAR       szThreadName[EC_RAS_EXCEPTION_THREAD_NAME_SIZE];   /**< [in] Thread name */
} EC_PACKED(1) EC_T_RAS_EXCEPTIONDESC, *EC_PT_RAS_EXCEPTIONDESC;
#include EC_PACKED_INCLUDESTOP

/*-FUNCTION DECLARATION------------------------------------------------------*/
EC_API const EC_T_CHAR* EC_API_FNCALL emRasErrorText(EC_T_DWORD dwError);
EC_API const EC_T_CHAR* EC_API_FNCALL emRasEventText(EC_T_DWORD dwEvent);

EC_API const EC_T_CHAR* EC_API_FNCALL esRasErrorText(EC_T_DWORD dwError);
EC_API const EC_T_CHAR* EC_API_FNCALL esRasEventText(EC_T_DWORD dwEvent);

#endif /* INC_ECRASERROR */

/*-END OF SOURCE FILE--------------------------------------------------------*/
