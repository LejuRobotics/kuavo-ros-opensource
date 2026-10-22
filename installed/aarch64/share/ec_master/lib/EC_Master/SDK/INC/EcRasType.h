/*-----------------------------------------------------------------------------
 * EcRasType.h            file
 * Copyright                acontis technologies GmbH, Ravensburg, Germany
 * Response                 Willig, Andreas
 * Description              description of file
 * Date                     2007/5/4::13:58
 *---------------------------------------------------------------------------*/

#ifndef INC_ECMASTERRASTYPE
#define INC_ECMASTERRASTYPE 1

/*-INCLUDES------------------------------------------------------------------*/
#ifndef INC_ECOS
#include "EcOs.h"
#endif

/*-COMPILER SETTINGS---------------------------------------------------------*/
#ifdef __cplusplus
extern "C"
{
#endif

/*-DEFINES-------------------------------------------------------------------*/
#undef INCLUDE_RAS_TRACESUPPORT

#if (!defined ECMASTERRAS_DEFAULT_PORT)
#if (defined EC_SOCKET_IP_SUPPORTED)
  #if (defined EC_SIMULATOR)
  #define ECMASTERRAS_DEFAULT_PORT     ((EC_T_WORD)6001)
  #else
  #define ECMASTERRAS_DEFAULT_PORT     ((EC_T_WORD)6000)
  #endif
#elif (defined EC_SOCKET_MSGQUEUE_RTOSSHM_SUPPORTED)
#define ECMASTERRAS_DEFAULT_PORT       ((EC_T_WORD)2)
#elif (defined EC_SOCKET_SHM_SUPPORTED)
#define ECMASTERRAS_DEFAULT_PORT       ((EC_T_WORD)1)
#elif (defined EC_SOCKET_MSGQUEUE_WIN32_SUPPORTED)
#define ECMASTERRAS_DEFAULT_PORT       ((EC_T_WORD)0)
#elif (defined EC_SOCKET_RTOSLIB_SUPPORTED)
#define ECMASTERRAS_DEFAULT_PORT       ((EC_T_WORD)3)
#else
#error Unknown socket implementation
#endif /* EC_SOCKET_..._SUPPORTED */
#endif /* ECMASTERRAS_DEFAULT_PORT */

/*-TYPEDEFS------------------------------------------------------------------*/

/*-COMPILER SETTINGS---------------------------------------------------------*/
#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* INC_ECMASTERRASTYPE */

/*-END OF SOURCE FILE--------------------------------------------------------*/
