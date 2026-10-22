/*-----------------------------------------------------------------------------
* EcSdoServices.h
* Copyright                acontis technologies GmbH, Ravensburg, Germany
* Response                 Holger Oelhaf
* Description
*---------------------------------------------------------------------------*/

#ifndef INC_ECSDOSERVICES_H
#define INC_ECSDOSERVICES_H 1

/*-FUNCTION DECLARATION------------------------------------------------------*/
EC_T_DWORD CoeReadObjectDictionary(
    T_EC_DEMO_APP_CONTEXT* pAppContext,
    EC_T_BOOL*               pbStopReading,   /**< [in]   Pointer to "Stop Reading" flag */
    EC_T_DWORD               dwNodeId,        /**< [in]   Slave Id to query ODL from  */
    EC_T_BOOL                bPerformUpload,  /**< [in]   EC_TRUE: do SDO Upload */
    EC_T_DWORD               dwTimeout        /**< [in]   Individual call time-out */
);

#endif

/*-END OF SOURCE FILE--------------------------------------------------------*/
