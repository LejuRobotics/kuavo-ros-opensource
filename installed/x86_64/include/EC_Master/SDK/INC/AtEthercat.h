/*-----------------------------------------------------------------------------
 * AtEthercat.h
 * Copyright                acontis technologies GmbH, Ravensburg, Germany
 * Response                 Stefan Zintgraf
 * Description              interface to the ethercat master
 *---------------------------------------------------------------------------*/

#ifndef INC_ATETHERCAT
#define INC_ATETHERCAT 1

/*-INCLUDE-------------------------------------------------------------------*/
#ifndef INSTANCE_MASTER_DEFAULT                     /* can be overriden in AtEmRasClnt.h */
#define INSTANCE_MASTER_DEFAULT                     ((EC_T_DWORD)0)  /* default master instance to be used by emXXXXX API to access same instance as ecatXXXXX API */
#endif

#ifndef INC_ECOS
#include "EcOs.h"
#endif
#ifndef INC_ECINTERFACECOMMON
#include "EcInterfaceCommon.h"
#endif
#ifndef INC_ECLINK
#include "EcLink.h"
#endif

/*-DEFINES/MACROS------------------------------------------------------------*/
#define ATECAT_SIGNATURE_PATTERN                    0xAEC00000
#define ATECAT_SIGNATURE (  ATECAT_SIGNATURE_PATTERN       \
                         | (ATECAT_VERS_MAJ         << 16) \
                         | (ATECAT_VERS_MIN         << 12) \
                         | (ATECAT_VERS_SERVICEPACK <<  8) \
                         | (ATECAT_VERS_BUILD       <<  0) \
                         )

/*-TYPEDEFS/ENUMS------------------------------------------------------------*/

#include EC_PACKED_API_INCLUDESTART
typedef struct _EC_T_INIT_MASTER_PARMS
{
    EC_T_DWORD  dwSignature;                        /**< [in] set to ATECAT_SIGNATURE */
    EC_T_DWORD  dwSize;                             /**< [in] set to sizeof(EC_T_INIT_MASTER_PARMS) */

    struct _EC_T_OS_PARMS*      pOsParms;           /**< [in] OS layer parameters */
    struct _EC_T_LINK_PARMS*    pLinkParms;         /**< [in] Link layer parameters */
    struct _EC_T_LINK_PARMS*    pLinkParmsRed;      /**< [in] Link layer parameters for red device (cable redundancy) */

    EC_T_DWORD  dwBusCycleTimeUsec;                 /**< [in] [usec] bus cycle time in microseconds */

    /* memory */
    EC_T_DWORD  dwMaxBusSlaves;                     /**< [in] maximum pre-allocated bus slave objects */
    EC_T_DWORD  dwMaxAcycFramesQueued;              /**< [in] maximum queued acyclic frames */
    EC_T_DWORD  dwAdditionalEoEEndpoints;           /**< [in] additional EoE endpoints */

    /* bus load */
    EC_T_DWORD  dwMaxAcycBytesPerCycle;             /**< [in] maximum bytes sent during eUsrJob_SendAcycFrames per cycle */

    /* CPU load */
    EC_T_DWORD  dwMaxAcycFramesPerCycle;            /**< [in] maximum frames sent during eUsrJob_SendAcycFrames per cycle */
    EC_T_DWORD  dwMaxAcycCmdsPerCycle;              /**< [in] maximum commands sent during eUsrJob_SendAcycFrames per cycle */
    EC_T_DWORD  dwMaxSlavesProcessedPerCycle;       /**< [in] maximum slave-related state machine calls per cycle */

    /* retry and timeouts */
    EC_T_DWORD  dwEcatCmdMaxRetries;                /**< [in] maximum retries to send pending ethercat command frames */
    EC_T_DWORD  dwEcatCmdTimeout;                   /**< [in] timeout to send pending ethercat command frames */

    /* VLAN */
    EC_T_BOOL   bVLANEnable;                        /**< [in] E=enable (1/0) */
    EC_T_WORD   wVLANId;                            /**< [in] I=VLAN Id (12Bit)*/
    EC_T_BYTE   byVLANPrio;                         /**< [in] P=Prio (3Bit) */

    EC_T_LOG_PARMS EC_PACKED_API_MEMBER LogParms;           /**< [in] Logging parameters */

    EC_T_MASTER_RED_PARMS EC_PACKED_API_MEMBER MasterRedParms; /**< [in] Master Redundancy parameters */

    /* Slave to slave mailbox communication */
    EC_T_DWORD  dwMaxS2SMbxSize;                    /**< [in] Size of the queued S2S mailbox in bytes */
    EC_T_DWORD  dwMaxQueuedS2SMbxTfer;              /**< [in] S2S Fifo number of entries */

    EC_T_WORD   wMaxSlavesProcessedPerBusScanStep;  /**< [in] maximum slave-related calls per cycle during bus scans */
    EC_T_WORD   wReserved;

    EC_T_BOOL   bApiLockByApp;                      /**< [in] EC_TRUE: Don't lock pending API calls to increase performance */

    /* Performance Measurements */
    EC_T_PERF_MEAS_INTERNAL_PARMS PerfMeasInternalParms; /**< [in] Internal performance measurement parameters */
} EC_PACKED_API EC_T_INIT_MASTER_PARMS;
#include EC_PACKED_INCLUDESTOP

/* link layer debug message */
#include EC_PACKED_INCLUDESTART(1)
typedef struct _EC_T_LINKLAYER_DBG_MSG_DESC
{
    EC_T_BYTE   byEthTypeByte0;                     /**< [in] Ethernet type byte 0 */
    EC_T_BYTE   byEthTypeByte1;                     /**< [in] Ethernet type byte 0 */
    EC_T_WORD   wRes;
    EC_T_CHAR*  szMsg;                              /**< [in] message to send to link layer */
} EC_PACKED(1) EC_T_LINKLAYER_DBG_MSG_DESC;
#include EC_PACKED_INCLUDESTOP

/*-FUNCTION DECLARATIONS-----------------------------------------------------*/
#ifdef __cplusplus
extern "C"
{
#endif

EC_API const EC_T_CHAR* EC_API_FNCALL emGetText(  EC_T_DWORD dwInstanceID, EC_T_DWORD dwTextId);
EC_API const EC_T_CHAR* EC_API_FNCALL ecatGetText(EC_T_DWORD dwTextId);

EC_API const EC_T_CHAR* EC_API_FNCALL emGetNotifyText(  EC_T_DWORD dwInstanceID, EC_T_DWORD dwNotifyCode);
EC_API const EC_T_CHAR* EC_API_FNCALL ecatGetNotifyText(EC_T_DWORD dwNotifyCode);

EC_API EC_T_DWORD EC_API_FNCALL emConvertEcErrorToAdsError(  EC_T_DWORD dwInstanceID, EC_T_DWORD dwErrorCode);
EC_API EC_T_DWORD EC_API_FNCALL ecatConvertEcErrorToAdsError(EC_T_DWORD dwErrorCode);

EC_API EC_T_DWORD EC_API_FNCALL emConvertEcErrorToFoeError(  EC_T_DWORD dwInstanceID, EC_T_DWORD dwErrorCode);
EC_API EC_T_DWORD EC_API_FNCALL ecatConvertEcErrorToFoeError(EC_T_DWORD dwErrorCode);

EC_API EC_T_DWORD EC_API_FNCALL emConvertEcErrorToSoeError(  EC_T_DWORD dwInstanceID, EC_T_DWORD dwErrorCode);
EC_API EC_T_DWORD EC_API_FNCALL ecatConvertEcErrorToSoeError(EC_T_DWORD dwErrorCode);

EC_API EC_T_DWORD EC_API_FNCALL emConvertEcErrorToCoeError(  EC_T_DWORD dwInstanceID, EC_T_DWORD dwErrorCode);
EC_API EC_T_DWORD EC_API_FNCALL ecatConvertEcErrorToCoeError(EC_T_DWORD dwErrorCode);


/* Perf measurement API functions */

EC_API EC_T_VOID EC_API_FNCALL emPerfMeasInit(   EC_T_DWORD dwInstanceID, EC_T_TSC_MEAS_DESC* pTscMeasDesc, EC_T_UINT64 dwlFreqSet, EC_T_DWORD dwNumMeas, EC_T_FNMESSAGE pfnMessage);
EC_API EC_T_VOID EC_API_FNCALL ecatPerfMeasInit( EC_T_TSC_MEAS_DESC* pTscMeasDesc, EC_T_UINT64 dwlFreqSet, EC_T_DWORD dwNumMeas, EC_T_FNMESSAGE pfnMessage);

EC_API EC_T_VOID EC_API_FNCALL emPerfMeasDeinit(   EC_T_DWORD dwInstanceID, EC_T_TSC_MEAS_DESC* pTscMeasDesc);
EC_API EC_T_VOID EC_API_FNCALL ecatPerfMeasDeinit( EC_T_TSC_MEAS_DESC* pTscMeasDesc );

EC_API EC_T_VOID EC_API_FNCALL emPerfMeasEnable(   EC_T_DWORD dwInstanceID, EC_T_TSC_MEAS_DESC* pTscMeasDesc);
EC_API EC_T_VOID EC_API_FNCALL ecatPerfMeasEnable( EC_T_TSC_MEAS_DESC* pTscMeasDesc );

EC_API EC_T_VOID EC_API_FNCALL emPerfMeasDisable(   EC_T_DWORD dwInstanceID, EC_T_TSC_MEAS_DESC* pTscMeasDesc);
EC_API EC_T_VOID EC_API_FNCALL ecatPerfMeasDisable( EC_T_TSC_MEAS_DESC* pTscMeasDesc );

EC_API EC_T_VOID EC_API_FNCALL emPerfMeasStart(   EC_T_DWORD dwInstanceID, EC_T_TSC_MEAS_DESC* pTscMeasDesc, EC_T_DWORD dwIndex);
EC_API EC_T_VOID EC_API_FNCALL ecatPerfMeasStart( EC_T_TSC_MEAS_DESC* pTscMeasDesc, EC_T_DWORD dwIndex );

EC_API EC_T_TSC_TIME* EC_API_FNCALL emPerfMeasEnd(   EC_T_DWORD dwInstanceID, EC_T_TSC_MEAS_DESC* pTscMeasDesc, EC_T_DWORD dwIndex);
EC_API EC_T_TSC_TIME* EC_API_FNCALL ecatPerfMeasEnd( EC_T_TSC_MEAS_DESC* pTscMeasDesc, EC_T_DWORD dwIndex );

EC_API EC_T_VOID EC_API_FNCALL emPerfMeasReset(   EC_T_DWORD dwInstanceID, EC_T_TSC_MEAS_DESC* pTscMeasDesc, EC_T_DWORD dwIndex);
EC_API EC_T_VOID EC_API_FNCALL ecatPerfMeasReset( EC_T_TSC_MEAS_DESC* pTscMeasDesc, EC_T_DWORD dwIndex );

EC_API EC_T_VOID EC_API_FNCALL emPerfMeasShow(   EC_T_DWORD dwInstanceID, EC_T_TSC_MEAS_DESC* pTscMeasDesc, EC_T_DWORD dwIndex, EC_T_CHAR** aszMeasInfo);
EC_API EC_T_VOID EC_API_FNCALL ecatPerfMeasShow( EC_T_TSC_MEAS_DESC* pTscMeasDesc, EC_T_DWORD dwIndex, EC_T_CHAR** aszMeasInfo );

EC_API EC_T_VOID EC_API_FNCALL ecatPerfMeasSetIrqCtlEnabled(  EC_T_BOOL bEnabled        );


/* New Perf measurement API functions */

EC_API EC_T_DWORD EC_API_FNCALL emPerfMeasAppCreate(EC_T_DWORD dwInstanceID, EC_T_PERF_MEAS_APP_PARMS* pPerfMeasAppParms, EC_T_VOID** ppvPerfMeas);
#ifndef ECAT_INTERFACE
static EC_INLINESTART EC_T_DWORD ecatPerfMeasAppCreate(EC_T_PERF_MEAS_APP_PARMS* pPerfMeasAppParms, EC_T_VOID** ppvPerfMeas)
{
    return emPerfMeasAppCreate(INSTANCE_MASTER_DEFAULT, pPerfMeasAppParms, ppvPerfMeas);
} EC_INLINESTOP
#endif

EC_API EC_T_DWORD EC_API_FNCALL emPerfMeasAppDelete(EC_T_DWORD dwInstanceID, EC_T_VOID* pvPerfMeas);
#ifndef ECAT_INTERFACE
static EC_INLINESTART EC_T_DWORD ecatPerfMeasAppDelete(EC_T_VOID* pvPerfMeas)
{
    return emPerfMeasAppDelete(INSTANCE_MASTER_DEFAULT, pvPerfMeas);
} EC_INLINESTOP
#endif

EC_API EC_T_DWORD EC_API_FNCALL emPerfMeasAppStart(EC_T_DWORD dwInstanceID, EC_T_VOID* pvPerfMeas, EC_T_DWORD dwIndex);
#ifndef ECAT_INTERFACE
static EC_INLINESTART EC_T_DWORD ecatPerfMeasAppStart(EC_T_VOID* pvPerfMeas, EC_T_DWORD dwIndex)
{
    return emPerfMeasAppStart(INSTANCE_MASTER_DEFAULT, pvPerfMeas, dwIndex);
} EC_INLINESTOP
#endif

EC_API EC_T_DWORD EC_API_FNCALL emPerfMeasAppEnd(EC_T_DWORD dwInstanceID, EC_T_VOID* pvPerfMeas, EC_T_DWORD dwIndex);
#ifndef ECAT_INTERFACE
static EC_INLINESTART EC_T_DWORD ecatPerfMeasAppEnd(EC_T_VOID* pvPerfMeas, EC_T_DWORD dwIndex)
{
    return emPerfMeasAppEnd(INSTANCE_MASTER_DEFAULT, pvPerfMeas, dwIndex);
} EC_INLINESTOP
#endif

EC_API EC_T_DWORD EC_API_FNCALL emPerfMeasAppReset(EC_T_DWORD dwInstanceID, EC_T_VOID* pvPerfMeas, EC_T_DWORD dwIndex);
#ifndef ECAT_INTERFACE
static EC_INLINESTART EC_T_DWORD ecatPerfMeasAppReset(EC_T_VOID* pvPerfMeas, EC_T_DWORD dwIndex)
{
    return emPerfMeasAppReset(INSTANCE_MASTER_DEFAULT, pvPerfMeas, dwIndex);
} EC_INLINESTOP
#endif

EC_API EC_T_DWORD EC_API_FNCALL emPerfMeasAppGetRaw(EC_T_DWORD dwInstanceID, EC_T_VOID* pvPerfMeas, EC_T_DWORD dwIndex, EC_T_PERF_MEAS_VAL* pPerfMeasVal, EC_T_PERF_MEAS_HISTOGRAM* pPerfMeasHistogram, EC_T_DWORD dwPerfMeasNumOf);
#ifndef ECAT_INTERFACE
static EC_INLINESTART EC_T_DWORD ecatPerfMeasAppGetRaw(EC_T_VOID* pvPerfMeas, EC_T_DWORD dwIndex, EC_T_PERF_MEAS_VAL* pPerfMeasVal, EC_T_PERF_MEAS_HISTOGRAM* pPerfMeasHistogram, EC_T_DWORD dwPerfMeasNumOf)
{
    return emPerfMeasAppGetRaw(INSTANCE_MASTER_DEFAULT, pvPerfMeas, dwIndex, pPerfMeasVal, pPerfMeasHistogram, dwPerfMeasNumOf);
} EC_INLINESTOP
#endif

EC_API EC_T_DWORD EC_API_FNCALL emPerfMeasAppGetInfo(EC_T_DWORD dwInstanceID, EC_T_VOID* pvPerfMeas, EC_T_DWORD dwIndex, EC_T_PERF_MEAS_INFO* pPerfMeasInfo, EC_T_DWORD dwPerfMeasNumOf);
#ifndef ECAT_INTERFACE
static EC_INLINESTART EC_T_DWORD ecatPerfMeasAppGetInfo(EC_T_VOID* pvPerfMeas, EC_T_DWORD dwIndex, EC_T_PERF_MEAS_INFO* pPerfMeasInfo, EC_T_DWORD dwPerfMeasNumOf)
{
    return emPerfMeasAppGetInfo(INSTANCE_MASTER_DEFAULT, pvPerfMeas, dwIndex, pPerfMeasInfo, dwPerfMeasNumOf);
} EC_INLINESTOP
#endif

EC_API EC_T_DWORD EC_API_FNCALL emPerfMeasAppGetNumOf(EC_T_DWORD dwInstanceID, EC_T_VOID* pvPerfMeas, EC_T_DWORD* pdwNumOf);
#ifndef ECAT_INTERFACE
static EC_INLINESTART EC_T_DWORD ecatPerfMeasAppGetNumOf(EC_T_VOID* pvPerfMeas, EC_T_DWORD* pdwNumOf)
{
    return emPerfMeasAppGetNumOf(INSTANCE_MASTER_DEFAULT, pvPerfMeas, pdwNumOf);
} EC_INLINESTOP
#endif

EC_API EC_T_DWORD EC_API_FNCALL emPerfMeasInternalResetByTaskId(EC_T_DWORD dwInstanceID, EC_T_DWORD dwTaskId, EC_T_DWORD dwIndex);
#ifndef ECAT_INTERFACE
static EC_INLINESTART EC_T_DWORD ecatPerfMeasInternalResetByTaskId(EC_T_DWORD dwTaskId, EC_T_DWORD dwIndex)
{
    return emPerfMeasInternalResetByTaskId(INSTANCE_MASTER_DEFAULT, dwTaskId, dwIndex);
} EC_INLINESTOP
#endif

#ifndef emPerfMeasInternalReset
static EC_INLINESTART EC_T_DWORD emPerfMeasInternalReset(EC_T_DWORD dwInstanceID, EC_T_DWORD dwIndex)
{
    return emPerfMeasInternalResetByTaskId(dwInstanceID, 0, dwIndex);
} EC_INLINESTOP
#define emPerfMeasInternalReset emPerfMeasInternalReset
#endif
#ifndef ECAT_INTERFACE
static EC_INLINESTART EC_T_DWORD ecatPerfMeasInternalReset(EC_T_DWORD dwIndex)
{
    return emPerfMeasInternalResetByTaskId(INSTANCE_MASTER_DEFAULT, 0, dwIndex);
} EC_INLINESTOP
#endif

EC_API EC_T_DWORD EC_API_FNCALL emPerfMeasInternalGetRawByTaskId(EC_T_DWORD dwInstanceID, EC_T_DWORD dwTaskId, EC_T_DWORD dwIndex, EC_T_PERF_MEAS_VAL* pPerfMeasVal, EC_T_PERF_MEAS_HISTOGRAM* pPerfMeasHistogram, EC_T_DWORD dwPerfMeasNumOf);
#ifndef ECAT_INTERFACE
static EC_INLINESTART EC_T_DWORD ecatPerfMeasInternalGetRawByTaskId(EC_T_DWORD dwTaskId, EC_T_DWORD dwIndex, EC_T_PERF_MEAS_VAL* pPerfMeasVal, EC_T_PERF_MEAS_HISTOGRAM* pPerfMeasHistogram, EC_T_DWORD dwPerfMeasNumOf)
{
    return emPerfMeasInternalGetRawByTaskId(INSTANCE_MASTER_DEFAULT, dwTaskId, dwIndex, pPerfMeasVal, pPerfMeasHistogram, dwPerfMeasNumOf);
} EC_INLINESTOP
#endif

#ifndef emPerfMeasInternalGetRaw
static EC_INLINESTART EC_T_DWORD emPerfMeasInternalGetRaw(EC_T_DWORD dwInstanceID, EC_T_DWORD dwIndex, EC_T_PERF_MEAS_VAL* pPerfMeasVal, EC_T_PERF_MEAS_HISTOGRAM* pPerfMeasHistogram, EC_T_DWORD dwPerfMeasNumOf)
{
    return emPerfMeasInternalGetRawByTaskId(dwInstanceID, 0, dwIndex, pPerfMeasVal, pPerfMeasHistogram, dwPerfMeasNumOf);
} EC_INLINESTOP
#define emPerfMeasInternalGetRaw emPerfMeasInternalGetRaw
#endif
#ifndef ECAT_INTERFACE
static EC_INLINESTART EC_T_DWORD ecatPerfMeasInternalGetRaw(EC_T_DWORD dwIndex, EC_T_PERF_MEAS_VAL* pPerfMeasVal, EC_T_PERF_MEAS_HISTOGRAM* pPerfMeasHistogram, EC_T_DWORD dwPerfMeasNumOf)
{
    return emPerfMeasInternalGetRawByTaskId(INSTANCE_MASTER_DEFAULT, 0, dwIndex, pPerfMeasVal, pPerfMeasHistogram, dwPerfMeasNumOf);
} EC_INLINESTOP
#endif

EC_API EC_T_DWORD EC_API_FNCALL emPerfMeasInternalGetInfoByTaskId(EC_T_DWORD dwInstanceID, EC_T_DWORD dwTaskId, EC_T_DWORD dwIndex, EC_T_PERF_MEAS_INFO* pPerfMeasInfo, EC_T_DWORD dwPerfMeasNumOf);
#ifndef ECAT_INTERFACE
static EC_INLINESTART EC_T_DWORD ecatPerfMeasInternalGetInfoByTaskId(EC_T_DWORD dwTaskId, EC_T_DWORD dwIndex, EC_T_PERF_MEAS_INFO* pPerfMeasInfo, EC_T_DWORD dwPerfMeasNumOf)
{
    return emPerfMeasInternalGetInfoByTaskId(INSTANCE_MASTER_DEFAULT, dwTaskId, dwIndex, pPerfMeasInfo, dwPerfMeasNumOf);
} EC_INLINESTOP
#endif

#ifndef emPerfMeasInternalGetInfo
static EC_INLINESTART EC_T_DWORD emPerfMeasInternalGetInfo(EC_T_DWORD dwInstanceID, EC_T_DWORD dwIndex, EC_T_PERF_MEAS_INFO* pPerfMeasInfo, EC_T_DWORD dwPerfMeasNumOf)
{
    return emPerfMeasInternalGetInfoByTaskId(dwInstanceID, 0, dwIndex, pPerfMeasInfo, dwPerfMeasNumOf);
} EC_INLINESTOP
#define emPerfMeasInternalGetInfo emPerfMeasInternalGetInfo
#endif
#ifndef ECAT_INTERFACE
static EC_INLINESTART EC_T_DWORD ecatPerfMeasInternalGetInfo(EC_T_DWORD dwIndex, EC_T_PERF_MEAS_INFO* pPerfMeasInfo, EC_T_DWORD dwPerfMeasNumOf)
{
    return emPerfMeasInternalGetInfoByTaskId(INSTANCE_MASTER_DEFAULT, 0, dwIndex, pPerfMeasInfo, dwPerfMeasNumOf);
} EC_INLINESTOP
#endif

EC_API EC_T_DWORD EC_API_FNCALL emPerfMeasInternalGetNumOfByTaskId(EC_T_DWORD dwInstanceID, EC_T_DWORD dwTaskId, EC_T_DWORD* pdwNumOf);
#ifndef ECAT_INTERFACE
static EC_INLINESTART EC_T_DWORD ecatPerfMeasInternalGetNumOfByTaskId(EC_T_DWORD dwTaskId, EC_T_DWORD* pdwNumOf)
{
    return emPerfMeasInternalGetNumOfByTaskId(INSTANCE_MASTER_DEFAULT, dwTaskId, pdwNumOf);
} EC_INLINESTOP
#endif

#ifndef emPerfMeasInternalGetNumOf
static EC_INLINESTART EC_T_DWORD emPerfMeasInternalGetNumOf(EC_T_DWORD dwInstanceID, EC_T_DWORD* pdwNumOf)
{
    return emPerfMeasInternalGetNumOfByTaskId(dwInstanceID, 0, pdwNumOf);
} EC_INLINESTOP
#define emPerfMeasInternalGetNumOf emPerfMeasInternalGetNumOf
#endif
#ifndef ECAT_INTERFACE
static EC_INLINESTART EC_T_DWORD ecatPerfMeasInternalGetNumOf(EC_T_DWORD* pdwNumOf)
{
    return emPerfMeasInternalGetNumOfByTaskId(INSTANCE_MASTER_DEFAULT, 0, pdwNumOf);
} EC_INLINESTOP
#endif

/* Multi instance API functions */

EC_API EC_T_DWORD EC_API_FNCALL emInitMaster(     EC_T_DWORD              dwInstanceID,
                                                    EC_T_INIT_MASTER_PARMS* pParms          );
static EC_INLINESTART  EC_T_DWORD ecatInitMaster(   EC_T_INIT_MASTER_PARMS* pParms          )
{
    return emInitMaster(INSTANCE_MASTER_DEFAULT, pParms);
} EC_INLINESTOP

EC_API EC_T_DWORD EC_API_FNCALL emDeinitMaster(   EC_T_DWORD              dwInstanceID    );
static EC_INLINESTART  EC_T_DWORD ecatDeinitMaster( EC_T_VOID                               )
{
    return emDeinitMaster(INSTANCE_MASTER_DEFAULT);
} EC_INLINESTOP

EC_API EC_T_DWORD EC_API_FNCALL emGetMasterParms( EC_T_DWORD              dwInstanceID,
                                                    EC_T_INIT_MASTER_PARMS* pParms,
                                                    EC_T_DWORD              dwParmsBufSize);
static EC_INLINESTART  EC_T_DWORD ecatGetMasterParms(EC_T_INIT_MASTER_PARMS* pParms, EC_T_DWORD dwParmsBufSize)
{
    return emGetMasterParms(INSTANCE_MASTER_DEFAULT, pParms, dwParmsBufSize);
} EC_INLINESTOP
EC_API EC_T_DWORD EC_API_FNCALL emSetMasterParms( EC_T_DWORD              dwInstanceID,
                                                    EC_T_INIT_MASTER_PARMS* pParms          );
static EC_INLINESTART  EC_T_DWORD ecatSetMasterParms(EC_T_INIT_MASTER_PARMS* pParms         )
{
    return emSetMasterParms(INSTANCE_MASTER_DEFAULT, pParms);
} EC_INLINESTOP

EC_API EC_T_DWORD EC_API_FNCALL emSetMasterRedStateReq(   EC_T_DWORD      dwInstanceID,
                                                            EC_T_BOOL       bActive);
#ifndef ECAT_INTERFACE
static EC_INLINESTART  EC_T_DWORD ecatSetMasterRedStateReq( EC_T_BOOL       bActive)
{
    return emSetMasterRedStateReq(INSTANCE_MASTER_DEFAULT, bActive);
} EC_INLINESTOP
#endif

EC_API EC_T_DWORD EC_API_FNCALL emGetMasterRedState(  EC_T_DWORD          dwInstanceID,
                                                        EC_T_BOOL*          pbActive);
#ifndef ECAT_INTERFACE
static EC_INLINESTART  EC_T_DWORD ecatGetMasterRedState(EC_T_BOOL*          pbActive)
{
    return emGetMasterRedState(INSTANCE_MASTER_DEFAULT, pbActive);
} EC_INLINESTOP
#endif

EC_API EC_T_BYTE* EC_API_FNCALL emGetMasterRedProcessImageInputPtr(EC_T_DWORD dwInstanceID);
#ifndef ECAT_INTERFACE
static EC_INLINESTART  EC_T_BYTE* ecatGetMasterRedProcessImageInputPtr(EC_T_VOID)
{
    return emGetMasterRedProcessImageInputPtr(INSTANCE_MASTER_DEFAULT);
} EC_INLINESTOP
#endif

EC_API EC_T_BYTE* EC_API_FNCALL emGetMasterRedProcessImageOutputPtr(EC_T_DWORD dwInstanceID);
#ifndef ECAT_INTERFACE
static EC_INLINESTART  EC_T_BYTE* ecatGetMasterRedProcessImageOutputPtr(EC_T_VOID)
{
    return emGetMasterRedProcessImageOutputPtr(INSTANCE_MASTER_DEFAULT);
} EC_INLINESTOP
#endif

EC_API EC_T_DWORD EC_API_FNCALL emIoControl(      EC_T_DWORD              dwInstanceID,
                                                    EC_T_DWORD              dwCode,
                                                    EC_T_IOCTLPARMS*        pParms          );
#ifndef ECAT_INTERFACE
static EC_INLINESTART  EC_T_DWORD ecatIoControl(    EC_T_DWORD              dwCode,
                                                    EC_T_IOCTLPARMS*        pParms          )
{
    return emIoControl(INSTANCE_MASTER_DEFAULT, dwCode, pParms);
} EC_INLINESTOP
#endif

EC_API EC_T_DWORD EC_API_FNCALL emScanBus(        EC_T_DWORD              dwInstanceID,
                                                    EC_T_DWORD              dwTimeout       );
#ifndef ECAT_INTERFACE
static EC_INLINESTART  EC_T_DWORD ecatScanBus(      EC_T_DWORD              dwTimeout       )
{
    return emScanBus(INSTANCE_MASTER_DEFAULT, dwTimeout);
} EC_INLINESTOP
#endif

EC_API EC_T_DWORD EC_API_FNCALL emRescueScan(     EC_T_DWORD               dwInstanceID,
                                                    EC_T_DWORD               dwTimeout      );
#ifndef ECAT_INTERFACE
static EC_INLINESTART  EC_T_DWORD ecatRescueScan(   EC_T_DWORD               dwTimeout      )
{
    return emRescueScan(INSTANCE_MASTER_DEFAULT, dwTimeout);
} EC_INLINESTOP
#endif

EC_API EC_T_DWORD EC_API_FNCALL emGetMasterInfo(  EC_T_DWORD               dwInstanceID,
                                                    EC_T_MASTER_INFO*        pMasterInfo    );
#ifndef ECAT_INTERFACE
static EC_INLINESTART  EC_T_DWORD ecatGetMasterInfo(EC_T_MASTER_INFO*        pMasterInfo    )
{
    return emGetMasterInfo(INSTANCE_MASTER_DEFAULT, pMasterInfo);
} EC_INLINESTOP
#endif

EC_API EC_T_DWORD EC_API_FNCALL emConfigureMaster(EC_T_DWORD              dwInstanceID,
                                                    EC_T_CNF_TYPE           eCnfType,
                                                    EC_T_PBYTE              pbyCnfData,
                                                    EC_T_DWORD              dwCnfDataLen    );
#ifndef ECAT_INTERFACE
static EC_INLINESTART  EC_T_DWORD ecatConfigureMaster(EC_T_CNF_TYPE         eCnfType,
                                                    EC_T_PBYTE              pbyCnfData,
                                                    EC_T_DWORD              dwCnfDataLen    )
{
    return emConfigureMaster(INSTANCE_MASTER_DEFAULT, eCnfType, pbyCnfData, dwCnfDataLen);
} EC_INLINESTOP
#endif

EC_API EC_T_DWORD EC_API_FNCALL emConfigLoad(     EC_T_DWORD              dwInstanceID,
                                                    EC_T_CNF_TYPE           eCnfType,
                                                    EC_T_PBYTE              pbyCnfData,
                                                    EC_T_DWORD              dwCnfDataLen    );
#ifndef ECAT_INTERFACE
static EC_INLINESTART  EC_T_DWORD ecatConfigLoad(   EC_T_CNF_TYPE           eCnfType,
                                                    EC_T_PBYTE              pbyCnfData,
                                                    EC_T_DWORD              dwCnfDataLen    )
{
    return emConfigLoad(INSTANCE_MASTER_DEFAULT, eCnfType, pbyCnfData, dwCnfDataLen);
} EC_INLINESTOP
#endif

EC_API EC_T_DWORD EC_API_FNCALL emConfigGet(        EC_T_DWORD    dwInstanceID,
                                                    EC_T_BYTE**   ppbyCnfData,
                                                    EC_T_DWORD*   pdwCnfDataLen             );
#ifndef ECAT_INTERFACE
static EC_INLINESTART  EC_T_DWORD ecatConfigGet(    EC_T_BYTE** ppbyCnfData,
                                                    EC_T_DWORD* pdwCnfDataLen               )
{
    return emConfigGet(INSTANCE_MASTER_DEFAULT, ppbyCnfData, pdwCnfDataLen);
} EC_INLINESTOP
#endif

EC_API EC_T_DWORD EC_API_FNCALL emConfigExcludeSlave(EC_T_DWORD           dwInstanceID,
                                                     EC_T_WORD              wStationAddress );
#ifndef ECAT_INTERFACE
static EC_INLINESTART  EC_T_DWORD ecatConfigExcludeSlave(EC_T_WORD          wStationAddress )
{
    return emConfigExcludeSlave(INSTANCE_MASTER_DEFAULT, wStationAddress);
} EC_INLINESTOP
#endif

EC_API EC_T_DWORD EC_API_FNCALL emConfigIncludeSlave(EC_T_DWORD           dwInstanceID,
                                                     EC_T_WORD              wStationAddress );
#ifndef ECAT_INTERFACE
static EC_INLINESTART  EC_T_DWORD ecatConfigIncludeSlave(EC_T_WORD          wStationAddress )
{
    return emConfigIncludeSlave(INSTANCE_MASTER_DEFAULT, wStationAddress);
} EC_INLINESTOP
#endif

EC_API EC_T_DWORD EC_API_FNCALL emConfigSetPreviousPort(EC_T_DWORD        dwInstanceID,
                                                    EC_T_WORD               wStationAddress,
                                                    EC_T_WORD               wStationAddressPrev,
                                                    EC_T_WORD               wPortPrev       );
#ifndef ECAT_INTERFACE
static EC_INLINESTART  EC_T_DWORD ecatConfigSetPreviousPort(EC_T_WORD       wStationAddress,
                                                    EC_T_WORD               wStationAddressPrev,
                                                    EC_T_WORD               wPortPrev       )
{
    return emConfigSetPreviousPort(INSTANCE_MASTER_DEFAULT, wStationAddress, wStationAddressPrev, wPortPrev);
} EC_INLINESTOP
#endif

EC_API EC_T_DWORD EC_API_FNCALL emConfigAddJunctionRedundancyConnection(
                                                    EC_T_DWORD              dwInstanceID,
                                                    EC_T_WORD               wHeadStationAddress,
                                                    EC_T_WORD               wHeadRedPort,
                                                    EC_T_WORD               wTailStationAddress,
                                                    EC_T_WORD               wTailRedPort    );
#ifndef ECAT_INTERFACE
static EC_INLINESTART  EC_T_DWORD ecatConfigAddJunctionRedundancyConnection(
                                                    EC_T_WORD               wHeadStationAddress,
                                                    EC_T_WORD               wHeadRedPort,
                                                    EC_T_WORD               wTailStationAddress,
                                                    EC_T_WORD               wTailRedPort    )
{
    return emConfigAddJunctionRedundancyConnection(INSTANCE_MASTER_DEFAULT, wHeadStationAddress, wHeadRedPort, wTailStationAddress, wTailRedPort);
} EC_INLINESTOP
#endif

EC_API EC_T_DWORD EC_API_FNCALL emConfigApply(    EC_T_DWORD              dwInstanceID      );
#ifndef ECAT_INTERFACE
static EC_INLINESTART  EC_T_DWORD ecatConfigApply(  EC_T_VOID                               )
{
    return emConfigApply(INSTANCE_MASTER_DEFAULT);
} EC_INLINESTOP
#endif

EC_API EC_T_DWORD EC_API_FNCALL emConfigExtend(     EC_T_DWORD              dwInstanceID,
                                                    EC_T_BOOL               bResetConfig,
                                                    EC_T_DWORD              dwTimeout       );
#ifndef ECAT_INTERFACE
static EC_INLINESTART  EC_T_DWORD ecatConfigExtend( EC_T_BOOL               bResetConfig,
                                                    EC_T_DWORD              dwTimeout       )
{
    return emConfigExtend(INSTANCE_MASTER_DEFAULT, bResetConfig, dwTimeout);
} EC_INLINESTOP
#endif

EC_API EC_T_DWORD EC_API_FNCALL emIsConfigured(     EC_T_DWORD              dwInstanceID,
                                                    EC_T_BOOL*              pbIsConfigured  );
#ifndef ECAT_INTERFACE
static EC_INLINESTART  EC_T_DWORD ecatIsConfigured( EC_T_BOOL*              pbIsConfigured  )
{
    return emIsConfigured(INSTANCE_MASTER_DEFAULT, pbIsConfigured);
} EC_INLINESTOP
#endif

EC_API EC_T_DWORD EC_API_FNCALL emSetMasterState( EC_T_DWORD              dwInstanceID,
                                                    EC_T_DWORD              dwTimeout,
                                                    EC_T_STATE              eReqState       );
#ifndef ECAT_INTERFACE
static EC_INLINESTART  EC_T_DWORD ecatSetMasterState(EC_T_DWORD             dwTimeout,
                                                    EC_T_STATE              eReqState       )
{
    return emSetMasterState(INSTANCE_MASTER_DEFAULT, dwTimeout, eReqState);
} EC_INLINESTOP
#endif

EC_API EC_T_STATE EC_API_FNCALL emGetMasterState( EC_T_DWORD              dwInstanceID    );
#ifndef ECAT_INTERFACE
static EC_INLINESTART  EC_T_STATE ecatGetMasterState(EC_T_VOID                               )
{
    return emGetMasterState(INSTANCE_MASTER_DEFAULT);
} EC_INLINESTOP
#endif

EC_API EC_T_DWORD EC_API_FNCALL emGetMasterStateEx(EC_T_DWORD              dwInstanceID,
                                                    EC_T_WORD*              pwCurrState,
                                                    EC_T_WORD*              pwReqState      );
#ifndef ECAT_INTERFACE
static EC_INLINESTART  EC_T_DWORD ecatGetMasterStateEx(EC_T_WORD*              pwCurrState,
                                                       EC_T_WORD*              pwReqState   )
{
    return emGetMasterStateEx(INSTANCE_MASTER_DEFAULT, pwCurrState, pwReqState);
} EC_INLINESTOP
#endif


EC_API EC_T_DWORD EC_API_FNCALL emStart(          EC_T_DWORD              dwInstanceID,
                                                    EC_T_DWORD              dwTimeout       );
#ifndef ECAT_INTERFACE
static EC_INLINESTART  EC_T_DWORD ecatStart(        EC_T_DWORD              dwTimeout       )
{
    return emStart(INSTANCE_MASTER_DEFAULT, dwTimeout);
} EC_INLINESTOP
#endif

EC_API EC_T_DWORD EC_API_FNCALL emStop(           EC_T_DWORD              dwInstanceID,
                                                    EC_T_DWORD              dwTimeout       );
#ifndef ECAT_INTERFACE
static EC_INLINESTART  EC_T_DWORD ecatStop(         EC_T_DWORD              dwTimeout       )
{
    return emStop(INSTANCE_MASTER_DEFAULT, dwTimeout);
} EC_INLINESTOP
#endif

EC_API EC_T_DWORD EC_API_FNCALL emGetSlaveId(     EC_T_DWORD              dwInstanceID,
                                                    EC_T_WORD               wStationAddress );
#ifndef ECAT_INTERFACE
static EC_INLINESTART  EC_T_DWORD ecatGetSlaveId(   EC_T_WORD               wStationAddress )
{
    return emGetSlaveId(INSTANCE_MASTER_DEFAULT, wStationAddress);
} EC_INLINESTOP
#endif

EC_API EC_T_DWORD EC_API_FNCALL emGetSlaveFixedAddr(EC_T_DWORD            dwInstanceID,
                                                    EC_T_DWORD              dwSlaveId,
                                                    EC_T_WORD*              pwFixedAddr     );
#ifndef ECAT_INTERFACE
static EC_INLINESTART  EC_T_DWORD ecatGetSlaveFixedAddr( EC_T_DWORD         dwSlaveId,
                                                    EC_T_WORD*              pwFixedAddr     )
{
    return emGetSlaveFixedAddr(INSTANCE_MASTER_DEFAULT, dwSlaveId, pwFixedAddr);
} EC_INLINESTOP
#endif

EC_API EC_T_DWORD EC_API_FNCALL emGetSlaveIdAtPosition(EC_T_DWORD         dwInstanceID,
                                                    EC_T_WORD               wAutoIncAddress );
#ifndef ECAT_INTERFACE
static EC_INLINESTART  EC_T_DWORD ecatGetSlaveIdAtPosition(EC_T_WORD        wAutoIncAddress )
{
    return emGetSlaveIdAtPosition(INSTANCE_MASTER_DEFAULT, wAutoIncAddress);
} EC_INLINESTOP
#endif

EC_API EC_T_BOOL EC_API_FNCALL emGetSlaveProp(   EC_T_DWORD              dwInstanceID,
                                                    EC_T_DWORD              dwSlaveId,
                                                    EC_T_SLAVE_PROP*        pSlaveProp      );
#ifndef ECAT_INTERFACE
static EC_INLINESTART  EC_T_BOOL  ecatGetSlaveProp( EC_T_DWORD              dwSlaveId,
                                                    EC_T_SLAVE_PROP*        pSlaveProp      )
{
    return emGetSlaveProp(INSTANCE_MASTER_DEFAULT, dwSlaveId, pSlaveProp);
} EC_INLINESTOP
#endif

EC_API EC_T_DWORD EC_API_FNCALL emGetSlavePortState(EC_T_DWORD            dwInstanceID,
                                                    EC_T_DWORD              dwSlaveId,
                                                    EC_T_WORD*              pwPortState     );
#ifndef ECAT_INTERFACE
static EC_INLINESTART  EC_T_DWORD ecatGetSlavePortState( EC_T_DWORD         dwSlaveId,
                                                    EC_T_WORD*              pwPortState     )
{
    return emGetSlavePortState(INSTANCE_MASTER_DEFAULT, dwSlaveId, pwPortState);
} EC_INLINESTOP
#endif

EC_API EC_T_DWORD EC_API_FNCALL emGetSlaveState(  EC_T_DWORD              dwInstanceID,
                                                    EC_T_DWORD              dwSlaveId,
                                                    EC_T_WORD*              pwCurrDevState,
                                                    EC_T_WORD*              pwReqDevState   );
#ifndef ECAT_INTERFACE
static EC_INLINESTART  EC_T_DWORD ecatGetSlaveState(EC_T_DWORD              dwSlaveId,
                                                    EC_T_WORD*              pwCurrDevState,
                                                    EC_T_WORD*              pwReqDevState   )
{
    return emGetSlaveState(INSTANCE_MASTER_DEFAULT, dwSlaveId, pwCurrDevState, pwReqDevState);
} EC_INLINESTOP
#endif

EC_API EC_T_DWORD EC_API_FNCALL emSetSlaveState(    EC_T_DWORD              dwInstanceID,
                                                    EC_T_DWORD              dwSlaveId,
                                                    EC_T_WORD               wNewReqDevState,
                                                    EC_T_DWORD              dwTimeout       );
#ifndef ECAT_INTERFACE
static EC_INLINESTART  EC_T_DWORD ecatSetSlaveState(EC_T_DWORD              dwSlaveId,
                                                    EC_T_WORD               wNewReqDevState,
                                                    EC_T_DWORD              dwTimeout       )
{
    return emSetSlaveState(INSTANCE_MASTER_DEFAULT, dwSlaveId, wNewReqDevState, dwTimeout);
} EC_INLINESTOP
#endif

EC_API EC_T_DWORD EC_API_FNCALL emTferSingleRawCmd( EC_T_DWORD              dwInstanceID,
                                                    EC_T_BYTE               byCmd,
                                                    EC_T_DWORD              dwMemoryAddress,
                                                    EC_T_BYTE*              pbyData,
                                                    EC_T_WORD               wLen,
                                                    EC_T_DWORD              dwTimeout       );
#ifndef ECAT_INTERFACE
static EC_INLINESTART  EC_T_DWORD ecatTferSingleRawCmd(EC_T_BYTE            byCmd,
                                                    EC_T_DWORD              dwMemoryAddress,
                                                    EC_T_BYTE*              pbyData,
                                                    EC_T_WORD               wLen,
                                                    EC_T_DWORD              dwTimeout       )
{
    return emTferSingleRawCmd(INSTANCE_MASTER_DEFAULT, byCmd, dwMemoryAddress, pbyData, wLen, dwTimeout);
} EC_INLINESTOP
#endif

EC_API EC_T_DWORD EC_API_FNCALL emReadSlaveRegister(EC_T_DWORD            dwInstanceID,
                                                    EC_T_BOOL               bFixedAddressing,
                                                    EC_T_WORD               wSlaveAddress,
                                                    EC_T_WORD               wRegisterOffset,
                                                    EC_T_BYTE*              pbyData,
                                                    EC_T_WORD               wLen,
                                                    EC_T_DWORD              dwTimeout       );
#ifndef ECAT_INTERFACE
static EC_INLINESTART  EC_T_DWORD ecatReadSlaveRegister(EC_T_BOOL           bFixedAddressing,
                                                    EC_T_WORD               wSlaveAddress,
                                                    EC_T_WORD               wRegisterOffset,
                                                    EC_T_BYTE*              pbyData,
                                                    EC_T_WORD               wLen,
                                                    EC_T_DWORD              dwTimeout       )
{
   return emReadSlaveRegister(
            INSTANCE_MASTER_DEFAULT, bFixedAddressing, wSlaveAddress, wRegisterOffset, pbyData, wLen, dwTimeout);
} EC_INLINESTOP
#endif

EC_API EC_T_DWORD EC_API_FNCALL emReadSlaveRegisterReq(EC_T_DWORD         dwInstanceID,
                                                    EC_T_DWORD              dwClientId,
                                                    EC_T_DWORD              dwTferId,
                                                    EC_T_BOOL               bFixedAddressing,
                                                    EC_T_WORD               wSlaveAddress,
                                                    EC_T_WORD               wRegisterOffset,
                                                    EC_T_BYTE*              pbyData,
                                                    EC_T_WORD               wLen            );
#ifndef ECAT_INTERFACE
static EC_INLINESTART  EC_T_DWORD ecatReadSlaveRegisterReq(EC_T_DWORD       dwClientId,
                                                    EC_T_DWORD              dwTferId,
                                                    EC_T_BOOL               bFixedAddressing,
                                                    EC_T_WORD               wSlaveAddress,
                                                    EC_T_WORD               wRegisterOffset,
                                                    EC_T_BYTE*              pbyData,
                                                    EC_T_WORD               wLen            )
{
    return emReadSlaveRegisterReq(
        INSTANCE_MASTER_DEFAULT, dwClientId, dwTferId, bFixedAddressing, wSlaveAddress, wRegisterOffset, pbyData, wLen);
} EC_INLINESTOP
#endif

EC_API EC_T_DWORD EC_API_FNCALL emWriteSlaveRegister(EC_T_DWORD           dwInstanceID,
                                                    EC_T_BOOL               bFixedAddressing,
                                                    EC_T_WORD               wSlaveAddress,
                                                    EC_T_WORD               wRegisterOffset,
                                                    EC_T_BYTE*              pbyData,
                                                    EC_T_WORD               wLen,
                                                    EC_T_DWORD              dwTimeout       );
#ifndef ECAT_INTERFACE
static EC_INLINESTART  EC_T_DWORD ecatWriteSlaveRegister(EC_T_BOOL          bFixedAddressing,
                                                    EC_T_WORD               wSlaveAddress,
                                                    EC_T_WORD               wRegisterOffset,
                                                    EC_T_BYTE*              pbyData,
                                                    EC_T_WORD               wLen,
                                                    EC_T_DWORD              dwTimeout       )
{
   return emWriteSlaveRegister(
            INSTANCE_MASTER_DEFAULT, bFixedAddressing,wSlaveAddress, wRegisterOffset, pbyData, wLen, dwTimeout);
} EC_INLINESTOP
#endif

EC_API EC_T_DWORD EC_API_FNCALL emWriteSlaveRegisterReq(EC_T_DWORD        dwInstanceID,
                                                    EC_T_DWORD              dwClientId,
                                                    EC_T_DWORD              dwTferId,
                                                    EC_T_BOOL               bFixedAddressing,
                                                    EC_T_WORD               wSlaveAddress,
                                                    EC_T_WORD               wRegisterOffset,
                                                    EC_T_BYTE*              pbyData,
                                                    EC_T_WORD               wLen            );
#ifndef ECAT_INTERFACE
static EC_INLINESTART  EC_T_DWORD ecatWriteSlaveRegisterReq(EC_T_DWORD      dwClientId,
                                                    EC_T_DWORD              dwTferId,
                                                    EC_T_BOOL               bFixedAddressing,
                                                    EC_T_WORD               wSlaveAddress,
                                                    EC_T_WORD               wRegisterOffset,
                                                    EC_T_BYTE*              pbyData,
                                                    EC_T_WORD               wLen            )
{
    return emWriteSlaveRegisterReq(
        INSTANCE_MASTER_DEFAULT, dwClientId, dwTferId, bFixedAddressing, wSlaveAddress, wRegisterOffset, pbyData, wLen);
} EC_INLINESTOP
#endif

EC_API EC_T_DWORD EC_API_FNCALL emQueueRawCmd(      EC_T_DWORD              dwInstanceID,
                                                    EC_T_WORD               wInvokeId,
                                                    EC_T_BYTE               byCmd,
                                                    EC_T_DWORD              dwMemoryAddress,
                                                    EC_T_BYTE*              pbyData,
                                                    EC_T_WORD               wLen            );
#ifndef ECAT_INTERFACE
static EC_INLINESTART  EC_T_DWORD ecatQueueRawCmd(  EC_T_WORD               wInvokeId,
                                                    EC_T_BYTE               byCmd,
                                                    EC_T_DWORD              dwMemoryAddress,
                                                    EC_T_BYTE*              pbyData,
                                                    EC_T_WORD               wLen            )
{
    return emQueueRawCmd(INSTANCE_MASTER_DEFAULT, wInvokeId, byCmd, dwMemoryAddress, pbyData, wLen);
} EC_INLINESTOP
#endif

EC_API EC_T_DWORD EC_API_FNCALL emClntQueueRawCmd(  EC_T_DWORD              dwInstanceID,
                                                    EC_T_DWORD              dwClntId,
                                                    EC_T_WORD               wInvokeId,
                                                    EC_T_BYTE               byCmd,
                                                    EC_T_DWORD              dwMemoryAddress,
                                                    EC_T_BYTE*              pbyData,
                                                    EC_T_WORD               wLen            );
#ifndef ECAT_INTERFACE
static EC_INLINESTART  EC_T_DWORD ecatClntQueueRawCmd(EC_T_DWORD            dwClntId,
                                                    EC_T_WORD               wInvokeId,
                                                    EC_T_BYTE               byCmd,
                                                    EC_T_DWORD              dwMemoryAddress,
                                                    EC_T_BYTE*              pbyData,
                                                    EC_T_WORD               wLen            )
{
    return emClntQueueRawCmd(INSTANCE_MASTER_DEFAULT, dwClntId, wInvokeId, byCmd, dwMemoryAddress, pbyData, wLen);
} EC_INLINESTOP
#endif

EC_API EC_T_DWORD EC_API_FNCALL emGetNumConfiguredSlaves(EC_T_DWORD       dwInstanceID    );
#ifndef ECAT_INTERFACE
static EC_INLINESTART  EC_T_DWORD ecatGetNumConfiguredSlaves(EC_T_VOID                      )
{
    return emGetNumConfiguredSlaves(INSTANCE_MASTER_DEFAULT);
} EC_INLINESTOP
#endif

EC_API EC_T_MBXTFER* EC_API_FNCALL emMbxTferCreate(EC_T_DWORD             dwInstanceID,
                                                    EC_T_MBXTFER_DESC*      pMbxTferDesc    );
#ifndef ECAT_INTERFACE
static EC_INLINESTART  EC_T_MBXTFER* ecatMbxTferCreate(EC_T_MBXTFER_DESC*   pMbxTferDesc    )
{
    return emMbxTferCreate(INSTANCE_MASTER_DEFAULT, pMbxTferDesc);
} EC_INLINESTOP
#endif

EC_API EC_T_DWORD    emMbxTferAbort(EC_T_DWORD              dwInstanceID,
                                                    EC_T_MBXTFER*           pMbxTfer        );
#ifndef ECAT_INTERFACE
static EC_INLINESTART  EC_T_DWORD    ecatMbxTferAbort(EC_T_MBXTFER*         pMbxTfer        )
{
    return emMbxTferAbort(INSTANCE_MASTER_DEFAULT, pMbxTfer);
} EC_INLINESTOP
#endif

EC_API EC_T_VOID EC_API_FNCALL emMbxTferDelete(EC_T_DWORD             dwInstanceID,
                                                    EC_T_MBXTFER*           pMbxTfer        );
#ifndef ECAT_INTERFACE
static EC_INLINESTART  EC_T_VOID     ecatMbxTferDelete(  EC_T_MBXTFER*      pMbxTfer        )
{
    emMbxTferDelete(INSTANCE_MASTER_DEFAULT, pMbxTfer);
} EC_INLINESTOP
#endif

EC_API EC_T_DWORD EC_API_FNCALL emClntSendRawMbx(EC_T_DWORD            dwInstanceID,
                                                    EC_T_DWORD              dwClntId,
                                                    EC_T_BYTE*              pbyMbxCmd,
                                                    EC_T_DWORD              dwMbxCmdLen,
                                                    EC_T_DWORD              dwTimeout       );
#ifndef ECAT_INTERFACE
static EC_INLINESTART  EC_T_DWORD    ecatClntSendRawMbx(EC_T_DWORD          dwClntId,
                                                    EC_T_BYTE*              pbyMbxCmd,
                                                    EC_T_DWORD              dwMbxCmdLen,
                                                    EC_T_DWORD              dwTimeout       )
{
    return emClntSendRawMbx(INSTANCE_MASTER_DEFAULT, dwClntId, pbyMbxCmd, dwMbxCmdLen, dwTimeout);
} EC_INLINESTOP
#endif

EC_API EC_T_DWORD EC_API_FNCALL emCoeSdoDownloadReq(EC_T_DWORD           dwInstanceID,
                                                    EC_T_MBXTFER*           pMbxTfer,
                                                    EC_T_DWORD              dwSlaveId,
                                                    EC_T_WORD               wObIndex,
                                                    EC_T_BYTE               byObSubIndex,
                                                    EC_T_DWORD              dwTimeout,
                                                    EC_T_DWORD              dwFlags         );
#ifndef ECAT_INTERFACE
static EC_INLINESTART  EC_T_DWORD  ecatCoeSdoDownloadReq(EC_T_MBXTFER*      pMbxTfer,
                                                    EC_T_DWORD              dwSlaveId,
                                                    EC_T_WORD               wObIndex,
                                                    EC_T_BYTE               byObSubIndex,
                                                    EC_T_DWORD              dwTimeout,
                                                    EC_T_DWORD              dwFlags         )
{
    return emCoeSdoDownloadReq(INSTANCE_MASTER_DEFAULT, pMbxTfer, dwSlaveId, wObIndex, byObSubIndex, dwTimeout, dwFlags);
} EC_INLINESTOP
#endif

EC_API EC_T_DWORD EC_API_FNCALL  emCoeSdoDownload(EC_T_DWORD             dwInstanceID
                                                   ,EC_T_DWORD              dwSlaveId
                                                   ,EC_T_WORD               wObIndex
                                                   ,EC_T_BYTE               byObSubIndex
                                                   ,EC_T_BYTE*              pbyData
                                                   ,EC_T_DWORD              dwDataLen
                                                   ,EC_T_DWORD              dwTimeout
                                                   ,EC_T_DWORD              dwFlags         );

#ifndef ECAT_INTERFACE
static EC_INLINESTART  EC_T_DWORD   ecatCoeSdoDownload(EC_T_DWORD           dwSlaveId
                                                   ,EC_T_WORD               wObIndex
                                                   ,EC_T_BYTE               byObSubIndex
                                                   ,EC_T_BYTE*              pbyData
                                                   ,EC_T_DWORD              dwDataLen
                                                   ,EC_T_DWORD              dwTimeout
                                                   ,EC_T_DWORD              dwFlags         )
{
    return emCoeSdoDownload(INSTANCE_MASTER_DEFAULT, dwSlaveId, wObIndex, byObSubIndex, pbyData, dwDataLen, dwTimeout, dwFlags);
} EC_INLINESTOP
#endif

EC_API EC_T_DWORD EC_API_FNCALL emCoeSdoUploadReq(EC_T_DWORD             dwInstanceID,
                                                    EC_T_MBXTFER*           pMbxTfer,
                                                    EC_T_DWORD              dwSlaveId,
                                                    EC_T_WORD               wObIndex,
                                                    EC_T_BYTE               byObSubIndex,
                                                    EC_T_DWORD              dwTimeout,
                                                    EC_T_DWORD              dwFlags         );
#ifndef ECAT_INTERFACE
static EC_INLINESTART  EC_T_DWORD  ecatCoeSdoUploadReq(EC_T_MBXTFER*        pMbxTfer,
                                                    EC_T_DWORD              dwSlaveId,
                                                    EC_T_WORD               wObIndex,
                                                    EC_T_BYTE               byObSubIndex,
                                                    EC_T_DWORD              dwTimeout,
                                                    EC_T_DWORD              dwFlags         )
{
    return emCoeSdoUploadReq(INSTANCE_MASTER_DEFAULT, pMbxTfer, dwSlaveId, wObIndex, byObSubIndex, dwTimeout, dwFlags);
} EC_INLINESTOP
#endif

EC_API EC_T_DWORD EC_API_FNCALL emCoeSdoUpload(  EC_T_DWORD              dwInstanceID
                                                   ,EC_T_DWORD              dwSlaveId
                                                   ,EC_T_WORD               wObIndex
                                                   ,EC_T_BYTE               byObSubIndex
                                                   ,EC_T_BYTE*              pbyData
                                                   ,EC_T_DWORD              dwDataLen
                                                   ,EC_T_DWORD*             pdwOutDataLen
                                                   ,EC_T_DWORD              dwTimeout
                                                   ,EC_T_DWORD              dwFlags         );
#ifndef ECAT_INTERFACE
static EC_INLINESTART  EC_T_DWORD  ecatCoeSdoUpload(EC_T_DWORD              dwSlaveId
                                                   ,EC_T_WORD               wObIndex
                                                   ,EC_T_BYTE               byObSubIndex
                                                   ,EC_T_BYTE*              pbyData
                                                   ,EC_T_DWORD              dwDataLen
                                                   ,EC_T_DWORD*             pdwOutDataLen
                                                   ,EC_T_DWORD              dwTimeout
                                                   ,EC_T_DWORD              dwFlags         )
{
    return emCoeSdoUpload(INSTANCE_MASTER_DEFAULT, dwSlaveId, wObIndex, byObSubIndex, pbyData, dwDataLen, pdwOutDataLen, dwTimeout, dwFlags);
} EC_INLINESTOP
#endif


EC_API EC_T_DWORD EC_API_FNCALL emCoeGetODList(  EC_T_DWORD              dwInstanceID,
                                                    EC_T_MBXTFER*           pMbxTfer,
                                                    EC_T_DWORD              dwSlaveId,
                                                    EC_T_COE_ODLIST_TYPE    eListType,
                                                    EC_T_DWORD              dwTimeout       );
#ifndef ECAT_INTERFACE
static EC_INLINESTART  EC_T_DWORD  ecatCoeGetODList(EC_T_MBXTFER*           pMbxTfer,
                                                    EC_T_DWORD              dwSlaveId,
                                                    EC_T_COE_ODLIST_TYPE    eListType,
                                                    EC_T_DWORD              dwTimeout       )
{
    return emCoeGetODList(INSTANCE_MASTER_DEFAULT, pMbxTfer, dwSlaveId, eListType, dwTimeout);
} EC_INLINESTOP
#endif

EC_API EC_T_DWORD EC_API_FNCALL emCoeGetObjectDesc(EC_T_DWORD            dwInstanceID,
                                                    EC_T_MBXTFER*           pMbxTfer,
                                                    EC_T_DWORD              dwSlaveId,
                                                    EC_T_WORD               wObIndex,
                                                    EC_T_DWORD              dwTimeout       );
#ifndef ECAT_INTERFACE
static EC_INLINESTART  EC_T_DWORD  ecatCoeGetObjectDesc(EC_T_MBXTFER*       pMbxTfer,
                                                    EC_T_DWORD              dwSlaveId,
                                                    EC_T_WORD               wObIndex,
                                                    EC_T_DWORD              dwTimeout       )
{
    return emCoeGetObjectDesc(INSTANCE_MASTER_DEFAULT, pMbxTfer, dwSlaveId, wObIndex, dwTimeout);
} EC_INLINESTOP
#endif

EC_API EC_T_DWORD EC_API_FNCALL emCoeGetEntryDesc(EC_T_DWORD             dwInstanceID,
                                                    EC_T_MBXTFER*           pMbxTfer,
                                                    EC_T_DWORD              dwSlaveId,
                                                    EC_T_WORD               wObIndex,
                                                    EC_T_BYTE               byObSubIndex,
                                                    EC_T_BYTE               byValueInfo,
                                                    EC_T_DWORD              dwTimeout       );

#ifndef ECAT_INTERFACE
static EC_INLINESTART  EC_T_DWORD  ecatCoeGetEntryDesc(EC_T_MBXTFER*        pMbxTfer,
                                                    EC_T_DWORD              dwSlaveId,
                                                    EC_T_WORD               wObIndex,
                                                    EC_T_BYTE               byObSubIndex,
                                                    EC_T_BYTE               byValueInfo,
                                                    EC_T_DWORD              dwTimeout       )
{
    return emCoeGetEntryDesc(INSTANCE_MASTER_DEFAULT, pMbxTfer, dwSlaveId, wObIndex, byObSubIndex, byValueInfo, dwTimeout);
} EC_INLINESTOP
#endif

EC_API EC_T_DWORD EC_API_FNCALL emCoeRxPdoTfer(   EC_T_DWORD              dwInstanceID,
                                                    EC_T_MBXTFER*           pMbxTfer,
                                                    EC_T_DWORD              dwSlaveId,
                                                    EC_T_DWORD              dwNumber,
                                                    EC_T_DWORD              dwTimeout       );

#ifndef ECAT_INTERFACE
static EC_INLINESTART  EC_T_DWORD ecatCoeRxPdoTfer( EC_T_MBXTFER*           pMbxTfer,
                                                    EC_T_DWORD              dwSlaveId,
                                                    EC_T_DWORD              dwNumber,
                                                    EC_T_DWORD              dwTimeout       )
{
    return emCoeRxPdoTfer(INSTANCE_MASTER_DEFAULT, pMbxTfer, dwSlaveId, dwNumber, dwTimeout);
} EC_INLINESTOP
#endif

EC_API EC_T_DWORD EC_API_FNCALL emCoeProfileGetChannelInfo(EC_T_DWORD       dwInstanceID,
                                                    EC_T_BOOL               bStationAddress,
                                                    EC_T_WORD               wSlaveAddress,
                                                    EC_T_DWORD              dwChannel,
                                                    EC_T_PROFILE_CHANNEL_INFO* pInfo);
#ifndef ECAT_INTERFACE
static EC_INLINESTART  EC_T_DWORD ecatCoeProfileGetChannelInfo(EC_T_BOOL    bStationAddress,
                                                    EC_T_WORD               wSlaveAddress,
                                                    EC_T_DWORD              dwChannel,
                                                    EC_T_PROFILE_CHANNEL_INFO* pInfo)
{
    return emCoeProfileGetChannelInfo(INSTANCE_MASTER_DEFAULT, bStationAddress, wSlaveAddress, dwChannel, pInfo);
} EC_INLINESTOP
#endif

EC_API EC_T_DWORD EC_API_FNCALL emFoeFileUpload( EC_T_DWORD              dwInstanceID,
                                                    EC_T_DWORD              dwSlaveId,
                                                    const EC_T_CHAR*        achFileName,
                                                    EC_T_DWORD              dwFileNameLen,
                                                    EC_T_BYTE*              pbyData,
                                                    EC_T_DWORD              dwDataLen,
                                                    EC_T_DWORD*             pdwOutDataLen,
                                                    EC_T_DWORD              dwPassword,
                                                    EC_T_DWORD              dwTimeout       );

#ifndef ECAT_INTERFACE
static EC_INLINESTART  EC_T_DWORD  ecatFoeFileUpload(EC_T_DWORD             dwSlaveId,
                                                    const EC_T_CHAR*        achFileName,
                                                    EC_T_DWORD              dwFileNameLen,
                                                    EC_T_BYTE*              pbyData,
                                                    EC_T_DWORD              dwDataLen,
                                                    EC_T_DWORD*             pdwOutDataLen,
                                                    EC_T_DWORD              dwPassword,
                                                    EC_T_DWORD              dwTimeout       )
{
    return emFoeFileUpload(INSTANCE_MASTER_DEFAULT, dwSlaveId, achFileName, dwFileNameLen, pbyData, dwDataLen, pdwOutDataLen, dwPassword, dwTimeout);
} EC_INLINESTOP
#endif

EC_API EC_T_DWORD EC_API_FNCALL emFoeFileDownload(EC_T_DWORD             dwInstanceID,
                                                    EC_T_DWORD              dwSlaveId,
                                                    const EC_T_CHAR*        achFileName,
                                                    EC_T_DWORD              dwFileNameLen,
                                                    EC_T_BYTE*              pbyData,
                                                    EC_T_DWORD              dwDataLen,
                                                    EC_T_DWORD              dwPassword,
                                                    EC_T_DWORD              dwTimeout       );

#ifndef ECAT_INTERFACE
static EC_INLINESTART  EC_T_DWORD  ecatFoeFileDownload(EC_T_DWORD           dwSlaveId,
                                                    const EC_T_CHAR*        achFileName,
                                                    EC_T_DWORD              dwFileNameLen,
                                                    EC_T_BYTE*              pbyData,
                                                    EC_T_DWORD              dwDataLen,
                                                    EC_T_DWORD              dwPassword,
                                                    EC_T_DWORD              dwTimeout       )
{
    return emFoeFileDownload(INSTANCE_MASTER_DEFAULT, dwSlaveId, achFileName, dwFileNameLen, pbyData, dwDataLen, dwPassword, dwTimeout);
} EC_INLINESTOP
#endif

EC_API EC_T_DWORD EC_API_FNCALL emFoeUploadReq(  EC_T_DWORD              dwInstanceID,
                                                    EC_T_MBXTFER*           pMbxTfer,
                                                    EC_T_DWORD              dwSlaveId,
                                                    const EC_T_CHAR*        achFileName,
                                                    EC_T_DWORD              dwFileNameLen,
                                                    EC_T_DWORD              dwPassword,
                                                    EC_T_DWORD              dwTimeout       );

#ifndef ECAT_INTERFACE
static EC_INLINESTART  EC_T_DWORD  ecatFoeUploadReq(EC_T_MBXTFER*           pMbxTfer,
                                                    EC_T_DWORD              dwSlaveId,
                                                    const EC_T_CHAR*        achFileName,
                                                    EC_T_DWORD              dwFileNameLen,
                                                    EC_T_DWORD              dwPassword,
                                                    EC_T_DWORD              dwTimeout       )
{
    return emFoeUploadReq(INSTANCE_MASTER_DEFAULT, pMbxTfer, dwSlaveId, achFileName, dwFileNameLen, dwPassword, dwTimeout);
} EC_INLINESTOP
#endif

EC_API EC_T_DWORD EC_API_FNCALL emFoeSegmentedUploadReq(EC_T_DWORD        dwInstanceID,
                                                    EC_T_MBXTFER*           pMbxTfer,
                                                    EC_T_DWORD              dwSlaveId,
                                                    const EC_T_CHAR*        szFileName,
                                                    EC_T_DWORD              dwFileNameLen,
                                                    EC_T_DWORD              dwFileSize,
                                                    EC_T_DWORD              dwPassword,
                                                    EC_T_DWORD              dwTimeout       );
#ifndef ECAT_INTERFACE
static EC_INLINESTART  EC_T_DWORD  ecatFoeSegmentedUploadReq(EC_T_MBXTFER* pMbxTfer,
                                                    EC_T_DWORD              dwSlaveId,
                                                    const EC_T_CHAR*        szFileName,
                                                    EC_T_DWORD              dwFileNameLen,
                                                    EC_T_DWORD              dwFileSize,
                                                    EC_T_DWORD              dwPassword,
                                                    EC_T_DWORD              dwTimeout       )
{
    return emFoeSegmentedUploadReq(INSTANCE_MASTER_DEFAULT, pMbxTfer, dwSlaveId, szFileName, dwFileNameLen, dwFileSize, dwPassword, dwTimeout);
} EC_INLINESTOP
#endif

EC_API EC_T_DWORD EC_API_FNCALL emFoeDownloadReq(EC_T_DWORD              dwInstanceID,
                                                    EC_T_MBXTFER*           pMbxTfer,
                                                    EC_T_DWORD              dwSlaveId,
                                                    const EC_T_CHAR*        achFileName,
                                                    EC_T_DWORD              dwFileNameLen,
                                                    EC_T_DWORD              dwPassword,
                                                    EC_T_DWORD              dwTimeout       );

#ifndef ECAT_INTERFACE
static EC_INLINESTART  EC_T_DWORD  ecatFoeDownloadReq(EC_T_MBXTFER*         pMbxTfer,
                                                    EC_T_DWORD              dwSlaveId,
                                                    const EC_T_CHAR*        achFileName,
                                                    EC_T_DWORD              dwFileNameLen,
                                                    EC_T_DWORD              dwPassword,
                                                    EC_T_DWORD              dwTimeout       )
{
    return emFoeDownloadReq(INSTANCE_MASTER_DEFAULT, pMbxTfer, dwSlaveId, achFileName, dwFileNameLen, dwPassword, dwTimeout);
} EC_INLINESTOP
#endif

EC_API EC_T_DWORD EC_API_FNCALL emFoeSegmentedDownloadReq(EC_T_DWORD     dwInstanceID,
                                                    EC_T_MBXTFER*           pMbxTfer,
                                                    EC_T_DWORD              dwSlaveId,
                                                    const EC_T_CHAR*        szFileName,
                                                    EC_T_DWORD              dwFileNameLen,
                                                    EC_T_DWORD              dwFileSize,
                                                    EC_T_DWORD              dwPassword,
                                                    EC_T_DWORD              dwTimeout       );
#ifndef ECAT_INTERFACE
static EC_INLINESTART  EC_T_DWORD  ecatFoeSegmentedDownloadReq(EC_T_MBXTFER* pMbxTfer,
                                                    EC_T_DWORD              dwSlaveId,
                                                    const EC_T_CHAR*        szFileName,
                                                    EC_T_DWORD              dwFileNameLen,
                                                    EC_T_DWORD              dwFileSize,
                                                    EC_T_DWORD              dwPassword,
                                                    EC_T_DWORD              dwTimeout       )
{
    return emFoeSegmentedDownloadReq(INSTANCE_MASTER_DEFAULT, pMbxTfer, dwSlaveId, szFileName, dwFileNameLen, dwFileSize, dwPassword, dwTimeout);
} EC_INLINESTOP
#endif

EC_API EC_T_DWORD EC_API_FNCALL emEoeRegisterEndpoint(EC_T_DWORD         dwInstanceID,
                                                    const EC_T_CHAR*        szEoEDrvIdent,
                                                    EC_T_LINK_DRV_DESC*     pLinkDrvDesc    );
#ifndef ECAT_INTERFACE
static EC_INLINESTART  EC_T_DWORD  ecatEoeRegisterEndpoint(const EC_T_CHAR* szEoEDrvIdent,
                                                    EC_T_LINK_DRV_DESC*     pLinkDrvDesc    )
{
    return emEoeRegisterEndpoint(INSTANCE_MASTER_DEFAULT, szEoEDrvIdent, pLinkDrvDesc);
} EC_INLINESTOP
#endif

EC_API EC_T_DWORD EC_API_FNCALL emEoeUnregisterEndpoint(EC_T_DWORD       dwInstanceID,
                                                    EC_T_LINK_DRV_DESC*     pLinkDrvDesc    );
#ifndef ECAT_INTERFACE
static EC_INLINESTART  EC_T_DWORD  ecatEoeUnregisterEndpoint(EC_T_LINK_DRV_DESC* pLinkDrvDesc)
{
    return emEoeUnregisterEndpoint(INSTANCE_MASTER_DEFAULT, pLinkDrvDesc);
} EC_INLINESTOP
#endif

EC_API EC_T_DWORD EC_API_FNCALL emSoeWrite(      EC_T_DWORD              dwInstanceID,
                                                    EC_T_DWORD              dwSlaveId,
                                                    EC_T_BYTE               byDriveNo,
                                                    EC_T_BYTE*              pbyElementFlags,
                                                    EC_T_WORD               wIDN,
                                                    EC_T_BYTE*              pbyData,
                                                    EC_T_DWORD              dwDataLen,
                                                    EC_T_DWORD              dwTimeout        );
#ifndef ECAT_INTERFACE
static EC_INLINESTART   EC_T_DWORD ecatSoeWrite(    EC_T_DWORD              dwSlaveId,
                                                    EC_T_BYTE               byDriveNo,
                                                    EC_T_BYTE*              pbyElementFlags,
                                                    EC_T_WORD               wIDN,
                                                    EC_T_BYTE*              pbyData,
                                                    EC_T_DWORD              dwDataLen,
                                                    EC_T_DWORD              dwTimeout        )
{
    return emSoeWrite(INSTANCE_MASTER_DEFAULT, dwSlaveId, byDriveNo, pbyElementFlags, wIDN, pbyData, dwDataLen, dwTimeout);
} EC_INLINESTOP
#endif

EC_API EC_T_DWORD EC_API_FNCALL emSoeRead(       EC_T_DWORD              dwInstanceID,
                                                    EC_T_DWORD              dwSlaveId,
                                                    EC_T_BYTE               byDriveNo,
                                                    EC_T_BYTE*              pbyElementFlags,
                                                    EC_T_WORD               wIDN,
                                                    EC_T_BYTE*              pbyData,
                                                    EC_T_DWORD              dwDataLen,
                                                    EC_T_DWORD*             pdwOutDataLen,
                                                    EC_T_DWORD              dwTimeout        );
#ifndef ECAT_INTERFACE
static EC_INLINESTART   EC_T_DWORD ecatSoeRead(     EC_T_DWORD              dwSlaveId,
                                                    EC_T_BYTE               byDriveNo,
                                                    EC_T_BYTE*              pbyElementFlags,
                                                    EC_T_WORD               wIDN,
                                                    EC_T_BYTE*              pbyData,
                                                    EC_T_DWORD              dwDataLen,
                                                    EC_T_DWORD*             pdwOutDataLen,
                                                    EC_T_DWORD              dwTimeout        )
{
    return emSoeRead(INSTANCE_MASTER_DEFAULT, dwSlaveId, byDriveNo, pbyElementFlags, wIDN, pbyData, dwDataLen, pdwOutDataLen, dwTimeout);
} EC_INLINESTOP
#endif

EC_API EC_T_DWORD EC_API_FNCALL emSoeAbortProcCmd(EC_T_DWORD             dwInstanceID,
                                                    EC_T_DWORD              dwSlaveId,
                                                    EC_T_BYTE               byDriveNo,
                                                    EC_T_BYTE*              pbyElementFlags,
                                                    EC_T_WORD               wIDN,
                                                    EC_T_DWORD              dwTimeout        );
#ifndef ECAT_INTERFACE
static EC_INLINESTART   EC_T_DWORD ecatSoeAbortProcCmd(EC_T_DWORD           dwSlaveId,
                                                    EC_T_BYTE               byDriveNo,
                                                    EC_T_BYTE*              pbyElementFlags,
                                                    EC_T_WORD               wIDN,
                                                    EC_T_DWORD              dwTimeout        )
{
    return emSoeAbortProcCmd(INSTANCE_MASTER_DEFAULT, dwSlaveId, byDriveNo, pbyElementFlags, wIDN, dwTimeout);
} EC_INLINESTOP
#endif

EC_API EC_T_DWORD EC_API_FNCALL emSoeWriteReq(   EC_T_DWORD              dwInstanceID,
                                                    EC_T_MBXTFER*           pMbxTfer,
                                                    EC_T_DWORD              dwSlaveId,
                                                    EC_T_BYTE               byDriveNo,
                                                    EC_T_BYTE*              pbyElementFlags,
                                                    EC_T_WORD               wIDN,
                                                    EC_T_DWORD              dwTimeout        );
#ifndef ECAT_INTERFACE
static EC_INLINESTART   EC_T_DWORD ecatSoeWriteReq( EC_T_MBXTFER*           pMbxTfer,
                                                    EC_T_DWORD              dwSlaveId,
                                                    EC_T_BYTE               byDriveNo,
                                                    EC_T_BYTE*              pbyElementFlags,
                                                    EC_T_WORD               wIDN,
                                                    EC_T_DWORD              dwTimeout        )
{
    return emSoeWriteReq(INSTANCE_MASTER_DEFAULT, pMbxTfer, dwSlaveId, byDriveNo, pbyElementFlags, wIDN, dwTimeout);
} EC_INLINESTOP
#endif

EC_API EC_T_DWORD EC_API_FNCALL emSoeReadReq(    EC_T_DWORD              dwInstanceID,
                                                    EC_T_MBXTFER*           pMbxTfer,
                                                    EC_T_DWORD              dwSlaveId,
                                                    EC_T_BYTE               byDriveNo,
                                                    EC_T_BYTE*              pbyElementFlags,
                                                    EC_T_WORD               wIDN,
                                                    EC_T_DWORD              dwTimeout        );
#ifndef ECAT_INTERFACE
static EC_INLINESTART   EC_T_DWORD ecatSoeReadReq(  EC_T_MBXTFER*           pMbxTfer,
                                                    EC_T_DWORD              dwSlaveId,
                                                    EC_T_BYTE               byDriveNo,
                                                    EC_T_BYTE*              pbyElementFlags,
                                                    EC_T_WORD               wIDN,
                                                    EC_T_DWORD              dwTimeout        )
{
    return emSoeReadReq(INSTANCE_MASTER_DEFAULT, pMbxTfer, dwSlaveId, byDriveNo, pbyElementFlags, wIDN, dwTimeout);
} EC_INLINESTOP
#endif

EC_API EC_T_DWORD EC_API_FNCALL emAoeGetSlaveNetId(EC_T_DWORD             dwInstanceID,
                                                    EC_T_DWORD              dwSlaveId,
                                                    EC_T_AOE_NETID*         poAoeNetId       );
#ifndef ECAT_INTERFACE
static EC_INLINESTART  EC_T_DWORD ecatAoeGetSlaveNetId(EC_T_DWORD           dwSlaveId,
                                                    EC_T_AOE_NETID*         poAoeNetId       )
{
    return emAoeGetSlaveNetId(INSTANCE_MASTER_DEFAULT, dwSlaveId, poAoeNetId);
} EC_INLINESTOP
#endif

EC_API EC_T_DWORD EC_API_FNCALL emAoeRead(       EC_T_DWORD              dwInstanceID,
                                                    EC_T_DWORD              dwSlaveId,
                                                    EC_T_AOE_NETID*         poTargetNetId,
                                                    EC_T_WORD               wTargetPort,
                                                    EC_T_DWORD              dwIndexGroup,
                                                    EC_T_DWORD              dwIndexOffset,
                                                    EC_T_DWORD              dwDataLen,
                                                    EC_T_BYTE*              pbyData,
                                                    EC_T_DWORD*             pdwDataOutLen,
                                                    EC_T_DWORD*             pdwErrorCode,
                                                    EC_T_DWORD*             pdwCmdResult,
                                                    EC_T_DWORD              dwTimeout        );
#ifndef ECAT_INTERFACE
static EC_INLINESTART  EC_T_DWORD  ecatAoeRead(     EC_T_DWORD              dwSlaveId,
                                                    EC_T_AOE_NETID*         poTargetNetId,
                                                    EC_T_WORD               wTargetPort,
                                                    EC_T_DWORD              dwIndexGroup,
                                                    EC_T_DWORD              dwIndexOffset,
                                                    EC_T_DWORD              dwDataLen,
                                                    EC_T_BYTE*              pbyData,
                                                    EC_T_DWORD*             pdwDataOutLen,
                                                    EC_T_DWORD*             pdwErrorCode,
                                                    EC_T_DWORD*             pdwCmdResult,
                                                    EC_T_DWORD              dwTimeout        )
{
    return emAoeRead(INSTANCE_MASTER_DEFAULT, dwSlaveId, poTargetNetId, wTargetPort, dwIndexGroup, dwIndexOffset, dwDataLen, pbyData, pdwDataOutLen, pdwErrorCode, pdwCmdResult, dwTimeout);
} EC_INLINESTOP
#endif

EC_API EC_T_DWORD EC_API_FNCALL emAoeReadReq(    EC_T_DWORD              dwInstanceID,
                                                    EC_T_MBXTFER*           pMbxTfer,
                                                    EC_T_DWORD              dwSlaveId,
                                                    EC_T_AOE_NETID*         poTargetNetId,
                                                    EC_T_WORD               wTargetPort,
                                                    EC_T_DWORD              dwIndexGroup,
                                                    EC_T_DWORD              dwIndexOffset,
                                                    EC_T_DWORD              dwTimeout       );
#ifndef ECAT_INTERFACE
static EC_INLINESTART  EC_T_DWORD  ecatAoeReadReq(  EC_T_MBXTFER*           pMbxTfer,
                                                    EC_T_DWORD              dwSlaveId,
                                                    EC_T_AOE_NETID*         poTargetNetId,
                                                    EC_T_WORD               wTargetPort,
                                                    EC_T_DWORD              dwIndexGroup,
                                                    EC_T_DWORD              dwIndexOffset,
                                                    EC_T_DWORD              dwTimeout       )
{
    return emAoeReadReq(INSTANCE_MASTER_DEFAULT, pMbxTfer, dwSlaveId, poTargetNetId, wTargetPort, dwIndexGroup, dwIndexOffset, dwTimeout);
} EC_INLINESTOP
#endif

EC_API EC_T_DWORD EC_API_FNCALL emAoeWrite(      EC_T_DWORD              dwInstanceID,
                                                    EC_T_DWORD              dwSlaveId,
                                                    EC_T_AOE_NETID*         poTargetNetId,
                                                    EC_T_WORD               wTargetPort,
                                                    EC_T_DWORD              dwIndexGroup,
                                                    EC_T_DWORD              dwIndexOffset,
                                                    EC_T_DWORD              dwDataLen,
                                                    EC_T_BYTE*              pbyData,
                                                    EC_T_DWORD*             pdwErrorCode,
                                                    EC_T_DWORD*             pdwCmdResult,
                                                    EC_T_DWORD              dwTimeout       );
#ifndef ECAT_INTERFACE
static EC_INLINESTART  EC_T_DWORD  ecatAoeWrite(    EC_T_DWORD              dwSlaveId,
                                                    EC_T_AOE_NETID*         poTargetNetId,
                                                    EC_T_WORD               wTargetPort,
                                                    EC_T_DWORD              dwIndexGroup,
                                                    EC_T_DWORD              dwIndexOffset,
                                                    EC_T_DWORD              dwDataLen,
                                                    EC_T_BYTE*              pbyData,
                                                    EC_T_DWORD*             pdwErrorCode,
                                                    EC_T_DWORD*             pdwCmdResult,
                                                    EC_T_DWORD              dwTimeout       )
{
    return emAoeWrite(INSTANCE_MASTER_DEFAULT, dwSlaveId, poTargetNetId, wTargetPort, dwIndexGroup, dwIndexOffset, dwDataLen, pbyData, pdwErrorCode, pdwCmdResult, dwTimeout);
} EC_INLINESTOP
#endif

EC_API EC_T_DWORD EC_API_FNCALL emAoeWriteReq(    EC_T_DWORD              dwInstanceID,
                                                    EC_T_MBXTFER*           pMbxTfer,
                                                    EC_T_DWORD              dwSlaveId,
                                                    EC_T_AOE_NETID*         poTargetNetId,
                                                    EC_T_WORD               wTargetPort,
                                                    EC_T_DWORD              dwIndexGroup,
                                                    EC_T_DWORD              dwIndexOffset,
                                                    EC_T_DWORD              dwTimeout       );
#ifndef ECAT_INTERFACE
static EC_INLINESTART  EC_T_DWORD ecatAoeWriteReq(  EC_T_MBXTFER*           pMbxTfer,
                                                    EC_T_DWORD              dwSlaveId,
                                                    EC_T_AOE_NETID*         poTargetNetId,
                                                    EC_T_WORD               wTargetPort,
                                                    EC_T_DWORD              dwIndexGroup,
                                                    EC_T_DWORD              dwIndexOffset,
                                                    EC_T_DWORD              dwTimeout       )
{
    return emAoeWriteReq(INSTANCE_MASTER_DEFAULT, pMbxTfer, dwSlaveId, poTargetNetId, wTargetPort, dwIndexGroup, dwIndexOffset, dwTimeout);
} EC_INLINESTOP
#endif

EC_API EC_T_DWORD EC_API_FNCALL emAoeReadWrite(  EC_T_DWORD              dwInstanceID,
                                                    EC_T_DWORD              dwSlaveId,
                                                    EC_T_AOE_NETID*         poTargetNetId,
                                                    EC_T_WORD               wTargetPort,
                                                    EC_T_DWORD              dwIndexGroup,
                                                    EC_T_DWORD              dwIndexOffset,
                                                    EC_T_DWORD              dwReadDataLen,
                                                    EC_T_DWORD              dwWriteDataLen,
                                                    EC_T_BYTE*              pbyData,
                                                    EC_T_DWORD*             pdwDataOutLen,
                                                    EC_T_DWORD*             pdwErrorCode,
                                                    EC_T_DWORD*             pdwCmdResult,
                                                    EC_T_DWORD              dwTimeout        );
#ifndef ECAT_INTERFACE
static EC_INLINESTART  EC_T_DWORD  ecatAoeReadWrite(EC_T_DWORD              dwSlaveId,
                                                    EC_T_AOE_NETID*         poTargetNetId,
                                                    EC_T_WORD               wTargetPort,
                                                    EC_T_DWORD              dwIndexGroup,
                                                    EC_T_DWORD              dwIndexOffset,
                                                    EC_T_DWORD              dwReadDataLen,
                                                    EC_T_DWORD              dwWriteDataLen,
                                                    EC_T_BYTE*              pbyData,
                                                    EC_T_DWORD*             pdwDataOutLen,
                                                    EC_T_DWORD*             pdwErrorCode,
                                                    EC_T_DWORD*             pdwCmdResult,
                                                    EC_T_DWORD              dwTimeout        )
{
    return emAoeReadWrite(INSTANCE_MASTER_DEFAULT, dwSlaveId, poTargetNetId, wTargetPort, dwIndexGroup, dwIndexOffset,
        dwReadDataLen, dwWriteDataLen, pbyData, pdwDataOutLen, pdwErrorCode, pdwCmdResult, dwTimeout);
} EC_INLINESTOP
#endif

EC_API EC_T_DWORD EC_API_FNCALL emAoeWriteControl(EC_T_DWORD              dwInstanceID,
                                                    EC_T_DWORD              dwSlaveId,
                                                    EC_T_AOE_NETID*         poTargetNetId,
                                                    EC_T_WORD               wTargetPort,
                                                    EC_T_WORD               wAoEState,
                                                    EC_T_WORD               wDeviceState,
                                                    EC_T_DWORD              dwDataLen,
                                                    EC_T_BYTE*              pbyData,
                                                    EC_T_DWORD*             pdwErrorCode,
                                                    EC_T_DWORD*             pdwCmdResult,
                                                    EC_T_DWORD              dwTimeout       );
#ifndef ECAT_INTERFACE
static EC_INLINESTART  EC_T_DWORD  ecatAoeWriteControl(EC_T_DWORD           dwSlaveId,
                                                    EC_T_AOE_NETID*         poTargetNetId,
                                                    EC_T_WORD               wTargetPort,
                                                    EC_T_WORD               wAoEState,
                                                    EC_T_WORD               wDeviceState,
                                                    EC_T_DWORD              dwDataLen,
                                                    EC_T_BYTE*              pbyData,
                                                    EC_T_DWORD*             pdwErrorCode,
                                                    EC_T_DWORD*             pdwCmdResult,
                                                    EC_T_DWORD              dwTimeout       )
{
    return emAoeWriteControl(INSTANCE_MASTER_DEFAULT, dwSlaveId, poTargetNetId, wTargetPort, wAoEState, wDeviceState, dwDataLen, pbyData, pdwErrorCode, pdwCmdResult, dwTimeout);
} EC_INLINESTOP
#endif

EC_API EC_T_DWORD EC_API_FNCALL emVoeRead(       EC_T_DWORD              dwInstanceID,
                                                    EC_T_DWORD              dwSlaveId,
                                                    EC_T_BYTE*              pbyData,
                                                    EC_T_DWORD              dwDataLen,
                                                    EC_T_DWORD*             pdwOutDataLen,
                                                    EC_T_DWORD              dwTimeout       );
#ifndef ECAT_INTERFACE
static EC_INLINESTART  EC_T_DWORD  ecatVoeRead(          EC_T_DWORD         dwSlaveId,
                                                    EC_T_BYTE*              pbyData,
                                                    EC_T_DWORD              dwDataLen,
                                                    EC_T_DWORD*             pdwOutDataLen,
                                                    EC_T_DWORD              dwTimeout       )
{
    return emVoeRead(INSTANCE_MASTER_DEFAULT, dwSlaveId, pbyData, dwDataLen, pdwOutDataLen, dwTimeout);
} EC_INLINESTOP
#endif

EC_API EC_T_DWORD EC_API_FNCALL emVoeWrite(      EC_T_DWORD              dwInstanceID,
                                                    EC_T_DWORD              dwSlaveId,
                                                    EC_T_BYTE*              pbyData,
                                                    EC_T_DWORD              dwDataLen,
                                                    EC_T_DWORD              dwTimeout       );
#ifndef ECAT_INTERFACE
static EC_INLINESTART  EC_T_DWORD  ecatVoeWrite(    EC_T_DWORD              dwSlaveId,
                                                    EC_T_BYTE*              pbyData,
                                                    EC_T_DWORD              dwDataLen,
                                                    EC_T_DWORD              dwTimeout       )
{
    return emVoeWrite(INSTANCE_MASTER_DEFAULT, dwSlaveId, pbyData, dwDataLen, dwTimeout);
} EC_INLINESTOP
#endif

EC_API EC_T_DWORD EC_API_FNCALL emVoeWriteReq(   EC_T_DWORD              dwInstanceID,
                                                    EC_T_MBXTFER*           pMbxTfer,
                                                    EC_T_DWORD              dwSlaveId,
                                                    EC_T_DWORD              dwTimeout       );
#ifndef ECAT_INTERFACE
static EC_INLINESTART  EC_T_DWORD  ecatVoeWriteReq( EC_T_MBXTFER*           pMbxTfer,
                                                    EC_T_DWORD              dwSlaveId,
                                                    EC_T_DWORD              dwTimeout       )
{
    return emVoeWriteReq(INSTANCE_MASTER_DEFAULT, pMbxTfer, dwSlaveId, dwTimeout);
} EC_INLINESTOP
#endif

EC_API EC_T_DWORD EC_API_FNCALL emExecJob(       EC_T_DWORD              dwInstanceID,
                                                    EC_T_USER_JOB           eUserJob,
                                                    EC_T_USER_JOB_PARMS*    pUserJobParms   );
#ifndef ECAT_INTERFACE
static EC_INLINESTART  EC_T_DWORD  ecatExecJob(     EC_T_USER_JOB           eUserJob,
                                                    EC_T_USER_JOB_PARMS*    pUserJobParms   )
{
    return emExecJob(INSTANCE_MASTER_DEFAULT, eUserJob, pUserJobParms);
} EC_INLINESTOP
#endif

EC_API EC_T_DWORD EC_API_FNCALL emGetProcessData(EC_T_DWORD              dwInstanceID,
                                                    EC_T_BOOL               bOutputData,
                                                    EC_T_DWORD              dwOffset,
                                                    EC_T_BYTE*              pbyData,
                                                    EC_T_DWORD              dwLength,
                                                    EC_T_DWORD              dwTimeout       );
#ifndef ECAT_INTERFACE
static EC_INLINESTART  EC_T_DWORD  ecatGetProcessData(EC_T_BOOL             bOutputData,
                                                    EC_T_DWORD              dwOffset,
                                                    EC_T_BYTE*              pbyData,
                                                    EC_T_DWORD              dwLength,
                                                    EC_T_DWORD              dwTimeout       )
{
    return emGetProcessData(INSTANCE_MASTER_DEFAULT, bOutputData, dwOffset, pbyData, dwLength, dwTimeout);
} EC_INLINESTOP
#endif

EC_API EC_T_DWORD EC_API_FNCALL emSetProcessData(EC_T_DWORD              dwInstanceID,
                                                    EC_T_BOOL               bOutputData,
                                                    EC_T_DWORD              dwOffset,
                                                    EC_T_BYTE*              pbyData,
                                                    EC_T_DWORD              dwLength,
                                                    EC_T_DWORD              dwTimeout       );
#ifndef ECAT_INTERFACE
static EC_INLINESTART  EC_T_DWORD  ecatSetProcessData(EC_T_BOOL             bOutputData,
                                                    EC_T_DWORD              dwOffset,
                                                    EC_T_BYTE*              pbyData,
                                                    EC_T_DWORD              dwLength,
                                                    EC_T_DWORD              dwTimeout       )
{
    return emSetProcessData(INSTANCE_MASTER_DEFAULT, bOutputData, dwOffset, pbyData, dwLength, dwTimeout);
} EC_INLINESTOP
#endif

EC_API EC_T_DWORD EC_API_FNCALL emSetProcessDataBits(EC_T_DWORD           dwInstanceID,
                                                    EC_T_BOOL               bOutputData,
                                                    EC_T_DWORD              dwBitOffsetPd,
                                                    EC_T_BYTE*              pbyDataSrc,
                                                    EC_T_DWORD              dwBitLengthSrc,
                                                    EC_T_DWORD              dwTimeout       );
#ifndef ECAT_INTERFACE
static EC_INLINESTART  EC_T_DWORD ecatSetProcessDataBits(EC_T_BOOL          bOutputData,
                                                    EC_T_DWORD              dwBitOffsetPd,
                                                    EC_T_BYTE*              pbyDataSrc,
                                                    EC_T_DWORD              dwBitLengthSrc,
                                                    EC_T_DWORD              dwTimeout       )
{
    return emSetProcessDataBits(INSTANCE_MASTER_DEFAULT, bOutputData, dwBitOffsetPd, pbyDataSrc, dwBitLengthSrc, dwTimeout);
} EC_INLINESTOP
#endif

EC_API EC_T_DWORD EC_API_FNCALL emGetProcessDataBits(EC_T_DWORD           dwInstanceID,
                                                    EC_T_BOOL               bOutputData,
                                                    EC_T_DWORD              dwBitOffsetPd,
                                                    EC_T_BYTE*              pbyDataDst,
                                                    EC_T_DWORD              dwBitLengthDst,
                                                    EC_T_DWORD              dwTimeout       );
#ifndef ECAT_INTERFACE
static EC_INLINESTART  EC_T_DWORD ecatGetProcessDataBits(EC_T_BOOL          bOutputData,
                                                    EC_T_DWORD              dwBitOffsetPd,
                                                    EC_T_BYTE*              pbyDataDst,
                                                    EC_T_DWORD              dwBitLengthDst,
                                                    EC_T_DWORD              dwTimeout       )
{
    return emGetProcessDataBits(INSTANCE_MASTER_DEFAULT, bOutputData, dwBitOffsetPd, pbyDataDst, dwBitLengthDst, dwTimeout);
} EC_INLINESTOP
#endif

EC_API EC_T_DWORD EC_API_FNCALL emForceProcessDataBits(EC_T_DWORD         dwInstanceID,
                                                    EC_T_DWORD              dwClientId,
                                                    EC_T_BOOL               bOutputData,
                                                    EC_T_DWORD              dwBitOffsetPd,
                                                    EC_T_WORD               wBitLength,
                                                    EC_T_BYTE*              pbyData,
                                                    EC_T_DWORD              dwTimeout);
#ifndef ECAT_INTERFACE
static EC_INLINESTART  EC_T_DWORD ecatForceProcessDataBits(EC_T_DWORD       dwClientId,
                                                    EC_T_BOOL               bOutputData,
                                                    EC_T_DWORD              dwBitOffsetPd,
                                                    EC_T_WORD               wBitLength,
                                                    EC_T_BYTE*              pbyData,
                                                    EC_T_DWORD              dwTimeout)
{
    return emForceProcessDataBits(INSTANCE_MASTER_DEFAULT, dwClientId, bOutputData, dwBitOffsetPd, wBitLength, pbyData, dwTimeout);
} EC_INLINESTOP
#endif

EC_API EC_T_DWORD EC_API_FNCALL emReleaseProcessDataBits(EC_T_DWORD       dwInstanceID,
                                                    EC_T_DWORD              dwClientId,
                                                    EC_T_BOOL               bOutputData,
                                                    EC_T_DWORD              dwBitOffsetPd,
                                                    EC_T_WORD               wBitLength,
                                                    EC_T_DWORD              dwTimeout       );
#ifndef ECAT_INTERFACE
static EC_INLINESTART  EC_T_DWORD ecatReleaseProcessDataBits(EC_T_DWORD     dwClientId,
                                                    EC_T_BOOL               bOutputData,
                                                    EC_T_DWORD              dwBitOffsetPd,
                                                    EC_T_WORD               wBitLength,
                                                    EC_T_DWORD              dwTimeout       )
{
    return emReleaseProcessDataBits(INSTANCE_MASTER_DEFAULT, dwClientId, bOutputData, dwBitOffsetPd, wBitLength, dwTimeout);
} EC_INLINESTOP
#endif

EC_API EC_T_DWORD EC_API_FNCALL emReleaseAllProcessDataBits(EC_T_DWORD       dwInstanceID,
                                                    EC_T_DWORD              dwClientId,
                                                    EC_T_DWORD              dwTimeout       );
#ifndef ECAT_INTERFACE
static EC_INLINESTART  EC_T_DWORD ecatReleaseAllProcessDataBits(EC_T_DWORD  dwClientId,
                                                    EC_T_DWORD              dwTimeout       )
{
    return emReleaseAllProcessDataBits(INSTANCE_MASTER_DEFAULT, dwClientId, dwTimeout);
} EC_INLINESTOP
#endif

EC_API EC_T_DWORD EC_API_FNCALL emGetNumConnectedSlaves(EC_T_DWORD       dwInstanceID    );
#ifndef ECAT_INTERFACE
static EC_INLINESTART  EC_T_DWORD  ecatGetNumConnectedSlaves(EC_T_VOID                      )

{
    return emGetNumConnectedSlaves(INSTANCE_MASTER_DEFAULT);
} EC_INLINESTOP
#endif

EC_API EC_T_DWORD EC_API_FNCALL emGetNumConnectedSlavesMain(EC_T_DWORD   dwInstanceID    );
#ifndef ECAT_INTERFACE
static EC_INLINESTART  EC_T_DWORD  ecatGetNumConnectedSlavesMain(EC_T_VOID                  )
{
    return emGetNumConnectedSlavesMain(INSTANCE_MASTER_DEFAULT);
} EC_INLINESTOP
#endif

EC_API EC_T_DWORD EC_API_FNCALL emGetNumConnectedSlavesRed(EC_T_DWORD    dwInstanceID    );
#ifndef ECAT_INTERFACE
static EC_INLINESTART  EC_T_DWORD  ecatGetNumConnectedSlavesRed(EC_T_VOID                   )
{
    return emGetNumConnectedSlavesRed(INSTANCE_MASTER_DEFAULT);
} EC_INLINESTOP
#endif

EC_API EC_T_DWORD EC_API_FNCALL emReadSlaveEEPRom(EC_T_DWORD             dwInstanceID,
                                                    EC_T_BOOL               bFixedAddressing,
                                                    EC_T_WORD               wSlaveAddress,
                                                    EC_T_WORD               wEEPRomStartOffset,
                                                    EC_T_WORD*              pwReadData,
                                                    EC_T_DWORD              dwReadLen,
                                                    EC_T_DWORD*             pdwNumOutData,
                                                    EC_T_DWORD              dwTimeout       );
#ifndef ECAT_INTERFACE
static EC_INLINESTART  EC_T_DWORD  ecatReadSlaveEEPRom(EC_T_BOOL            bFixedAddressing,
                                                    EC_T_WORD               wSlaveAddress,
                                                    EC_T_WORD               wEEPRomStartOffset,
                                                    EC_T_WORD*              pwReadData,
                                                    EC_T_DWORD              dwReadLen,
                                                    EC_T_DWORD*             pdwNumOutData,
                                                    EC_T_DWORD              dwTimeout       )
{
    return emReadSlaveEEPRom(INSTANCE_MASTER_DEFAULT, bFixedAddressing, wSlaveAddress, wEEPRomStartOffset, pwReadData, dwReadLen, pdwNumOutData, dwTimeout);
} EC_INLINESTOP
#endif

EC_API EC_T_DWORD EC_API_FNCALL emReadSlaveEEPRomReq(EC_T_DWORD          dwInstanceID,
                                                    EC_T_DWORD              dwClientId,
                                                    EC_T_DWORD              dwTferId,
                                                    EC_T_BOOL               bFixedAddressing,
                                                    EC_T_WORD               wSlaveAddress,
                                                    EC_T_WORD               wEEPRomStartOffset,
                                                    EC_T_WORD*              pwReadData,
                                                    EC_T_DWORD              dwReadLen,
                                                    EC_T_DWORD*             pdwNumOutData,
                                                    EC_T_DWORD              dwTimeout      );
#ifndef ECAT_INTERFACE
static EC_INLINESTART  EC_T_DWORD  ecatReadSlaveEEPRomReq(EC_T_DWORD        dwClientId,
                                                    EC_T_DWORD              dwTferId,
                                                    EC_T_BOOL               bFixedAddressing,
                                                    EC_T_WORD               wSlaveAddress,
                                                    EC_T_WORD               wEEPRomStartOffset,
                                                    EC_T_WORD*              pwReadData,
                                                    EC_T_DWORD              dwReadLen,
                                                    EC_T_DWORD*             pdwNumOutData,
                                                    EC_T_DWORD              dwTimeout      )
{
    return emReadSlaveEEPRomReq(INSTANCE_MASTER_DEFAULT, dwClientId, dwTferId, bFixedAddressing, wSlaveAddress, wEEPRomStartOffset, pwReadData, dwReadLen, pdwNumOutData, dwTimeout);
} EC_INLINESTOP
#endif

EC_API EC_T_DWORD EC_API_FNCALL emWriteSlaveEEPRom(EC_T_DWORD            dwInstanceID,
                                                    EC_T_BOOL               bFixedAddressing,
                                                    EC_T_WORD               wSlaveAddress,
                                                    EC_T_WORD               wEEPRomStartOffset,
                                                    EC_T_WORD*              pwWriteData,
                                                    EC_T_DWORD              dwWriteLen,
                                                    EC_T_DWORD              dwTimeout       );
#ifndef ECAT_INTERFACE
static EC_INLINESTART  EC_T_DWORD ecatWriteSlaveEEPRom(EC_T_BOOL            bFixedAddressing,
                                                    EC_T_WORD               wSlaveAddress,
                                                    EC_T_WORD               wEEPRomStartOffset,
                                                    EC_T_WORD*              pwWriteData,
                                                    EC_T_DWORD              dwWriteLen,
                                                    EC_T_DWORD              dwTimeout       )
{
    return emWriteSlaveEEPRom(INSTANCE_MASTER_DEFAULT, bFixedAddressing, wSlaveAddress, wEEPRomStartOffset, pwWriteData, dwWriteLen, dwTimeout);
} EC_INLINESTOP
#endif

EC_API EC_T_DWORD EC_API_FNCALL emWriteSlaveEEPRomReq(EC_T_DWORD         dwInstanceID,
                                                    EC_T_DWORD              dwClientId,
                                                    EC_T_DWORD              dwTferId,
                                                    EC_T_BOOL               bFixedAddressing,
                                                    EC_T_WORD               wSlaveAddress,
                                                    EC_T_WORD               wEEPRomStartOffset,
                                                    EC_T_WORD*              pwWriteData,
                                                    EC_T_DWORD              dwWriteLen,
                                                    EC_T_DWORD              dwTimeout       );
#ifndef ECAT_INTERFACE
static EC_INLINESTART  EC_T_DWORD ecatWriteSlaveEEPRomReq(EC_T_DWORD        dwClientId,
                                                    EC_T_DWORD              dwTferId,
                                                    EC_T_BOOL               bFixedAddressing,
                                                    EC_T_WORD               wSlaveAddress,
                                                    EC_T_WORD               wEEPRomStartOffset,
                                                    EC_T_WORD*              pwWriteData,
                                                    EC_T_DWORD              dwWriteLen,
                                                    EC_T_DWORD              dwTimeout)
{
    return emWriteSlaveEEPRomReq(INSTANCE_MASTER_DEFAULT, dwClientId, dwTferId, bFixedAddressing, wSlaveAddress, wEEPRomStartOffset, pwWriteData, dwWriteLen, dwTimeout);
} EC_INLINESTOP
#endif

EC_API EC_T_DWORD EC_API_FNCALL emReloadSlaveEEPRom(EC_T_DWORD            dwInstanceID,
                                                    EC_T_BOOL               bFixedAddressing,
                                                    EC_T_WORD               wSlaveAddress,
                                                    EC_T_DWORD              dwTimeout       );
#ifndef ECAT_INTERFACE
static EC_INLINESTART  EC_T_DWORD ecatReloadSlaveEEPRom(EC_T_BOOL           bFixedAddressing,
                                                    EC_T_WORD               wSlaveAddress,
                                                    EC_T_DWORD              dwTimeout       )
{
    return emReloadSlaveEEPRom(INSTANCE_MASTER_DEFAULT, bFixedAddressing, wSlaveAddress, dwTimeout);
} EC_INLINESTOP
#endif

EC_API EC_T_DWORD EC_API_FNCALL emReloadSlaveEEPRomReq(EC_T_DWORD         dwInstanceID,
                                                    EC_T_DWORD              dwClientId,
                                                    EC_T_DWORD              dwTferId,
                                                    EC_T_BOOL               bFixedAddressing,
                                                    EC_T_WORD               wSlaveAddress,
                                                    EC_T_DWORD              dwTimeout       );
#ifndef ECAT_INTERFACE
static EC_INLINESTART  EC_T_DWORD ecatReloadSlaveEEPRomReq(EC_T_DWORD       dwClientId,
                                                    EC_T_DWORD              dwTferId,
                                                    EC_T_BOOL               bFixedAddressing,
                                                    EC_T_WORD               wSlaveAddress,
                                                    EC_T_DWORD              dwTimeout       )
{
    return emReloadSlaveEEPRomReq(INSTANCE_MASTER_DEFAULT, dwClientId, dwTferId, bFixedAddressing, wSlaveAddress, dwTimeout);
} EC_INLINESTOP
#endif

EC_API EC_T_DWORD EC_API_FNCALL emResetSlaveController(EC_T_DWORD         dwInstanceID,
                                                    EC_T_BOOL               bFixedAddressing,
                                                    EC_T_WORD               wSlaveAddress,
                                                    EC_T_DWORD              dwTimeout       );
#ifndef ECAT_INTERFACE
static EC_INLINESTART  EC_T_DWORD ecatResetSlaveController(EC_T_BOOL        bFixedAddressing,
                                                    EC_T_WORD               wSlaveAddress,
                                                    EC_T_DWORD              dwTimeout       )
{
    return emResetSlaveController(INSTANCE_MASTER_DEFAULT, bFixedAddressing, wSlaveAddress, dwTimeout);
} EC_INLINESTOP
#endif

EC_API EC_T_DWORD EC_API_FNCALL emAssignSlaveEEPRom(EC_T_DWORD            dwInstanceID,
                                                    EC_T_BOOL               bFixedAddressing,
                                                    EC_T_WORD               wSlaveAddress,
                                                    EC_T_BOOL               bSlavePDIAccessEnable,
                                                    EC_T_BOOL               bForceAssign,
                                                    EC_T_DWORD              dwTimeout       );
#ifndef ECAT_INTERFACE
static EC_INLINESTART  EC_T_DWORD ecatAssignSlaveEEPRom(EC_T_BOOL           bFixedAddressing,
                                                    EC_T_WORD               wSlaveAddress,
                                                    EC_T_BOOL               bSlavePDIAccessEnable,
                                                    EC_T_BOOL               bForceAssign,
                                                    EC_T_DWORD              dwTimeout       )
{
    return emAssignSlaveEEPRom(INSTANCE_MASTER_DEFAULT, bFixedAddressing, wSlaveAddress, bSlavePDIAccessEnable, bForceAssign, dwTimeout);
} EC_INLINESTOP
#endif

EC_API EC_T_DWORD EC_API_FNCALL emAssignSlaveEEPRomReq(EC_T_DWORD         dwInstanceID,
                                                    EC_T_DWORD              dwClientId,
                                                    EC_T_DWORD              dwTferId,
                                                    EC_T_BOOL               bFixedAddressing,
                                                    EC_T_WORD               wSlaveAddress,
                                                    EC_T_BOOL               bSlavePDIAccessEnable,
                                                    EC_T_BOOL               bForceAssign,
                                                    EC_T_DWORD              dwTimeout       );
#ifndef ECAT_INTERFACE
static EC_INLINESTART  EC_T_DWORD ecatAssignSlaveEEPRomReq(EC_T_DWORD       dwClientId,
                                                    EC_T_DWORD              dwTferId,
                                                    EC_T_BOOL               bFixedAddressing,
                                                    EC_T_WORD               wSlaveAddress,
                                                    EC_T_BOOL               bSlavePDIAccessEnable,
                                                    EC_T_BOOL               bForceAssign,
                                                    EC_T_DWORD              dwTimeout       )
{
    return emAssignSlaveEEPRomReq(INSTANCE_MASTER_DEFAULT, dwClientId, dwTferId,bFixedAddressing, wSlaveAddress, bSlavePDIAccessEnable, bForceAssign, dwTimeout);
} EC_INLINESTOP
#endif

EC_API EC_T_DWORD EC_API_FNCALL emActiveSlaveEEPRom(EC_T_DWORD            dwInstanceID,
                                                    EC_T_BOOL               bFixedAddressing,
                                                    EC_T_WORD               wSlaveAddress,
                                                    EC_T_BOOL*              pbSlavePDIAccessActive,
                                                    EC_T_DWORD              dwTimeout       );
#ifndef ECAT_INTERFACE
static EC_INLINESTART  EC_T_DWORD ecatActiveSlaveEEPRom(EC_T_BOOL           bFixedAddressing,
                                                    EC_T_WORD               wSlaveAddress,
                                                    EC_T_BOOL*              pbSlavePDIAccessActive,
                                                    EC_T_DWORD              dwTimeout       )
{
    return emActiveSlaveEEPRom(INSTANCE_MASTER_DEFAULT, bFixedAddressing, wSlaveAddress, pbSlavePDIAccessActive, dwTimeout);
} EC_INLINESTOP
#endif

EC_API EC_T_DWORD EC_API_FNCALL emActiveSlaveEEPRomReq(EC_T_DWORD         dwInstanceID,
                                                    EC_T_DWORD              dwClientId,
                                                    EC_T_DWORD              dwTferId,
                                                    EC_T_BOOL               bFixedAddressing,
                                                    EC_T_WORD               wSlaveAddress,
                                                    EC_T_BOOL*              pbSlavePDIAccessActive,
                                                    EC_T_DWORD              dwTimeout      );
#ifndef ECAT_INTERFACE
static EC_INLINESTART  EC_T_DWORD ecatActiveSlaveEEPRomReq(EC_T_DWORD       dwClientId,
                                                    EC_T_DWORD              dwTferId,
                                                    EC_T_BOOL               bFixedAddressing,
                                                    EC_T_WORD               wSlaveAddress,
                                                    EC_T_BOOL*              pbSlavePDIAccessActive,
                                                    EC_T_DWORD              dwTimeout      )
{
    return emActiveSlaveEEPRomReq(INSTANCE_MASTER_DEFAULT, dwClientId, dwTferId, bFixedAddressing, wSlaveAddress, pbSlavePDIAccessActive, dwTimeout);
} EC_INLINESTOP
#endif

EC_API EC_T_DWORD EC_API_FNCALL emHCAcceptTopoChange(EC_T_DWORD           dwInstanceID    );
#ifndef ECAT_INTERFACE
static EC_INLINESTART  EC_T_DWORD ecatHCAcceptTopoChange(EC_T_VOID                          )
{
    return emHCAcceptTopoChange(INSTANCE_MASTER_DEFAULT);
} EC_INLINESTOP
#endif

EC_API EC_T_DWORD EC_API_FNCALL emHCGetNumGroupMembers(EC_T_DWORD         dwInstanceID,
                                                    EC_T_DWORD              dwGroupIndex    );
#ifndef ECAT_INTERFACE
static EC_INLINESTART  EC_T_DWORD ecatHCGetNumGroupMembers(EC_T_DWORD       dwGroupIndex    )
{
    return emHCGetNumGroupMembers( INSTANCE_MASTER_DEFAULT, dwGroupIndex );
} EC_INLINESTOP
#endif

EC_API EC_T_DWORD EC_API_FNCALL emHCGetSlaveIdsOfGroup(EC_T_DWORD         dwInstanceID,
                                                    EC_T_DWORD              dwGroupIndex,
                                                    EC_T_DWORD*             adwSlaveId,
                                                    EC_T_DWORD              dwMaxNumSlaveIds );
#ifndef ECAT_INTERFACE
static EC_INLINESTART  EC_T_DWORD ecatHCGetSlaveIdsOfGroup(EC_T_DWORD       dwGroupIndex,
                                                    EC_T_DWORD*             adwSlaveId,
                                                    EC_T_DWORD              dwMaxNumSlaveIds )
{
    return emHCGetSlaveIdsOfGroup( INSTANCE_MASTER_DEFAULT, dwGroupIndex, adwSlaveId, dwMaxNumSlaveIds );
} EC_INLINESTOP
#endif

EC_API EC_T_DWORD EC_API_FNCALL emSetSlavePortState(EC_T_DWORD            dwInstanceID,
                                                    EC_T_DWORD              dwSlaveId,
                                                    EC_T_WORD               wPort,
                                                    EC_T_BOOL               bClose,
                                                    EC_T_BOOL               bForce,
                                                    EC_T_DWORD              dwTimeout       );
#ifndef ECAT_INTERFACE
static EC_INLINESTART  EC_T_DWORD ecatSetSlavePortState(EC_T_DWORD          dwSlaveId,
                                                    EC_T_WORD               wPort,
                                                    EC_T_BOOL               bClose,
                                                    EC_T_BOOL               bForce,
                                                    EC_T_DWORD              dwTimeout       )
{
    return emSetSlavePortState(INSTANCE_MASTER_DEFAULT, dwSlaveId, wPort, bClose, bForce, dwTimeout);
} EC_INLINESTOP
#endif

EC_API EC_T_DWORD EC_API_FNCALL emSetSlavePortStateReq(EC_T_DWORD         dwInstanceID,
                                                    EC_T_DWORD              dwClientId,
                                                    EC_T_DWORD              dwTferId,
                                                    EC_T_DWORD              dwSlaveId,
                                                    EC_T_WORD               wPort,
                                                    EC_T_BOOL               bClose,
                                                    EC_T_BOOL               bForce,
                                                    EC_T_DWORD              dwTimeout       );
#ifndef ECAT_INTERFACE
static EC_INLINESTART  EC_T_DWORD ecatSetSlavePortStateReq(EC_T_DWORD       dwClientId,
                                                    EC_T_DWORD              dwTferId,
                                                    EC_T_DWORD              dwSlaveId,
                                                    EC_T_WORD               wPort,
                                                    EC_T_BOOL               bClose,
                                                    EC_T_BOOL               bForce,
                                                    EC_T_DWORD              dwTimeout       )
{
    return emSetSlavePortStateReq(INSTANCE_MASTER_DEFAULT, dwClientId, dwTferId, dwSlaveId, wPort, bClose, bForce, dwTimeout);
} EC_INLINESTOP
#endif

EC_API EC_T_DWORD EC_API_FNCALL emSlaveSerializeMbxTfers(EC_T_DWORD      dwInstanceID,
                                                    EC_T_DWORD              dwSlaveId       );
#ifndef ECAT_INTERFACE
static EC_INLINESTART  EC_T_DWORD  ecatSlaveSerializeMbxTfers(EC_T_DWORD    dwSlaveId       )
{
    return emSlaveSerializeMbxTfers(INSTANCE_MASTER_DEFAULT, dwSlaveId);
} EC_INLINESTOP
#endif

EC_API EC_T_DWORD EC_API_FNCALL emSlaveParallelMbxTfers(EC_T_DWORD       dwInstanceID,
                                                    EC_T_DWORD              dwSlaveId       );
#ifndef ECAT_INTERFACE
static EC_INLINESTART  EC_T_DWORD  ecatSlaveParallelMbxTfers(EC_T_DWORD     dwSlaveId       )
{
    return emSlaveParallelMbxTfers(INSTANCE_MASTER_DEFAULT, dwSlaveId);
} EC_INLINESTOP
#endif

EC_API EC_T_DWORD EC_API_FNCALL emRegisterClient( EC_T_DWORD              dwInstanceID,
                                                    EC_PF_NOTIFY            pfnNotify,
                                                    EC_T_VOID*              pCallerData,
                                                    EC_T_REGISTERRESULTS*   pRegResults     );
#ifndef ECAT_INTERFACE
static EC_INLINESTART  EC_T_DWORD ecatRegisterClient(EC_PF_NOTIFY           pfnNotify,
                                                    EC_T_VOID*              pCallerData,
                                                    EC_T_REGISTERRESULTS*   pRegResults     )
{
    return emRegisterClient(INSTANCE_MASTER_DEFAULT, pfnNotify, pCallerData, pRegResults);
} EC_INLINESTOP
#endif

EC_API EC_T_DWORD EC_API_FNCALL emUnregisterClient(EC_T_DWORD             dwInstanceID,
                                                    EC_T_DWORD              dwClntId        );
#ifndef ECAT_INTERFACE
static EC_INLINESTART  EC_T_DWORD ecatUnregisterClient(EC_T_DWORD           dwClntId        )
{
    return emUnregisterClient(INSTANCE_MASTER_DEFAULT, dwClntId);
} EC_INLINESTOP
#endif

EC_API EC_T_DWORD EC_API_FNCALL emDcEnable(       EC_T_DWORD              dwInstanceID    );
#ifndef ECAT_INTERFACE
static EC_INLINESTART  EC_T_DWORD ecatDcEnable(     EC_T_VOID                               )
{
    return emDcEnable(INSTANCE_MASTER_DEFAULT);
} EC_INLINESTOP
#endif

EC_API EC_T_DWORD EC_API_FNCALL emDcDisable(      EC_T_DWORD              dwInstanceID    );
#ifndef ECAT_INTERFACE
static EC_INLINESTART  EC_T_DWORD ecatDcDisable(    EC_T_VOID                               )
{
    return emDcDisable(INSTANCE_MASTER_DEFAULT);
} EC_INLINESTOP
#endif

EC_API EC_T_DWORD EC_API_FNCALL emDcIsEnabled(    EC_T_DWORD              dwInstanceID,
                                                    EC_T_BOOL*              pbDcIsEnabled);
#ifndef ECAT_INTERFACE
static EC_INLINESTART  EC_T_DWORD ecatDcIsEnabled(  EC_T_BOOL*           pbDcIsEnabled)
{
    return emDcIsEnabled(INSTANCE_MASTER_DEFAULT, pbDcIsEnabled);
} EC_INLINESTOP
#endif

EC_API EC_T_DWORD EC_API_FNCALL emDcConfigure(    EC_T_DWORD              dwInstanceID,
                                                    struct _EC_T_DC_CONFIGURE* pDcConfigure );
#ifndef ECAT_INTERFACE
static EC_INLINESTART  EC_T_DWORD ecatDcConfigure(  struct _EC_T_DC_CONFIGURE* pDcConfigure )
{
    return emDcConfigure(INSTANCE_MASTER_DEFAULT, pDcConfigure);
} EC_INLINESTOP
#endif

EC_API EC_T_DWORD EC_API_FNCALL emDcContDelayCompEnable(EC_T_DWORD        dwInstanceID    );
#ifndef ECAT_INTERFACE
static EC_INLINESTART  EC_T_DWORD ecatDcContDelayCompEnable(EC_T_VOID                       )
{
    return emDcContDelayCompEnable(INSTANCE_MASTER_DEFAULT);
} EC_INLINESTOP
#endif

EC_API EC_T_DWORD EC_API_FNCALL emDcContDelayCompDisable(EC_T_DWORD       dwInstanceID    );
#ifndef ECAT_INTERFACE
static EC_INLINESTART  EC_T_DWORD ecatDcContDelayCompDisable(EC_T_VOID                      )
{
    return emDcContDelayCompDisable(INSTANCE_MASTER_DEFAULT);
} EC_INLINESTOP
#endif

EC_API EC_T_DWORD EC_API_FNCALL emDcmConfigure(   EC_T_DWORD              dwInstanceID,
                                                    struct _EC_T_DCM_CONFIG* pDcmConfig,
                                                    EC_T_DWORD              dwInSyncTimeout );
#ifndef ECAT_INTERFACE
static EC_INLINESTART  EC_T_DWORD ecatDcmConfigure(struct _EC_T_DCM_CONFIG* pDcmConfig,
                                                    EC_T_DWORD              dwInSyncTimeout )
{
    return emDcmConfigure(INSTANCE_MASTER_DEFAULT, pDcmConfig, dwInSyncTimeout);
} EC_INLINESTOP
#endif

EC_API EC_T_DWORD EC_API_FNCALL emDcmGetStatus(   EC_T_DWORD              dwInstanceID,
                                                    EC_T_DWORD*             pdwErrorCode,
                                                    EC_T_INT*               pnDiffCur,
                                                    EC_T_INT*               pnDiffAvg,
                                                    EC_T_INT*               pnDiffMax       );
#ifndef ECAT_INTERFACE
static EC_INLINESTART  EC_T_DWORD ecatDcmGetStatus( EC_T_DWORD*             pdwErrorCode,
                                                    EC_T_INT*               pnDiffCur,
                                                    EC_T_INT*               pnDiffAvg,
                                                    EC_T_INT*               pnDiffMax       )
{
    return emDcmGetStatus(INSTANCE_MASTER_DEFAULT, pdwErrorCode, pnDiffCur, pnDiffAvg, pnDiffMax);
} EC_INLINESTOP
#endif

EC_API EC_T_DWORD EC_API_FNCALL emDcxGetStatus(   EC_T_DWORD              dwInstanceID,
                                                    EC_T_DWORD*             pdwErrorCode,
                                                    EC_T_INT*               pnDiffCur,
                                                    EC_T_INT*               pnDiffAvg,
                                                    EC_T_INT*               pnDiffMax,
                                                    EC_T_INT64*             pnTimeStampDiff);

#ifndef ECAT_INTERFACE
static EC_INLINESTART  EC_T_DWORD ecatDcxGetStatus( EC_T_DWORD*             pdwErrorCode,
                                                    EC_T_INT*               pnDiffCur,
                                                    EC_T_INT*               pnDiffAvg,
                                                    EC_T_INT*               pnDiffMax,
                                                    EC_T_INT64*             pnTimeStampDiff)
{
    return emDcxGetStatus(INSTANCE_MASTER_DEFAULT, pdwErrorCode, pnDiffCur, pnDiffAvg, pnDiffMax, pnTimeStampDiff);
} EC_INLINESTOP
#endif

EC_API EC_T_DWORD EC_API_FNCALL emDcmResetStatus(  EC_T_DWORD             dwInstanceID    );
#ifndef ECAT_INTERFACE
static EC_INLINESTART  EC_T_DWORD ecatDcmResetStatus(EC_T_VOID                              )
{
    return emDcmResetStatus(INSTANCE_MASTER_DEFAULT);
} EC_INLINESTOP
#endif

EC_API EC_T_DWORD EC_API_FNCALL emDcmGetBusShiftConfigured(  EC_T_DWORD             dwInstanceID,
                                                               EC_T_BOOL*             pbBusShiftConfigured);
#ifndef ECAT_INTERFACE
static EC_INLINESTART  EC_T_DWORD ecatDcmGetBusShiftConfigured(EC_T_BOOL*             pbBusShiftConfigured)
{
    return emDcmGetBusShiftConfigured(INSTANCE_MASTER_DEFAULT, pbBusShiftConfigured);
} EC_INLINESTOP
#endif

EC_API EC_T_DWORD EC_API_FNCALL emDcmGetLog(      EC_T_DWORD              dwInstanceID,
                                                    EC_T_CHAR**             pszLog          );
#ifndef ECAT_INTERFACE
static EC_INLINESTART  EC_T_DWORD ecatDcmGetLog(    EC_T_CHAR**             pszLog          )
{
    return emDcmGetLog(INSTANCE_MASTER_DEFAULT, pszLog);
} EC_INLINESTOP
#endif

EC_API EC_T_DWORD EC_API_FNCALL emDcmShowStatus(  EC_T_DWORD              dwInstanceID    );
#ifndef ECAT_INTERFACE
static EC_INLINESTART  EC_T_DWORD ecatDcmShowStatus(EC_T_VOID                               )
{
    return emDcmShowStatus(INSTANCE_MASTER_DEFAULT);
} EC_INLINESTOP
#endif

EC_API EC_T_DWORD EC_API_FNCALL emDcmGetAdjust(   EC_T_DWORD              dwInstanceID,
                                                    EC_T_INT*               pnAdjustPermil  );
#ifndef ECAT_INTERFACE
static EC_INLINESTART  EC_T_DWORD ecatDcmGetAdjust( EC_T_INT*               pnAdjustPermil  )
{
    return emDcmGetAdjust(INSTANCE_MASTER_DEFAULT, pnAdjustPermil);
} EC_INLINESTOP
#endif

EC_API EC_T_DWORD EC_API_FNCALL emGetSlaveInfo(   EC_T_DWORD              dwInstanceID,
                                                    EC_T_BOOL               bFixedAddressing,
                                                    EC_T_WORD               wSlaveAddress,
                                                    EC_T_GET_SLAVE_INFO*    pGetSlaveInfo   );
#ifndef ECAT_INTERFACE
static EC_INLINESTART  EC_T_DWORD ecatGetSlaveInfo( EC_T_BOOL               bFixedAddressing,
                                                    EC_T_WORD               wSlaveAddress,
                                                    EC_T_GET_SLAVE_INFO*    pGetSlaveInfo   )
{
    return emGetSlaveInfo(INSTANCE_MASTER_DEFAULT, bFixedAddressing, wSlaveAddress, pGetSlaveInfo);
} EC_INLINESTOP
#endif

EC_API EC_T_DWORD EC_API_FNCALL emGetCfgSlaveInfo(EC_T_DWORD              dwInstanceID,
                                                    EC_T_BOOL               bStationAddress,
                                                    EC_T_WORD               wSlaveAddress,
                                                    EC_T_CFG_SLAVE_INFO*    pSlaveInfo      );
#ifndef ECAT_INTERFACE
static EC_INLINESTART  EC_T_DWORD ecatGetCfgSlaveInfo(EC_T_BOOL             bStationAddress,
                                                    EC_T_WORD               wSlaveAddress,
                                                    EC_T_CFG_SLAVE_INFO*    pSlaveInfo      )
{
    return emGetCfgSlaveInfo(INSTANCE_MASTER_DEFAULT, bStationAddress, wSlaveAddress, pSlaveInfo);
} EC_INLINESTOP
#endif

EC_API EC_T_DWORD EC_API_FNCALL emGetCfgSlaveEoeInfo(EC_T_DWORD                 dwInstanceID,
                                                    EC_T_BOOL                   bStationAddress,
                                                    EC_T_WORD                   wSlaveAddress,
                                                    EC_T_CFG_SLAVE_EOE_INFO*    pSlaveEoeInfo);
#ifndef ECAT_INTERFACE
static EC_INLINESTART  EC_T_DWORD ecatGetCfgSlaveEoeInfo(EC_T_BOOL              bStationAddress,
                                                    EC_T_WORD                   wSlaveAddress,
                                                    EC_T_CFG_SLAVE_EOE_INFO*    pSlaveEoeInfo)
{
    return emGetCfgSlaveEoeInfo(INSTANCE_MASTER_DEFAULT, bStationAddress, wSlaveAddress, pSlaveEoeInfo);
} EC_INLINESTOP
#endif

EC_API EC_T_DWORD EC_API_FNCALL emGetBusSlaveInfo(EC_T_DWORD              dwInstanceID,
                                                    EC_T_BOOL               bStationAddress,
                                                    EC_T_WORD               wSlaveAddress,
                                                    EC_T_BUS_SLAVE_INFO*    pSlaveInfo      );
#ifndef ECAT_INTERFACE
static EC_INLINESTART  EC_T_DWORD  ecatGetBusSlaveInfo(EC_T_BOOL            bStationAddress,
                                                    EC_T_WORD               wSlaveAddress,
                                                    EC_T_BUS_SLAVE_INFO*    pSlaveInfo      )
{
    return emGetBusSlaveInfo(INSTANCE_MASTER_DEFAULT, bStationAddress, wSlaveAddress, pSlaveInfo);
} EC_INLINESTOP
#endif

EC_API EC_T_DWORD EC_API_FNCALL emGetSlaveInpVarInfoNumOf(EC_T_DWORD      dwInstanceID,
                                                    EC_T_BOOL               bFixedAddressing,
                                                    EC_T_WORD               wSlaveAddress,
                                                    EC_T_WORD*              pwSlaveInpVarInfoNumOf);
#ifndef ECAT_INTERFACE
static EC_INLINESTART  EC_T_DWORD ecatGetSlaveInpVarInfoNumOf(EC_T_BOOL     bFixedAddressing,
                                                    EC_T_WORD               wSlaveAddress,
                                                    EC_T_WORD*              pwSlaveInpVarInfoNumOf)
{
    return emGetSlaveInpVarInfoNumOf(INSTANCE_MASTER_DEFAULT, bFixedAddressing, wSlaveAddress, pwSlaveInpVarInfoNumOf);
} EC_INLINESTOP
#endif

EC_API EC_T_DWORD EC_API_FNCALL emGetSlaveOutpVarInfoNumOf(EC_T_DWORD     dwInstanceID,
                                                    EC_T_BOOL               bFixedAddressing,
                                                    EC_T_WORD               wSlaveAddress,
                                                    EC_T_WORD*              pwSlaveOutpVarInfoNumOf );
#ifndef ECAT_INTERFACE
static EC_INLINESTART  EC_T_DWORD ecatGetSlaveOutpVarInfoNumOf(EC_T_BOOL    bFixedAddressing,
                                                    EC_T_WORD               wSlaveAddress,
                                                    EC_T_WORD*              pwSlaveOutpVarInfoNumOf)
{
    return emGetSlaveOutpVarInfoNumOf(INSTANCE_MASTER_DEFAULT, bFixedAddressing, wSlaveAddress, pwSlaveOutpVarInfoNumOf);
} EC_INLINESTOP
#endif

EC_API EC_T_DWORD EC_API_FNCALL emGetSlaveInpVarInfo(EC_T_DWORD           dwInstanceID,
                                                    EC_T_BOOL               bFixedAddressing,
                                                    EC_T_WORD               wSlaveAddress,
                                                    EC_T_WORD               wNumOfVarsToRead,
                                                    EC_T_PROCESS_VAR_INFO*  pSlaveProcVarInfoEntries,
                                                    EC_T_WORD*              pwReadEntries    );
#ifndef ECAT_INTERFACE
static EC_INLINESTART  EC_T_DWORD ecatGetSlaveInpVarInfo(EC_T_BOOL          bFixedAddressing,
                                                    EC_T_WORD               wSlaveAddress,
                                                    EC_T_WORD               wNumOfVarsToRead,
                                                    EC_T_PROCESS_VAR_INFO*  pSlaveProcVarInfoEntries,
                                                    EC_T_WORD*              pwReadEntries    )
{
    return emGetSlaveInpVarInfo(INSTANCE_MASTER_DEFAULT, bFixedAddressing, wSlaveAddress, wNumOfVarsToRead, pSlaveProcVarInfoEntries, pwReadEntries);
} EC_INLINESTOP
#endif

EC_API EC_T_DWORD EC_API_FNCALL emGetSlaveInpVarInfoEx(EC_T_DWORD         dwInstanceID,
                                                    EC_T_BOOL               bFixedAddressing,
                                                    EC_T_WORD               wSlaveAddress,
                                                    EC_T_WORD               wNumOfVarsToRead,
                                                    EC_T_PROCESS_VAR_INFO_EX* pSlaveProcVarInfoEntries,
                                                    EC_T_WORD*              pwReadEntries    );
#ifndef ECAT_INTERFACE
static EC_INLINESTART  EC_T_DWORD ecatGetSlaveInpVarInfoEx(EC_T_BOOL        bFixedAddressing,
                                                    EC_T_WORD               wSlaveAddress,
                                                    EC_T_WORD               wNumOfVarsToRead,
                                                    EC_T_PROCESS_VAR_INFO_EX* pSlaveProcVarInfoEntries,
                                                    EC_T_WORD*              pwReadEntries    )
{
    return emGetSlaveInpVarInfoEx(INSTANCE_MASTER_DEFAULT, bFixedAddressing, wSlaveAddress, wNumOfVarsToRead, pSlaveProcVarInfoEntries, pwReadEntries);
} EC_INLINESTOP
#endif

EC_API EC_T_DWORD EC_API_FNCALL emGetSlaveOutpVarInfo(EC_T_DWORD          dwInstanceID,
                                                    EC_T_BOOL               bFixedAddressing,
                                                    EC_T_WORD               wSlaveAddress,
                                                    EC_T_WORD               wNumOfVarsToRead,
                                                    EC_T_PROCESS_VAR_INFO*  pSlaveProcVarInfoEntries,
                                                    EC_T_WORD*              pwReadEntries    );
#ifndef ECAT_INTERFACE
static EC_INLINESTART  EC_T_DWORD ecatGetSlaveOutpVarInfo(EC_T_BOOL         bFixedAddressing,
                                                    EC_T_WORD               wSlaveAddress,
                                                    EC_T_WORD               wNumOfVarsToRead,
                                                    EC_T_PROCESS_VAR_INFO*  pSlaveProcVarInfoEntries,
                                                    EC_T_WORD*              pwReadEntries    )
{
    return emGetSlaveOutpVarInfo(INSTANCE_MASTER_DEFAULT, bFixedAddressing, wSlaveAddress, wNumOfVarsToRead, pSlaveProcVarInfoEntries, pwReadEntries);
} EC_INLINESTOP
#endif

EC_API EC_T_DWORD EC_API_FNCALL emGetSlaveOutpVarInfoEx(EC_T_DWORD        dwInstanceID,
                                                    EC_T_BOOL               bFixedAddressing,
                                                    EC_T_WORD               wSlaveAddress,
                                                    EC_T_WORD               wNumOfVarsToRead,
                                                    EC_T_PROCESS_VAR_INFO_EX* pSlaveProcVarInfoEntries,
                                                    EC_T_WORD*              pwReadEntries    );
#ifndef ECAT_INTERFACE
static EC_INLINESTART  EC_T_DWORD ecatGetSlaveOutpVarInfoEx(EC_T_BOOL       bFixedAddressing,
                                                    EC_T_WORD               wSlaveAddress,
                                                    EC_T_WORD               wNumOfVarsToRead,
                                                    EC_T_PROCESS_VAR_INFO_EX* pSlaveProcVarInfoEntries,
                                                    EC_T_WORD*              pwReadEntries    )
{
    return emGetSlaveOutpVarInfoEx(INSTANCE_MASTER_DEFAULT, bFixedAddressing, wSlaveAddress, wNumOfVarsToRead, pSlaveProcVarInfoEntries, pwReadEntries);
} EC_INLINESTOP
#endif

EC_API EC_T_DWORD EC_API_FNCALL emGetSlaveOutpVarByObjectEx(EC_T_DWORD    dwInstanceID,
                                                    EC_T_BOOL               bFixedAddressing,
                                                    EC_T_WORD               wSlaveAddress,
                                                    EC_T_WORD               wIndex,
                                                    EC_T_WORD               wSubIndex,
                                                    EC_T_PROCESS_VAR_INFO_EX* pProcessVarInfoEntry);
#ifndef ECAT_INTERFACE
static EC_INLINESTART  EC_T_DWORD ecatGetSlaveOutpVarByObjectEx(EC_T_BOOL   bFixedAddressing,
                                                    EC_T_WORD               wSlaveAddress,
                                                    EC_T_WORD               wIndex,
                                                    EC_T_WORD               wSubIndex,
                                                    EC_T_PROCESS_VAR_INFO_EX* pProcessVarInfoEntry)
{
    return emGetSlaveOutpVarByObjectEx(INSTANCE_MASTER_DEFAULT, bFixedAddressing, wSlaveAddress, wIndex, wSubIndex, pProcessVarInfoEntry);
} EC_INLINESTOP
#endif

EC_API EC_T_DWORD EC_API_FNCALL emGetSlaveInpVarByObjectEx(EC_T_DWORD     dwInstanceID,
                                                    EC_T_BOOL               bFixedAddressing,
                                                    EC_T_WORD               wSlaveAddress,
                                                    EC_T_WORD               wIndex,
                                                    EC_T_WORD               wSubIndex,
                                                    EC_T_PROCESS_VAR_INFO_EX* pProcessVarInfoEntry);
#ifndef ECAT_INTERFACE
static EC_INLINESTART  EC_T_DWORD ecatGetSlaveInpVarByObjectEx(EC_T_BOOL    bFixedAddressing,
                                                    EC_T_WORD               wSlaveAddress,
                                                    EC_T_WORD               wIndex,
                                                    EC_T_WORD               wSubIndex,
                                                    EC_T_PROCESS_VAR_INFO_EX* pProcessVarInfoEntry)
{
    return emGetSlaveInpVarByObjectEx(INSTANCE_MASTER_DEFAULT, bFixedAddressing, wSlaveAddress, wIndex, wSubIndex, pProcessVarInfoEntry);
} EC_INLINESTOP
#endif

EC_API EC_T_DWORD EC_API_FNCALL emFindOutpVarByName(EC_T_DWORD            dwInstanceID,
                                                    const EC_T_CHAR*      szVariableName,
                                                    EC_T_PROCESS_VAR_INFO*  pSlaveOutpVarInfo);
#ifndef ECAT_INTERFACE
static EC_INLINESTART  EC_T_DWORD ecatFindOutpVarByName(const EC_T_CHAR*    szVariableName,
                                                    EC_T_PROCESS_VAR_INFO*  pProcessVarInfoEntry)
{
    return emFindOutpVarByName(INSTANCE_MASTER_DEFAULT, szVariableName, pProcessVarInfoEntry);
} EC_INLINESTOP
#endif

EC_API EC_T_DWORD EC_API_FNCALL emFindOutpVarByNameEx(EC_T_DWORD          dwInstanceID,
                                                    const EC_T_CHAR*      szVariableName,
                                                    EC_T_PROCESS_VAR_INFO_EX* pProcessVarInfoEntry);
#ifndef ECAT_INTERFACE
static EC_INLINESTART  EC_T_DWORD ecatFindOutpVarByNameEx(const EC_T_CHAR*    szVariableName,
                                                    EC_T_PROCESS_VAR_INFO_EX* pProcessVarInfoEntry)
{
    return emFindOutpVarByNameEx(INSTANCE_MASTER_DEFAULT, szVariableName, pProcessVarInfoEntry);
} EC_INLINESTOP
#endif

EC_API EC_T_DWORD EC_API_FNCALL emFindInpVarByName(EC_T_DWORD             dwInstanceID,
                                                    const EC_T_CHAR*      szVariableName,
                                                    EC_T_PROCESS_VAR_INFO*  pProcessVarInfoEntry);
#ifndef ECAT_INTERFACE
static EC_INLINESTART  EC_T_DWORD ecatFindInpVarByName(const EC_T_CHAR*     szVariableName,
                                                    EC_T_PROCESS_VAR_INFO*  pProcessVarInfoEntry)
{
    return emFindInpVarByName(INSTANCE_MASTER_DEFAULT, szVariableName, pProcessVarInfoEntry);
} EC_INLINESTOP
#endif

EC_API EC_T_DWORD EC_API_FNCALL emFindInpVarByNameEx(EC_T_DWORD             dwInstanceID,
                                                    const EC_T_CHAR*        szVariableName,
                                                    EC_T_PROCESS_VAR_INFO_EX*   pProcessVarInfoEntry);
#ifndef ECAT_INTERFACE
static EC_INLINESTART  EC_T_DWORD ecatFindInpVarByNameEx(const EC_T_CHAR*       szVariableName,
                                                    EC_T_PROCESS_VAR_INFO_EX*   pProcessVarInfoEntry)
{
    return emFindInpVarByNameEx(INSTANCE_MASTER_DEFAULT, szVariableName, pProcessVarInfoEntry);
} EC_INLINESTOP
#endif

EC_API EC_T_DWORD EC_API_FNCALL emEthDbgMsg(      EC_T_DWORD              dwInstanceID,
                                                    EC_T_BYTE               byEthTypeByte0,
                                                    EC_T_BYTE               byEthTypeByte1,
                                                    EC_T_CHAR*              szMsg           );
#ifndef ECAT_INTERFACE
static EC_INLINESTART  EC_T_DWORD ecatEthDbgMsg(    EC_T_BYTE               byEthTypeByte0,
                                                    EC_T_BYTE               byEthTypeByte1,
                                                    EC_T_CHAR*              szMsg           )
{
    return emEthDbgMsg(INSTANCE_MASTER_DEFAULT, byEthTypeByte0, byEthTypeByte1, szMsg);
} EC_INLINESTOP
#endif

EC_API EC_T_DWORD EC_API_FNCALL emBlockNode(      EC_T_DWORD              dwInstanceID,
                                                    EC_T_SB_MISMATCH_DESC*  pMisMatch,
                                                    EC_T_DWORD              dwTimeout       );
#ifndef ECAT_INTERFACE
static EC_INLINESTART  EC_T_DWORD ecatBlockNode(    EC_T_SB_MISMATCH_DESC*  pMisMatch,
                                                    EC_T_DWORD              dwTimeout       )
{
    return emBlockNode(INSTANCE_MASTER_DEFAULT, pMisMatch, dwTimeout);
} EC_INLINESTOP
#endif

EC_API EC_T_DWORD EC_API_FNCALL emOpenBlockedPorts(EC_T_DWORD             dwInstanceID,
                                                    EC_T_DWORD              dwTimeout       );
#ifndef ECAT_INTERFACE
static EC_INLINESTART  EC_T_DWORD ecatOpenBlockedPorts(EC_T_DWORD           dwTimeout       )
{
    return emOpenBlockedPorts(INSTANCE_MASTER_DEFAULT, dwTimeout);
} EC_INLINESTOP
#endif

EC_API EC_T_DWORD EC_API_FNCALL emForceTopologyChange(EC_T_DWORD          dwInstanceID    );
#ifndef ECAT_INTERFACE
static EC_INLINESTART  EC_T_DWORD ecatForceTopologyChange(EC_T_VOID                         )
{
    return emForceTopologyChange(INSTANCE_MASTER_DEFAULT);
} EC_INLINESTOP
#endif

EC_API EC_T_DWORD EC_API_FNCALL emIsTopologyChangeDetected(EC_T_DWORD     dwInstanceID,
                                                           EC_T_BOOL*     pbTopologyChangeDetected);
#ifndef ECAT_INTERFACE
static EC_INLINESTART  EC_T_DWORD ecatIsTopologyChangeDetected(EC_T_BOOL*   pbTopologyChangeDetected)
{
    return emIsTopologyChangeDetected(INSTANCE_MASTER_DEFAULT, pbTopologyChangeDetected);
} EC_INLINESTOP
#endif

EC_API EC_T_DWORD EC_API_FNCALL emIsTopologyKnown(EC_T_DWORD  dwInstanceID,
                                                  EC_T_BOOL*  pbTopologyKnown);
#ifndef ECAT_INTERFACE
static EC_INLINESTART EC_T_DWORD ecatIsTopologyKnown(EC_T_BOOL* pbTopologyKnown)
{
    return emIsTopologyKnown(INSTANCE_MASTER_DEFAULT, pbTopologyKnown);
} EC_INLINESTOP
#endif

EC_API EC_T_DWORD EC_API_FNCALL emGetBusTime(EC_T_DWORD      dwInstanceID,
                                             EC_T_UINT64*    pqwBusTime);
#ifndef ECAT_INTERFACE
static EC_INLINESTART EC_T_DWORD ecatGetBusTime(EC_T_UINT64* pqwBusTime)
{
    return emGetBusTime(INSTANCE_MASTER_DEFAULT, pqwBusTime);
} EC_INLINESTOP
#endif

EC_API EC_T_DWORD EC_API_FNCALL emIsSlavePresent( EC_T_DWORD              dwInstanceID,
                                                    EC_T_DWORD              dwSlaveId,
                                                    EC_T_BOOL*              pbPresence      );
#ifndef ECAT_INTERFACE
static EC_INLINESTART  EC_T_DWORD ecatIsSlavePresent(EC_T_DWORD             dwSlaveId,
                                                    EC_T_BOOL*              pbPresence      )
{
    return emIsSlavePresent(INSTANCE_MASTER_DEFAULT, dwSlaveId, pbPresence);
} EC_INLINESTOP
#endif

EC_API EC_PTS_STATE emPassThroughSrvGetStatus(EC_T_DWORD    dwInstanceID    );
#ifndef ECAT_INTERFACE
static EC_INLINESTART  EC_PTS_STATE ecatPassThroughSrvGetStatus(EC_T_VOID                   )
{
    return emPassThroughSrvGetStatus(INSTANCE_MASTER_DEFAULT);
} EC_INLINESTOP
#endif

EC_API EC_T_DWORD EC_API_FNCALL emPassThroughSrvStart(EC_T_DWORD          dwInstanceID,
                                                    EC_T_PTS_SRV_START_PARMS* poPtsStartParams,
                                                    EC_T_DWORD              dwTimeout       );
#ifndef ECAT_INTERFACE
static EC_INLINESTART  EC_T_DWORD ecatPassThroughSrvStart(EC_T_PTS_SRV_START_PARMS* poPtsStartParams,
                                                    EC_T_DWORD              dwTimeout       )
{
    return emPassThroughSrvStart(INSTANCE_MASTER_DEFAULT, poPtsStartParams, dwTimeout);
} EC_INLINESTOP
#endif

EC_API EC_T_DWORD EC_API_FNCALL emPassThroughSrvStop(EC_T_DWORD           dwInstanceID,
                                                    EC_T_DWORD              dwTimeout       );
#ifndef ECAT_INTERFACE
static EC_INLINESTART  EC_T_DWORD ecatPassThroughSrvStop(EC_T_DWORD         dwTimeout       )
{
    return emPassThroughSrvStop(INSTANCE_MASTER_DEFAULT, dwTimeout);
} EC_INLINESTOP
#endif

EC_API EC_T_DWORD EC_API_FNCALL emPassThroughSrvEnable(EC_T_DWORD         dwInstanceID,
                                                    EC_T_DWORD              dwTimeout       );
#ifndef ECAT_INTERFACE
static EC_INLINESTART  EC_T_DWORD ecatPassThroughSrvEnable(EC_T_DWORD       dwTimeout       )
{
    return emPassThroughSrvEnable(INSTANCE_MASTER_DEFAULT, dwTimeout);
} EC_INLINESTOP
#endif

EC_API EC_T_DWORD EC_API_FNCALL emPassThroughSrvDisable(EC_T_DWORD        dwInstanceID,
                                                    EC_T_DWORD              dwTimeout       );
#ifndef ECAT_INTERFACE
static EC_INLINESTART  EC_T_DWORD ecatPassThroughSrvDisable(EC_T_DWORD      dwTimeout       )
{
    return emPassThroughSrvDisable(INSTANCE_MASTER_DEFAULT, dwTimeout);
} EC_INLINESTOP
#endif

EC_API EC_T_DWORD EC_API_FNCALL emAdsAdapterStart(EC_T_DWORD              dwInstanceID,
                                                    EC_T_ADS_ADAPTER_START_PARMS* poStartParams,
                                                    EC_T_DWORD              dwTimeout       );
#ifndef ECAT_INTERFACE
static EC_INLINESTART  EC_T_DWORD ecatAdsAdapterStart(EC_T_ADS_ADAPTER_START_PARMS* poStartParams,
                                                    EC_T_DWORD              dwTimeout       )
{
    return emAdsAdapterStart(INSTANCE_MASTER_DEFAULT, poStartParams, dwTimeout);
} EC_INLINESTOP
#endif

EC_API EC_T_DWORD EC_API_FNCALL emAdsAdapterStop(EC_T_DWORD               dwInstanceID,
                                                    EC_T_DWORD              dwTimeout       );
#ifndef ECAT_INTERFACE
static EC_INLINESTART  EC_T_DWORD ecatAdsAdapterStop(EC_T_DWORD             dwTimeout       )
{
    return emAdsAdapterStop(INSTANCE_MASTER_DEFAULT, dwTimeout);
} EC_INLINESTOP
#endif

EC_API EC_T_BYTE* EC_API_FNCALL emGetProcessImageInputPtr(EC_T_DWORD      dwInstanceID    );
#ifndef ECAT_INTERFACE
static EC_INLINESTART  EC_T_BYTE* ecatGetProcessImageInputPtr(EC_T_VOID                     )
{
    return emGetProcessImageInputPtr(INSTANCE_MASTER_DEFAULT);
} EC_INLINESTOP
#endif

EC_API EC_T_BYTE* EC_API_FNCALL emGetProcessImageOutputPtr(EC_T_DWORD     dwInstanceID    );
#ifndef ECAT_INTERFACE
static EC_INLINESTART  EC_T_BYTE* ecatGetProcessImageOutputPtr(EC_T_VOID                    )
{
    return emGetProcessImageOutputPtr(INSTANCE_MASTER_DEFAULT);
} EC_INLINESTOP
#endif

EC_API EC_T_BYTE* EC_API_FNCALL emGetDiagnosisImagePtr(EC_T_DWORD         dwInstanceID    );
#ifndef ECAT_INTERFACE
static EC_INLINESTART  EC_T_BYTE* ecatGetDiagnosisImagePtr(EC_T_VOID                        )
{
    return emGetDiagnosisImagePtr(INSTANCE_MASTER_DEFAULT);
} EC_INLINESTOP
#endif

EC_API EC_T_DWORD EC_API_FNCALL emNotifyApp(      EC_T_DWORD              dwInstanceID,
                                                    EC_T_DWORD              dwCode,
                                                    EC_T_NOTIFYPARMS*       pParms          );
#ifndef ECAT_INTERFACE
static EC_INLINESTART  EC_T_DWORD ecatNotifyApp(    EC_T_DWORD              dwCode,
                                                    EC_T_NOTIFYPARMS*       pParms          )
{
    return emNotifyApp(INSTANCE_MASTER_DEFAULT, dwCode, pParms);
} EC_INLINESTOP
#endif

EC_API EC_T_DWORD EC_API_FNCALL emLogFrameEnable( EC_T_DWORD              dwInstanceID,
                                                    EC_T_PFLOGFRAME_CB      pvLogFrameCallBack,
                                                    EC_T_VOID*              pvContext       );
#ifndef ECAT_INTERFACE
static EC_INLINESTART  EC_T_DWORD ecatLogFrameEnable(EC_T_PFLOGFRAME_CB     pvLogFrameCallBack,
                                                    EC_T_VOID*              pvContext       )
{
    return emLogFrameEnable(INSTANCE_MASTER_DEFAULT, pvLogFrameCallBack, pvContext);
} EC_INLINESTOP
#endif

EC_API EC_T_DWORD EC_API_FNCALL emLogFrameDisable(EC_T_DWORD              dwInstanceID    );
#ifndef ECAT_INTERFACE
static EC_INLINESTART  EC_T_DWORD ecatLogFrameDisable(EC_T_VOID                             )
{
    return emLogFrameDisable(INSTANCE_MASTER_DEFAULT);
} EC_INLINESTOP
#endif

EC_API EC_T_DWORD EC_API_FNCALL emGetSrcMacAddress(EC_T_DWORD             dwInstanceID,
                                                    ETHERNET_ADDRESS*       pMacSrc         );
#ifndef ECAT_INTERFACE
static EC_INLINESTART  EC_T_DWORD ecatGetSrcMacAddress(ETHERNET_ADDRESS*    pMacSrc         )
{
    return emGetSrcMacAddress(INSTANCE_MASTER_DEFAULT, pMacSrc);
} EC_INLINESTOP
#endif

EC_API EC_T_DWORD EC_API_FNCALL emSetLicenseKey(  EC_T_DWORD              dwInstanceID,
                                                  const EC_T_CHAR*        pszLicenseKey   );
#ifndef ECAT_INTERFACE
static EC_INLINESTART  EC_T_DWORD ecatSetLicenseKey(const EC_T_CHAR*      pszLicenseKey   )
{
    return emSetLicenseKey(INSTANCE_MASTER_DEFAULT, pszLicenseKey);
} EC_INLINESTOP
#endif

EC_API EC_T_DWORD EC_API_FNCALL emGetMasterProperties(EC_T_DWORD          dwInstanceID,
                                                    EC_T_DWORD*             pdwMasterPropNumEntries,
                                                    EC_T_MASTER_PROP_DESC** ppaMasterPropEntries);
#ifndef ECAT_INTERFACE
static EC_INLINESTART  EC_T_DWORD ecatGetMasterProperties(EC_T_DWORD*       pdwMasterPropNumEntries,
                                                    EC_T_MASTER_PROP_DESC** ppaMasterPropEntries)
{
    return emGetMasterProperties(INSTANCE_MASTER_DEFAULT, pdwMasterPropNumEntries, ppaMasterPropEntries);
} EC_INLINESTOP
#endif

EC_API EC_T_DWORD EC_API_FNCALL emGetVersion(     EC_T_DWORD              dwInstanceID,
                                                    EC_T_DWORD*             pdwVersion      );
#ifndef ECAT_INTERFACE
static EC_INLINESTART  EC_T_DWORD ecatGetVersion(   EC_T_DWORD*             pdwVersion      )
{
    return emGetVersion(INSTANCE_MASTER_DEFAULT, pdwVersion);
} EC_INLINESTOP
#endif

EC_API EC_T_DWORD EC_API_FNCALL emTraceDataConfig(    EC_T_DWORD          dwInstanceID,
                                                        EC_T_WORD           wTraceDataSize  );
#ifndef ECAT_INTERFACE
static EC_INLINESTART  EC_T_DWORD ecatTraceDataConfig(  EC_T_WORD           wTraceDataSize  )
{
    return emTraceDataConfig(INSTANCE_MASTER_DEFAULT, wTraceDataSize);
} EC_INLINESTOP
#endif

EC_API EC_T_DWORD EC_API_FNCALL emTraceDataGetInfo(   EC_T_DWORD          dwInstanceID,
                                                        EC_T_TRACE_DATA_INFO* pTraceDataInfo);
#ifndef ECAT_INTERFACE
static EC_INLINESTART  EC_T_DWORD ecatTraceDataGetInfo( EC_T_TRACE_DATA_INFO* pTraceDataInfo)
{
    return emTraceDataGetInfo(INSTANCE_MASTER_DEFAULT, pTraceDataInfo);
} EC_INLINESTOP
#endif

EC_API EC_T_DWORD EC_API_FNCALL emFastModeInit(       EC_T_DWORD          dwInstanceID);
#ifndef ECAT_INTERFACE
static EC_INLINESTART  EC_T_DWORD ecatFastModeInit(EC_T_VOID)
{
    return emFastModeInit(INSTANCE_MASTER_DEFAULT);
} EC_INLINESTOP
#endif

EC_API EC_T_DWORD EC_API_FNCALL emFastSendAllCycFrames(EC_T_DWORD         dwInstanceID);
#ifndef ECAT_INTERFACE
static EC_INLINESTART  EC_T_DWORD ecatFastSendAllCycFrames(EC_T_VOID)
{
    return emFastSendAllCycFrames(INSTANCE_MASTER_DEFAULT);
} EC_INLINESTOP
#endif

EC_API EC_T_DWORD EC_API_FNCALL emFastProcessAllRxFrames(EC_T_DWORD      dwInstanceID,
                                                            EC_T_BOOL*      pbAreAllCycFramesProcessed);
#ifndef ECAT_INTERFACE
static EC_INLINESTART  EC_T_DWORD ecatFastProcessAllRxFrames(EC_T_BOOL*     pbAreAllCycFramesProcessed)
{
    return emFastProcessAllRxFrames(INSTANCE_MASTER_DEFAULT, pbAreAllCycFramesProcessed);
} EC_INLINESTOP
#endif

EC_API EC_T_DWORD EC_API_FNCALL emReadSlaveIdentification(   EC_T_DWORD   dwInstanceID,
                                                               EC_T_BOOL    bFixedAddressing,
                                                               EC_T_WORD    wSlaveAddress,
                                                               EC_T_WORD    wAdo,
                                                               EC_T_WORD*   pwValue,
                                                               EC_T_DWORD   dwTimeout       );
#ifndef ECAT_INTERFACE
static EC_INLINESTART  EC_T_DWORD ecatReadSlaveIdentification( EC_T_BOOL    bFixedAddressing,
                                                               EC_T_WORD    wSlaveAddress,
                                                               EC_T_WORD    wAdo,
                                                               EC_T_WORD*   pwValue,
                                                               EC_T_DWORD   dwTimeout       )
{
    return emReadSlaveIdentification(INSTANCE_MASTER_DEFAULT, bFixedAddressing, wSlaveAddress, wAdo, pwValue, dwTimeout);
} EC_INLINESTOP
#endif

EC_API EC_T_DWORD EC_API_FNCALL emReadSlaveIdentificationReq(EC_T_DWORD   dwInstanceID,
                                                               EC_T_DWORD   dwClientId,
                                                               EC_T_DWORD   dwTferId,
                                                               EC_T_BOOL    bFixedAddressing,
                                                               EC_T_WORD    wSlaveAddress,
                                                               EC_T_WORD    wAdo,
                                                               EC_T_WORD*   pwValue,
                                                               EC_T_DWORD   dwTimeout);
#ifndef ECAT_INTERFACE
static EC_INLINESTART  EC_T_DWORD ecatReadSlaveIdentificationReq(EC_T_DWORD dwClientId,
                                                               EC_T_DWORD   dwTferId,
                                                               EC_T_BOOL    bFixedAddressing,
                                                               EC_T_WORD    wSlaveAddress,
                                                               EC_T_WORD    wAdo,
                                                               EC_T_WORD*   pwValue,
                                                               EC_T_DWORD   dwTimeout)
{
    return emReadSlaveIdentificationReq(INSTANCE_MASTER_DEFAULT, dwClientId, dwTferId, bFixedAddressing, wSlaveAddress, wAdo, pwValue, dwTimeout);
} EC_INLINESTOP
#endif

EC_API EC_T_DWORD EC_API_FNCALL emSetSlaveDisabled(  EC_T_DWORD           dwInstanceID,
                                                       EC_T_BOOL            bFixedAddressing,
                                                       EC_T_WORD            wSlaveAddress,
                                                       EC_T_BOOL            bDisabled);
#ifndef ECAT_INTERFACE
static EC_INLINESTART  EC_T_DWORD ecatSetSlaveDisabled(EC_T_BOOL            bFixedAddressing,
                                                       EC_T_WORD            wSlaveAddress,
                                                       EC_T_BOOL            bDisabled       )
{
    return emSetSlaveDisabled(INSTANCE_MASTER_DEFAULT, bFixedAddressing, wSlaveAddress, bDisabled);
} EC_INLINESTOP
#endif

EC_API EC_T_DWORD EC_API_FNCALL emSetSlavesDisabled(  EC_T_DWORD              dwInstanceID,
                                                        EC_T_BOOL               bFixedAddressing,
                                                        EC_T_WORD               wSlaveAddress,
                                                        EC_T_SLAVE_SELECTION    eSlaveSelection,
                                                        EC_T_BOOL               bDisabled);
#ifndef ECAT_INTERFACE
static EC_INLINESTART  EC_T_DWORD ecatSetSlavesDisabled(EC_T_BOOL               bFixedAddressing,
                                                        EC_T_WORD               wSlaveAddress,
                                                        EC_T_SLAVE_SELECTION    eSlaveSelection,
                                                        EC_T_BOOL               bDisabled)
{
    return emSetSlavesDisabled(INSTANCE_MASTER_DEFAULT, bFixedAddressing, wSlaveAddress, eSlaveSelection, bDisabled);
} EC_INLINESTOP
#endif

EC_API EC_T_DWORD EC_API_FNCALL emSetSlaveDisconnected(  EC_T_DWORD       dwInstanceID,
                                                           EC_T_BOOL        bFixedAddressing,
                                                           EC_T_WORD        wSlaveAddress,
                                                           EC_T_BOOL        bDisconnected   );
#ifndef ECAT_INTERFACE
static EC_INLINESTART  EC_T_DWORD ecatSetSlaveDisconnected(EC_T_BOOL        bFixedAddressing,
                                                           EC_T_WORD        wSlaveAddress,
                                                           EC_T_BOOL        bDisconnected   )
{
    return emSetSlaveDisconnected(INSTANCE_MASTER_DEFAULT, bFixedAddressing, wSlaveAddress, bDisconnected);
} EC_INLINESTOP
#endif

EC_API EC_T_DWORD EC_API_FNCALL emSetSlavesDisconnected(  EC_T_DWORD              dwInstanceID,
                                                            EC_T_BOOL               bFixedAddressing,
                                                            EC_T_WORD               wSlaveAddress,
                                                            EC_T_SLAVE_SELECTION    eSlaveSelection,
                                                            EC_T_BOOL               bDisconnected);
#ifndef ECAT_INTERFACE
static EC_INLINESTART  EC_T_DWORD ecatSetSlavesDisconnected(EC_T_BOOL               bFixedAddressing,
                                                            EC_T_WORD               wSlaveAddress,
                                                            EC_T_SLAVE_SELECTION    eSlaveSelection,
                                                            EC_T_BOOL               bDisconnected)
{
    return emSetSlavesDisconnected(INSTANCE_MASTER_DEFAULT, bFixedAddressing, wSlaveAddress, eSlaveSelection, bDisconnected);
} EC_INLINESTOP
#endif

EC_API EC_T_DWORD EC_API_FNCALL emGetMemoryUsage(  EC_T_DWORD         dwInstanceID,
                                                     EC_T_DWORD*        pdwCurrentUsage,
                                                     EC_T_DWORD*        pdwMaxUsage);
#ifndef ECAT_INTERFACE
static EC_INLINESTART  EC_T_DWORD ecatGetMemoryUsage(EC_T_DWORD*        pdwCurrentUsage,
                                                     EC_T_DWORD*        pdwMaxUsage)
{
    return emGetMemoryUsage(INSTANCE_MASTER_DEFAULT, pdwCurrentUsage, pdwMaxUsage);
} EC_INLINESTOP
#endif

EC_API EC_T_DWORD EC_API_FNCALL emGetSlaveStatistics(    EC_T_DWORD   dwInstanceID,
                                                           EC_T_DWORD   dwSlaveId,
                                                           EC_T_SLVSTATISTICS_DESC* pSlaveStatisticsDesc);
#ifndef ECAT_INTERFACE
static EC_INLINESTART  EC_T_DWORD ecatGetSlaveStatistics(  EC_T_DWORD   dwSlaveId,
                                                           EC_T_SLVSTATISTICS_DESC* pSlaveStatisticsDesc)
{
    return emGetSlaveStatistics(INSTANCE_MASTER_DEFAULT, dwSlaveId, pSlaveStatisticsDesc);
} EC_INLINESTOP
#endif

EC_API EC_T_DWORD EC_API_FNCALL emClearSlaveStatistics(  EC_T_DWORD   dwInstanceID,
                                                           EC_T_DWORD   dwSlaveId   );
#ifndef ECAT_INTERFACE
static EC_INLINESTART  EC_T_DWORD ecatClearSlaveStatistics(EC_T_DWORD   dwSlaveId   )
{
    return emClearSlaveStatistics(INSTANCE_MASTER_DEFAULT, dwSlaveId);
} EC_INLINESTOP
#endif

EC_API EC_T_DWORD EC_API_FNCALL emGetMasterSyncUnitInfoNumOf(EC_T_DWORD  dwInstanceID);
#ifndef ECAT_INTERFACE
static EC_INLINESTART  EC_T_DWORD ecatGetMasterSyncUnitInfoNumOf(EC_T_VOID)
{
    return emGetMasterSyncUnitInfoNumOf(INSTANCE_MASTER_DEFAULT);
} EC_INLINESTOP
#endif

EC_API EC_T_DWORD EC_API_FNCALL emGetMasterSyncUnitInfo(  EC_T_DWORD     dwInstanceID,
                                                            EC_T_WORD      wMsuId,
                                                            EC_T_MSU_INFO* pMsuInfo);
#ifndef ECAT_INTERFACE
static EC_INLINESTART  EC_T_DWORD ecatGetMasterSyncUnitInfo(EC_T_WORD      wMsuId,
                                                            EC_T_MSU_INFO* pMsuInfo)
{
    return emGetMasterSyncUnitInfo(INSTANCE_MASTER_DEFAULT, wMsuId, pMsuInfo);
} EC_INLINESTOP
#endif

EC_API EC_T_DWORD EC_API_FNCALL emGetMasterDump(    EC_T_DWORD      dwInstanceID,
                                                    EC_T_BYTE*      pbyBuffer,
                                                    EC_T_DWORD      dwBufferSize,
                                                    EC_T_DWORD*     pdwDumpSize);
#ifndef ECAT_INTERFACE
static EC_INLINESTART EC_T_DWORD ecatGetMasterDump( EC_T_BYTE*      pbyBuffer,
                                                    EC_T_DWORD      dwBufferSize,
                                                    EC_T_DWORD*     pdwDumpSize)
{
    return emGetMasterDump(INSTANCE_MASTER_DEFAULT, pbyBuffer, dwBufferSize, pdwDumpSize);
} EC_INLINESTOP
#endif

/*-INLINE METHODS------------------------------------------------------------*/
#ifndef ECAT_INTERFACE
static EC_INLINESTART const EC_T_CHAR* ecatStateToStr(EC_T_STATE eState)
{
    return eState == eEcatState_INIT ? "INIT":
     (eState == eEcatState_PREOP ? "PREOP":
      (eState == eEcatState_SAFEOP ? "SAFEOP":
       (eState == eEcatState_OP ? "OP":
        (eState == eEcatState_BOOTSTRAP ? "BOOTSTRAP":
         (eState == eEcatState_UNKNOWN ? "UNKNOWN":
          "STATE INVALID")))));
} EC_INLINESTOP
#define ecatDeviceStateText(eState)     ecatGetText(((EC_T_DWORD)(EC_TXT_DEVICE_STATE_BASE+(eState))))
#endif

#ifndef ECAT_INTERFACE
static EC_INLINESTART const EC_T_CHAR* ecatSlaveStateText(EC_T_WORD nState)
{
    if (nState & DEVICE_STATE_ERROR)
        return ecatGetText(((EC_T_DWORD)(EC_TXT_SLAVE_STATE_ERROR_BASE+(nState & DEVICE_STATE_MASK))));
    else
        return ecatGetText(((EC_T_DWORD)(EC_TXT_DEVICE_STATE_BASE+(nState & DEVICE_STATE_MASK))));
} EC_INLINESTOP
#endif

static EC_INLINESTART EC_T_DWORD emIoCtl(
    EC_T_DWORD      dwInstanceID, /**< [in] Instance ID (Multiple EtherCAT Network Support) */
    EC_T_DWORD      dwCode,       /**< [in]  see EC_IOCTL_... */
    EC_T_VOID*      pbyInBuf,     /**< [in]  input data buffer */
    EC_T_DWORD      dwInBufSize,  /**< [in]  size of input data buffer in byte */
    EC_T_VOID*      pbyOutBuf,    /**< [out] output data buffer */
    EC_T_DWORD      dwOutBufSize, /**< [in]  size of output data buffer in byte */
    EC_T_DWORD*     pdwNumOutData /**< [out] number of output data bytes stored in output data buffer */
)
{
    EC_T_IOCTLPARMS oIoCtlParms;
    EC_T_DWORD      dwNumOutData = 0;

    oIoCtlParms.pbyInBuf      = (EC_T_BYTE*)pbyInBuf;
    oIoCtlParms.dwInBufSize   = dwInBufSize;
    oIoCtlParms.pbyOutBuf     = (EC_T_BYTE*)pbyOutBuf;
    oIoCtlParms.dwOutBufSize  = dwOutBufSize;
    oIoCtlParms.pdwNumOutData = (EC_NULL != pdwNumOutData) ? pdwNumOutData : &dwNumOutData;
    return emIoControl(dwInstanceID, dwCode, &oIoCtlParms);
} EC_INLINESTOP
#ifndef ECAT_INTERFACE
static EC_INLINESTART  EC_T_DWORD ecatIoCtl(
    EC_T_DWORD      dwCode,       /**< [in]  see EC_IOCTL_... */
    EC_T_VOID*      pbyInBuf,     /**< [in]  input data buffer */
    EC_T_DWORD      dwInBufSize,  /**< [in]  size of input data buffer in byte */
    EC_T_VOID*      pbyOutBuf,    /**< [out] output data buffer */
    EC_T_DWORD      dwOutBufSize, /**< [in]  size of output data buffer in byte */
    EC_T_DWORD*     pdwNumOutData /**< [out] number of output data bytes stored in output data buffer */
)
{
    return emIoCtl(INSTANCE_MASTER_DEFAULT, dwCode, pbyInBuf, dwInBufSize, pbyOutBuf, dwOutBufSize, pdwNumOutData);
} EC_INLINESTOP
#endif

/**
\brief Provide OEM Key needed for OEM Masters to parse ENI files and provide access via RAS.
       Must be called after initialization and before configuration
*/
static EC_INLINESTART  EC_T_DWORD emSetOemKey(      EC_T_DWORD              dwInstanceID,   /**< [in] Instance ID (Multiple EtherCAT Network Support) */
                                                    EC_T_UINT64             qwOemKey        /**< [in] 64 bit OEM key */
                                             )
{
    return emIoCtl(dwInstanceID, EC_IOCTL_SET_OEM_KEY, &qwOemKey, sizeof(EC_T_UINT64), EC_NULL, 0, EC_NULL);
} EC_INLINESTOP
#ifndef ECAT_INTERFACE
static EC_INLINESTART  EC_T_DWORD ecatSetOemKey(    EC_T_UINT64             qwOemKey        )
{
    return emSetOemKey(INSTANCE_MASTER_DEFAULT, qwOemKey);
} EC_INLINESTOP
#endif

/** \brief Checks if OEM Key from parameter matches the OEM Key from emSetOemKey
*
* \return 
        - #EC_E_NOERROR
        - #EC_E_OEM_KEY_MISSING if OEM Key not set
        - #EC_E_OEM_KEY_MISMATCH if OEM Key differs from Client
*/
static EC_INLINESTART  EC_T_DWORD emCheckOemKey(    EC_T_DWORD              dwInstanceID,   /**< [in] Instance ID (Multiple EtherCAT Network Support) */
                                                    EC_T_UINT64             qwOemKey        /**< [in] 64 bit OEM key */   
                                               )
{
    return emIoCtl(dwInstanceID, EC_IOCTL_CHECK_OEM_KEY, &qwOemKey, sizeof(EC_T_UINT64), EC_NULL, 0, EC_NULL);
} EC_INLINESTOP
#ifndef ECAT_INTERFACE
static EC_INLINESTART  EC_T_DWORD ecatCheckOemKey(  EC_T_UINT64             qwOemKey        )
{
    return emCheckOemKey(INSTANCE_MASTER_DEFAULT, qwOemKey);
} EC_INLINESTOP
#endif

/**
\brief Clears all error counters (0x0300 - 0x0313) of all slaves.
*/
static EC_INLINESTART  EC_T_DWORD emBadConnectionsReset(   EC_T_DWORD      dwInstanceID     /**< [in] Instance ID (Multiple EtherCAT Network Support) */
                                                       )
{
    return emIoCtl(dwInstanceID, EC_IOCTL_CLR_SLVSTATISTICS, EC_NULL, 0, EC_NULL, 0, EC_NULL);
}
static EC_INLINESTART  EC_T_DWORD ecatBadConnectionsReset( EC_T_VOID                        )
{
    return emBadConnectionsReset(INSTANCE_MASTER_DEFAULT);
} EC_INLINESTOP

EC_API EC_T_DWORD EC_API_FNCALL   emBadConnectionsDetect(  EC_T_DWORD      dwInstanceID,
                                                           EC_T_DWORD      dwTimeout        );
static EC_INLINESTART  EC_T_DWORD ecatBadConnectionsDetect(EC_T_DWORD      dwTimeout        )
{
    return emBadConnectionsDetect(INSTANCE_MASTER_DEFAULT, dwTimeout);
} EC_INLINESTOP

EC_API EC_T_DWORD EC_API_FNCALL   emSelfTestScan(   EC_T_DWORD                  dwInstanceID,
                                                    EC_T_SELFTESTSCAN_PARMS*    pParms);
static EC_INLINESTART  EC_T_DWORD ecatSelfTestScan( EC_T_SELFTESTSCAN_PARMS*    pParms)
{
    return emSelfTestScan(INSTANCE_MASTER_DEFAULT, pParms);
} EC_INLINESTOP

#ifdef __cplusplus
} /* extern "C" */
#endif

#ifndef ECAT_INTERFACE
#define ECAT_INTERFACE
#endif

#endif /* INC_ATETHERCAT */

/*-END OF SOURCE FILE--------------------------------------------------------*/
