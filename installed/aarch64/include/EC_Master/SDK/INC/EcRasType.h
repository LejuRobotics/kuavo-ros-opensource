/** ---------------------------------------------------------------------------
 * \file
 * \author      Timo Nussbaumer
 * \copyright   acontis technologies GmbH, Weingarten, Germany
 *---------------------------------------------------------------------------*/

#ifndef INC_ECRASTYPE
#define INC_ECRASTYPE 1

/*-INCLUDES------------------------------------------------------------------*/
#ifndef INC_ECOS
#include "EcOs.h"
#endif
#ifndef INC_ECRASERROR
#include "EcRasError.h"
#endif
#ifndef INC_ECINTERFACECOMMON
#include "EcInterfaceCommon.h"
#endif

/*-COMPILER SETTINGS---------------------------------------------------------*/
#ifdef __cplusplus
extern "C"
{
#endif

/*-DEFINES-------------------------------------------------------------------*/
#undef INCLUDE_RAS_TRACESUPPORT

#if (!defined EC_RAS_DEFAULT_PORT)
#if (defined EC_SOCKET_IP_SUPPORTED)
  #if (defined INCLUDE_EC_SIMULATOR_HIL)
  #define EC_RAS_DEFAULT_PORT     ((EC_T_WORD)6001)
  #else
  #define EC_RAS_DEFAULT_PORT     ((EC_T_WORD)6000)
  #endif
#elif (defined EC_SOCKET_MSGQUEUE_RTOSSHM_SUPPORTED)
#define EC_RAS_DEFAULT_PORT       ((EC_T_WORD)2)
#elif (defined EC_SOCKET_SHM_SUPPORTED)
#define EC_RAS_DEFAULT_PORT       ((EC_T_WORD)1)
#elif (defined EC_SOCKET_MSGQUEUE_WIN32_SUPPORTED)
#define EC_RAS_DEFAULT_PORT       ((EC_T_WORD)0)
#elif (defined EC_SOCKET_RTOSLIB_SUPPORTED)
#define EC_RAS_DEFAULT_PORT       ((EC_T_WORD)3)
#else
#error Unknown socket implementation
#endif /* EC_SOCKET_..._SUPPORTED */
#endif /* EC_RAS_DEFAULT_PORT */

/*-TYPEDEFS------------------------------------------------------------------*/
typedef enum _EC_T_TLS_CERT_TYPE
{
    eTlsCertType_Unknown = 0,
    eTlsCertType_Filename = 1,            /**< TLS certificate type is a filename */
    eTlsCertType_Data = 2,                /**< TLS certificate type is a buffer */
    /* Borland C++ datatype alignment correction */
    eTlsCertType_BCppDummy = 0xFFFFFFFF
} EC_T_TLS_CERT_TYPE;

typedef enum _EC_T_TLS_PRIVKEY_TYPE
{
    eTlsPrivKeyType_Unknown = 0,
    eTlsPrivKeyType_Filename = 1,            /**< TLS private key type is a filename */
    eTlsPrivKeyType_Data = 2,                /**< TLS private key type is a buffer */
    /* Borland C++ datatype alignment correction */
    eTlsPrivKeyType_BCppDummy = 0xFFFFFFFF
} EC_T_TLS_PRIVKEY_TYPE;

#include EC_PACKED_INCLUDESTART(4)
#if (defined INCLUDE_RAS_SPOCSUPPORT)
typedef enum _EC_T_RAS_ORDINAL
{
    ord_emInitMaster                     = 201,  /* 0x00C9 */
    ord_emDeinitMaster                   = 202,  /* 0x00CA */
    ord_emStart                          = 203,  /* 0x00CB */
    ord_emStop                           = 204,  /* 0x00CC */
    ord_emIoCtl                          = 205,  /* 0x00CD */
    ord_emGetSlaveId                     = 207,  /* 0x00CF */
    ord_emMbxTferCreate                  = 208,  /* 0x00D0 */
    ord_emMbxTferDelete                  = 209,  /* 0x00D1 */
    ord_emCoeSdoDownloadReq              = 210,  /* 0x00D2 */
    ord_emCoeSdoUploadReq                = 211,  /* 0x00D3 */
    ord_emCoeGetODListReq                = 212,  /* 0x00D4 */
    ord_emCoeGetObjectDescReq            = 213,  /* 0x00D5 */
    ord_emCoeGetEntryDescReq             = 214,  /* 0x00D6 */
    ord_emGetSlaveProp                   = 218,  /* 0x00DA */
    ord_emGetSlaveState                  = 219,  /* 0x00DB */
    ord_emSetSlaveState                  = 220,  /* 0x00DC */
    ord_emTferSingleRawCmd               = 221,  /* 0x00DD */
    ord_emGetSlaveIdAtPosition           = 225,  /* 0x00E1 */
    ord_emGetNumConfiguredSlaves         = 226,  /* 0x00E2 */
    ord_emConfigureNetwork               = 227,  /* 0x00E3 */
    ord_emSetMasterState                 = 228,  /* 0x00E4 */
    ord_emQueueRawCmd                    = 229,  /* 0x00E5 */
    ord_emCoeRxPdoTfer                   = 230,  /* 0x00E6 */
    ord_emExecJob                        = 231,  /* 0x00E7 */
    ord_emGetProcessData                 = 234,  /* 0x00EA */
    ord_emSetProcessData                 = 235,  /* 0x00EB */
    ord_emGetMasterState                 = 236,  /* 0x00EC */
    ord_emFoeFileUpload                  = 237,  /* 0x00ED */
    ord_emFoeFileDownload                = 238,  /* 0x00EE */
    ord_emFoeUpoadReq                    = 239,  /* 0x00EF */
    ord_emFoeDownloadReq                 = 240,  /* 0x00F0 */
    ord_emCoeSdoDownload                 = 241,  /* 0x00F1 */
    ord_emCoeSdoUpload                   = 242,  /* 0x00F2 */
    ord_emGetNumConnectedSlaves          = 243,  /* 0x00F3 */
    ord_emResetSlaveController           = 244,  /* 0x00F4 */
    ord_emGetSlaveInfo                   = 245,  /* 0x00F5 */
    ord_emIsSlavePresent                 = 246,  /* 0x00F6 */
    ord_emAoeWriteReq                    = 247,  /* 0x00F7 */
    ord_emAoeReadReq                     = 248,  /* 0x00F8 */
    ord_emAoeWrite                       = 249,  /* 0x00F9 */
    ord_emAoeRead                        = 250,  /* 0x00FA */
    ord_emAoeGetSlaveNetId               = 251,  /* 0x00FB */
    ord_emGetFixedAddr                   = 252,  /* 0x00FC */
    ord_emGetSlaveProcVarInfoNumOf       = 253,  /* 0x00FD */
    ord_emGetSlaveProcVarInfo            = 254,  /* 0x00FE */
    ord_emFindProcVarByName              = 255,  /* 0x00FF */
    ord_emGetProcessDataBits             = 256,  /* 0x0100 */
    ord_emSetProcessDataBits             = 257,  /* 0x0101 */
    ord_emReloadSlaveEEPRom              = 258,  /* 0x0102 */
    ord_emReadSlaveEEPRom                = 259,  /* 0x0103 */
    ord_emWriteSlaveEEPRom               = 260,  /* 0x0104 */
    ord_emAssignSlaveEEPRom              = 261,  /* 0x0105 */
    ord_emSoeRead                        = 262,  /* 0x0106 */
    ord_emSoeWrite                       = 263,  /* 0x0107 */
    ord_emSoeAbortProcCmd                = 264,  /* 0x0108 */
    ord_emGetNumConnectedSlavesMain      = 265,  /* 0x0109 */
    ord_emGetNumConnectedSlavesRed       = 266,  /* 0x010A */
    ord_emNotifyApp                      = 267,  /* 0x010B */
    ord_emAoeReadWriteReq                = 268,  /* 0x010C */
    ord_emAoeReadWrite                   = 269,  /* 0x010D */
    ord_emGetCfgSlaveInfo                = 270,  /* 0x010E */
    ord_emGetBusSlaveInfo                = 271,  /* 0x010F */
    ord_emReadSlaveIdentification        = 272,  /* 0x0110 */
    ord_emSetSlaveDisabled               = 273,  /* 0x0111 */
    ord_emSetSlaveDisconnected           = 274,  /* 0x0112 */
    ord_emRescueScan                     = 275,  /* 0x0113 */
    ord_emGetMasterInfo                  = 276,  /* 0x0114 */
    ord_emConfigExtend                   = 277,  /* 0x0115 */
    ord_emAoeWriteControl                = 278,  /* 0x0116 */
    ord_emSetSlavesDisabled              = 279,  /* 0x0117 */
    ord_emSetSlavesDisconnected          = 280,  /* 0x0118 */
    ord_emSetMbxProtocolsSerialize       = 281,  /* 0x0119 */
    ord_emBadConnectionsDetect           = 282,  /* 0x011A */
    ord_emIsConfigured                   = 283,  /* 0x011B */
    ord_emPerfMeasReset                  = 284,  /* 0x011C */
    ord_emPerfMeasGetRaw                 = 285,  /* 0x011D */
    ord_emPerfMeasGetInfo                = 286,  /* 0x011E */
    ord_emPerfMeasGetNumOf               = 287,  /* 0x011F */
    ord_emSelfTestScan                   = 288,  /* 0x0120 */
    ord_emScanBus                        = 289,  /* 0x0121 */
    ord_emGetMemoryUsage                 = 290,  /* 0x0122 */
    ord_emGetCfgSlaveSmInfo              = 291,  /* 0x0123 */
    ord_emPerfMeasAppGetInfo             = 292,  /* 0x0124 */
    ord_emPerfMeasAppGetRaw              = 293,  /* 0x0125 */
    ord_emPerfMeasAppGetNumOf            = 294,  /* 0x0126 */

    ord_emCoeGetODList                   = 310,  /* 0x0136 */
    ord_emCoeGetObjectDesc               = 311,  /* 0x0137 */
    ord_emCoeGetEntryDesc                = 312,  /* 0x0138 */

    ord_esConnectPorts                   = 605,  /* 0x025D */
    ord_esDisconnectPort                 = 606,  /* 0x025E */
    ord_esPowerSlave                     = 607,  /* 0x025F */
    ord_esSetErrorAtSlavePort                 = 616,  /* 0x0268 */
    ord_esSetErrorGenerationAtSlavePort       = 617,  /* 0x0269 */
    ord_esResetErrorGenerationAtSlavePorts    = 618,  /* 0x026A */
    ord_esSetLinkDownAtSlavePort              = 619,  /* 0x026B */
    ord_esSetLinkDownGenerationAtSlavePort    = 620,  /* 0x026C */
    ord_esResetLinkDownGenerationAtSlavePorts = 621,  /* 0x026D */

    ord_emGetMasterStateEx                    = 622,  /* 0x026E */

    ord_esSendSlaveCoeEmergency               = 623,  /* 0x026F */
    ord_esVoeSend                             = 624,  /* 0x0270 */
    ord_esGetSimSlaveInfo                     = 625,  /* 0x0271 */
    ord_esSetSimSlaveState                    = 630,  /* 0x0276 */

    ord_emGetHcGroupNumOf                     = 663,  /* 0x0297 */
    ord_emGetHcGroupInfo                      = 664,  /* 0x0298 */

    ord_emSetMasterStateReq                   = 766,  /* 0x02FE */
    ord_emSetSlaveStateReq                    = 767,  /* 0x02FF */

    /* See also EC-Master API, EC-Simulator API and other branches! */

    /* Borland C++ datatype alignment correction */
    ord_BCppDummy                   = 0xFFFFFFFF
} EC_T_RAS_ORDINAL;

#define PARAMETER_IGNORE        ((EC_T_DWORD)0xffffffff)


/* Default RAS SPOC access configuration */
#define RASSPOCCFGINITDEFAULT \
{ \
    /* \
      Level required                    Ordinal                           Index                                       Subindex            Reserved \
    */ \
    { EC_RAS_ACCESS_LEVEL_ALLOW_ALL,  ord_emStart,                    PARAMETER_IGNORE,                           PARAMETER_IGNORE, 0 /* reserved */ }, \
    { EC_RAS_ACCESS_LEVEL_ALLOW_ALL,  ord_emStop,                     PARAMETER_IGNORE,                           PARAMETER_IGNORE, 0 /* reserved */ }, \
    { EC_RAS_ACCESS_LEVEL_ALLOW_ALL,  ord_emConfigureNetwork,         PARAMETER_IGNORE,                           PARAMETER_IGNORE, 0 /* reserved */ }, \
    { EC_RAS_ACCESS_LEVEL_ALLOW_ALL,  ord_emSetMasterState,           PARAMETER_IGNORE,                           PARAMETER_IGNORE, 0 /* reserved */ }, \
    { EC_RAS_ACCESS_LEVEL_READONLY,   ord_emGetMasterState,           PARAMETER_IGNORE,                           PARAMETER_IGNORE, 0 /* reserved */ }, \
    { EC_RAS_ACCESS_LEVEL_READONLY,   ord_emGetMasterStateEx,         PARAMETER_IGNORE,                           PARAMETER_IGNORE, 0 /* reserved */ }, \
    { EC_RAS_ACCESS_LEVEL_ALLOW_ALL,  ord_emSetSlaveState,            PARAMETER_IGNORE,                           PARAMETER_IGNORE, 0 /* reserved */ }, \
    { EC_RAS_ACCESS_LEVEL_READONLY,   ord_emGetSlaveState,            PARAMETER_IGNORE,                           PARAMETER_IGNORE, 0 /* reserved */ }, \
    { EC_RAS_ACCESS_LEVEL_ALLOW_ALL,  ord_emSetMasterStateReq,        PARAMETER_IGNORE,                           PARAMETER_IGNORE, 0 /* reserved */ }, \
    { EC_RAS_ACCESS_LEVEL_ALLOW_ALL,  ord_emSetSlaveStateReq,         PARAMETER_IGNORE,                           PARAMETER_IGNORE, 0 /* reserved */ }, \
    /* Info configured slaves */ \
    { EC_RAS_ACCESS_LEVEL_READONLY,   ord_emGetFixedAddr,             PARAMETER_IGNORE,                           PARAMETER_IGNORE, 0 /* reserved */ }, \
    { EC_RAS_ACCESS_LEVEL_READONLY,   ord_emGetSlaveId,               PARAMETER_IGNORE,                           PARAMETER_IGNORE, 0 /* reserved */ }, \
    { EC_RAS_ACCESS_LEVEL_READONLY,   ord_emGetNumConnectedSlaves,    PARAMETER_IGNORE,                           PARAMETER_IGNORE, 0 /* reserved */ }, \
    { EC_RAS_ACCESS_LEVEL_READONLY,   ord_emGetNumConfiguredSlaves,   PARAMETER_IGNORE,                           PARAMETER_IGNORE, 0 /* reserved */ }, \
    { EC_RAS_ACCESS_LEVEL_READONLY,   ord_emIsSlavePresent,           PARAMETER_IGNORE,                           PARAMETER_IGNORE, 0 /* reserved */ }, \
    { EC_RAS_ACCESS_LEVEL_ALLOW_ALL,  ord_emSetSlaveDisabled,         PARAMETER_IGNORE,                           PARAMETER_IGNORE, 0 /* reserved */ }, \
    { EC_RAS_ACCESS_LEVEL_READONLY,   ord_emGetSlaveProp,             PARAMETER_IGNORE,                           PARAMETER_IGNORE, 0 /* reserved */ }, \
    { EC_RAS_ACCESS_LEVEL_READONLY,   ord_emGetSlaveIdAtPosition,     PARAMETER_IGNORE,                           PARAMETER_IGNORE, 0 /* reserved */ }, \
    { EC_RAS_ACCESS_LEVEL_READONLY,   ord_emGetSlaveInfo,             PARAMETER_IGNORE,                           PARAMETER_IGNORE, 0 /* reserved */ }, \
    { EC_RAS_ACCESS_LEVEL_READONLY,   ord_emIsSlavePresent,           PARAMETER_IGNORE,                           PARAMETER_IGNORE, 0 /* reserved */ }, \
    /* Process data */ \
    { EC_RAS_ACCESS_LEVEL_READONLY,   ord_emGetProcessData,           PARAMETER_IGNORE,                           PARAMETER_IGNORE, 0 /* reserved */ }, \
    { EC_RAS_ACCESS_LEVEL_ALLOW_ALL,  ord_emSetProcessData,           PARAMETER_IGNORE,                           PARAMETER_IGNORE, 0 /* reserved */ }, \
    { EC_RAS_ACCESS_LEVEL_READONLY,   ord_emGetProcessDataBits,       PARAMETER_IGNORE,                           PARAMETER_IGNORE, 0 /* reserved */ }, \
    { EC_RAS_ACCESS_LEVEL_ALLOW_ALL,  ord_emSetProcessDataBits,       PARAMETER_IGNORE,                           PARAMETER_IGNORE, 0 /* reserved */ }, \
    { EC_RAS_ACCESS_LEVEL_READONLY,   ord_emGetSlaveProcVarInfoNumOf, PARAMETER_IGNORE,                           PARAMETER_IGNORE, 0 /* reserved */ }, \
    { EC_RAS_ACCESS_LEVEL_READONLY,   ord_emGetSlaveProcVarInfo,      PARAMETER_IGNORE,                           PARAMETER_IGNORE, 0 /* reserved */ }, \
    { EC_RAS_ACCESS_LEVEL_READONLY,   ord_emFindProcVarByName,        PARAMETER_IGNORE,                           PARAMETER_IGNORE, 0 /* reserved */ }, \
    /* Mailbox protocols */ \
    { EC_RAS_ACCESS_LEVEL_READONLY,   ord_emMbxTferCreate,            PARAMETER_IGNORE,                           PARAMETER_IGNORE, 0 /* reserved */ }, \
    { EC_RAS_ACCESS_LEVEL_READONLY,   ord_emMbxTferDelete,            PARAMETER_IGNORE,                           PARAMETER_IGNORE, 0 /* reserved */ }, \
    { EC_RAS_ACCESS_LEVEL_READWRITE,  ord_emCoeSdoDownloadReq,        PARAMETER_IGNORE,                           PARAMETER_IGNORE, 0 /* reserved */ }, \
    { EC_RAS_ACCESS_LEVEL_READONLY,   ord_emCoeSdoUploadReq,          PARAMETER_IGNORE,                           PARAMETER_IGNORE, 0 /* reserved */ }, \
    { EC_RAS_ACCESS_LEVEL_READONLY,   ord_emCoeGetODListReq,          PARAMETER_IGNORE,                           PARAMETER_IGNORE, 0 /* reserved */ }, \
    { EC_RAS_ACCESS_LEVEL_READONLY,   ord_emCoeGetObjectDescReq,      PARAMETER_IGNORE,                           PARAMETER_IGNORE, 0 /* reserved */ }, \
    { EC_RAS_ACCESS_LEVEL_READONLY,   ord_emCoeGetEntryDescReq,       PARAMETER_IGNORE,                           PARAMETER_IGNORE, 0 /* reserved */ }, \
    { EC_RAS_ACCESS_LEVEL_READONLY,   ord_emCoeSdoUpload,             PARAMETER_IGNORE,                           PARAMETER_IGNORE, 0 /* reserved */ }, \
    { EC_RAS_ACCESS_LEVEL_READWRITE,  ord_emCoeSdoDownload,           PARAMETER_IGNORE,                           PARAMETER_IGNORE, 0 /* reserved */ }, \
    { EC_RAS_ACCESS_LEVEL_READONLY,   ord_emFoeFileUpload,            PARAMETER_IGNORE,                           PARAMETER_IGNORE, 0 /* reserved */ }, \
    { EC_RAS_ACCESS_LEVEL_READWRITE,  ord_emFoeFileDownload,          PARAMETER_IGNORE,                           PARAMETER_IGNORE, 0 /* reserved */ }, \
    { EC_RAS_ACCESS_LEVEL_READONLY,   ord_emFoeUpoadReq,              PARAMETER_IGNORE,                           PARAMETER_IGNORE, 0 /* reserved */ }, \
    { EC_RAS_ACCESS_LEVEL_READWRITE,  ord_emFoeDownloadReq,           PARAMETER_IGNORE,                           PARAMETER_IGNORE, 0 /* reserved */ }, \
    { EC_RAS_ACCESS_LEVEL_READWRITE,  ord_emAoeWriteReq,              PARAMETER_IGNORE,                           PARAMETER_IGNORE, 0 /* reserved */ }, \
    { EC_RAS_ACCESS_LEVEL_READONLY,   ord_emAoeReadReq,               PARAMETER_IGNORE,                           PARAMETER_IGNORE, 0 /* reserved */ }, \
    { EC_RAS_ACCESS_LEVEL_READWRITE,  ord_emAoeWrite,                 PARAMETER_IGNORE,                           PARAMETER_IGNORE, 0 /* reserved */ }, \
    { EC_RAS_ACCESS_LEVEL_READONLY,   ord_emAoeRead,                  PARAMETER_IGNORE,                           PARAMETER_IGNORE, 0 /* reserved */ }, \
    { EC_RAS_ACCESS_LEVEL_READONLY,   ord_emAoeGetSlaveNetId,         PARAMETER_IGNORE,                           PARAMETER_IGNORE, 0 /* reserved */ }, \
    { EC_RAS_ACCESS_LEVEL_READWRITE,  ord_emAoeReadWrite,             PARAMETER_IGNORE,                           PARAMETER_IGNORE, 0 /* reserved */ }, \
    { EC_RAS_ACCESS_LEVEL_READONLY,   ord_emSoeRead,                  PARAMETER_IGNORE,                           PARAMETER_IGNORE, 0 /* reserved */ }, \
    { EC_RAS_ACCESS_LEVEL_READWRITE,  ord_emSoeWrite,                 PARAMETER_IGNORE,                           PARAMETER_IGNORE, 0 /* reserved */ }, \
    { EC_RAS_ACCESS_LEVEL_READWRITE,  ord_emSoeAbortProcCmd,          PARAMETER_IGNORE,                           PARAMETER_IGNORE, 0 /* reserved */ }, \
    { EC_RAS_ACCESS_LEVEL_READWRITE,  ord_emAoeWriteControl,          PARAMETER_IGNORE,                           PARAMETER_IGNORE, 0 /* reserved */ }, \
    { EC_RAS_ACCESS_LEVEL_READWRITE,  ord_emSetMbxProtocolsSerialize, PARAMETER_IGNORE,                           PARAMETER_IGNORE, 0 /* reserved */ }, \
    /* Misc.*/ \
    { EC_RAS_ACCESS_LEVEL_ALLOW_ALL,  ord_emResetSlaveController,     PARAMETER_IGNORE,                           PARAMETER_IGNORE, 0 /* reserved */ }, \
    { EC_RAS_ACCESS_LEVEL_ALLOW_ALL,  ord_emReloadSlaveEEPRom,        PARAMETER_IGNORE,                           PARAMETER_IGNORE, 0 /* reserved */ }, \
    { EC_RAS_ACCESS_LEVEL_READONLY,   ord_emReadSlaveEEPRom,          PARAMETER_IGNORE,                           PARAMETER_IGNORE, 0 /* reserved */ }, \
    { EC_RAS_ACCESS_LEVEL_ALLOW_ALL,  ord_emWriteSlaveEEPRom,         PARAMETER_IGNORE,                           PARAMETER_IGNORE, 0 /* reserved */ }, \
    { EC_RAS_ACCESS_LEVEL_ALLOW_ALL,  ord_emAssignSlaveEEPRom,        PARAMETER_IGNORE,                           PARAMETER_IGNORE, 0 /* reserved */ }, \
    /* EC_IOCTL_REGISTERCLIENT/EC_IOCTL_UNREGISTERCLIENT were removed from the public EcIoCtl.h in V3.3, kept here as literals for RAS wire compatibility */ \
    { EC_RAS_ACCESS_LEVEL_READONLY,   ord_emIoCtl,                    (EC_IOCTL_GENERIC |  2) /* EC_IOCTL_REGISTERCLIENT */,   PARAMETER_IGNORE, 0 /* reserved */ }, \
    { EC_RAS_ACCESS_LEVEL_READONLY,   ord_emIoCtl,                    (EC_IOCTL_GENERIC |  3) /* EC_IOCTL_UNREGISTERCLIENT */, PARAMETER_IGNORE, 0 /* reserved */ }, \
    { EC_RAS_ACCESS_LEVEL_READONLY,   ord_emIoCtl,                    EC_IOCTL_ISLINK_CONNECTED,                  PARAMETER_IGNORE, 0 /* reserved */ }, \
    { EC_RAS_ACCESS_LEVEL_READONLY,   ord_emIoCtl,                    EC_IOCTL_GET_PDMEMORYSIZE,                  PARAMETER_IGNORE, 0 /* reserved */ }, \
    { EC_RAS_ACCESS_LEVEL_ALLOW_ALL,  ord_emIoCtl,                    EC_IOCTL_SLAVE_LINKMESSAGES,                PARAMETER_IGNORE, 0 /* reserved */ }, \
    { EC_RAS_ACCESS_LEVEL_READONLY,   ord_emIoCtl,                    EC_IOCTL_DC_SLV_SYNC_STATUS_GET,            PARAMETER_IGNORE, 0 /* reserved */ }, \
    { EC_RAS_ACCESS_LEVEL_ALLOW_ALL,  ord_emIoCtl,                    EC_IOCTL_SB_RESTART,                        PARAMETER_IGNORE, 0 /* reserved */ }, \
    { EC_RAS_ACCESS_LEVEL_READONLY,   ord_emIoCtl,                    EC_IOCTL_SB_STATUS_GET,                     PARAMETER_IGNORE, 0 /* reserved */ }, \
    { EC_RAS_ACCESS_LEVEL_ALLOW_ALL,  ord_emIoCtl,                    EC_IOCTL_SB_SET_BUSCNF_VERIFY,              PARAMETER_IGNORE, 0 /* reserved */ }, \
    { EC_RAS_ACCESS_LEVEL_ALLOW_ALL,  ord_emIoCtl,                    EC_IOCTL_SB_SET_BUSCNF_VERIFY_PROP,         PARAMETER_IGNORE, 0 /* reserved */ }, \
    { EC_RAS_ACCESS_LEVEL_READONLY,   ord_emIoCtl,                    EC_IOCTL_SB_BUSCNF_GETSLAVE_INFO,           PARAMETER_IGNORE, 0 /* reserved */ }, \
    { EC_RAS_ACCESS_LEVEL_READONLY,   ord_emIoCtl,                    EC_IOCTL_SB_BUSCNF_GETSLAVE_INFO_EEP,       PARAMETER_IGNORE, 0 /* reserved */ }, \
    { EC_RAS_ACCESS_LEVEL_ALLOW_ALL,  ord_emIoCtl,                    EC_IOCTL_SB_ENABLE,                         PARAMETER_IGNORE, 0 /* reserved */ }, \
    { EC_RAS_ACCESS_LEVEL_READONLY,   ord_emIoCtl,                    EC_IOCTL_SB_BUSCNF_GETSLAVE_INFO_EX,        PARAMETER_IGNORE, 0 /* reserved */ }, \
    { EC_RAS_ACCESS_LEVEL_ALLOW_ALL,  ord_emIoCtl,                    EC_IOCTL_SLV_ALIAS_ENABLE,                  PARAMETER_IGNORE, 0 /* reserved */ }, \
    { EC_RAS_ACCESS_LEVEL_ALLOW_ALL,  ord_emIoCtl,                    EC_IOCTL_SET_SLVSTAT_PERIOD,                PARAMETER_IGNORE, 0 /* reserved */ }, \
    { EC_RAS_ACCESS_LEVEL_ALLOW_ALL,  ord_emIoCtl,                    EC_IOCTL_GET_SLVSTAT_PERIOD,                PARAMETER_IGNORE, 0 /* reserved */ }, \
    { EC_RAS_ACCESS_LEVEL_ALLOW_ALL,  ord_emIoCtl,                    EC_IOCTL_FORCE_SLVSTAT_COLLECTION,          PARAMETER_IGNORE, 0 /* reserved */ }, \
    { EC_RAS_ACCESS_LEVEL_READONLY,   ord_emIoCtl,                    EC_IOCTL_GET_SLVSTATISTICS,                 PARAMETER_IGNORE, 0 /* reserved */ }, \
    { EC_RAS_ACCESS_LEVEL_BLOCK_ALL,  ord_emTferSingleRawCmd,         0x00000000,                                 PARAMETER_IGNORE, 0 /* reserved */ }, \
    { EC_RAS_ACCESS_LEVEL_READONLY,   ord_emTferSingleRawCmd,         0x00000001,                                 PARAMETER_IGNORE, 0 /* reserved */ }, \
    { EC_RAS_ACCESS_LEVEL_READONLY,   ord_emTferSingleRawCmd,         0x00000004,                                 PARAMETER_IGNORE, 0 /* reserved */ }, \
    { EC_RAS_ACCESS_LEVEL_READONLY,   ord_emTferSingleRawCmd,         0x00000007,                                 PARAMETER_IGNORE, 0 /* reserved */ }, \
    { EC_RAS_ACCESS_LEVEL_READONLY,   ord_emTferSingleRawCmd,         0x0000000A,                                 PARAMETER_IGNORE, 0 /* reserved */ }, \
    { EC_RAS_ACCESS_LEVEL_ALLOW_ALL,  ord_emTferSingleRawCmd,         0x00000002,                                 PARAMETER_IGNORE, 0 /* reserved */ }, \
    { EC_RAS_ACCESS_LEVEL_ALLOW_ALL,  ord_emTferSingleRawCmd,         0x00000003,                                 PARAMETER_IGNORE, 0 /* reserved */ }, \
    { EC_RAS_ACCESS_LEVEL_ALLOW_ALL,  ord_emTferSingleRawCmd,         0x00000005,                                 PARAMETER_IGNORE, 0 /* reserved */ }, \
    { EC_RAS_ACCESS_LEVEL_ALLOW_ALL,  ord_emTferSingleRawCmd,         0x00000006,                                 PARAMETER_IGNORE, 0 /* reserved */ }, \
    { EC_RAS_ACCESS_LEVEL_ALLOW_ALL,  ord_emTferSingleRawCmd,         0x00000008,                                 PARAMETER_IGNORE, 0 /* reserved */ }, \
    { EC_RAS_ACCESS_LEVEL_ALLOW_ALL,  ord_emTferSingleRawCmd,         0x00000009,                                 PARAMETER_IGNORE, 0 /* reserved */ }, \
    { EC_RAS_ACCESS_LEVEL_ALLOW_ALL,  ord_emTferSingleRawCmd,         0x0000000B,                                 PARAMETER_IGNORE, 0 /* reserved */ }, \
    { EC_RAS_ACCESS_LEVEL_ALLOW_ALL,  ord_emTferSingleRawCmd,         0x0000000C,                                 PARAMETER_IGNORE, 0 /* reserved */ }, \
    { EC_RAS_ACCESS_LEVEL_ALLOW_ALL,  ord_emTferSingleRawCmd,         0x0000000D,                                 PARAMETER_IGNORE, 0 /* reserved */ }, \
    { EC_RAS_ACCESS_LEVEL_ALLOW_ALL,  ord_emTferSingleRawCmd,         0x0000000E,                                 PARAMETER_IGNORE, 0 /* reserved */ }, \
    { EC_RAS_ACCESS_LEVEL_BLOCK_ALL,  ord_emQueueRawCmd,              eEcatCmd_NOP,                            PARAMETER_IGNORE, 0 /* reserved */ }, \
    { EC_RAS_ACCESS_LEVEL_READONLY,   ord_emQueueRawCmd,              eEcatCmd_APRD,                           PARAMETER_IGNORE, 0 /* reserved */ }, \
    { EC_RAS_ACCESS_LEVEL_READONLY,   ord_emQueueRawCmd,              eEcatCmd_FPRD,                           PARAMETER_IGNORE, 0 /* reserved */ }, \
    { EC_RAS_ACCESS_LEVEL_READONLY,   ord_emQueueRawCmd,              eEcatCmd_BRD,                            PARAMETER_IGNORE, 0 /* reserved */ }, \
    { EC_RAS_ACCESS_LEVEL_READONLY,   ord_emQueueRawCmd,              eEcatCmd_LRD,                            PARAMETER_IGNORE, 0 /* reserved */ }, \
    { EC_RAS_ACCESS_LEVEL_READWRITE,  ord_emQueueRawCmd,              eEcatCmd_APWR,                           PARAMETER_IGNORE, 0 /* reserved */ }, \
    { EC_RAS_ACCESS_LEVEL_READWRITE,  ord_emQueueRawCmd,              eEcatCmd_APRW,                           PARAMETER_IGNORE, 0 /* reserved */ }, \
    { EC_RAS_ACCESS_LEVEL_READWRITE,  ord_emQueueRawCmd,              eEcatCmd_FPWR,                           PARAMETER_IGNORE, 0 /* reserved */ }, \
    { EC_RAS_ACCESS_LEVEL_READWRITE,  ord_emQueueRawCmd,              eEcatCmd_FPRW,                           PARAMETER_IGNORE, 0 /* reserved */ }, \
    { EC_RAS_ACCESS_LEVEL_READWRITE,  ord_emQueueRawCmd,              eEcatCmd_BWR,                            PARAMETER_IGNORE, 0 /* reserved */ }, \
    { EC_RAS_ACCESS_LEVEL_READWRITE,  ord_emQueueRawCmd,              eEcatCmd_BRW,                            PARAMETER_IGNORE, 0 /* reserved */ }, \
    { EC_RAS_ACCESS_LEVEL_READWRITE,  ord_emQueueRawCmd,              eEcatCmd_LWR,                            PARAMETER_IGNORE, 0 /* reserved */ }, \
    { EC_RAS_ACCESS_LEVEL_READWRITE,  ord_emQueueRawCmd,              eEcatCmd_LRW,                            PARAMETER_IGNORE, 0 /* reserved */ }, \
    { EC_RAS_ACCESS_LEVEL_READWRITE,  ord_emQueueRawCmd,              eEcatCmd_ARMW,                           PARAMETER_IGNORE, 0 /* reserved */ }, \
    { EC_RAS_ACCESS_LEVEL_READWRITE,  ord_emQueueRawCmd,              eEcatCmd_FRMW,                           PARAMETER_IGNORE, 0 /* reserved */ }, \
    { EC_RAS_ACCESS_LEVEL_READONLY,   ord_emGetCfgSlaveInfo,          PARAMETER_IGNORE,                           PARAMETER_IGNORE, 0 /* reserved */ }, \
    { EC_RAS_ACCESS_LEVEL_READONLY,   ord_emGetBusSlaveInfo,          PARAMETER_IGNORE,                           PARAMETER_IGNORE, 0 /* reserved */ }, \
    { EC_RAS_ACCESS_LEVEL_READWRITE,  ord_emReadSlaveIdentification,  PARAMETER_IGNORE,                           PARAMETER_IGNORE, 0 /* reserved */ }, \
    { EC_RAS_ACCESS_LEVEL_ALLOW_ALL,  ord_emSetSlaveDisconnected,     PARAMETER_IGNORE,                           PARAMETER_IGNORE, 0 /* reserved */ }, \
    { EC_RAS_ACCESS_LEVEL_ALLOW_ALL,  ord_emRescueScan,               PARAMETER_IGNORE,                           PARAMETER_IGNORE, 0 /* reserved */ }, \
    { EC_RAS_ACCESS_LEVEL_ALLOW_ALL,  ord_emConfigExtend,             PARAMETER_IGNORE,                           PARAMETER_IGNORE, 0 /* reserved */ }, \
    { EC_RAS_ACCESS_LEVEL_ALLOW_ALL,  ord_emSetSlavesDisabled,        PARAMETER_IGNORE,                           PARAMETER_IGNORE, 0 /* reserved */ }, \
    { EC_RAS_ACCESS_LEVEL_ALLOW_ALL,  ord_emSetSlavesDisconnected,    PARAMETER_IGNORE,                           PARAMETER_IGNORE, 0 /* reserved */ }, \
    { EC_RAS_ACCESS_LEVEL_READONLY,   ord_emIsConfigured,             PARAMETER_IGNORE,                           PARAMETER_IGNORE, 0 /* reserved */ }, \
    { EC_RAS_ACCESS_LEVEL_ALLOW_ALL,  ord_emPerfMeasReset,            PARAMETER_IGNORE,                           PARAMETER_IGNORE, 0 /* reserved */ }, \
    { EC_RAS_ACCESS_LEVEL_READONLY,   ord_emPerfMeasGetRaw,           PARAMETER_IGNORE,                           PARAMETER_IGNORE, 0 /* reserved */ }, \
    { EC_RAS_ACCESS_LEVEL_READONLY,   ord_emPerfMeasGetInfo,          PARAMETER_IGNORE,                           PARAMETER_IGNORE, 0 /* reserved */ }, \
    { EC_RAS_ACCESS_LEVEL_READONLY,   ord_emPerfMeasGetNumOf,         PARAMETER_IGNORE,                           PARAMETER_IGNORE, 0 /* reserved */ }, \
    { EC_RAS_ACCESS_LEVEL_ALLOW_ALL,  ord_emScanBus,                  PARAMETER_IGNORE,                           PARAMETER_IGNORE, 0 /* reserved */ }, \
    { EC_RAS_ACCESS_LEVEL_READONLY,   ord_emGetMemoryUsage,           PARAMETER_IGNORE,                           PARAMETER_IGNORE, 0 /* reserved */ }, \
    { EC_RAS_ACCESS_LEVEL_ALLOW_ALL,  ord_esPowerSlave,               PARAMETER_IGNORE,                           PARAMETER_IGNORE, 0 /* reserved */ }, \
    { EC_RAS_ACCESS_LEVEL_ALLOW_ALL,  ord_esSetErrorAtSlavePort,      PARAMETER_IGNORE,                           PARAMETER_IGNORE, 0 /* reserved */ }, \
    { EC_RAS_ACCESS_LEVEL_ALLOW_ALL,  ord_esSetErrorGenerationAtSlavePort,       PARAMETER_IGNORE,                PARAMETER_IGNORE, 0 /* reserved */ }, \
    { EC_RAS_ACCESS_LEVEL_ALLOW_ALL,  ord_esResetErrorGenerationAtSlavePorts,    PARAMETER_IGNORE,                PARAMETER_IGNORE, 0 /* reserved */ }, \
    { EC_RAS_ACCESS_LEVEL_ALLOW_ALL,  ord_esSetLinkDownAtSlavePort,   PARAMETER_IGNORE,                           PARAMETER_IGNORE, 0 /* reserved */ }, \
    { EC_RAS_ACCESS_LEVEL_ALLOW_ALL,  ord_esSetLinkDownGenerationAtSlavePort,    PARAMETER_IGNORE,                PARAMETER_IGNORE, 0 /* reserved */ }, \
    { EC_RAS_ACCESS_LEVEL_ALLOW_ALL,  ord_esResetLinkDownGenerationAtSlavePorts, PARAMETER_IGNORE,                PARAMETER_IGNORE, 0 /* reserved */ }, \
    { EC_RAS_ACCESS_LEVEL_ALLOW_ALL,  ord_esSendSlaveCoeEmergency,    PARAMETER_IGNORE,                           PARAMETER_IGNORE, 0 /* reserved */ }, \
    { EC_RAS_ACCESS_LEVEL_READWRITE,  ord_esVoeSend,                  PARAMETER_IGNORE,                           PARAMETER_IGNORE, 0 /* reserved */ }, \
    { EC_RAS_ACCESS_LEVEL_READONLY,   ord_esGetSimSlaveInfo,          PARAMETER_IGNORE,                           PARAMETER_IGNORE, 0 /* reserved */ }, \
    { EC_RAS_ACCESS_LEVEL_ALLOW_ALL,  ord_esSetSimSlaveState,         PARAMETER_IGNORE,                           PARAMETER_IGNORE, 0 /* reserved */ }, \
    { EC_RAS_ACCESS_LEVEL_ALLOW_ALL,  ord_esSetSimSlaveState,         PARAMETER_IGNORE,                           PARAMETER_IGNORE, 0 /* reserved */ }, \
    { EC_RAS_ACCESS_LEVEL_READONLY,   ord_emPerfMeasAppGetInfo,       PARAMETER_IGNORE,                           PARAMETER_IGNORE, 0 /* reserved */ }, \
    { EC_RAS_ACCESS_LEVEL_READONLY,   ord_emPerfMeasAppGetRaw,        PARAMETER_IGNORE,                           PARAMETER_IGNORE, 0 /* reserved */ }, \
    { EC_RAS_ACCESS_LEVEL_READONLY,   ord_emPerfMeasAppGetNumOf,      PARAMETER_IGNORE,                           PARAMETER_IGNORE, 0 /* reserved */ }, \
    { EC_RAS_ACCESS_LEVEL_READONLY,   ord_emGetHcGroupNumOf,          PARAMETER_IGNORE,                           PARAMETER_IGNORE, 0 /* reserved */ }, \
    { EC_RAS_ACCESS_LEVEL_READONLY,   ord_emGetHcGroupInfo,           PARAMETER_IGNORE,                           PARAMETER_IGNORE, 0 /* reserved */ }\
}
#endif /* INCLUDE_RAS_SPOCSUPPORT */

typedef struct _EC_T_RAS_SPOCCFG
{
    EC_T_DWORD              dwAccessLevel; /**< [in]   see EC_RAS_ACCESS_LEVEL_..., e.g. EC_RAS_ACCESS_LEVEL_READWRITE */
    EC_T_DWORD              dwOrdinal;     /**< [in]   see ord_..., e.g. ord_emSetMasterState */
    EC_T_DWORD              dwIndex;       /**< [in]   set to PARAMETER_IGNORE, if not needed */
    EC_T_DWORD              dwSubIndex;    /**< [in]   set to PARAMETER_IGNORE, if not needed */
    EC_T_DWORD              dwReserved;
} EC_PACKED(4) EC_T_RAS_SPOCCFG;

#define EC_RAS_SERVER_SIGNATURE_PATTERN 0xEAE00000
#define EC_RAS_SERVER_SIGNATURE EC_SIGNATURE_MAKE(EC_RAS_SERVER_SIGNATURE_PATTERN, EC_VERSION_MAJ, EC_VERSION_MIN, EC_VERSION_SERVICEPACK, EC_VERSION_BUILD)

typedef EC_T_DWORD(*EC_PF_CHECK_TOKEN)(EC_T_VOID* pvCheckTokenContext, EC_T_BYTE* pbyToken, EC_T_DWORD dwTokenSize);

#if __GNUC__ >= 8
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpacked-not-aligned"
#endif
typedef struct _EC_T_RAS_SERVER_PARMS
{
    EC_T_DWORD          dwSignature;            /**< [in]   Set to EC_RAS_SERVER_SIGNATURE */
    EC_T_DWORD          dwSize;                 /**< [in]   Set to sizeof(EC_T_RAS_SERVER_PARMS) */
    EC_T_LOG_PARMS      LogParms;               /**< [in]   Logging parameters */

    EC_T_IPADDR         oAddr;                  /**< [in]   Remote Access Server (RAS) listen IP address */
    EC_T_WORD           wPort;                  /**< [in]   Remote Access Server (RAS) listen port */
    EC_T_WORD           wMaxClientCnt;          /**< [in]   Max. clients in parallel (0: unlimited) */
    EC_T_DWORD          dwCycleTime;            /**< [in]   Cycle Time of RAS Network access (acceptor, worker) */

    EC_T_DWORD          dwCommunicationTimeout; /**< [in]   Timeout [ms] before automatically closing connection */

    /* Settings for thread accepting new client connections and spawning corresponding worker threads */
    EC_T_CPUSET         oAcceptorThreadCpuAffinityMask;     /**< [in]   Acceptor Thread CPU affinity mask */
    EC_T_DWORD          dwAcceptorThreadPrio;               /**< [in]   Acceptor Thread Priority */
    EC_T_DWORD          dwAcceptorThreadStackSize;          /**< [in]   Acceptor Thread Stack Size */

    /* Settings for worker threads handling requests from corresponding client connections */
    EC_T_CPUSET         oClientWorkerThreadCpuAffinityMask; /**< [in]   Client Worker Thread CPU affinity mask */
    EC_T_DWORD          dwClientWorkerThreadPrio;           /**< [in]   Client Worker Thread Priority */
    EC_T_DWORD          dwClientWorkerThreadStackSize;      /**< [in]   Client Worker Thread Stack Size */

    EC_T_DWORD          dwMaxQueuedNotificationCnt;         /**< [in]   Amount of concurrently queue able Notifications */

    EC_T_DWORD          dwMaxParallelMbxTferCnt;            /**< [in]   Amount of concurrent active mailbox transfers */

    EC_PF_NOTIFY        pfnRasNotify;                       /**< [in]   Function pointer called to notify error and status
                                                             *          information generated by Remote API Layer */
    EC_T_VOID*          pvRasNotifyCtxt;                    /**< [in]   Notification context returned while calling pfNotification */
    EC_T_DWORD          dwCycErrInterval;                   /**< [in]   Interval which allows cyclic Notifications */
    EC_T_DWORD          dwMaxQueuedNotificationSize;        /**< [in]   Size of concurrent active mailbox transfers */
    EC_PF_CHECK_TOKEN   pfCheckToken;                       /**< [in]   Function pointer called to check token  */
    EC_T_VOID*          pvCheckTokenContext;                /**< [in]   Check token context  */
    EC_T_BYTE*                pbyTlsCert;                         /**< [in]   TLS certificate filename string */
    EC_T_DWORD                dwTlsCertSize;                      /**< [in]   Size of TLS certeficate */
    EC_T_TLS_CERT_TYPE        eTlsCertType;                       /**< [in]   TLS certeficate type */
    EC_T_BYTE*                pbyTlsPrivKey;                      /**< [in]   TLS private key filename string */
    EC_T_DWORD                dwTlsPrivKeySize;                   /**< [in]   Size of TLS private key */
    EC_T_TLS_PRIVKEY_TYPE     eTlsPrivKeyType;                    /**< [in]   TLS certeficate type */
} EC_PACKED(4) EC_T_RAS_SERVER_PARMS, *EC_PT_RAS_SERVER_PARMS;
#if __GNUC__ >= 8
#pragma GCC diagnostic pop
#endif
#include EC_PACKED_INCLUDESTOP

/*-TYPEDEFS RAS CLIENT------------------------------------------------------------*/

#define EC_RAS_CLIENT_CONDESC_SIGNATURE_PATTERN 0x8BE00000
#define EC_RAS_CLIENT_CONDESC_SIGNATURE EC_SIGNATURE_MAKE(EC_RAS_CLIENT_CONDESC_SIGNATURE_PATTERN, EC_VERSION_MAJ, EC_VERSION_MIN, EC_VERSION_SERVICEPACK, EC_VERSION_BUILD)

#include EC_PACKED_INCLUDESTART(4)
#if __GNUC__ >= 8
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpacked-not-aligned"
#endif
typedef struct _EC_T_RAS_CLIENT_CONDESC
{
    EC_T_DWORD          dwSignature;            /**< [in]   Set to EC_RAS_CLIENT_CONDESC_SIGNATURE */
    EC_T_DWORD          dwSize;                 /**< [in]   Set to sizeof(EC_T_RAS_CLIENT_CONDESC) */

    EC_T_LOG_PARMS      LogParms;

    EC_T_IPADDR         oAddr;              /**< [in]   Remote Access Server (RAS) connect IP address */
    EC_T_WORD           wPort;              /**< [in]   Remote Access Server (RAS) connect port */
    EC_T_DWORD          dwWatchDog;         /**< [in]   Watchdog interval when to send IDL packets */
    EC_T_DWORD          dwCycleTime;        /**< [in]   Cycle Time for Recv Polling */
    EC_T_DWORD          dwWDTOLimit;        /**< [in]   Amount of cycles without receiving commands (idles) before
                                              *         Entering state wdexpired
                                              */
    EC_T_CPUSET         cpuAffinityMask;    /**< [in]   CPU affinity mask */
    EC_T_DWORD          dwRecvPrio;         /**< [in]   Thread Priority of RAS Receive Thread */
    EC_T_DWORD          dwPktAdminSize;     /**< [in]   Slave Packet Administrative List Size */
    EC_T_PVOID          pvConHandle;        /**< [in]   Connection Handle needed for disconnect */

    EC_T_DWORD          dwInstanceID;       /**< [in]   OEM Master / Simulator Instance. Only used if OEM Key given. */
    EC_T_UINT64         qwOemKey;           /**< [in]   OEM Key */
    EC_T_BYTE*          pbyToken;           /**< [in]   Token buffer */
    EC_T_DWORD          dwTokenSize;        /**< [in]   Token size */
    EC_T_BYTE*                 pbyTlsCert;                 /**< [in] Client TLS certificate */
    EC_T_DWORD                 dwTlsCertSize;              /**< [in] Client TLS certificate size */
    EC_T_TLS_CERT_TYPE         eTlsCertType;               /**< [in] TLS certificate type: file or buffer */
    EC_T_BOOL                  bIsRootCert;                /**< [in] EC_TRUE for root certificate and EC_FALSE for others */
    EC_T_BYTE*                 pbyTlsPrivKey;              /**< [in] Client TLS private key  */
    EC_T_DWORD                 dwTlsPrivKeySize;           /**< [in] Client TLS private key size */
    EC_T_TLS_PRIVKEY_TYPE      eTlsPrivKeyType;            /**< [in] TLS private key type: file or buffer */
} EC_PACKED(4) EC_T_RAS_CLIENT_CONDESC, *EC_PT_RAS_CLIENT_CONDESC;
#if __GNUC__ >= 8
#pragma GCC diagnostic pop
#endif
#include EC_PACKED_INCLUDESTOP

#define EC_RAS_CLIENT_SIGNATURE_PATTERN 0x9C300000
#define EC_RAS_CLIENT_SIGNATURE EC_SIGNATURE_MAKE(EC_RAS_CLIENT_SIGNATURE_PATTERN, EC_VERSION_MAJ, EC_VERSION_MIN, EC_VERSION_SERVICEPACK, EC_VERSION_BUILD)

#include EC_PACKED_INCLUDESTART(4)
#if __GNUC__ >= 8
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpacked-not-aligned"
#endif
typedef struct _EC_T_RAS_CLIENT_PARMS
{
    EC_T_DWORD          dwSignature;        /**< [in]   Set to EC_RAS_CLIENT_SIGNATURE */
    EC_T_DWORD          dwSize;             /**< [in]   Set to sizeof(EC_T_RAS_CLIENT_PARMS) */
    EC_T_LOG_PARMS      LogParms;

    EC_T_CPUSET         cpuAffinityMask;    /**< [in]   CPU affinity mask */
    EC_T_DWORD          dwAdmPrio;          /**< [in]   Priority of Administrative task */
    EC_T_DWORD          dwAdmStackSize;     /**< [in]   Stack size of Administrative task */
    EC_T_DWORD          dwCommThreadStackSize; /**< [in] Stack size of Communication task */

    EC_T_PVOID          pvNotifCtxt;        /**< [in]   Notification context returned while calling pfNotification */
    EC_PF_NOTIFY        pfNotification;     /**< [in]   Function pointer called to notify error and status
                                              *         information generated by Remote API Layer */
} EC_PACKED(4) EC_T_RAS_CLIENT_PARMS, *EC_PT_RAS_CLIENT_PARMS;
#if __GNUC__ >= 8
#pragma GCC diagnostic pop
#endif
#include EC_PACKED_INCLUDESTOP

/*-COMPILER SETTINGS---------------------------------------------------------*/
#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* INC_ECRASTYPE */

/*-END OF SOURCE FILE--------------------------------------------------------*/
