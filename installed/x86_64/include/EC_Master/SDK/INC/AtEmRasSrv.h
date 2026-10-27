/*-----------------------------------------------------------------------------
 * AtEmRasSrv.h             file
 * Copyright                acontis technologies GmbH, Ravensburg, Germany
 * Response                 Willig, Andreas
 * Description              description of file
 * Date                     2007/5/4::7:18
 *---------------------------------------------------------------------------*/

#ifndef INC_ATEMRASSRV
#define INC_ATEMRASSRV 1

/*-INCLUDES------------------------------------------------------------------*/
#ifndef INC_ECTYPE
#include "EcType.h"
#endif
#ifndef INC_ATEMRASERROR
#include "AtEmRasError.h"
#endif
#ifndef INC_ECOS
#include "EcOs.h"
#endif
#ifndef INC_ATEMRASTYPE
#include "AtEmRasType.h"
#endif
#ifndef INC_ATEMRASSRVVERSION
#include "AtEmRasSrvVersion.h"
#endif
#ifndef INC_ECINTERFACECOMMON
#include "EcInterfaceCommon.h"
#endif

/* legacy */
#ifndef EC_API
#define EC_API ATECAT_API
#endif
#ifndef EC_API_FNCALL
#define EC_API_FNCALL
#endif

/*-TYPEDEFS------------------------------------------------------------------*/
#include EC_PACKED_INCLUDESTART(4)
#if (defined INCLUDE_RAS_SPOCSUPPORT)
typedef enum _ATEMRAS_T_ORDINAL
{
    ord_emInitMaster                     = 201,  /* 0x00C9 */
    ord_emDeinitMaster                   = 202,  /* 0x00CA */
    ord_emStart                          = 203,  /* 0x00CB */
    ord_emStop                           = 204,  /* 0x00CC */
    ord_emIoControl                      = 205,  /* 0x00CD */
    ord_emGetSlaveId                     = 207,  /* 0x00CF */
    ord_emMbxTferCreate                  = 208,  /* 0x00D0 */
    ord_emMbxTferDelete                  = 209,  /* 0x00D1 */
    ord_emCoeSdoDownloadReq              = 210,  /* 0x00D2 */
    ord_emCoeSdoUploadReq                = 211,  /* 0x00D3 */
    ord_emCoeGetODList                   = 212,  /* 0x00D4 */
    ord_emCoeGetObjectDesc               = 213,  /* 0x00D5 */
    ord_emCoeGetEntryDesc                = 214,  /* 0x00D6 */
    ord_emGetSlaveProp                   = 218,  /* 0x00DA */
    ord_emGetSlaveState                  = 219,  /* 0x00DB */
    ord_emSetSlaveState                  = 220,  /* 0x00DC */
    ord_emTferSingleRawCmd               = 221,  /* 0x00DD */
    ord_emGetSlaveIdAtPosition           = 225,  /* 0x00E1 */
    ord_emGetNumConfiguredSlaves         = 226,  /* 0x00E2 */
    ord_emConfigureMaster                = 227,  /* 0x00E3 */
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
    ord_emPerfMeasInternalReset          = 284,  /* 0x011C */
    ord_emPerfMeasInternalGetRaw         = 285,  /* 0x011D */
    ord_emPerfMeasInternalGetInfo        = 286,  /* 0x011E */
    ord_emPerfMeasInternalGetNumOf       = 287,  /* 0x011F */
    ord_emSelfTestScan                   = 288,  /* 0x0120 */
    ord_emScanBus                        = 289,  /* 0x0121 */
    ord_emGetMemoryUsage                 = 290,  /* 0x0122 */

    ord_esConnectPorts                   = 605,  /* 0x025D */
    ord_esDisconnectPort                 = 606,  /* 0x025E */
    ord_esPowerSlave                     = 607,  /* 0x025F */
    ord_esSetErrorAtSlavePort                 = 616,  /* 0x0268 */
    ord_esSetErrorGenerationAtSlavePort       = 617,  /* 0x0269 */
    ord_esResetErrorGenerationAtSlavePorts    = 618,  /* 0x026A */
    ord_esSetLinkDownAtSlavePort              = 619,  /* 0x026B */
    ord_esSetLinkDownGenerationAtSlavePort    = 620,  /* 0x026C */
    ord_esResetLinkDownGenerationAtSlavePorts = 621,  /* 0x026D */

    ord_emGetMasterStateEx               = 622,  /* 0x026E */


    /* See also EC-Master API, EC-Simulator API and other branches! */

    /* Borland C++ datatype alignment correction */
    ord_BCppDummy                   = 0xFFFFFFFF
} ATEMRAS_T_ORDINAL;

#define PARAMETER_IGNORE        ((EC_T_DWORD)0xffffffff)


/* Default RAS SPOC access configuration */
#define RASSPOCCFGINITDEFAULT \
{ \
    /* \
      Level required                    Ordinal                           Index                                       Subindex            Reserved \
    */ \
    { ATEMRAS_ACCESS_LEVEL_ALLOW_ALL,  ord_emStart,                    PARAMETER_IGNORE,                           PARAMETER_IGNORE, 0 /* reserved */ }, \
    { ATEMRAS_ACCESS_LEVEL_ALLOW_ALL,  ord_emStop,                     PARAMETER_IGNORE,                           PARAMETER_IGNORE, 0 /* reserved */ }, \
    { ATEMRAS_ACCESS_LEVEL_ALLOW_ALL,  ord_emConfigureMaster,          PARAMETER_IGNORE,                           PARAMETER_IGNORE, 0 /* reserved */ }, \
    { ATEMRAS_ACCESS_LEVEL_ALLOW_ALL,  ord_emSetMasterState,           PARAMETER_IGNORE,                           PARAMETER_IGNORE, 0 /* reserved */ }, \
    { ATEMRAS_ACCESS_LEVEL_READONLY,   ord_emGetMasterState,           PARAMETER_IGNORE,                           PARAMETER_IGNORE, 0 /* reserved */ }, \
    { ATEMRAS_ACCESS_LEVEL_READONLY,   ord_emGetMasterStateEx,         PARAMETER_IGNORE,                           PARAMETER_IGNORE, 0 /* reserved */ }, \
    { ATEMRAS_ACCESS_LEVEL_ALLOW_ALL,  ord_emSetSlaveState,            PARAMETER_IGNORE,                           PARAMETER_IGNORE, 0 /* reserved */ }, \
    { ATEMRAS_ACCESS_LEVEL_READONLY,   ord_emGetSlaveState,            PARAMETER_IGNORE,                           PARAMETER_IGNORE, 0 /* reserved */ }, \
    /* Info configured slaves */ \
    { ATEMRAS_ACCESS_LEVEL_READONLY,   ord_emGetFixedAddr,             PARAMETER_IGNORE,                           PARAMETER_IGNORE, 0 /* reserved */ }, \
    { ATEMRAS_ACCESS_LEVEL_READONLY,   ord_emGetSlaveId,               PARAMETER_IGNORE,                           PARAMETER_IGNORE, 0 /* reserved */ }, \
    { ATEMRAS_ACCESS_LEVEL_READONLY,   ord_emGetNumConnectedSlaves,    PARAMETER_IGNORE,                           PARAMETER_IGNORE, 0 /* reserved */ }, \
    { ATEMRAS_ACCESS_LEVEL_READONLY,   ord_emGetNumConfiguredSlaves,   PARAMETER_IGNORE,                           PARAMETER_IGNORE, 0 /* reserved */ }, \
    { ATEMRAS_ACCESS_LEVEL_READONLY,   ord_emIsSlavePresent,           PARAMETER_IGNORE,                           PARAMETER_IGNORE, 0 /* reserved */ }, \
    { ATEMRAS_ACCESS_LEVEL_ALLOW_ALL,  ord_emSetSlaveDisabled,         PARAMETER_IGNORE,                           PARAMETER_IGNORE, 0 /* reserved */ }, \
    { ATEMRAS_ACCESS_LEVEL_READONLY,   ord_emGetSlaveProp,             PARAMETER_IGNORE,                           PARAMETER_IGNORE, 0 /* reserved */ }, \
    { ATEMRAS_ACCESS_LEVEL_READONLY,   ord_emGetSlaveIdAtPosition,     PARAMETER_IGNORE,                           PARAMETER_IGNORE, 0 /* reserved */ }, \
    { ATEMRAS_ACCESS_LEVEL_READONLY,   ord_emGetSlaveInfo,             PARAMETER_IGNORE,                           PARAMETER_IGNORE, 0 /* reserved */ }, \
    { ATEMRAS_ACCESS_LEVEL_READONLY,   ord_emIsSlavePresent,           PARAMETER_IGNORE,                           PARAMETER_IGNORE, 0 /* reserved */ }, \
    /* Process data */ \
    { ATEMRAS_ACCESS_LEVEL_READONLY,   ord_emGetProcessData,           PARAMETER_IGNORE,                           PARAMETER_IGNORE, 0 /* reserved */ }, \
    { ATEMRAS_ACCESS_LEVEL_ALLOW_ALL,  ord_emSetProcessData,           PARAMETER_IGNORE,                           PARAMETER_IGNORE, 0 /* reserved */ }, \
    { ATEMRAS_ACCESS_LEVEL_READONLY,   ord_emGetProcessDataBits,       PARAMETER_IGNORE,                           PARAMETER_IGNORE, 0 /* reserved */ }, \
    { ATEMRAS_ACCESS_LEVEL_ALLOW_ALL,  ord_emSetProcessDataBits,       PARAMETER_IGNORE,                           PARAMETER_IGNORE, 0 /* reserved */ }, \
    { ATEMRAS_ACCESS_LEVEL_READONLY,   ord_emGetSlaveProcVarInfoNumOf, PARAMETER_IGNORE,                           PARAMETER_IGNORE, 0 /* reserved */ }, \
    { ATEMRAS_ACCESS_LEVEL_READONLY,   ord_emGetSlaveProcVarInfo,      PARAMETER_IGNORE,                           PARAMETER_IGNORE, 0 /* reserved */ }, \
    { ATEMRAS_ACCESS_LEVEL_READONLY,   ord_emFindProcVarByName,        PARAMETER_IGNORE,                           PARAMETER_IGNORE, 0 /* reserved */ }, \
    /* Mailbox protocols */ \
    { ATEMRAS_ACCESS_LEVEL_READONLY,   ord_emMbxTferCreate,            PARAMETER_IGNORE,                           PARAMETER_IGNORE, 0 /* reserved */ }, \
    { ATEMRAS_ACCESS_LEVEL_READONLY,   ord_emMbxTferDelete,            PARAMETER_IGNORE,                           PARAMETER_IGNORE, 0 /* reserved */ }, \
    { ATEMRAS_ACCESS_LEVEL_READWRITE,  ord_emCoeSdoDownloadReq,        PARAMETER_IGNORE,                           PARAMETER_IGNORE, 0 /* reserved */ }, \
    { ATEMRAS_ACCESS_LEVEL_READONLY,   ord_emCoeSdoUploadReq,          PARAMETER_IGNORE,                           PARAMETER_IGNORE, 0 /* reserved */ }, \
    { ATEMRAS_ACCESS_LEVEL_READONLY,   ord_emCoeGetODList,             PARAMETER_IGNORE,                           PARAMETER_IGNORE, 0 /* reserved */ }, \
    { ATEMRAS_ACCESS_LEVEL_READONLY,   ord_emCoeGetObjectDesc,         PARAMETER_IGNORE,                           PARAMETER_IGNORE, 0 /* reserved */ }, \
    { ATEMRAS_ACCESS_LEVEL_READONLY,   ord_emCoeGetEntryDesc,          PARAMETER_IGNORE,                           PARAMETER_IGNORE, 0 /* reserved */ }, \
    { ATEMRAS_ACCESS_LEVEL_READONLY,   ord_emCoeSdoUpload,             PARAMETER_IGNORE,                           PARAMETER_IGNORE, 0 /* reserved */ }, \
    { ATEMRAS_ACCESS_LEVEL_READWRITE,  ord_emCoeSdoDownload,           PARAMETER_IGNORE,                           PARAMETER_IGNORE, 0 /* reserved */ }, \
    { ATEMRAS_ACCESS_LEVEL_READONLY,   ord_emFoeFileUpload,            PARAMETER_IGNORE,                           PARAMETER_IGNORE, 0 /* reserved */ }, \
    { ATEMRAS_ACCESS_LEVEL_READWRITE,  ord_emFoeFileDownload,          PARAMETER_IGNORE,                           PARAMETER_IGNORE, 0 /* reserved */ }, \
    { ATEMRAS_ACCESS_LEVEL_READONLY,   ord_emFoeUpoadReq,              PARAMETER_IGNORE,                           PARAMETER_IGNORE, 0 /* reserved */ }, \
    { ATEMRAS_ACCESS_LEVEL_READWRITE,  ord_emFoeDownloadReq,           PARAMETER_IGNORE,                           PARAMETER_IGNORE, 0 /* reserved */ }, \
    { ATEMRAS_ACCESS_LEVEL_READWRITE,  ord_emAoeWriteReq,              PARAMETER_IGNORE,                           PARAMETER_IGNORE, 0 /* reserved */ }, \
    { ATEMRAS_ACCESS_LEVEL_READONLY,   ord_emAoeReadReq,               PARAMETER_IGNORE,                           PARAMETER_IGNORE, 0 /* reserved */ }, \
    { ATEMRAS_ACCESS_LEVEL_READWRITE,  ord_emAoeWrite,                 PARAMETER_IGNORE,                           PARAMETER_IGNORE, 0 /* reserved */ }, \
    { ATEMRAS_ACCESS_LEVEL_READONLY,   ord_emAoeRead,                  PARAMETER_IGNORE,                           PARAMETER_IGNORE, 0 /* reserved */ }, \
    { ATEMRAS_ACCESS_LEVEL_READONLY,   ord_emAoeGetSlaveNetId,         PARAMETER_IGNORE,                           PARAMETER_IGNORE, 0 /* reserved */ }, \
    { ATEMRAS_ACCESS_LEVEL_READWRITE,  ord_emAoeReadWrite,             PARAMETER_IGNORE,                           PARAMETER_IGNORE, 0 /* reserved */ }, \
    { ATEMRAS_ACCESS_LEVEL_READONLY,   ord_emSoeRead,                  PARAMETER_IGNORE,                           PARAMETER_IGNORE, 0 /* reserved */ }, \
    { ATEMRAS_ACCESS_LEVEL_READWRITE,  ord_emSoeWrite,                 PARAMETER_IGNORE,                           PARAMETER_IGNORE, 0 /* reserved */ }, \
    { ATEMRAS_ACCESS_LEVEL_READWRITE,  ord_emSoeAbortProcCmd,          PARAMETER_IGNORE,                           PARAMETER_IGNORE, 0 /* reserved */ }, \
    { ATEMRAS_ACCESS_LEVEL_READWRITE,  ord_emAoeWriteControl,          PARAMETER_IGNORE,                           PARAMETER_IGNORE, 0 /* reserved */ }, \
    { ATEMRAS_ACCESS_LEVEL_READWRITE,  ord_emSetMbxProtocolsSerialize, PARAMETER_IGNORE,                           PARAMETER_IGNORE, 0 /* reserved */ }, \
    /* Misc.*/ \
    { ATEMRAS_ACCESS_LEVEL_ALLOW_ALL,  ord_emResetSlaveController,     PARAMETER_IGNORE,                           PARAMETER_IGNORE, 0 /* reserved */ }, \
    { ATEMRAS_ACCESS_LEVEL_ALLOW_ALL,  ord_emReloadSlaveEEPRom,        PARAMETER_IGNORE,                           PARAMETER_IGNORE, 0 /* reserved */ }, \
    { ATEMRAS_ACCESS_LEVEL_READONLY,   ord_emReadSlaveEEPRom,          PARAMETER_IGNORE,                           PARAMETER_IGNORE, 0 /* reserved */ }, \
    { ATEMRAS_ACCESS_LEVEL_ALLOW_ALL,  ord_emWriteSlaveEEPRom,         PARAMETER_IGNORE,                           PARAMETER_IGNORE, 0 /* reserved */ }, \
    { ATEMRAS_ACCESS_LEVEL_ALLOW_ALL,  ord_emAssignSlaveEEPRom,        PARAMETER_IGNORE,                           PARAMETER_IGNORE, 0 /* reserved */ }, \
    { ATEMRAS_ACCESS_LEVEL_READONLY,   ord_emIoControl,                EC_IOCTL_REGISTERCLIENT,                    PARAMETER_IGNORE, 0 /* reserved */ }, \
    { ATEMRAS_ACCESS_LEVEL_READONLY,   ord_emIoControl,                EC_IOCTL_UNREGISTERCLIENT,                  PARAMETER_IGNORE, 0 /* reserved */ }, \
    { ATEMRAS_ACCESS_LEVEL_READONLY,   ord_emIoControl,                EC_IOCTL_ISLINK_CONNECTED,                  PARAMETER_IGNORE, 0 /* reserved */ }, \
    { ATEMRAS_ACCESS_LEVEL_READONLY,   ord_emIoControl,                EC_IOCTL_GET_PDMEMORYSIZE,                  PARAMETER_IGNORE, 0 /* reserved */ }, \
    { ATEMRAS_ACCESS_LEVEL_ALLOW_ALL,  ord_emIoControl,                EC_IOCTL_SLAVE_LINKMESSAGES,                PARAMETER_IGNORE, 0 /* reserved */ }, \
    { ATEMRAS_ACCESS_LEVEL_READONLY,   ord_emIoControl,                EC_IOCTL_DC_SLV_SYNC_STATUS_GET,            PARAMETER_IGNORE, 0 /* reserved */ }, \
    { ATEMRAS_ACCESS_LEVEL_ALLOW_ALL,  ord_emIoControl,                EC_IOCTL_SB_RESTART,                        PARAMETER_IGNORE, 0 /* reserved */ }, \
    { ATEMRAS_ACCESS_LEVEL_READONLY,   ord_emIoControl,                EC_IOCTL_SB_STATUS_GET,                     PARAMETER_IGNORE, 0 /* reserved */ }, \
    { ATEMRAS_ACCESS_LEVEL_ALLOW_ALL,  ord_emIoControl,                EC_IOCTL_SB_SET_BUSCNF_VERIFY,              PARAMETER_IGNORE, 0 /* reserved */ }, \
    { ATEMRAS_ACCESS_LEVEL_ALLOW_ALL,  ord_emIoControl,                EC_IOCTL_SB_SET_BUSCNF_VERIFY_PROP,         PARAMETER_IGNORE, 0 /* reserved */ }, \
    { ATEMRAS_ACCESS_LEVEL_READONLY,   ord_emIoControl,                EC_IOCTL_SB_BUSCNF_GETSLAVE_INFO,           PARAMETER_IGNORE, 0 /* reserved */ }, \
    { ATEMRAS_ACCESS_LEVEL_READONLY,   ord_emIoControl,                EC_IOCTL_SB_BUSCNF_GETSLAVE_INFO_EEP,       PARAMETER_IGNORE, 0 /* reserved */ }, \
    { ATEMRAS_ACCESS_LEVEL_ALLOW_ALL,  ord_emIoControl,                EC_IOCTL_SB_ENABLE,                         PARAMETER_IGNORE, 0 /* reserved */ }, \
    { ATEMRAS_ACCESS_LEVEL_READONLY,   ord_emIoControl,                EC_IOCTL_SB_BUSCNF_GETSLAVE_INFO_EX,        PARAMETER_IGNORE, 0 /* reserved */ }, \
    { ATEMRAS_ACCESS_LEVEL_ALLOW_ALL,  ord_emIoControl,                EC_IOCTL_SLV_ALIAS_ENABLE,                  PARAMETER_IGNORE, 0 /* reserved */ }, \
    { ATEMRAS_ACCESS_LEVEL_ALLOW_ALL,  ord_emIoControl,                EC_IOCTL_SET_SLVSTAT_PERIOD,                PARAMETER_IGNORE, 0 /* reserved */ }, \
    { ATEMRAS_ACCESS_LEVEL_ALLOW_ALL,  ord_emIoControl,                EC_IOCTL_GET_SLVSTAT_PERIOD,                PARAMETER_IGNORE, 0 /* reserved */ }, \
    { ATEMRAS_ACCESS_LEVEL_ALLOW_ALL,  ord_emIoControl,                EC_IOCTL_FORCE_SLVSTAT_COLLECTION,          PARAMETER_IGNORE, 0 /* reserved */ }, \
    { ATEMRAS_ACCESS_LEVEL_READONLY,   ord_emIoControl,                EC_IOCTL_GET_SLVSTATISTICS,                 PARAMETER_IGNORE, 0 /* reserved */ }, \
    { ATEMRAS_ACCESS_LEVEL_BLOCK_ALL,  ord_emTferSingleRawCmd,         0x00000000,                                 PARAMETER_IGNORE, 0 /* reserved */ }, \
    { ATEMRAS_ACCESS_LEVEL_READONLY,   ord_emTferSingleRawCmd,         0x00000001,                                 PARAMETER_IGNORE, 0 /* reserved */ }, \
    { ATEMRAS_ACCESS_LEVEL_READONLY,   ord_emTferSingleRawCmd,         0x00000004,                                 PARAMETER_IGNORE, 0 /* reserved */ }, \
    { ATEMRAS_ACCESS_LEVEL_READONLY,   ord_emTferSingleRawCmd,         0x00000007,                                 PARAMETER_IGNORE, 0 /* reserved */ }, \
    { ATEMRAS_ACCESS_LEVEL_READONLY,   ord_emTferSingleRawCmd,         0x0000000A,                                 PARAMETER_IGNORE, 0 /* reserved */ }, \
    { ATEMRAS_ACCESS_LEVEL_ALLOW_ALL,  ord_emTferSingleRawCmd,         0x00000002,                                 PARAMETER_IGNORE, 0 /* reserved */ }, \
    { ATEMRAS_ACCESS_LEVEL_ALLOW_ALL,  ord_emTferSingleRawCmd,         0x00000003,                                 PARAMETER_IGNORE, 0 /* reserved */ }, \
    { ATEMRAS_ACCESS_LEVEL_ALLOW_ALL,  ord_emTferSingleRawCmd,         0x00000005,                                 PARAMETER_IGNORE, 0 /* reserved */ }, \
    { ATEMRAS_ACCESS_LEVEL_ALLOW_ALL,  ord_emTferSingleRawCmd,         0x00000006,                                 PARAMETER_IGNORE, 0 /* reserved */ }, \
    { ATEMRAS_ACCESS_LEVEL_ALLOW_ALL,  ord_emTferSingleRawCmd,         0x00000008,                                 PARAMETER_IGNORE, 0 /* reserved */ }, \
    { ATEMRAS_ACCESS_LEVEL_ALLOW_ALL,  ord_emTferSingleRawCmd,         0x00000009,                                 PARAMETER_IGNORE, 0 /* reserved */ }, \
    { ATEMRAS_ACCESS_LEVEL_ALLOW_ALL,  ord_emTferSingleRawCmd,         0x0000000B,                                 PARAMETER_IGNORE, 0 /* reserved */ }, \
    { ATEMRAS_ACCESS_LEVEL_ALLOW_ALL,  ord_emTferSingleRawCmd,         0x0000000C,                                 PARAMETER_IGNORE, 0 /* reserved */ }, \
    { ATEMRAS_ACCESS_LEVEL_ALLOW_ALL,  ord_emTferSingleRawCmd,         0x0000000D,                                 PARAMETER_IGNORE, 0 /* reserved */ }, \
    { ATEMRAS_ACCESS_LEVEL_ALLOW_ALL,  ord_emTferSingleRawCmd,         0x0000000E,                                 PARAMETER_IGNORE, 0 /* reserved */ }, \
    { ATEMRAS_ACCESS_LEVEL_BLOCK_ALL,  ord_emQueueRawCmd,              EC_CMD_TYPE_NOP,                            PARAMETER_IGNORE, 0 /* reserved */ }, \
    { ATEMRAS_ACCESS_LEVEL_READONLY,   ord_emQueueRawCmd,              EC_CMD_TYPE_APRD,                           PARAMETER_IGNORE, 0 /* reserved */ }, \
    { ATEMRAS_ACCESS_LEVEL_READONLY,   ord_emQueueRawCmd,              EC_CMD_TYPE_FPRD,                           PARAMETER_IGNORE, 0 /* reserved */ }, \
    { ATEMRAS_ACCESS_LEVEL_READONLY,   ord_emQueueRawCmd,              EC_CMD_TYPE_BRD,                            PARAMETER_IGNORE, 0 /* reserved */ }, \
    { ATEMRAS_ACCESS_LEVEL_READONLY,   ord_emQueueRawCmd,              EC_CMD_TYPE_LRD,                            PARAMETER_IGNORE, 0 /* reserved */ }, \
    { ATEMRAS_ACCESS_LEVEL_READWRITE,  ord_emQueueRawCmd,              EC_CMD_TYPE_APWR,                           PARAMETER_IGNORE, 0 /* reserved */ }, \
    { ATEMRAS_ACCESS_LEVEL_READWRITE,  ord_emQueueRawCmd,              EC_CMD_TYPE_APRW,                           PARAMETER_IGNORE, 0 /* reserved */ }, \
    { ATEMRAS_ACCESS_LEVEL_READWRITE,  ord_emQueueRawCmd,              EC_CMD_TYPE_FPWR,                           PARAMETER_IGNORE, 0 /* reserved */ }, \
    { ATEMRAS_ACCESS_LEVEL_READWRITE,  ord_emQueueRawCmd,              EC_CMD_TYPE_FPRW,                           PARAMETER_IGNORE, 0 /* reserved */ }, \
    { ATEMRAS_ACCESS_LEVEL_READWRITE,  ord_emQueueRawCmd,              EC_CMD_TYPE_BWR,                            PARAMETER_IGNORE, 0 /* reserved */ }, \
    { ATEMRAS_ACCESS_LEVEL_READWRITE,  ord_emQueueRawCmd,              EC_CMD_TYPE_BRW,                            PARAMETER_IGNORE, 0 /* reserved */ }, \
    { ATEMRAS_ACCESS_LEVEL_READWRITE,  ord_emQueueRawCmd,              EC_CMD_TYPE_LWR,                            PARAMETER_IGNORE, 0 /* reserved */ }, \
    { ATEMRAS_ACCESS_LEVEL_READWRITE,  ord_emQueueRawCmd,              EC_CMD_TYPE_LRW,                            PARAMETER_IGNORE, 0 /* reserved */ }, \
    { ATEMRAS_ACCESS_LEVEL_READWRITE,  ord_emQueueRawCmd,              EC_CMD_TYPE_ARMW,                           PARAMETER_IGNORE, 0 /* reserved */ }, \
    { ATEMRAS_ACCESS_LEVEL_READWRITE,  ord_emQueueRawCmd,              EC_CMD_TYPE_FRMW,                           PARAMETER_IGNORE, 0 /* reserved */ }, \
    { ATEMRAS_ACCESS_LEVEL_READONLY,   ord_emGetCfgSlaveInfo,          PARAMETER_IGNORE,                           PARAMETER_IGNORE, 0 /* reserved */ }, \
    { ATEMRAS_ACCESS_LEVEL_READONLY,   ord_emGetBusSlaveInfo,          PARAMETER_IGNORE,                           PARAMETER_IGNORE, 0 /* reserved */ }, \
    { ATEMRAS_ACCESS_LEVEL_READWRITE,  ord_emReadSlaveIdentification,  PARAMETER_IGNORE,                           PARAMETER_IGNORE, 0 /* reserved */ }, \
    { ATEMRAS_ACCESS_LEVEL_ALLOW_ALL,  ord_emSetSlaveDisconnected,     PARAMETER_IGNORE,                           PARAMETER_IGNORE, 0 /* reserved */ }, \
    { ATEMRAS_ACCESS_LEVEL_ALLOW_ALL,  ord_emRescueScan,               PARAMETER_IGNORE,                           PARAMETER_IGNORE, 0 /* reserved */ }, \
    { ATEMRAS_ACCESS_LEVEL_ALLOW_ALL,  ord_emConfigExtend,             PARAMETER_IGNORE,                           PARAMETER_IGNORE, 0 /* reserved */ }, \
    { ATEMRAS_ACCESS_LEVEL_ALLOW_ALL,  ord_emSetSlavesDisabled,        PARAMETER_IGNORE,                           PARAMETER_IGNORE, 0 /* reserved */ }, \
    { ATEMRAS_ACCESS_LEVEL_ALLOW_ALL,  ord_emSetSlavesDisconnected,    PARAMETER_IGNORE,                           PARAMETER_IGNORE, 0 /* reserved */ }, \
    { ATEMRAS_ACCESS_LEVEL_READONLY,   ord_emIsConfigured,             PARAMETER_IGNORE,                           PARAMETER_IGNORE, 0 /* reserved */ }, \
    { ATEMRAS_ACCESS_LEVEL_ALLOW_ALL,  ord_emPerfMeasInternalReset,    PARAMETER_IGNORE,                           PARAMETER_IGNORE, 0 /* reserved */ }, \
    { ATEMRAS_ACCESS_LEVEL_READONLY,   ord_emPerfMeasInternalGetRaw,   PARAMETER_IGNORE,                           PARAMETER_IGNORE, 0 /* reserved */ }, \
    { ATEMRAS_ACCESS_LEVEL_READONLY,   ord_emPerfMeasInternalGetInfo,  PARAMETER_IGNORE,                           PARAMETER_IGNORE, 0 /* reserved */ }, \
    { ATEMRAS_ACCESS_LEVEL_READONLY,   ord_emPerfMeasInternalGetNumOf, PARAMETER_IGNORE,                           PARAMETER_IGNORE, 0 /* reserved */ }, \
    { ATEMRAS_ACCESS_LEVEL_ALLOW_ALL,  ord_emScanBus,                  PARAMETER_IGNORE,                           PARAMETER_IGNORE, 0 /* reserved */ }, \
    { ATEMRAS_ACCESS_LEVEL_ALLOW_ALL,  ord_emGetMemoryUsage,           PARAMETER_IGNORE,                           PARAMETER_IGNORE, 0 /* reserved */ } \
}
#endif /* INCLUDE_RAS_SPOCSUPPORT */

typedef struct _ATEMRAS_T_SPOCCFG
{
    EC_T_DWORD              dwAccessLevel; /**< [in]   see ATEMRAS_ACCESS_LEVEL_..., e.g. ATEMRAS_ACCESS_LEVEL_READWRITE */
    EC_T_DWORD              dwOrdinal;     /**< [in]   see ord_..., e.g. ord_emSetMasterState */
    EC_T_DWORD              dwIndex;       /**< [in]   set to PARAMETER_IGNORE, if not needed */
    EC_T_DWORD              dwSubIndex;    /**< [in]   set to PARAMETER_IGNORE, if not needed */
    EC_T_DWORD              dwReserved;
} EC_PACKED(4) ATEMRAS_T_SPOCCFG;

#define ATEMRASSRV_SIGNATURE_PATTERN                    0xEAC00000
#define ATEMRASSRV_SIGNATURE (  ATEMRASSRV_SIGNATURE_PATTERN       \
                             | (ATEMRASSRV_VERS_MAJ         << 16) \
                             | (ATEMRASSRV_VERS_MIN         << 12) \
                             | (ATEMRASSRV_VERS_SERVICEPACK <<  8) \
                             | (ATEMRASSRV_VERS_BUILD       <<  0) \
                             )
typedef struct _ATEMRAS_T_SRVPARMS
{
    EC_T_DWORD          dwSignature;            /**< [in]   Set to ATEMRASSRV_SIGNATURE */
    EC_T_DWORD          dwSize;                 /**< [in]   Set to sizeof(ATEMRAS_T_SRVPARMS) */
    EC_T_LOG_PARMS      LogParms;               /**< [in]   Logging parameters */

    ATEMRAS_T_IPADDR    oAddr;                  /**< [in]   Server Bind IP Address */
    EC_T_WORD           wPort;                  /**< [in]   Server Bind IP Port */
    EC_T_WORD           wMaxClientCnt;          /**< [in]   Max. clients in parallel (0: unlimited) */
    EC_T_DWORD          dwCycleTime;            /**< [in]   Cycle Time of RAS Network access (acceptor, worker) */

    EC_T_DWORD          dwCommunicationTimeout; /**< [in]   timeout before automatically closing connection */

    /* Settings for thread accepting new client connections and spawning corresponding worker threads */
    EC_T_CPUSET         oAcceptorThreadCpuAffinityMask;     /**< [in]   Acceptor Thread CPU affinity mask */
    EC_T_DWORD          dwAcceptorThreadPrio;               /**< [in]   Acceptor Thread Priority */
    EC_T_DWORD          dwAcceptorThreadStackSize;          /**< [in]   Acceptor Thread Stack Size */

    /* Settings for worker threads handling requests from corresponding client connections */
    EC_T_CPUSET         oClientWorkerThreadCpuAffinityMask; /**< [in]   Client Worker Thread CPU affinity mask */
    EC_T_DWORD          dwClientWorkerThreadPrio;           /**< [in]   Client Worker Thread Priority */
    EC_T_DWORD          dwClientWorkerThreadStackSize;      /**< [in]   Client Worker Thread Stack Size */

    EC_T_DWORD          dwMaxQueuedNotificationCnt;     /**< [in]   Amount of concurrently queue able Notifications */

    EC_T_DWORD          dwMaxParallelMbxTferCnt; /**< [in]   Amount of concurrent active mailbox transfers */

    EC_PF_NOTIFY        pfnRasNotify;            /**< [in]   Function pointer called to notify error and status
                                                  *          information generated by Remote API Layer */
    EC_T_VOID*          pvRasNotifyCtxt;         /**< [in]   Notification context returned while calling pfNotification */
    EC_T_DWORD          dwCycErrInterval;        /**< [in]   Interval which allows cyclic Notifications */
} EC_PACKED(4) ATEMRAS_T_SRVPARMS, *ATEMRAS_PT_SRVPARMS;
#include EC_PACKED_INCLUDESTOP

/*-FUNCTION DECLARATION------------------------------------------------------*/
#ifdef __cplusplus
extern "C"
{
#endif

/* AtemRasSrv API used with EC-Master */
EC_API EC_T_DWORD EC_API_FNCALL emRasSrvGetVersion( EC_T_VOID );
EC_API EC_T_DWORD EC_API_FNCALL emRasSrvStart( ATEMRAS_T_SRVPARMS* pParms, EC_T_PVOID* ppHandle);
EC_API EC_T_DWORD EC_API_FNCALL emRasSrvStop( EC_T_PVOID pvHandle, EC_T_DWORD dwTimeout);
EC_API const EC_T_CHAR* EC_API_FNCALL emRasErrorText(EC_T_DWORD dwError);
EC_API const EC_T_CHAR* EC_API_FNCALL emRasEventText(EC_T_DWORD dwEvent);

#if (defined INCLUDE_RAS_TRACESUPPORT)
EC_API EC_T_DWORD EC_API_FNCALL emRasSrvTraceEnable(EC_T_BOOL bEnable);
#endif

EC_API EC_T_DWORD EC_API_FNCALL emRasSrvSetAccessLevel(EC_T_PVOID pvHandle, EC_T_DWORD dwAccessLevel);
EC_API EC_T_DWORD EC_API_FNCALL emRasSrvGetAccessLevel(EC_T_PVOID pvHandle, EC_T_DWORD* pdwAccessLevel);
EC_API EC_T_DWORD EC_API_FNCALL emRasSrvConfigAccessLevel(EC_T_PVOID pvHandle, ATEMRAS_T_SPOCCFG* pCfgData, EC_T_DWORD dwCfgDataCnt);
EC_API EC_T_DWORD EC_API_FNCALL emRasSrvSetAccessControl(EC_T_PVOID pvHandle, EC_T_BOOL bActive);
EC_API EC_T_DWORD EC_API_FNCALL emRasSrvSetCallAccessLevel(
    EC_T_PVOID      pvHandle        /**< [in]   Handle to previously started Server */
    , EC_T_DWORD    dwOrdinal       /**< [in]   Function call ID */
    , EC_T_DWORD    dwIndex         /**< [in]   Function call index */
    , EC_T_DWORD    dwSubIndex      /**< [in]   Function call subindex */
    , EC_T_DWORD    dwAccessLevel   /**< [in]   New Access level */
    );


EC_API EC_T_DWORD EC_API_FNCALL emRasGetMemoryUsage(EC_T_PVOID pvHandle, EC_T_DWORD* pdwCurrentUsage, EC_T_DWORD* pdwMaxUsage);

/* AtemRasSrv API used with EC-Simulator */
EC_API EC_T_DWORD EC_API_FNCALL esRasSrvGetVersion( EC_T_VOID );
EC_API EC_T_DWORD EC_API_FNCALL esRasSrvStart( ATEMRAS_T_SRVPARMS* pParms, EC_T_PVOID* ppHandle);
EC_API EC_T_DWORD EC_API_FNCALL esRasSrvStop( EC_T_PVOID pvHandle, EC_T_DWORD dwTimeout);
EC_API const EC_T_CHAR* EC_API_FNCALL esRasErrorText(EC_T_DWORD dwError);
EC_API const EC_T_CHAR* EC_API_FNCALL esRasEventText(EC_T_DWORD dwEvent);

#if (defined INCLUDE_RAS_TRACESUPPORT)
EC_API EC_T_DWORD EC_API_FNCALL esRasSrvTraceEnable(EC_T_BOOL bEnable);
#endif

EC_API EC_T_DWORD EC_API_FNCALL esRasSrvSetAccessLevel(EC_T_PVOID pvHandle, EC_T_DWORD dwAccessLevel);
EC_API EC_T_DWORD EC_API_FNCALL esRasSrvGetAccessLevel(EC_T_PVOID pvHandle, EC_T_DWORD* pdwAccessLevel);
EC_API EC_T_DWORD EC_API_FNCALL esRasSrvConfigAccessLevel(EC_T_PVOID pvHandle, ATEMRAS_T_SPOCCFG* poConfigData, EC_T_DWORD dwCnt);
EC_API EC_T_DWORD EC_API_FNCALL esRasSrvSetAccessControl(EC_T_PVOID pvHandle, EC_T_BOOL bActive);
EC_API EC_T_DWORD EC_API_FNCALL esRasSrvSetCallAccessLevel(
    EC_T_PVOID      pvHandle        /**< [in]   Handle to previously started Server */
    , EC_T_DWORD    dwOrdinal       /**< [in]   Function call ID */
    , EC_T_DWORD    dwIndex         /**< [in]   Function call index */
    , EC_T_DWORD    dwSubIndex      /**< [in]   Function call subindex */
    , EC_T_DWORD    dwAccessLevel   /**< [in]   New Access level */
    );

EC_API EC_T_DWORD EC_API_FNCALL esRasGetMemoryUsage(EC_T_PVOID pvHandle, EC_T_DWORD* pdwCurrentUsage, EC_T_DWORD* pdwMaxUsage);

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* INC_ATESRASSRV */

  /*-END OF SOURCE FILE--------------------------------------------------------*/
