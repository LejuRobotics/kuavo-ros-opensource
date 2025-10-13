/*-----------------------------------------------------------------------------
 * EcInterfaceCommon.h
 * Copyright                acontis technologies GmbH, Ravensburg, Germany
 * Response                 Stefan Zintgraf
 * Description              EtherCAT Master common definitions for interfaces
 *---------------------------------------------------------------------------*/

#ifndef INC_ECINTERFACECOMMON
#define INC_ECINTERFACECOMMON 1

/*-INCLUDES------------------------------------------------------------------*/
#if (!defined INC_ECOS) && (!defined INC_LINK_OS_LAYER)
#error EcOs.h / LinkOsLayer.h include missing!
#endif
#ifndef INC_ECVERSION
#include "EcVersion.h"
#endif
#ifndef INC_ECLOG
#include "EcLog.h"
#endif
#ifndef INC_ECESCREG
#include "EcEscReg.h"
#endif
#ifndef INC_ETHERNETSERVICES
#include "EthernetServices.h"
#endif

/*-COMPILER SETTINGS---------------------------------------------------------*/
#ifdef __cplusplus
extern "C"
{
#endif

/*-DEFINES/MACROS------------------------------------------------------------*/

/* EtherCat specific control codes */
#define EC_IOCTL_GENERIC                            0x00000000
#define EC_IOCTL_DC                                 0x00030000
#define EC_IOCTL_SB                                 0x00050000
#define EC_IOCTL_HC                                 0x00060000
#define EC_IOCTL_DCM                                0x00070000
#define EC_IOCTL_USER                               0x00F00000  /* for user extension */
#define EC_IOCTL_PRIVATE                            0x00FF0000  /* private, internal IOCTL values */
#define EC_IOCTL_LINKLAYER                          0xCA000000
#define EC_IOCTL_LINKLAYER_MAIN                     EC_IOCTL_LINKLAYER
#define EC_IOCTL_LINKLAYER_RED                      0xCB000000
#define EC_IOCTL_LINKLAYER_LAST                     0xCBFFFFFF
#define EC_IOCTL_SIMULATOR                          0xCC000000
#define EC_IOCTL_SIMULATOR_LAST                     0xCCFFFFFF
#define EC_IOCTL_PRIVATE2                           0xCD000000  /* private, internal IOCTL values */

#define EC_IOCTL_REGISTERCLIENT                         (EC_IOCTL_GENERIC |  2)
#define EC_IOCTL_UNREGISTERCLIENT                       (EC_IOCTL_GENERIC |  3)

#define EC_IOCTL_ISLINK_CONNECTED                       (EC_IOCTL_GENERIC |  6)

#define EC_IOCTL_SET_FRAME_RESPONSE_ERROR_NOTIFY_MASK   (EC_IOCTL_GENERIC |  8)

#define EC_IOCTL_LINKLAYER_DBG_MSG                      (EC_IOCTL_GENERIC | 10)

#define EC_IOCTL_RESET_SLAVE                            (EC_IOCTL_GENERIC | 13)
#define EC_IOCTL_SLAVE_LINKMESSAGES                     (EC_IOCTL_GENERIC | 14)
#define EC_IOCTL_GET_CYCLIC_CONFIG_INFO                 (EC_IOCTL_GENERIC | 15)
#define EC_IOCTL_GET_LINKLAYER_MODE                     (EC_IOCTL_GENERIC | 16)
#define EC_IOCTL_IS_SLAVETOSLAVE_COMM_CONFIGURED        (EC_IOCTL_GENERIC | 17)
#define EC_IOCTL_INITIATE_UPDATE_ALL_SLAVE_STATE        (EC_IOCTL_GENERIC | 19)
#define EC_IOCTL_ADD_BRD_SYNC_WINDOW_MONITORING         (EC_IOCTL_GENERIC | 20)
#define EC_IOCTL_ONLY_PROCESS_DATA_IN_IMAGE             (EC_IOCTL_GENERIC | 21)
#define EC_IOCTL_REGISTER_CYCFRAME_RX_CB                (EC_IOCTL_GENERIC | 22)
#define EC_IOCTL_SET_PD_OFFSET_COMPAT_MODE              (EC_IOCTL_GENERIC | 23)
#define EC_IOCTL_IS_MAIN_LINK_CONNECTED                 (EC_IOCTL_GENERIC | 24)
#define EC_IOCTL_IS_RED_LINK_CONNECTED                  (EC_IOCTL_GENERIC | 25)
#define EC_IOCTL_ADD_COE_INITCMD                        (EC_IOCTL_GENERIC | 26)

/* Memory Provider */
#define EC_IOCTL_GET_PDMEMORYSIZE                       (EC_IOCTL_GENERIC | 40)
#define EC_IOCTL_REGISTER_PDMEMORYPROVIDER              (EC_IOCTL_GENERIC | 41)

#define EC_IOCTL_FORCE_BROADCAST_DESTINATION            (EC_IOCTL_GENERIC | 42)     /* obsolete */

/* Slave Statistics Retrieval */
#define EC_IOCTL_SET_SLVSTAT_PERIOD                     (EC_IOCTL_GENERIC | 43)
#define EC_IOCTL_FORCE_SLVSTAT_COLLECTION               (EC_IOCTL_GENERIC | 44)
#define EC_IOCTL_GET_SLVSTATISTICS                      (EC_IOCTL_GENERIC | 45)
#define EC_IOCTL_CLR_SLVSTATISTICS                      (EC_IOCTL_GENERIC | 46)

#define EC_IOCTL_SET_MBX_RETRYACCESS_COUNT              (EC_IOCTL_GENERIC | 47)
#define EC_IOCTL_SET_MBX_RETRYACCESS_PERIOD             (EC_IOCTL_GENERIC | 48)

#define EC_IOCTL_ALL_SLAVES_MUST_REACH_MASTER_STATE     (EC_IOCTL_GENERIC | 49)

#define EC_IOCTL_SET_NOTIFICATION_CTL                   (EC_IOCTL_GENERIC | 50)

#define EC_IOCTL_MASTEROD_SET_VALUE                     (EC_IOCTL_GENERIC | 51)
#define EC_IOCTL_SET_CYCFRAME_LAYOUT                    (EC_IOCTL_GENERIC | 52)

#define EC_IOCTL_SET_NOTIFICATION_ENABLED               (EC_IOCTL_GENERIC | 53)
#define EC_IOCTL_GET_NOTIFICATION_ENABLED               (EC_IOCTL_GENERIC | 54)

#define EC_IOCTL_SET_MASTER_DEFAULT_TIMEOUTS                    (EC_IOCTL_GENERIC | 55)
#define EC_IOCTL_SET_COPYINFO_IN_SENDCYCFRAMES                  (EC_IOCTL_GENERIC | 56)
#define EC_IOCTL_SET_BUS_CYCLE_TIME                             (EC_IOCTL_GENERIC | 57)
#define EC_IOCTL_ADDITIONAL_VARIABLES_FOR_SPECIFIC_DATA_TYPES   (EC_IOCTL_GENERIC | 58)
#define EC_IOCTL_SET_IGNORE_INPUTS_ON_WKC_ERROR                 (EC_IOCTL_GENERIC | 59)
#define EC_IOCTL_SET_GENENI_ASSIGN_EEPROM_BACK_TO_ECAT          (EC_IOCTL_GENERIC | 60)
#define EC_IOCTL_SET_AUTO_ACK_AL_STATUS_ERROR_ENABLED           (EC_IOCTL_GENERIC | 61)
#define EC_IOCTL_SET_AUTO_ADJUST_CYCCMD_WKC_ENABLED             (EC_IOCTL_GENERIC | 62)
#define EC_IOCTL_CLEAR_MASTER_INFO_COUNTERS                     (EC_IOCTL_GENERIC | 63)
#define EC_IOCTL_SET_SPLIT_FRAME_PROCESSING_ENABLED             (EC_IOCTL_GENERIC | 64)
#define EC_IOCTL_SET_SPLITTED_FRAME_PROCESSING_ENABLED          EC_IOCTL_SET_SPLIT_FRAME_PROCESSING_ENABLED
#define EC_IOCTL_SET_ADJUST_CYCFRAMES_AFTER_SLAVES_STATE_CHANGE (EC_IOCTL_GENERIC | 65)
#define EC_IOCTL_GET_SLVSTAT_PERIOD                             (EC_IOCTL_GENERIC | 66)
#define EC_IOCTL_SET_EOE_DEFFERED_SWITCHING_ENABLED             (EC_IOCTL_GENERIC | 67)
#define EC_IOCTL_SET_NEW_BUSSLAVES_TO_INIT                      (EC_IOCTL_GENERIC | 68)
#define EC_IOCTL_SET_ZERO_INPUTS_ON_WKC_ZERO                    (EC_IOCTL_GENERIC | 69)
#define EC_IOCTL_SET_ZERO_INPUTS_ON_WKC_ERROR                   (EC_IOCTL_GENERIC | 70)
#define EC_IOCTL_SET_MAILBOX_POLLING_CYCLES                     (EC_IOCTL_GENERIC | 71)
#define EC_IOCTL_SET_IGNORE_SWAPDATA                            (EC_IOCTL_GENERIC | 72)
#define EC_IOCTL_SET_MASTER_MAX_STATE                           (EC_IOCTL_GENERIC | 73)
#define EC_IOCTL_GET_MASTER_MAX_STATE                           (EC_IOCTL_GENERIC | 74)
#define EC_IOCTL_SET_CONFIGDATA_MEMORY_POOL                     (EC_IOCTL_GENERIC | 75)
#define EC_IOCTL_SET_STOP_TRANSITION_ON_PDI_WATCHDOG            (EC_IOCTL_GENERIC | 76)
#define EC_IOCTL_SET_DIAGMSG_CODE_BASE                          (EC_IOCTL_GENERIC | 77)
#define EC_IOCTL_SET_BUS_DIAGNOSIS_COUNTERS_OVERFLOW_ENABLED    (EC_IOCTL_GENERIC | 78)
#define EC_IOCTL_SET_SENDCYCFRAMES_BEFORE_PROCESSALLRXFRAMES    (EC_IOCTL_GENERIC | 79)

/* Distributed Clocks (DC) */
#define EC_IOCTL_REG_DC_SLV_SYNC_NTFY               (EC_IOCTL_DC |  3)
#define EC_IOCTL_UNREG_DC_SLV_SYNC_NTFY             (EC_IOCTL_DC |  4)
#define EC_IOCTL_DC_SLV_SYNC_STATUS_GET             (EC_IOCTL_DC |  5)
#define EC_IOCTL_DC_SLV_SYNC_DEVLIMIT_SET           (EC_IOCTL_DC |  6)
#define EC_IOCTL_DC_SLV_SYNC_DEVLIMIT_GET           (EC_IOCTL_DC |  7)
#define EC_IOCTL_DC_SHIFT_SYSTIME                   (EC_IOCTL_DC | 16)
#define EC_IOCTL_DC_SETSYNCSTARTOFFSET              (EC_IOCTL_DC | 17)
#define EC_IOCTL_DC_FIRST_DC_SLV_AS_REF_CLOCK       (EC_IOCTL_DC | 18)
#define EC_IOCTL_DC_SLAVE_CONTROLLED_BY_PDI         (EC_IOCTL_DC | 19)
#define EC_IOCTL_DC_ENABLE_ALL_DC_SLV               (EC_IOCTL_DC | 20)
#define EC_IOCTL_DC_SET_RED_PROPAGDELAY             (EC_IOCTL_DC | 21)

/* DC Master Sync (DCM) */
#define EC_IOCTL_DCM_REGISTER_TIMESTAMP             (EC_IOCTL_DCM |  1)
#define EC_IOCTL_DCM_UNREGISTER_TIMESTAMP           (EC_IOCTL_DCM |  2)
#define EC_IOCTL_DCM_REGISTER_STARTSO_CALLBACK      (EC_IOCTL_DCM |  3)
#define EC_IOCTL_DCM_GET_LOG                        (EC_IOCTL_DCM |  4)

/* Scan Bus (SB) */
#define EC_IOCTL_SB_RESTART                         (EC_IOCTL_SB |  1)           /* 0x00050001 */
#define EC_IOCTL_SB_STATUS_GET                      (EC_IOCTL_SB |  2)           /* 0x00050002 */
#define EC_IOCTL_SB_SET_BUSCNF_VERIFY               (EC_IOCTL_SB |  3)           /* 0x00050003 */
#define EC_IOCTL_SB_SET_BUSCNF_VERIFY_PROP          (EC_IOCTL_SB |  4)           /* 0x00050004 */
#define EC_IOCTL_SB_BUSCNF_GETSLAVE_INFO            (EC_IOCTL_SB |  5)           /* 0x00050005 */
#define EC_IOCTL_SB_BUSCNF_GETSLAVE_INFO_EEP        (EC_IOCTL_SB |  6)           /* 0x00050006 */
#define EC_IOCTL_SB_ENABLE                          (EC_IOCTL_SB |  7)           /* 0x00050007 */
#define EC_IOCTL_SB_BUSCNF_GETSLAVE_INFO_EX         (EC_IOCTL_SB |  9)           /* 0x00050009 */
#define EC_IOCTL_SLV_ALIAS_ENABLE                   (EC_IOCTL_SB | 10)           /* 0x0005000A */
#define EC_IOCTL_SB_SET_BUSCNF_READ_PROP            (EC_IOCTL_SB | 12)           /* 0x0005000C */
#define EC_IOCTL_SB_SET_TOPOLOGY_CHANGED_DELAY      (EC_IOCTL_SB | 13)           /* 0x0005000D */
#define EC_IOCTL_SB_SET_ERROR_ON_CROSSED_LINES      (EC_IOCTL_SB | 14)           /* 0x0005000E */
#define EC_IOCTL_SB_SET_TOPOLOGY_CHANGE_AUTO_MODE   (EC_IOCTL_SB | 15)           /* 0x0005000F */
#define EC_IOCTL_SB_ACCEPT_TOPOLOGY_CHANGE          (EC_IOCTL_SB | 16)           /* 0x00050010 */
#define EC_IOCTL_SB_NOTIFY_UNEXPECTED_BUS_SLAVES    (EC_IOCTL_SB | 17)           /* 0x00050011 */
#define EC_IOCTL_SB_SET_RED_ENHANCED_LINE_CROSSED_DETECTION_ENABLED \
                                                    (EC_IOCTL_SB | 18)           /* 0x00050012 */
#define EC_IOCTL_SB_SET_NOTIFY_NOT_CONNECTED_PORT_A (EC_IOCTL_SB | 19)           /* 0x00050013 */
#define EC_IOCTL_SB_SET_NOTIFY_UNEXPECTED_CONNECTED_PORT \
                                                    (EC_IOCTL_SB | 20)           /* 0x00050014 */
#define EC_IOCTL_SB_SET_JUNCTION_REDUNDANCY_MODE    (EC_IOCTL_SB | 21)           /* 0x00050015 */
#define EC_IOCTL_SB_SET_JUNCTION_REDUNDANCY_ENABLED EC_IOCTL_SB_SET_JUNCTION_REDUNDANCY_MODE
#define EC_IOCTL_SB_GET_BUS_SLAVE_PORTS_INFO        (EC_IOCTL_SB | 22)           /* 0x00050016 */
#define EC_IOCTL_SB_SET_ERROR_ON_LINE_BREAK         (EC_IOCTL_SB | 23)           /* 0x00050017 */
#define EC_IOCTL_SB_SET_IDENTIFICATION_FALLBACK_ENABLED \
                                                    (EC_IOCTL_SB | 24)           /* 0x00050018 */
#define EC_IOCTL_SB_SET_NO_DC_SLAVES_AFTER_JUNCTION (EC_IOCTL_SB | 25)           /* 0x00050019 */
#define EC_IOCTL_SB_SET_TOPOLOGY_CHANGED_DELAYS     (EC_IOCTL_SB | 26)           /* 0x0005001A */

/* Hot Connect (HC) */
#define EC_IOCTL_HC_SETMODE                         (EC_IOCTL_HC | 1)
#define EC_IOCTL_HC_GETMODE                         (EC_IOCTL_HC | 2)
#define EC_IOCTL_HC_CONFIGURETIMEOUTS               (EC_IOCTL_HC | 3)

/* private (PRIVATE) */
#define EC_IOCTL_SET_FRAME_LOSS_SIMULATION          (EC_IOCTL_PRIVATE | 1)
#define EC_IOCTL_SET_RXFRAME_LOSS_SIMULATION        (EC_IOCTL_PRIVATE | 2)
#define EC_IOCTL_SET_TXFRAME_LOSS_SIMULATION        (EC_IOCTL_PRIVATE | 3)
#define EC_IOCTL_GET_FAST_CONTEXT                   (EC_IOCTL_PRIVATE | 4)
#define EC_IOCTL_SET_OEM_KEY                        (EC_IOCTL_PRIVATE | 5)
#define EC_IOCTL_CHECK_OEM_KEY                      (EC_IOCTL_PRIVATE | 6)

/** \defgroup EC_COE_ENTRY_VALUEINFO 
    EtherCat CoE entry description value information bit definitions
@{ */
#define EC_COE_ENTRY_ObjAccess                      0x01    /**<  Object access */
#define EC_COE_ENTRY_ObjCategory                    0x02    /**<  Object category */
#define EC_COE_ENTRY_PdoMapping                     0x04    /**<  PDO mapping */
#define EC_COE_ENTRY_UnitType                       0x08    /**<  Unit type */
#define EC_COE_ENTRY_DefaultValue                   0x10    /**<  Default value */
#define EC_COE_ENTRY_MinValue                       0x20    /**<  Minimum value */
#define EC_COE_ENTRY_MaxValue                       0x40    /**<  Maximum value */
/**@}*/

/** \defgroup EC_COE_ENTRY_OBJACCESS 
    EtherCat CoE entry access bit definitions
@{ */
#define EC_COE_ENTRY_Access_R_PREOP                 0x01    /**< Read access in Pre-Operational state */
#define EC_COE_ENTRY_Access_R_SAFEOP                0x02    /**< Read access in Safe-Operational state */
#define EC_COE_ENTRY_Access_R_OP                    0x04    /**< Read access in Operational state */
#define EC_COE_ENTRY_Access_W_PREOP                 0x08    /**< Write access in Pre-Operational state */
#define EC_COE_ENTRY_Access_W_SAFEOP                0x10    /**< Write access in Safe-Operational state */
#define EC_COE_ENTRY_Access_W_OP                    0x20    /**< Write access in Operational state */
/**@}*/

#define INVALID_SLAVE_ID                            ((EC_T_DWORD)0xFFFFFFFF)
#define INVALID_BUS_INDEX                           ((EC_T_DWORD)0xFFFFFFFF)
#define INVALID_FIXED_ADDR                          ((EC_T_WORD)0x0)
#define INVALID_AUTO_INC_ADDR                       ((EC_T_WORD)0x1)
#define INVALID_TABLE_INDEX                         ((EC_T_DWORD)0xFFFFFFFF)
#define INVALID_CLIENT_ID                           ((EC_T_DWORD)0xFFFFFFFF)

#define EC_ALL_CLIENTS_ID                     0

#ifndef MAX_NUMOF_MASTER_INSTANCES
#define MAX_NUMOF_MASTER_INSTANCES                  12               /* maximum number of master instances */
#endif

#define TRACE_DATA_CMD_ADO                          ((EC_T_WORD)0x4154)

#define MASTER_RED_MASTER_MASTER_PD_CMD_ADO         ((EC_T_WORD)0x4155)
#define MASTER_RED_MASTER_MASTER_EOE_CMD_ADO        ((EC_T_WORD)0x4156)

/* configuration for OD */
#define ECAT_DEVICE_NAMESIZE            80
#define MAX_SLAVE_DEVICENAME            80                      /* 0x3xxx Slave Device name len */
#define EC_OD_MAX_DRIVER_IDENT_LEN      40                      /* deprecated */
#define EC_OD_DRIVER_IDENT_MAXLEN       39                      /* maximum length of link layer name */
#define EC_OD_DRIVER_IDENT_SIZE         (EC_OD_DRIVER_IDENT_MAXLEN + 1) /*  link layer name is zero terminated */
#define EC_CFG_SLAVE_PD_SECTIONS        ((EC_T_DWORD)4)         /* amount of recv/send entries per slave processdata */

#define HISTORY_OBJECT_DIAGELE_SIZE     ((EC_T_WORD)0x0100)     /* size in bytes */
#define MAX_DIAG_MSG                    200
#define NOTIFICATION_TEXT_CODE          "Notification Message %03d - Code"
#define NOTIFICATION_TEXT_COUNT         "Notification Message %03d - Count"

#define NOTIFICATION_FLAGS_CLEAR        0x1                     /* Diagnosis flag: Send as emergency */

#define   DEFTYPE_BUSDIAGNOSTIC         0x0040                  /* Object 0x2002  Bus Diagnosis Object */
#define   DEFTYPE_MACADDRESS            0x0041                  /* Object 0x2005  MAC Address Object */
#define   DEFTYPE_MASTERINITPARM        0x0042                  /* Object 0x2020  Master Initialization parameters */
#define   DEFTYPE_SLAVECFGINFO          0x0043                  /* Object 0x3000 - 0x3FFF Slave Objects */
#define   DEFTYPE_BUSLOAD_STATISTICS    0x0044                  /* Object 0x2200  Bus Load Statistics Object */
#define   DEFTYPE_BUSLOADBASE           DEFTYPE_BUSLOAD_STATISTICS /* legacy */
#define   DEFTYPE_SLAVECFG              0x0045                  /* Object 0x8000 - 0x8FFF Slave Objects (configured slaves) */
#define   DEFTYPE_SLAVEINF              0x0046                  /* Object 0x9000 - 0x9FFF Slave Objects (connected slaves) */
#define   DEFTYPE_SLAVEDIAG             0x0047                  /* Object 0xA000 - 0xAFFF Diagnosis Data */
#define   DEFTYPE_DEVICEPROFILE         0x0048                  /* Object 0xA000 - 0xAFFF Diagnosis Data */
#define   DEFTYPE_DETECTMODCMD          0x0049                  /* Object 0xF002  Detect Modules Command */
#define   DEFTYPE_CONFADDRLIST          0x0050                  /* Object 0xF02x  Configured address list */
#define   DEFTYPE_CONNADDRLIST          0x0051                  /* Object 0xF04x  Detected address list */
#define   DEFTYPE_REDUNDANCY            0x0052                  /* Object 0x2003  Redundancy Diagnosis Object */
#define   DEFTYPE_NOTIFY_COUNTER        0x0053                  /* Object 0x2004  Notification Counter Object */
#define   DEFTYPE_MAILBOX_STATISTICS    0x0054                  /* Object 0x2006  Mailbox Statistics Object */
#define   DEFTYPE_DCM_BUS_SHIFT         0x0055                  /* Object 0x2102  DCM Bus Shift Object  */
#define   DEFTYPE_FRAMESTATISTICS       0x0056                  /* Object 0xF120  Frame Statistics */
#define   DEFTYPE_DIAGINTERFACECTL      0x0057                  /* Object 0xF200  Diag Interface Control */
#define   DEFTYPE_HISTORY_ADD_DIAGMSG   0x0058                  /* Object 0x2007  Add History Diagnosis Message Command */

#define SETDIAGNUMBER(number) \
    ((EC_T_DWORD)(((((EC_T_DWORD)(number))&0xFFF)|(0xE<<12)|((((((EC_T_DWORD)(number))>>12)==0)?1:((((EC_T_DWORD)(number))>>12)==0xFFFF)?0xFFFE:(((EC_T_DWORD)(number))>>12))<<16))))

#define DIAG_MSG_TYPE_MASK              0xF                     /* Diagnosis message type mask */
#define DIAG_MSG_TYPE_INFO              0x0                     /* Diagnosis message type info */
#define DIAG_MSG_TYPE_WARNING           0x1                     /* Diagnosis message type warning */
#define DIAG_MSG_TYPE_ERROR             0x2                     /* Diagnosis message type error */
#define DIAGFLAGINFO                    DIAG_MSG_TYPE_INFO
#define DIAGFLAGWARN                    DIAG_MSG_TYPE_WARNING
#define DIAGFLAGERROR                   DIAG_MSG_TYPE_ERROR

#define DIAG_MSG_PARAM_TYPE_MASK    0xF000 /* Diagnosis parameter type mask*/
#define DIAG_MSG_PARAM_TYPE_OFFSET  12 /* Diagnosis parameter type offset*/
#define DIAG_MSG_PARAM_TYPE_DATA    0x0 /* Diagnosis parameter type data*/
#define DIAG_MSG_PARAM_TYPE_B_ARRY  0x1 /* Diagnosis parameter type Byte Array*/
#define DIAG_MSG_PARAM_TYPE_ASCII   0x2 /* Diagnosis parameter type ASCII*/
#define DIAG_MSG_PARAM_TYPE_UNICODE 0x3 /* Diagnosis parameter type UNICODE*/
#define DIAG_MSG_PARAM_TYPE_TEXTID  0x4 /* Diagnosis parameter type Text ID */
/* this defines are only for compatibility reasons */
#define DIAGPARMTYPEDATATYPE    ((EC_T_WORD)(DIAG_MSG_PARAM_TYPE_DATA<<12))
#define DIAGPARMTYPEBYTEARRAY   ((EC_T_WORD)(DIAG_MSG_PARAM_TYPE_B_ARRY<<12))
#define DIAGPARMTYPEASCIISTRG   ((EC_T_WORD)(DIAG_MSG_PARAM_TYPE_ASCII<<12))
#define DIAGPARMTYPEUNICODESTRG ((EC_T_WORD)(DIAG_MSG_PARAM_TYPE_UNICODE<<12))
#define DIAGPARMTYPETEXTID      ((EC_T_WORD)(DIAG_MSG_PARAM_TYPE_TEXTID<<12))

/*0x1F03 SI5 flag values/masks*/
#define DIAG_SEND_AS_EMCY           0x1 /* Diagnosis flag : Send as emergency*/
#define DIAG_DISABLE_INFO_MSG       0x2 /* Diagnosis flag : Disable Info messages*/
#define DIAG_DISABLE_WARNING_MSG    0x4 /* Diagnosis flag : Disable Warning messages*/
#define DIAG_DISABLE_ERROR_MSG      0x8 /* Diagnosis flag : Disable Error messages*/
#define DIAG_OPERATION_MODE         0x10 /* Indicates the diagnosis history mode (0 "overwrite" Mode; 1 "acknowledge" Mode)*/
#define DIAG_OVERWRITE_DISCARD      0x20 /* Indicates if messages were overwritten ("overwrite" mode) or new messages were discard ("acknowledge" mode)*/

#define DIAG_MSG_DEFAULT_LEN        16   /* Default Length: DiagCode + Flags + TextID + TimeStamp*/

#define COEOBJID_0x800              ((EC_T_WORD)0x0800) /* Object 0x0800  ENUM */

#define COEOBJID_HISTORY_OBJECT     ((EC_T_WORD)0x10F3) /* Object 0x10F3  History Object */

#define COEOBJID_MAST_STATECHNG     ((EC_T_WORD)0x2000) /* Object 0x2000  Master State change Command */
#define COEOBJID_MAST_STATESUMMARY  ((EC_T_WORD)0x2001) /* Object 0x2000  Master State change Command */
#define COEOBJID_BUS_DIAGNOSIS      ((EC_T_WORD)0x2002) /* Object 0x2002  Bus Diagnosis Object */
#define COEOBJID_REDUNDANCY         ((EC_T_WORD)0x2003) /* Object 0x2003  Redundancy Diagnosis Object */
#define COEOBJID_NOTIFY_COUNTER     ((EC_T_WORD)0x2004) /* Object 0x2004  Notification Counter Object */
#define COEOBJID_MAC_ADDRESS        ((EC_T_WORD)0x2005) /* Object 0x2005  MAC Address Object */
#define COEOBJID_MAILBOX_STATISTICS ((EC_T_WORD)0x2006) /* Object 0x2006  Mailbox Statistics Object */
#define COEOBJID_HISTORY_ADD_DIAGMSG ((EC_T_WORD)0x2007)/* Object 0x2007  Add History Diagnosis Message Command */
#define COEOBJID_DEBUG_REGISTER     ((EC_T_WORD)0x2010) /* Object 0x2010  Debug Register */
#define COEOBJID_MASTER_INIT_PARM   ((EC_T_WORD)0x2020) /* Object 0x2020  Master Initialization Parameters */
#define COEOBJID_DC_DEVIATION_LIMIT ((EC_T_WORD)0x2100) /* Object 0x2100  DC Deviation Limit */
#define COEOBJID_DC_CURDEVIATION    ((EC_T_WORD)0x2101) /* Object 0x2101  DC Current Deviation */
#define COEOBJID_DCM_BUSSHIFT       ((EC_T_WORD)0x2102) /* Object 0x2102  DCM Bus Shift */


#define COEOBJID_BUSLOAD_STATISTICS ((EC_T_WORD)0x2200) /* Object 0x2200  Bus Load Statistics Object */
#define COEOBJID_BUSLOAD_BASE       COEOBJID_BUSLOAD_STATISTICS /* legacy */

#define COEOBJID_SLAVECFGINFOBASE   ((EC_T_WORD)0x3000) /* Object 0x3000 - 0x3FFF Slave Objects */

#define COEOBJID_SLAVECFGBASE       ((EC_T_WORD)0x8000) /* Object 0x8000 - 0x8FFF Slave Objects (configured slaves) "Modular Device Profiles" */
#define COEOBJID_SLAVEINFBASE       ((EC_T_WORD)0x9000) /* Object 0x9000 - 0x9FFF Slave Objects (connected slaves) "Modular Device Profiles" */
#define COEOBJID_SLAVEDIAGBASE      ((EC_T_WORD)0xA000) /* Object 0xA000 - 0xAFFF Diagnosis Data "Modular Device Profiles" */
#define COEOBJID_DEVICEPROFILE      ((EC_T_WORD)0xF000) /* Object 0xF000  Modular Device Profile object */
#define COEOBJID_DETECTMODCMD       ((EC_T_WORD)0xF002) /* Object 0xF002  Detect Modules Command "Modular Device Profiles" */
#define COEOBJID_CONFADDRLISTBASE   ((EC_T_WORD)0xF020) /* Object 0xF020 - 0xF02F Configured Address List "Modular Device Profiles" */
#define COEOBJID_CONNADDRLISTBASE   ((EC_T_WORD)0xF040) /* Object 0xF040 - 0xF04F Detected Address List "Modular Device Profiles" */
#define COEOBJID_FRAMESTATISTICS    ((EC_T_WORD)0xF120) /* Object 0xF120  Frame Statistics "Diagnosis Interface" */
#define COEOBJID_DIAGINTERFACECTL   ((EC_T_WORD)0xF200) /* Object 0xF200  Diag Interface Control "Diagnosis Interface" */

#define DEVICETYPE_ETHERNET_GATEWAY     ((EC_T_WORD)1000)
#define DEVICETYPE_ETHERCAT_MASTER      ((EC_T_WORD)1100)
#define DEVICETYPE_ETHERCAT_SLAVE       ((EC_T_WORD)1110)
#define DEVICETYPE_KBUS_MASTER          ((EC_T_WORD)1120)
#define DEVICETYPE_PROFIBUS_MASTER      ((EC_T_WORD)3100)
#define DEVICETYPE_PROFIBUS_SLAVE       ((EC_T_WORD)3110)
#define DEVICETYPE_INTERBUS_MASTER      ((EC_T_WORD)4100)
#define DEVICETYPE_INTERBUS_SLAVE       ((EC_T_WORD)4110)
#define DEVICETYPE_CANOPEN_MASTER       ((EC_T_WORD)5100)
#define DEVICETYPE_CANOPEN_SLAVE        ((EC_T_WORD)5110)
#define DEVICETYPE_DEVICENET_MASTER     ((EC_T_WORD)5200)
#define DEVICETYPE_DEVICENET_SLAVE      ((EC_T_WORD)5210)
#define DEVICETYPE_ASI_MASTER           ((EC_T_WORD)6200)
#define DEVICETYPE_IOLINK_MASTER        ((EC_T_WORD)6220)

#define MAX_ERRINFO_STRLEN  8   /* maximum length of error notification info strings */
#define MAX_SHORT_STRLEN    20  /* maximum length of short info string */
#define MAX_STD_STRLEN      80  /* maximum length of standard info string */

/* see also ATEMRAS_MAX_FILE_NAME_SIZE! */
#define EC_MAX_FILE_NAME_SIZE 64 /* maximum length of file name for FoE Download/Upload */

/** \defgroup EC_SLAVE_IDS
@{ */
#define MASTER_SLAVE_ID     ((EC_T_DWORD)0x00010000)
#define SIMULATOR_SLAVE_ID  ((EC_T_DWORD)0x00010000)
#define MASTER_RED_SLAVE_ID ((EC_T_DWORD)0x00020000)
#define EL9010_SLAVE_ID     ((EC_T_DWORD)0x00030000)
#define FRAMELOSS_SLAVE_ID  ((EC_T_DWORD)0x00040000)
#define JUNCTION_RED_FLAG   ((EC_T_DWORD)0x00100000)
/**@}*/

/* EtherCAT state */
typedef enum _EC_T_STATE
{
    eEcatState_UNKNOWN  = 0,                        /**< Unknown state */
    eEcatState_INIT     = 1,                        /**< EtherCAT state INIT */
    eEcatState_PREOP    = 2,                        /**< EtherCAT state PREOP (pre-operational) */
    eEcatState_SAFEOP   = 4,                        /**< EtherCAT state SAFEOP (safe operational) */
    eEcatState_OP       = 8,                        /**< EtherCAT state OP (operational) */

    eEcatState_BOOTSTRAP = 3,                       /**< EtherCAT state BOOTSTRAP */

    /* Borland C++ datatype alignment correction */
    eEcatState_BCppDummy   = 0xFFFFFFFF
} EC_T_STATE;

typedef enum EC_MAILBOX_FLAG
{
    EC_MAILBOX_FLAG_SDO_COMPLETE            = 1,

    /* Borland C++ datatype alignment correction */
    EC_MAILBOX_FLAG_BCppDummy                               = 0xFFFFFFFF
} EC_MAILBOX_FLAG;

/* EtherCat specific notify codes */
#define EC_NOTIFY_GENERIC                       0x00000000
#define EC_NOTIFY_ERROR                         0x00010000                  /* 0x00010000 ... 0x0001ffff */

#define EC_NOTIFY_MBOXRCV                       0x00020000
#define EC_SZTXT_NOTIFY_MBOXRCV                 "EC_NOTIFY_MBOXRCV"

#define EC_NOTIFY_SCANBUS                       0x00030000
#define EC_NOTIFY_HOTCONNECT                    0x00040000

#define EC_NOTIFY_APP                           0x00080000                  /* application specific codes used by ecatNotifyApp() */
#define EC_NOTIFY_APP_MAX_CODE                  0x0000FFFF                  /* max number app notify codes */

#define ATEMRAS_NOTIFY_GENERIC                  0x00100000
#define ATEMRAS_NOTIFY_ERROR                    0x00110000

#define EC_NOTIFY_STATECHANGED                  (EC_NOTIFY_GENERIC | 1)     /* 0x00000001: EtherCAT operational state change */
#define EC_SZTXT_NOTIFY_STATECHANGED            "EC_NOTIFY_STATECHANGED"

#define EC_NOTIFY_ETH_LINK_CONNECTED            (EC_NOTIFY_GENERIC | 2)     /* 0x00000002: Ethernet link (cable) connected */
#define EC_SZTXT_NOTIFY_ETH_LINK_CONNECTED      "EC_NOTIFY_ETH_LINK_CONNECTED"

#define EC_NOTIFY_SB_STATUS                     (EC_NOTIFY_GENERIC | 3)     /* 0x00000003: ScanBus finished */
#define EC_SZTXT_NOTIFY_SB_STATUS               "EC_NOTIFY_SB_STATUS"

#define EC_NOTIFY_DC_STATUS                     (EC_NOTIFY_GENERIC | 4)     /* 0x00000004: Distributed clocks initialized */
#define EC_SZTXT_NOTIFY_DC_STATUS               "EC_NOTIFY_DC_STATUS"

/* Distributed clocks (DC) */
#define EC_NOTIFY_DC_SLV_SYNC                   (EC_NOTIFY_GENERIC | 5)     /* 0x00000005: DC Slave Synchronization deviation notification */
#define EC_SZTXT_NOTIFY_DC_SLV_SYNC             "EC_NOTIFY_DC_SLV_SYNC"

/* Distributed Clocks Latching (DCL) */
#define EC_NOTIFY_DCL_STATUS                    (EC_NOTIFY_GENERIC | 8)     /* 0x00000008: DCL initialized */
#define EC_SZTXT_NOTIFY_DCL_STATUS              "EC_NOTIFY_DCL_STATUS"

/* Distributed clocks master sync (DCM) */
#define EC_NOTIFY_DCM_SYNC                      (EC_NOTIFY_GENERIC | 9)     /* 0x00000009: DCM InSync */
#define EC_SZTXT_NOTIFY_DCM_SYNC                "EC_NOTIFY_DCM_SYNC"

#define EC_NOTIFY_DCX_SYNC                      (EC_NOTIFY_GENERIC | 10)    /* 0x00000009: DCX InSync */
#define EC_SZTXT_NOTIFY_DCX_SYNC                "EC_NOTIFY_DCX_SYNC"

#define EC_NOTIFY_SLAVE_STATECHANGED            (EC_NOTIFY_GENERIC | 21)    /* 0x00000015: Slave finished successfully state transition */
#define EC_SZTXT_NOTIFY_SLAVE_STATECHANGED      "EC_NOTIFY_SLAVE_STATECHANGED"

#define EC_NOTIFY_SLAVES_STATECHANGED           (EC_NOTIFY_GENERIC | 22)    /* 0x00000016: Slaves finished successfully state transition */
#define EC_SZTXT_NOTIFY_SLAVES_STATECHANGED     "EC_NOTIFY_SLAVES_STATECHANGED"

#define EC_NOTIFY_RAWCMD_DONE                   (EC_NOTIFY_GENERIC | 100)   /* 0x00000064: Queue Raw Command Response Notification */
#define EC_SZTXT_NOTIFY_RAWCMD_DONE             "EC_NOTIFY_RAWCMD_DONE"

#define EC_NOTIFY_SLAVE_PRESENCE                (EC_NOTIFY_GENERIC | 101)   /* 0x00000065: Slave (dis)appeared */
#define EC_SZTXT_NOTIFY_SLAVE_PRESENCE          "EC_NOTIFY_SLAVE_PRESENCE"

#define EC_NOTIFY_SLAVES_PRESENCE               (EC_NOTIFY_GENERIC | 102)   /* 0x00000066: Slaves (dis)appeared */
#define EC_SZTXT_NOTIFY_SLAVES_PRESENCE         "EC_NOTIFY_SLAVES_PRESENCE"

#define EC_NOTIFY_REFCLOCK_PRESENCE             (EC_NOTIFY_GENERIC | 103)   /* 0x00000067: Reference clock (dis)appeared */
#define EC_SZTXT_NOTIFY_REFCLOCK_PRESENCE       "EC_NOTIFY_REFCLOCK_PRESENCE"

#define EC_NOTIFY_MASTER_RED_STATECHANGED       (EC_NOTIFY_GENERIC | 104)   /* 0x00000068: Change of Master Redundancy State */
#define EC_SZTXT_NOTIFY_MASTER_RED_STATECHANGED "EC_NOTIFY_MASTER_RED_STATECHANGE"

#define EC_NOTIFY_MASTER_RED_FOREIGN_SRC_MAC    (EC_NOTIFY_GENERIC | 105)   /* 0x00000069: Foreign communication detected */
#define EC_SZTXT_NOTIFY_MASTER_RED_FOREIGN_SRC_MAC \
                                                "EC_NOTIFY_MASTER_RED_FOREIGN_SRC_MAC"

#define EC_NOTIFY_SLAVE_REGISTER_TRANSFER       (EC_NOTIFY_GENERIC | 106)   /* 0x0000006A: Slave register read/write notification */
#define EC_SZTXT_NOTIFY_SLAVE_REGISTER_TRANSFER "EC_NOTIFY_SLAVE_REGISTER_TRANSFER"

#define EC_NOTIFY_EEPROM_OPERATION              (EC_NOTIFY_GENERIC | 107)   /* 0x0000006B: EEPRom operation notification */
#define EC_SZTXT_NOTIFY_EEPROM_OPERATION        "EC_NOTIFY_EEPROM_OPERATION"

#define EC_NOTIFY_PORT_OPERATION                (EC_NOTIFY_GENERIC | 108)   /* 0x0000006C: Port operation notification */
#define EC_SZTXT_NOTIFY_PORT_OPERATION          "EC_NOTIFY_PORT_OPERATION"

#define EC_NOTIFY_SLAVE_IDENTIFICATION          (EC_NOTIFY_GENERIC | 109)   /* 0x0000006D: Port operation notification */
#define EC_SZTXT_NOTIFY_SLAVE_IDENTIFICATION    "EC_NOTIFY_SLAVE_IDENTIFICATION"

#define EC_NOTIFY_RELEASE_FORCED_PROCESSDATA    (EC_NOTIFY_GENERIC | 110)   /* 0x0000006E: Release forced process data */
#define EC_SZTXT_NOTIFY_RELEASE_FORCED_PROCESSDATA "EC_NOTIFY_RELEASE_FORCED_PROCESSDATA"

/* mailbox */
#define EC_NOTIFY_COE_TX_PDO                    (EC_NOTIFY_MBOXRCV | 1)     /* 0x00020001: TxPDO notification */
#define EC_SZTXT_NOTIFY_COE_TX_PDO              "EC_NOTIFY_COE_TX_PDO"

#define EC_NOTIFY_RAWMBX_DONE                   (EC_NOTIFY_MBOXRCV | 2)     /* 0x00020002: Raw mailbox transfer response */
#define EC_SZTXT_NOTIFY_RAWMBX_DONE             "EC_NOTIFY_RAWMBX_DONE"

#define EC_NOTIFY_COE_INIT_CMD                  (EC_NOTIFY_MBOXRCV | 3)     /* 0x00020003: CoE init command */
#define EC_SZTXT_EC_NOTIFY_COE_INIT_CMD         "EC_NOTIFY_COE_INIT_CMD"

/* errors */
#define EC_NOTIFY_CYCCMD_WKC_ERROR              (EC_NOTIFY_ERROR | 1)       /* 0x00010001: cyclic command: working counter error */
#define EC_SZTXT_NOTIFY_CYCCMD_WKC_ERROR        "EC_NOTIFY_CYCCMD_WKC_ERROR"

#define EC_NOTIFY_MASTER_INITCMD_WKC_ERROR      (EC_NOTIFY_ERROR | 2)       /* 0x00010002: master init command: working counter error */
#define EC_SZTXT_NOTIFY_MASTER_INITCMD_WKC_ERROR "EC_NOTIFY_MASTER_INITCMD_WKC_ERROR"

#define EC_NOTIFY_SLAVE_INITCMD_WKC_ERROR       (EC_NOTIFY_ERROR | 3)       /* 0x00010003: slave init command: working counter error */
#define EC_SZTXT_NOTIFY_SLAVE_INITCMD_WKC_ERROR "EC_NOTIFY_SLAVE_INITCMD_WKC_ERROR"

#define EC_NOTIFY_EOE_MBXSND_WKC_ERROR          (EC_NOTIFY_ERROR | 7)       /* 0x00010007: EoE mbox send: working counter error */
#define EC_SZTXT_NOTIFY_EOE_MBXSND_WKC_ERROR    "EC_NOTIFY_EOE_MBXSND_WKC_ERROR"

#define EC_NOTIFY_COE_MBXSND_WKC_ERROR          (EC_NOTIFY_ERROR | 8)       /* 0x00010008: CoE mbox send: working counter error */
#define EC_SZTXT_NOTIFY_COE_MBXSND_WKC_ERROR    "EC_NOTIFY_COE_MBXSND_WKC_ERROR"

#define EC_NOTIFY_FOE_MBXSND_WKC_ERROR          (EC_NOTIFY_ERROR | 9)      /* 0x00010009: FoE mbox send: working counter error */
#define EC_SZTXT_NOTIFY_FOE_MBXSND_WKC_ERROR    "EC_NOTIFY_FOE_MBXSND_WKC_ERROR"

#define EC_NOTIFY_FRAME_RESPONSE_ERROR          (EC_NOTIFY_ERROR | 10)      /* 0x0001000a: Got no response on a sent Ethernet frame */
#define EC_SZTXT_NOTIFY_FRAME_RESPONSE_ERROR    "EC_NOTIFY_FRAME_RESPONSE_ERROR"

#define EC_NOTIFY_SLAVE_INITCMD_RESPONSE_ERROR  (EC_NOTIFY_ERROR | 11)      /* 0x0001000b: Got no or unexpected response on a sent ecat init command from slave */
#define EC_SZTXT_NOTIFY_SLAVE_INITCMD_RESPONSE_ERROR "EC_NOTIFY_SLAVE_INITCMD_RESPONSE_ERROR"

#define EC_NOTIFY_MASTER_INITCMD_RESPONSE_ERROR (EC_NOTIFY_ERROR | 12)      /* 0x0001000c: Got no response on a sent ecat master init command */
#define EC_SZTXT_NOTIFY_MASTER_INITCMD_RESPONSE_ERROR "EC_NOTIFY_MASTER_INITCMD_RESPONSE_ERROR"

#define EC_NOTIFY_MBSLAVE_INITCMD_TIMEOUT       (EC_NOTIFY_ERROR | 14)      /* 0x0001000e: Timeout when waiting for mailbox init command response */
#define EC_SZTXT_NOTIFY_MBSLAVE_INITCMD_TIMEOUT "EC_NOTIFY_MBSLAVE_INITCMD_TIMEOUT"

#define EC_NOTIFY_NOT_ALL_DEVICES_OPERATIONAL   (EC_NOTIFY_ERROR | 15)      /* 0x0001000f: Not all slave devices are in operational state when receiving cyclic frames */
#define EC_SZTXT_NOTIFY_NOT_ALL_DEVICES_OPERATIONAL "EC_NOTIFY_NOT_ALL_DEVICES_OPERATIONAL"

#define EC_NOTIFY_ETH_LINK_NOT_CONNECTED        (EC_NOTIFY_ERROR | 16)      /* 0x00010010: Ethernet link (cable) not connected */
#define EC_SZTXT_NOTIFY_ETH_LINK_NOT_CONNECTED  "EC_NOTIFY_ETH_LINK_NOT_CONNECTED"

#define EC_NOTIFY_RED_LINEBRK                   (EC_NOTIFY_ERROR | 18)      /* 0x00010012: Redundancy: line break detected */
#define EC_SZTXT_NOTIFY_RED_LINEBRK             "EC_NOTIFY_RED_LINEBRK"

#define EC_NOTIFY_STATUS_SLAVE_ERROR            (EC_NOTIFY_ERROR | 19)      /* 0x00010013: At least one slave is in error state when receiving cyclic frames (BRD AL-STATUS) */
#define EC_SZTXT_NOTIFY_STATUS_SLAVE_ERROR      "EC_NOTIFY_STATUS_SLAVE_ERROR"

#define EC_NOTIFY_SLAVE_ERROR_STATUS_INFO       (EC_NOTIFY_ERROR | 20)      /* 0x00010014: Slave error (AL status code) */
#define EC_SZTXT_NOTIFY_SLAVE_ERROR_STATUS_INFO "EC_NOTIFY_SLAVE_ERROR_STATUS_INFO"

#define EC_NOTIFY_SLAVE_NOT_ADDRESSABLE         (EC_NOTIFY_ERROR | 21)      /* 0x00010015: Obsolete */
#define EC_SZTXT_NOTIFY_SLAVE_NOT_ADDRESSABLE   "EC_NOTIFY_SLAVE_NOT_ADDRESSABLE"

#define EC_NOTIFY_SOE_MBXSND_WKC_ERROR          (EC_NOTIFY_ERROR | 23)      /* 0x00010017: SoE mbox send: working counter error */
#define EC_SZTXT_NOTIFY_SOE_MBXSND_WKC_ERROR    "EC_NOTIFY_SOE_MBXSND_WKC_ERROR"

#define EC_NOTIFY_SOE_WRITE_ERROR               (EC_NOTIFY_ERROR | 24)      /* 0x00010018: SoE mbox write responded with an error */
#define EC_SZTXT_NOTIFY_SOE_WRITE_ERROR         "EC_NOTIFY_SOE_WRITE_ERROR"

#define EC_NOTIFY_MBSLAVE_COE_SDO_ABORT         (EC_NOTIFY_ERROR | 25)      /* 0x00010019: CoE mbox SDO abort */
#define EC_SZTXT_NOTIFY_MBSLAVE_COE_SDO_ABORT   "EC_NOTIFY_MBSLAVE_COE_SDO_ABORT"

#define EC_NOTIFY_CLIENTREGISTRATION_DROPPED    (EC_NOTIFY_ERROR | 26)      /* 0x0001001a: Client registration dropped, possibly call to ecatConfigureMaster by other thread (RAS) */
#define EC_SZTXT_NOTIFY_CLIENTREGISTRATION_DROPPED "EC_NOTIFY_CLIENTREGISTRATION_DROPPED"

#define EC_NOTIFY_RED_LINEFIXED                 (EC_NOTIFY_ERROR | 27)      /* 0x0001001b: Redundancy: line is repaired */
#define EC_SZTXT_NOTIFY_RED_LINEFIXED           "EC_NOTIFY_RED_LINEFIXED"

#define EC_NOTIFY_FOE_MBSLAVE_ERROR             (EC_NOTIFY_ERROR | 28)      /* 0x0001001c: FoE mbox abort */
#define EC_SZTXT_NOTIFY_FOE_MBSLAVE_ERROR       "EC_NOTIFY_FOE_MBSLAVE_ERROR"

#define EC_NOTIFY_MBXRCV_INVALID_DATA           (EC_NOTIFY_ERROR | 29)      /* 0x0001001d: invalid mail box data received */
#define EC_SZTXT_NOTIFY_MBXRCV_INVALID_DATA     "EC_NOTIFY_MBXRCV_INVALID_DATA"

#define EC_NOTIFY_PDIWATCHDOG                   (EC_NOTIFY_ERROR | 30)      /* 0x0001001e: PDI Watchdog expired on slave, thrown by IST */
#define EC_SZTXT_NOTIFY_PDIWATCHDOG             "EC_NOTIFY_PDIWATCHDOG"

#define EC_NOTIFY_SLAVE_NOTSUPPORTED            (EC_NOTIFY_ERROR | 31)      /* 0x0001001f: slave not supported (if redundancy is activated and slave doesn't fully support autoclose */
#define EC_SZTXT_NOTIFY_SLAVE_NOTSUPPORTED      "EC_NOTIFY_SLAVE_NOTSUPPORTED"

#define EC_NOTIFY_SLAVE_UNEXPECTED_STATE        (EC_NOTIFY_ERROR | 32)      /* 0x00010020: slave in unexpected state */
#define EC_SZTXT_NOTIFY_SLAVE_UNEXPECTED_STATE  "EC_NOTIFY_SLAVE_UNEXPECTED_STATE"

#define EC_NOTIFY_ALL_DEVICES_OPERATIONAL       (EC_NOTIFY_ERROR | 33)      /* 0x00010021: All slave devices are in operational state */
#define EC_SZTXT_NOTIFY_ALL_DEVICES_OPERATIONAL "EC_NOTIFY_ALL_DEVICES_OPERATIONAL"

#define EC_NOTIFY_VOE_MBXSND_WKC_ERROR          (EC_NOTIFY_ERROR | 34)      /* 0x00010022: VOE mbox send: working counter error */
#define EC_SZTXT_NOTIFY_VOE_MBXSND_WKC_ERROR    "EC_NOTIFY_VOE_MBXSND_WKC_ERROR"

#define EC_NOTIFY_EEPROM_CHECKSUM_ERROR         (EC_NOTIFY_ERROR | 35)      /* 0x00010023: EEPROM checksum error detected */
#define EC_SZTXT_NOTIFY_EEPROM_CHECKSUM_ERROR   "EC_NOTIFY_EEPROM_CHECKSUM_ERROR"

#define EC_NOTIFY_LINE_CROSSED                  (EC_NOTIFY_ERROR | 36)      /* 0x00010024: Crossed lines detected */
#define EC_SZTXT_NOTIFY_LINE_CROSSED            "EC_NOTIFY_LINE_CROSSED"

#define EC_NOTIFY_JUNCTION_RED_CHANGE           (EC_NOTIFY_ERROR | 37)      /* 0x00010025: Junction redundancy change */
#define EC_SZTXT_NOTIFY_JUNCTION_RED_CHANGE     "EC_NOTIFY_JUNCTION_RED_CHANGE"

#define EC_NOTIFY_SLAVES_UNEXPECTED_STATE       (EC_NOTIFY_ERROR | 38)      /* 0x00010026: slaves in unexpected state */
#define EC_SZTXT_NOTIFY_SLAVES_UNEXPECTED_STATE "EC_NOTIFY_SLAVES_UNEXPECTED_STATE"

#define EC_NOTIFY_SLAVES_ERROR_STATUS           (EC_NOTIFY_ERROR | 39)      /* 0x00010027: Slaves error (AL status code) */
#define EC_SZTXT_NOTIFY_SLAVES_ERROR_STATUS     "EC_NOTIFY_SLAVES_ERROR_STATUS"

#define EC_NOTIFY_FRAMELOSS_AFTER_SLAVE         (EC_NOTIFY_ERROR | 40)      /* 0x00010028: Frameloss after Slave */
#define EC_SZTXT_NOTIFY_FRAMELOSS_AFTER_SLAVE   "EC_NOTIFY_FRAMELOSS_AFTER_SLAVE"

#define EC_NOTIFY_S2SMBX_ERROR                  (EC_NOTIFY_ERROR | 41)      /* 0x00010029: S2S Mailbox Error */
#define EC_SZTXT_NOTIFY_S2SMBX_ERROR            "EC_NOTIFY_S2SMBX_ERROR"

#define EC_NOTIFY_BAD_CONNECTION                (EC_NOTIFY_ERROR | 42)      /* 0x0001002A: Bad connection */
#define EC_SZTXT_NOTIFY_BAD_CONNECTION          "EC_NOTIFY_BAD_CONNECTION"

#define EC_NOTIFY_COMMUNICATION_TIMEOUT         (EC_NOTIFY_ERROR | 43)      /* 0x0001002B: Communication timeout */
#define EC_SZTXT_NOTIFY_COMMUNICATION_TIMEOUT   "EC_NOTIFY_COMMUNICATION_TIMEOUT"

#define EC_NOTIFY_TAP_LINK_STATUS               (EC_NOTIFY_ERROR | 44)      /* 0x0001002C: TAP link status */
#define EC_SZTXT_NOTIFY_TAP_LINK_STATUS         "NOTIFY_TAP_LINK_STATUS"

#define EC_NUM_ERROR_NOTIFICATIONS                                 44       /* number of error notifications in 0x00010000 ... 0x0001fff */

/* ScanBus Notification */
#define EC_NOTIFY_SB_MISMATCH                   (EC_NOTIFY_SCANBUS | 2)     /* 0x00030002: ScanBus mismatch */
#define EC_SZTXT_NOTIFY_SB_MISMATCH             "EC_NOTIFY_SB_MISMATCH"

#define EC_NOTIFY_SB_DUPLICATE_HC_NODE          (EC_NOTIFY_SCANBUS | 3)     /* 0x00030003: ScanBus mismatch. A duplicate HC group was detected equal product code, vendor id
                                                                               and alias address (or switch id)  */
#define EC_SZTXT_NOTIFY_SB_DUPLICATE_HC_NODE    "EC_NOTIFY_SB_DUPLICATE_HC_NODE"


/* Hot Connect Action Results */
/* #define EC_NOTIFY_HC_DETECTALLGROUPS         (EC_NOTIFY_HOTCONNECT | 1)   * 0x00040001: HC Detect All Groups done */
#define EC_NOTIFY_HC_DETECTADDGROUPS            (EC_NOTIFY_HOTCONNECT | 2)  /* 0x00040002: HC Enhance Detect All Groups done */
#define EC_SZTXT_NOTIFY_HC_DETECTADDGROUPS      "EC_NOTIFY_HC_DETECTADDGROUPS"

#define EC_NOTIFY_HC_PROBEALLGROUPS             (EC_NOTIFY_HOTCONNECT | 3)  /* 0x00040003: HC Probe All Groups done */
#define EC_SZTXT_NOTIFY_HC_PROBEALLGROUPS       "EC_NOTIFY_HC_PROBEALLGROUPS"

#define EC_NOTIFY_HC_TOPOCHGDONE                (EC_NOTIFY_HOTCONNECT | 4)  /* 0x00040004: HC Topology Change done */
#define EC_SZTXT_NOTIFY_HC_TOPOCHGDONE          "EC_NOTIFY_HC_TOPOCHGDONE"

/* Replaced by EC_NOTIFY_SLAVE_PRESENCE */
#define EC_NOTIFY_SLAVE_DISAPPEARS              (EC_NOTIFY_HOTCONNECT | 5)  /* 0x00040005: Slave disappears */
#define EC_NOTIFY_HC_SLAVE_PART EC_NOTIFY_SLAVE_DISAPPEARS
#define EC_SZTXT_NOTIFY_SLAVE_DISAPPEARS        "EC_NOTIFY_SLAVE_DISAPPEARS"

/* Replaced by EC_NOTIFY_SLAVE_PRESENCE */
#define EC_NOTIFY_SLAVE_APPEARS                 (EC_NOTIFY_HOTCONNECT | 6)  /* 0x00040006: Slave appears */
#define EC_NOTIFY_HC_SLAVE_JOIN EC_NOTIFY_SLAVE_APPEARS
#define EC_SZTXT_NOTIFY_SLAVE_APPEARS           "EC_NOTIFY_SLAVE_APPEARS"

/* Remote API */
#define ATEMRAS_NOTIFY_CONNECTION               (ATEMRAS_NOTIFY_GENERIC|0x0001) /* 0x00100001 */
#define EC_SZTXT_ATEMRAS_NOTIFY_CONNECTION      "ATEMRAS_NOTIFY_CONNECTION"

#define ATEMRAS_NOTIFY_REGISTER                 (ATEMRAS_NOTIFY_GENERIC|0x0002) /* 0x00100002 */
#define EC_SZTXT_ATEMRAS_NOTIFY_REGISTER        "ATEMRAS_NOTIFY_REGISTER"

#define ATEMRAS_NOTIFY_UNREGISTER               (ATEMRAS_NOTIFY_GENERIC|0x0003) /* 0x00100003 */
#define EC_SZTXT_ATEMRAS_NOTIFY_UNREGISTER      "ATEMRAS_NOTIFY_UNREGISTER"

#define ATEMRAS_NOTIFY_MARSHALERROR             (ATEMRAS_NOTIFY_ERROR  |0x0001) /* 0x00110001 */
#define EC_SZTXT_ATEMRAS_NOTIFY_MARSHALERROR    "ATEMRAS_NOTIFY_MARSHALERROR"

#define ATEMRAS_NOTIFY_ACKERROR                 (ATEMRAS_NOTIFY_ERROR  |0x0002) /* 0x00110002 */
#define EC_SZTXT_ATEMRAS_NOTIFY_ACKERROR        "ATEMRAS_NOTIFY_ACKERROR"

#define ATEMRAS_NOTIFY_NONOTIFYMEMORY           (ATEMRAS_NOTIFY_ERROR  |0x0003) /* 0x00110003 */
#define EC_SZTXT_ATEMRAS_NOTIFY_NONOTIFYMEMORY  "ATEMRAS_NOTIFY_NONOTIFYMEMORY"

#define ATEMRAS_NOTIFY_STDNOTIFYMEMORYSMALL     (ATEMRAS_NOTIFY_ERROR  |0x0004) /* 0x00110004 */
#define EC_SZTXT_ATEMRAS_NOTIFY_STDNOTIFYMEMORYSMALL    "ATEMRAS_NOTIFY_STDNOTIFYMEMORYSMALL"

#define ATEMRAS_NOTIFY_MBXNOTIFYMEMORYSMALL     (ATEMRAS_NOTIFY_ERROR  |0x0005) /* 0x00110005 */
#define EC_SZTXT_ATEMRAS_NOTIFY_MBXNOTIFYMEMORYSMALL    "ATEMRAS_NOTIFY_MBXNOTIFYMEMORYSMALL"

/** \defgroup EC_FRAME_RESPONSE_ERROR_NOTIFY_MASKS Frame response error notification bit masks
@{
*/
#define EC_FRAME_RESPONSE_ERROR_NOTIFY_MASK_UNDEFINED           (1 << eRspErr_UNDEFINED)        /**< Mask for eRspErr_UNDEFINED   notifications */
#define EC_FRAME_RESPONSE_ERROR_NOTIFY_MASK_NO_RESPONSE         (1 << eRspErr_NO_RESPONSE)      /**< Mask for eRspErr_NO_RESPONSE notifications */
#define EC_FRAME_RESPONSE_ERROR_NOTIFY_MASK_WRONG_IDX           (1 << eRspErr_WRONG_IDX)        /**< Mask for eRspErr_WRONG_IDX   notifications */
#define EC_FRAME_RESPONSE_ERROR_NOTIFY_MASK_UNEXPECTED          (1 << eRspErr_UNEXPECTED)       /**< Mask for eRspErr_UNEXPECTED  notifications */
#define EC_FRAME_RESPONSE_ERROR_NOTIFY_MASK_FRAME_RETRY         (1 << eRspErr_FRAME_RETRY)      /**< Mask for eRspErr_FRAME_RETRY notifications */
#define EC_FRAME_RESPONSE_ERROR_NOTIFY_MASK_RETRY_FAIL          (1 << eRspErr_RETRY_FAIL)       /**< Mask for eRspErr_RETRY_FAIL  notifications */
#define EC_FRAME_RESPONSE_ERROR_NOTIFY_MASK_FOREIGN_SRC_MAC     (1 << eRspErr_FOREIGN_SRC_MAC)  /**< Mask for eRspErr_FOREIGN_SRC_MAC notifications */
#define EC_FRAME_RESPONSE_ERROR_NOTIFY_MASK_NON_ECAT_FRAME      (1 << eRspErr_NON_ECAT_FRAME)   /**< Mask for eRspErr_NON_ECAT_FRAME  notifications */
#define EC_FRAME_RESPONSE_ERROR_NOTIFY_MASK_ALL                 (0xFFFFFFFF)                    /**< Mask for all notifications enabled except eRspErr_NON_ECAT_FRAME */
#define EC_FRAME_RESPONSE_ERROR_NOTIFY_MASK_DEFAULT             (EC_FRAME_RESPONSE_ERROR_NOTIFY_MASK_ALL & ~EC_FRAME_RESPONSE_ERROR_NOTIFY_MASK_NON_ECAT_FRAME) /**< Mask for all frame response error notifications */
/**@}*/

/** \defgroup EC_MBX_PROTOCOLS Supported mailbox protocols
@{
*/
#define EC_MBX_PROTOCOL_AOE 0x01
#define EC_MBX_PROTOCOL_EOE 0x02
#define EC_MBX_PROTOCOL_COE 0x04
#define EC_MBX_PROTOCOL_FOE 0x08
#define EC_MBX_PROTOCOL_SOE 0x10
#define EC_MBX_PROTOCOL_VOE 0x20
/**@}*/

/* CoE debug bit masks */
#define EC_COE_DBG_MASK_SDO_DOWNLOAD        0x00000001  /* mask for SDO download transfer debug support */
#define EC_COE_DBG_MASK_SDO_UPLOAD          0x00000002  /* mask for SDO upload transfer debug support */
#define EC_COE_DBG_MASK_GET_ODLIST          0x00000004  /* mask for CoE get object dictionary list transfer debug support */
#define EC_COE_DBG_MASK_GET_OBDESC          0x00000008  /* mask for CoE get object description transfer debug support */
#define EC_COE_DBG_MASK_GET_ENTRYDESC       0x00000010  /* mask for CoE get entry description transfer debug support */
#define EC_COE_DBG_MASK_RX_PDO              0x00000020  /* mask for RxPDO transfer debug support */
#define EC_COE_DBG_MASK_TX_PDO              0x00000040  /* mask for TxPDO transfer debug support */
#define EC_COE_DBG_MASK_ALL                 0x00000FFF  /* mask for all CoE transfers debug support */

/* FoE debug bit masks */
#define EC_FOE_DBG_MASK_FILE_DOWNLOAD       0x00001000  /* mask for FoE download transfer debug support */
#define EC_FOE_DBG_MASK_FILE_UPLOAD         0x00002000  /* mask for FoE download transfer debug support */
#define EC_FOE_DBG_MASK_ALL                 0x000FF000  /* mask for all FoE transfers debug support */

#define EC_MBX_DBG_MASK_ALL                 0x000FFFFF  /* mask for all CoE transfers debug support */


/* SoE debug bit masks */
#define EC_SOE_DBG_MASK_IDN_WRITE        0x00000001  /* mask for SoE download transfer debug support */
#define EC_SOE_DBG_MASK_IDN_READ         0x00000002  /* mask for SoE upload transfer debug support */
#define EC_SOE_DBG_MASK_ALL              0x00000FFF  /* mask for all SoE transfers debug support */

/* VoE debug bit masks */
#define EC_VOE_DBG_MASK_MBX_DOWNLOAD     0x00100000  /* mask for VoE write transfer debug support */
#define EC_VOE_DBG_MASK_MBX_UPLOAD       0x00200000  /* mask for VoE read transfer debug support */
#define EC_VOE_DBG_MASK_ALL              0x0FF00000  /* mask for all VoE transfers debug support */

#define MAX_PROCESS_VAR_NAME_LEN            ((EC_T_DWORD)72)  /* STD: Maximum length of a process variable name */
#define MAX_PROCESS_VAR_NAME_LEN_EX         ((EC_T_DWORD)128) /* EX:  Maximum length of a process variable name */

/* EtherCAT commands */
typedef enum
{
    EC_CMD_TYPE_NOP     = 0x00,
    EC_CMD_TYPE_APRD    = 0x01,
    EC_CMD_TYPE_APWR    = 0x02,
    EC_CMD_TYPE_APRW    = 0x03,
    EC_CMD_TYPE_FPRD    = 0x04,
    EC_CMD_TYPE_FPWR    = 0x05,
    EC_CMD_TYPE_FPRW    = 0x06,
    EC_CMD_TYPE_BRD     = 0x07,
    EC_CMD_TYPE_BWR     = 0x08,
    EC_CMD_TYPE_BRW     = 0x09,
    EC_CMD_TYPE_LRD     = 0x0A,
    EC_CMD_TYPE_LWR     = 0x0B,
    EC_CMD_TYPE_LRW     = 0x0C,
    EC_CMD_TYPE_ARMW    = 0x0D,
    EC_CMD_TYPE_FRMW    = 0x0E,
    EC_CMD_TYPE_EXT     = 0xFF,

    /* Borland C++ datatype alignment correction */
    EC_CMD_TYPE_BCppDummy   = 0xFFFFFFFF
} EC_CMD_TYPE;

/* textual description of EtherCat commands */
#define EcatCmdShortText(byCmd)                     \
    ((byCmd)==EC_CMD_TYPE_NOP?"NOP":                \
     ((byCmd)==EC_CMD_TYPE_APRD?"APRD":             \
      ((byCmd)==EC_CMD_TYPE_APWR?"APWR":            \
       ((byCmd)==EC_CMD_TYPE_APRW?"APRW":           \
        ((byCmd)==EC_CMD_TYPE_FPRD?"FPRD":          \
         ((byCmd)==EC_CMD_TYPE_FPWR?"FPWR":         \
          ((byCmd)==EC_CMD_TYPE_FPRW?"FPRW":        \
           ((byCmd)==EC_CMD_TYPE_BRD?"BRD":         \
            ((byCmd)==EC_CMD_TYPE_BWR?"BWR":        \
             ((byCmd)==EC_CMD_TYPE_BRW?"BRW":       \
              ((byCmd)==EC_CMD_TYPE_LRD?"LRD":      \
               ((byCmd)==EC_CMD_TYPE_LWR?"LWR":     \
                ((byCmd)==EC_CMD_TYPE_LRW?"LRW":    \
                 ((byCmd)==EC_CMD_TYPE_ARMW?"ARMW": \
                  ((byCmd)==EC_CMD_TYPE_FRMW?"FRMW":\
                  "INVALID ECAT CMD VALUE!!!"       \
    )))))))))))))))


/*-TYPEDEFS------------------------------------------------------------------*/
#include EC_PACKED_INCLUDESTART(1)

/* Object 0x10F3  History Object - Generic diagnosis message structure (0x10F3.SI6 : 0x10F3.MaxSubindex) */
/* Basic structure to handle message parameter */
typedef struct
{
   EC_T_WORD        wParamFlags;                    /* Parameter flags */
   EC_T_BYTE        aData[4];                       /* Parameter buffer: length depending on parameters */
} EC_PACKED(1) EC_T_DIAGMSGPARAM;
typedef struct _EC_T_OBJ10F3_DIAGMSG
{
   EC_T_DWORD           dwDiagNumber;               /* Message code */
   EC_T_WORD            wFlags;                     /* Message flags */
   EC_T_WORD            wTextId;                    /* Text ID */
   EC_T_UINT64          qwTimeStamp;                /* 0x08 */
   EC_T_DIAGMSGPARAM    oParameter;                 /* Handler of the first parameter */
} EC_PACKED(1) EC_T_OBJ10F3_DIAGMSG;
typedef struct _EC_T_OBJ10F3
{
    EC_T_WORD               wSubIndex0;             /* Subindex 000 */
    EC_T_BYTE               byMaxDiagMessages;      /* Subindex 001 */
    EC_T_BYTE               byNewestMessage;        /* Subindex 002 */
    EC_T_BYTE               byNewestAckMessage;     /* Subindex 003 */
    EC_T_BYTE               byNewDiagMessages;      /* Subindex 004 */
    EC_T_WORD               wFlags;                 /* Subindex 005 */
    /* EC_T_DIAGMESSAGE */                          /* Subindex 006ff */
} EC_PACKED(1) EC_T_OBJ10F3;

/* Object 0x2000  Master State change Command */
typedef enum _EC_T_OBJ2000_VALUES
{
    eMastStChng_init        = 1,
    eMastStChng_preop       = 2,
    eMastStChng_safeop      = 4,
    eMastStChng_op          = 8,

    /* Borland C++ datatype alignment correction */
    eMastStChng_BCppDummy   = 0xFFFFFFFF
} EC_T_OBJ2000_VALUES;

/* Object 0x2001  Master State Summary (EC_T_DWORD) */
#define OBJ2001_STATE_SUM_MASTER_OK    0x00000001   /* Bit 0: = 1 Master o.k. */
                                                    /* Bit 1-3: Reserved */
#define OBJ2001_STATE_SUM_MASTER_STATE 0x000000F0   /* Bit 4-7: Master State */
#define OBJ2001_STATE_SUM_SLAVE_REQ    0x00000100   /* Bit 8: Slaves in requested State */
#define OBJ2001_STATE_SUM_MASTER_REQ   0x00000200   /* Bit 9: Master in requested State */
#define OBJ2001_STATE_SUM_BUS_MATCH    0x00000400   /* Bit 10: Bus Scan Match */
#define OBJ2001_STATE_SUM_RES11        0x00000800   /* Bit 11: Reserved */
#define OBJ2001_STATE_SUM_DC_ENA       0x00001000   /* Bit 12: DC is enabled */
#define OBJ2001_STATE_SUM_DC_SYNC      0x00002000   /* Bit 13: DC In-Sync */
#define OBJ2001_STATE_SUM_DC_BUSY      0x00004000   /* Bit 14: DC Busy */
#define OBJ2001_STATE_SUM_RES15        0x00008000   /* Bit 15: Reserved */
#define OBJ2001_STATE_SUM_LINK_UP      0x00010000   /* Bit 16: Link Up  */
                                                    /* Bit 17-31: Reserved */

                                                             /* dword : .... .... .... ...L  .bID .BMS ssss ...O */
#define OBJ2001_STATE_SUM_MASK1  ((EC_T_DWORD)0x000107f0)    /* mask  : 0000 0000 0000 0001  0000 0111 1111 0000 */
#define OBJ2001_STATE_SUM_VALUE1 ((EC_T_DWORD)0x00010780)    /* value : 0000 0000 0000 0001  0000 0111 1000 0000 */
#define OBJ2001_STATE_SUM_MASK2  ((EC_T_DWORD)0x00007000)    /* mask2 : 0000 0000 0000 0000  0111 0000 0000 0000 */
#define OBJ2001_STATE_SUM_VALUE2 ((EC_T_DWORD)0x00003000)    /* value : 0000 0000 0000 0000  0011 0000 0000 0000 */

typedef struct _EC_T_OBJ2001
{
    EC_T_DWORD dwMasterStateSummary;    /**< Bit 0: Master o.k. \n
                                             Bit 1-3: Reserved \n
                                             Bit 4-7: Master State \n
                                             Bit 8: Slaves in requested State \n
                                             Bit 9: Master in requested State \n
                                             Bit 10: Bus Scan Match \n
                                             Bit 11: Reserved \n
                                             Bit 12: DC is enabled \n
                                             Bit 13: DC In-Sync \n
                                             Bit 14: DC Busy \n
                                             Bit 15: Reserved \n
                                             Bit 16: Link Up \n
                                             Bit 17-31: Reserved */
} EC_PACKED(1) EC_T_OBJ2001;

/* Object 0x2002  Bus Diagnosis Object */
#define OBJ2002_TXFRM_OFFSET    0x18
#define OBJ2002_RXFRM_OFFSET    0x1C
#define OBJ2002_LOSFRM_OFFSET   0x20
#define OBJ2002_CYCFRM_OFFSET   0x24
#define OBJ2002_CYCDGR_OFFSET   0x28
#define OBJ2002_ACYCFRM_OFFSET  0x2C
#define OBJ2002_ACYCDGR_OFFSET  0x30
typedef struct _EC_T_OBJ2002
{
    EC_T_WORD               wSubIndex0;             /* 0x00 */  /* Subindex 000 */
    EC_T_WORD               wReserved;              /* 0x02 */  /* Subindex 001 */
    EC_T_DWORD              dwCRC32ConfigCheckSum;  /* 0x04 */  /* Subindex 002 */
    EC_T_DWORD              dwNumSlavesFound;       /* 0x08 */  /* Subindex 003 */
    EC_T_DWORD              dwNumDCSlavesFound;     /* 0x0C */  /* Subindex 004 */
    EC_T_DWORD              dwNumCfgSlaves;         /* 0x10 */  /* Subindex 005 */
    EC_T_DWORD              dwNumMbxSlaves;         /* 0x14 */  /* Subindex 006 */

    EC_T_DWORD              dwTXFrames;             /* 0x18 */  /* Subindex 007 */
    EC_T_DWORD              dwRXFrames;             /* 0x1C */  /* Subindex 008 */
    EC_T_DWORD              dwLostFrames;           /* 0x20 */  /* Subindex 009 */

    EC_T_DWORD              dwCyclicFrames;         /* 0x24 */  /* Subindex 010 */
    EC_T_DWORD              dwCyclicDatagrams;      /* 0x28 */  /* Subindex 011 */
    EC_T_DWORD              dwAcyclicFrames;        /* 0x2C */  /* Subindex 012 */
    EC_T_DWORD              dwAcyclicDatagrams;     /* 0x30 */  /* Subindex 013 */
    EC_T_DWORD              dwClearCounters;        /* 0x34 */  /* Subindex 014 */
} EC_PACKED(1) EC_T_OBJ2002;

/* Object 0x2003  Redundancy Diagnosis Object */
typedef struct _EC_T_OBJ2003
{
    EC_T_WORD               wSubIndex0;             /* 0x00 */  /* Subindex 000 */
    EC_T_BYTE               byRedEnabled;           /* 0x01 */  /* Subindex 001 */
    EC_T_WORD               wNumOfMainSlaves;       /* 0x03 */  /* Subindex 002 */
    EC_T_WORD               wNumOfRedSlaves;        /* 0x05 */  /* Subindex 003 */
    EC_T_BYTE               byLineBreak;            /* 0x06 */  /* Subindex 004 */
} EC_PACKED(1) EC_T_OBJ2003;

/* Object 0x2004  Notification Counter Object */
typedef struct _EC_T_OBJ2004_NOTIFYMSG
{
  EC_T_DWORD              dwCode;
  EC_T_DWORD              dwCount;
} EC_PACKED(1) EC_T_OBJ2004_NOTIFYMSG;
typedef struct _EC_T_OBJ2004
{
  EC_T_WORD               wSubIndex0;               /* Subindex 000 */
  EC_T_BYTE               byMaxMessages;            /* Subindex 001 */
  EC_T_BYTE               byMessageCount;           /* Subindex 002 */
  EC_T_BYTE               byFlags;                  /* Subindex 003 */
  /* EC_T_OBJ2004_NOTIFYMSG */                      /* Subindex 004ff */
} EC_PACKED(1) EC_T_OBJ2004;

/* Object 0x2005  MAC Address Object */
typedef struct _EC_T_OBJ2005
{
    EC_T_WORD               wSubIndex0;             /* Subindex 000 */
    EC_T_BYTE               abyHardware[6];         /* Subindex 001 */
    EC_T_BYTE               abyRedHardware[6];      /* Subindex 002 */
    EC_T_BYTE               abyCfgSource[6];        /* Subindex 003 */
    EC_T_BYTE               abyCfgDestination[6];   /* Subindex 004 */
} EC_PACKED(1) EC_T_OBJ2005;

/* Object 0x2006  Mailbox Statistics Object */
typedef struct EC_T_OBJ2006
{
    EC_T_WORD               wSubIndex0;             /* Subindex 000 */
    EC_T_DWORD              dwCnt[64];              /* Subindex 001...064 Mailbox Statistics Counters (Read/Write, Total/Last Second) */
    EC_T_UINT64             qwClearCounters;        /* Subindex 065 Clear Counters */
} EC_PACKED(1) EC_T_OBJ2006;

/* Object 0x2007 Add History Diagnosis Message Command */
typedef struct _EC_T_OBJ2007_HISTORY_ADD_DIAGMSG
{
    EC_T_WORD               wSubIndex0;             /* Subindex 000 */
    EC_T_DWORD              dwDiagCode;             /* Subindex 001 Disgnosis code */
    EC_T_BYTE               byType;                 /* Subindex 002 Message type */
    EC_T_WORD               wTextID;                /* Subindex 003 Message text ID */
    EC_T_BYTE               byParmsLen;             /* Subindex 004 Length of the parameters data */
    EC_T_BYTE               abyParms[HISTORY_OBJECT_DIAGELE_SIZE];  /* Subindex 005 Parameters data according to Object 0x10F3 */
} EC_PACKED(1) EC_T_OBJ2007_HISTORY_ADD_DIAGMSG;

/* Object 0x2020  Master Initialization parameters */
typedef struct _EC_T_OBJ2020
{
    EC_T_WORD               wSubIndex0;             /* Subindex 000 */
    EC_T_DWORD              dwApplicationVersion;   /* Subindex 001 */
    EC_T_DWORD              dwMasterVersion;        /* Subindex 002 */
    EC_T_DWORD              dwMaxSlavesProcessedPerCycle; /* Subindex 003 */
    EC_T_DWORD              dwEcatCmdTimeout;       /* Subindex 004 */
    EC_T_DWORD              dwEcatCmdMaxRetries;    /* Subindex 005 */
    EC_T_DWORD              dwBusCycleTimeUsec;     /* Subindex 006 */
    EC_T_DWORD              dwEoeTimeout;           /* Subindex 007. Obsolete. */
    EC_T_DWORD              dwFoeBusyTimeout;       /* Subindex 008. Obsolete. */
    EC_T_DWORD              dwMaxAcycFramesQueued;  /* Subindex 009 */
    EC_T_DWORD              dwMaxAcycCmdsPerFrame;  /* Subindex 010 */
    EC_T_DWORD              dwMaxBusSlave;          /* Subindex 011 */
    EC_T_DWORD              dwReserved2;            /* Subindex 012 */
    EC_T_DWORD              dwStateChangeDebug;     /* Subindex 013 */
    EC_T_CHAR               szDriverIdent[EC_OD_DRIVER_IDENT_SIZE];
                                                    /* Subindex 014 */
    EC_T_BOOL               bPollingModeActive;     /* Subindex 015 */
    EC_T_BOOL               bAllocSendFrameActive;  /* Subindex 016 */
} EC_PACKED(1) EC_T_OBJ2020;

/* Object 0x2102  DCM Bus Shift */
typedef struct _EC_T_OBJ2102
{
    EC_T_WORD               wSubIndex0;             /* Subindex 000 */
    EC_T_DWORD              dwErrorCode;            /* Subindex 001: Error Code */
    EC_T_BOOL               bDcInSync;              /* Subindex 002: DC synchronized */
    EC_T_BOOL               bDcmInSync;             /* Subindex 003: DCM controller synchronized */
    EC_T_INT                nCtlSetVal;             /* Subindex 004: Controller Set Value [ns] */
    EC_T_INT                nCtlErrorFilt;          /* Subindex 005: Controller Error Filtered [ns] */
    EC_T_INT                nCtlErrorAvg;           /* Subindex 006: Controller Error Average [ns] */
    EC_T_INT                nCtlErrorMax;           /* Subindex 007: Controller Error Maximum [ns] */
} EC_PACKED(1) EC_T_OBJ2102;

/* Object 0x2200  Bus Load Statistics */
typedef struct _EC_T_OBJ2200
{
    EC_T_WORD               wSubIndex0;             /* Subindex 000 */

    EC_T_DWORD              dwBytesPerSecondAct;    /* Subindex 001: TX bytes/second actual value */
    EC_T_DWORD              dwBytesPerSecondMin;    /* Subindex 002: TX bytes/second min. value   */
    EC_T_DWORD              dwBytesPerSecondMax;    /* Subindex 003: TX bytes/second max. value   */

    EC_T_DWORD              dwBytesPerCycleAct;     /* Subindex 004: TX bytes/cycle actual value  */
    EC_T_DWORD              dwBytesPerCycleMin;     /* Subindex 005: TX bytes/cycle min. value    */
    EC_T_DWORD              dwBytesPerCycleMax;     /* Subindex 006: TX bytes/cycle max. value    */

    EC_T_WORD               wClearCounters;         /* Subindex 007: Clear Counters mask */
} EC_PACKED(1) EC_T_OBJ2200;

/* Object 0x3000 - 0x3FFF Slave Objects */
typedef struct _EC_T_OBJ3XXX
{
    EC_T_WORD               wSubIndex0;                         /* Subindex 000: (Offset   0) */
    EC_T_BOOL               bEntryValid;                        /* Subindex 001: (Offset   2) */
    EC_T_DWORD              dwVendorID;                         /* Subindex 002: (Offset   6) */
    EC_T_DWORD              dwProductCode;                      /* Subindex 003: (Offset  10) */
    EC_T_DWORD              dwRevisionNo;                       /* Subindex 004: (Offset  14) */
    EC_T_DWORD              dwSerialNo;                         /* Subindex 005: (Offset  18) */

    EC_T_CHAR               szDeviceName[MAX_SLAVE_DEVICENAME]; /* Subindex 006: (Offset  22) */
    EC_T_WORD               wAutoIncAddr;                       /* Subindex 007: (Offset 102) */
    EC_T_WORD               wPhysAddr;                          /* Subindex 008: (Offset 104) */
    EC_T_WORD               wConfigPhysAddr;                    /* Subindex 009: (Offset 106) */
    EC_T_WORD               wAliasAddr;                         /* Subindex 010: (Offset 108) */
    EC_T_WORD               wPortState;                         /* Subindex 011: (Offset 110) */
    EC_T_BOOL               bDCSupport;                         /* Subindex 012: (Offset 112) */
    EC_T_BOOL               bDC64Support;                       /* Subindex 013: (Offset 116) */

    EC_T_BOOL               bMailboxSupport;                    /* Subindex 014: (Offset 120) */
    EC_T_DWORD              dwReqState;                         /* Subindex 015: (Offset 124) */
    EC_T_DWORD              dwCurState;                         /* Subindex 016: (Offset 128) */
    EC_T_BOOL               bErrFlagSet;                        /* Subindex 017: (Offset 132) */
    EC_T_BOOL               bEnableLinkMsgs;                    /* Subindex 018: (Offset 136) */
    EC_T_DWORD              dwErrorCode;                        /* Subindex 019: (Offset 140) */
    EC_T_BOOL               bSyncPulseActive;                   /* Subindex 020: (Offset 144) */
    EC_T_DWORD              dwDCSync0Period;                    /* Subindex 021: (Offset 148) */
    EC_T_DWORD              dwDCSync1Period;                    /* Subindex 022: (Offset 152) */
    EC_T_DWORD              dwSBErrorCode;                      /* Subindex 023: (Offset 156) */

    EC_T_WORD               wRxErrorCounter0;                   /* Subindex 024: (Offset 160) */
    EC_T_WORD               wRxErrorCounter1;                   /* Subindex 025: (Offset 162) */
    EC_T_WORD               wRxErrorCounter2;                   /* Subindex 026: (Offset 164) */
    EC_T_WORD               wRxErrorCounter3;                   /* Subindex 027: (Offset 166) */
    EC_T_BYTE               byFwdRxErrorCounter0;               /* Subindex 028: (Offset 168) */
    EC_T_BYTE               byFwdRxErrorCounter1;               /* Subindex 029: (Offset 169) */
    EC_T_BYTE               byFwdRxErrorCounter2;               /* Subindex 030: (Offset 170) */
    EC_T_BYTE               byFwdRxErrorCounter3;               /* Subindex 031: (Offset 171) */

    EC_T_BYTE               byEcatProcUnitErrorCounter;         /* Subindex 032: (Offset 172) */
    EC_T_BYTE               byPDIErrorCounter;                  /* Subindex 033: (Offset 173) */
    EC_T_WORD               wMbxSupportedProtocols;             /* Subindex 034: (Offset 174) */
    EC_T_BYTE               byLostLinkCounter0;                 /* Subindex 035: (Offset 176) */
    EC_T_BYTE               byLostLinkCounter1;                 /* Subindex 036: (Offset 177) */
    EC_T_BYTE               byLostLinkCounter2;                 /* Subindex 037: (Offset 178) */
    EC_T_BYTE               byLostLinkCounter3;                 /* Subindex 038: (Offset 179) */
    EC_T_BYTE               byFmmusSupported;                   /* Subindex 039: (Offset 180) */
    EC_T_BYTE               bySyncManagersSupported;            /* Subindex 040: (Offset 181) */
    EC_T_BYTE               byRamSizeKb;                        /* Subindex 041: (Offset 182) */
    EC_T_BYTE               byPortDescriptor;                   /* Subindex 042: (Offset 183) */

    EC_T_BYTE               byESCType;                          /* Subindex 043: (Offset 184) */

    EC_T_BOOL               bSlaveIsOptional;                   /* Subindex 044: (Offset 185) */
    EC_T_BOOL               bSlaveIsPresent;                    /* Subindex 045: (Offset 189) */
    EC_T_DWORD              dwHotConnectGroupId;                /* Subindex 046: (Offset 193) */
    EC_T_DWORD              dwSystemTimeDifference;             /* Subindex 047: (Offset 197) */
    EC_T_DWORD              dwPdOffsIn;                         /* Subindex 048: (Offset 201) Process data offset of input data (in Bits) */
    EC_T_DWORD              dwPdSizeIn;                         /* Subindex 049: (Offset 205) Process data size of input data (in Bits) */
    EC_T_DWORD              dwPdOffsOut;                        /* Subindex 050: (Offset 209) Process data offset of output data (in Bits) */
    EC_T_DWORD              dwPdSizeOut;                        /* Subindex 051: (Offset 213) Process data size of output data (in Bits) */
} EC_PACKED(1) EC_T_OBJ3XXX;

/* Object 0x8000 - 0x8FFF Slave Objects (configured slaves) "Modular Device Profiles" */
typedef struct _EC_T_OBJ8XXX
{
    EC_T_WORD               wSubIndex0;                         /*   0 */       /* Subindex 000 */
    EC_T_WORD               wFixedStationAddr;                  /*   2 */       /* Subindex 001: Station Address of the first EtherCAT Slave (same value as 0xF020:01) */
    EC_T_CHAR               szType[64];                         /*   4 */       /* Subindex 002: Type of the first EtherCAT Slave configured */
    EC_T_CHAR               szName[64];                         /*  68 */       /* Subindex 003: Name of the first EtherCAT Slave configured (object 0x1008 of the EtherCAT slave) */
    EC_T_DWORD              dwDeviceType;                       /* 132 */       /* Subindex 004: Device Type of the first EtherCAT Slave configured (object 0x1000 of the EtherCAT slave) */
    EC_T_DWORD              dwVendorID;                         /* 136 */       /* Subindex 005: Vendor ID of the first EtherCAT Slave configured (entry 0x1018:01 of the EtherCAT slave) */
    EC_T_DWORD              dwProductCode;                      /* 140 */       /* Subindex 006: Product Code of the first EtherCAT Slave configured (entry 0x1018:02 of the EtherCAT slave) */
    EC_T_DWORD              dwRevision;                         /* 144 */       /* Subindex 007: Revision of the first EtherCAT Slave configured (entry 0x1018:03 of the EtherCAT slave) */
    EC_T_DWORD              dwSerial;                           /* 148 */       /* Subindex 008: Serial No of the first EtherCAT Slave configured (entry 0x1018:04 of the EtherCAT slave) */
    EC_T_WORD               wMailboxOutSize;                    /* 152 */       /* Subindex 033: Mailbox Write Size (SM0) of the first EtherCAT Slave configured */
    EC_T_WORD               wMailboxInSize;                     /* 154 */       /* Subindex 034: Mailbox Read Size (SM1) of the first EtherCAT Slave configured */
    EC_T_BYTE               byLinkPreset;                       /* 156 */       /* Subindex 036: Reports the expected physical link on each port of slave */
    EC_T_BYTE               byFlags;                            /* 157 */       /* Subindex 037: Provides additional topology information about slave */
} EC_PACKED(1) EC_T_OBJ8XXX;

/* Modular Device Profiles: EtherCAT Master  - internal slave object element (connected slaves) */
typedef struct _EC_T_OBJ9XXX
{
    EC_T_WORD               wSubIndex0;                         /*   0 */       /* Subindex 000 */
    EC_T_WORD               wFixedStationAddr;                  /*   2 */       /* Subindex 001: Fixed Station Address of the first EtherCAT Slave (same value as 0xF020:01) */
    EC_T_DWORD              dwVendorID;                         /*   4 */       /* Subindex 005: Vendor ID of the first EtherCAT Slave configured (entry 0x1018:01 of the EtherCAT slave) */
    EC_T_DWORD              dwProductCode;                      /*   8 */       /* Subindex 006: Product Code of the first EtherCAT Slave configured (entry 0x1018:02 of the EtherCAT slave) */
    EC_T_DWORD              dwRevision;                         /*  12 */       /* Subindex 007: Revision of the first EtherCAT Slave configured (entry 0x1018:03 of the EtherCAT slave) */
    EC_T_DWORD              dwSerial;                           /*  16 */       /* Subindex 008: Serial No. of the first EtherCAT Slave configured (entry 0x1018:04 of the EtherCAT slave) */
    EC_T_WORD               wDLStatus;                          /*  20 */       /* Subindex 032: DL Status (Register 0x110-0x111) of the first EtherCAT found */
} EC_PACKED(1) EC_T_OBJ9XXX;

/* Modular Device Profiles: EtherCAT Master  - internal slave object element (slave diagnosis) */
typedef struct _EC_T_OBJAXXX
{
    EC_T_WORD               wSubIndex0;                         /*   0 */       /* Subindex 000 */
    EC_T_WORD               wALStatus;                          /*   2 */       /* Subindex 001: AL Status (Register 0x130-0x131) of the first EtherCAT slave configured */
    EC_T_WORD               wALControl;                         /*   4 */       /* Subindex 002: AL Control (Register 0x120-0x121) of the first EtherCAT slave configured */
    EC_T_WORD               wLastALStatusCode;                                  /* Subindex 003: Last AL Status Code */
    EC_T_BYTE               byLinkConnStatus;                                   /* Subindex 004: Link Conn. Status */
    EC_T_BYTE               byLinkControl;                                      /* Subindex 005: Link Control */
    EC_T_WORD               wFixedAddressConnPort0;                             /* Subindex 006: Fixed Address Conn. Port 0 */
    EC_T_WORD               wFixedAddressConnPort1;                             /* Subindex 007: Fixed Address Conn. Port 1 */
    EC_T_WORD               wFixedAddressConnPort2;                             /* Subindex 008: Fixed Address Conn. Port 2 */
    EC_T_WORD               wFixedAddressConnPort3;                             /* Subindex 009: Fixed Address Conn. Port 3 */
    EC_T_DWORD              dwCRCErrorCounterPort0;                             /* Subindex 010: CRC Error Counter Port 0 */
    EC_T_DWORD              dwCRCErrorCounterPort1;                             /* Subindex 011: CRC Error Counter Port 1 */
    EC_T_DWORD              dwCRCErrorCounterPort2;                             /* Subindex 012: CRC Error Counter Port 2 */
    EC_T_DWORD              dwCRCErrorCounterPort3;                             /* Subindex 013: CRC Error Counter Port 3 */
    EC_T_DWORD              dwCyclicWCErrorCounter;                             /* Subindex 014: Cyclic WC Error Counter */
    EC_T_DWORD              dwSlaveNotPresentCounter;                           /* Subindex 015: Slave Not Present Counter */
    EC_T_DWORD              dwAbnormalStateCounter;                             /* Subindex 016: Abnormal State Counter */
    EC_T_BYTE               bDisableAutomaticLinkControl;                       /* Subindex 017: Disable Automatic Link Control */
} EC_PACKED(1) EC_T_OBJAXXX;

/* Modular Device Profiles: EtherCAT Master  - modular device profile */
typedef struct _EC_T_OBJF000
{
    EC_T_WORD               wSubIndex0;                         /*   0 */       /* Subindex 000 */
    EC_T_WORD               wIndexDistance;                     /*   2 */       /* Subindex 001: Index distance between two modules = 0x01 */
    EC_T_WORD               wMaxModuleCnt;                      /*   4 */       /* Subindex 002: Maximum number of EtherCAT Slaves connected to the EtherCAT Master = 4080 */
    EC_T_DWORD              dwGeneralCfg;                       /*   8 */       /* Subindex 003: Available entries in objects 0x8nn0 = 0x000000FF */
    EC_T_DWORD              dwGeneralInfo;                      /*  12 */       /* Subindex 004: Available entries in objects 0x9nn0 = 0x000000F1 (if information data supported) or 0x00000000 (if information data not supported) */
} EC_PACKED(1) EC_T_OBJF000;

/* Modular Device Profiles: EtherCAT Master  - detect modules command */
typedef struct _EC_T_OBJF002
{
    EC_T_WORD               wSubIndex0;                         /*   0 */       /* Subindex 000 */
    EC_T_BYTE               abyCmdRequest[2];                   /*   1 */       /* Subindex 001: When this subindex is written, the EtherCAT Master shall scan the EtherCAT bus and update the objects 0xF04x and 0x9nnn */
    EC_T_BYTE               byCmdStatus;                        /*   3 */       /* Subindex 002: 1: command is finished, no error
                                                                                                 3: command is finished, error
                                                                                           100-199: 0-99% of the command is done
                                                                                               255: command is executing */
    EC_T_BYTE               abyCmdResponse[6];                  /*   4 */       /* Subindex 003: Byte 0: like Subindex 2
                                                                                                 Byte 1: always 0
                                                                                                 Byte 2-3: 0: no error, > 0: vendor specific error code
                                                                                                 Byte 4-5: number of EtherCAT slaves found */
} EC_PACKED(1) EC_T_OBJF002;

/* Modular Device Profiles: EtherCAT Master  - configured address list */
typedef struct _EC_T_OBJF02X
{
    EC_T_WORD               wSubIndex0;                         /*   0 */       /* Subindex 000 */
    EC_T_WORD               wStationAddr[255];                  /*   2 */       /* Subindex 001 - 255 */
} EC_PACKED(1) EC_T_OBJF02X;

/* Modular Device Profiles: EtherCAT Master  - detected address list */
typedef struct _EC_T_OBJF04X
{
    EC_T_WORD               wSubIndex0;                         /*   0 */       /* Subindex 000 */
    EC_T_WORD               wStationAddr[255];                  /*   2 */       /* Subindex 001 - 255 */
} EC_PACKED(1) EC_T_OBJF04X;

/* Modular Device Profiles: EtherCAT Master  - frame statistics */
typedef struct _EC_T_OBJF120
{
    EC_T_WORD               wSubIndex0;                                         /* Subindex 000 */
    EC_T_DWORD              dwCyclicLostFrames;                                 /* Subindex 001: Number of cyclic lost frames. */
    EC_T_DWORD              dwAcyclicLostFrames;                                /* Subindex 001: Number of acyclic lost frames. */
} EC_PACKED(1) EC_T_OBJF120;

/* Modular Device Profiles: EtherCAT Master  - diag interface control */
typedef struct _EC_T_OBJF200
{
    EC_T_WORD               wSubIndex0;                                         /* Subindex 000 */
    EC_T_BOOL               bResetDiagInfo;                                     /* Subindex 016: A rising edge resets values to 0 of objects
                                                                                                 0xAnnn SI 03, 0xAnnn SI 04, 0xAnnn SI05-08, 0xAnnn SI08,
                                                                                                 0xAnnn SI09, 0xF120 SI01 and 0xF120 SI01.. */
} EC_PACKED(1) EC_T_OBJF200;
#include EC_PACKED_INCLUDESTOP/*(1)*/

#include EC_PACKED_INCLUDESTART(4)

/* EtherCAT slave properties */
typedef struct _EC_T_SLAVE_PROP
{
    EC_T_WORD   wStationAddress;        /**< station address or INVALID_FIXED_ADDR */
    EC_T_WORD   wAutoIncAddr;           /**< auto increment address or INVALID_AUTO_INC_ADDR*/
    EC_T_CHAR   achName[MAX_STD_STRLEN];/**< name of the slave device (NULL terminated string) */
} EC_PACKED(4) EC_T_SLAVE_PROP;


/* EtherCAT notify parameters */
typedef struct _EC_T_NOTIFYPARMS
{
    EC_T_VOID*      pCallerData;        /**< [in]  Client depending caller data parameter. This pointer is one of the parameters when the client registers */
    EC_T_BYTE*      pbyInBuf;           /**< [in]  Notification input parameters */
    EC_T_DWORD      dwInBufSize;        /**< [in]  Size of input buffer in byte */
    EC_T_BYTE*      pbyOutBuf;          /**< [out] Notification output (result) */
    EC_T_DWORD      dwOutBufSize;       /**< [in]  Size of output buffer in byte */
    EC_T_DWORD*     pdwNumOutData;      /**< [out] Amount of bytes written to the output buffer */
} EC_PACKED(4) EC_T_NOTIFYPARMS, *PEC_T_NOTIFYPARMS;

/* EtherCAT command WKC error descriptor */
typedef struct _EC_T_WKCERR_DESC
{
    EC_T_SLAVE_PROP SlaveProp;          /**< Slave properties, content is undefined in case of cyclic WKC_ERROR */
    EC_T_BYTE       byCmd;              /**< EtherCAT command type */
    EC_T_BYTE       byRsvd[3];
    EC_T_DWORD      dwAddr;             /**< Logical address or physical address (ADP/ADO) */
    EC_T_WORD       wWkcSet;            /**< Working counter set value */
    EC_T_WORD       wWkcAct;            /**< Working counter actual value */
} EC_PACKED(4) EC_T_WKCERR_DESC;

/* EtherCAT frame response error descriptor */
typedef enum _EC_T_FRAME_RSPERR_TYPE
{
    eRspErr_UNDEFINED       = 0,        /**< undefined */
    eRspErr_NO_RESPONSE     = 1,        /**< No Ethernet frame received (timeout, frame loss) */
    eRspErr_WRONG_IDX       = 2,        /**< Wrong IDX value in acyclic frame */
    eRspErr_UNEXPECTED      = 3,        /**< Unexpected frame was received */
    eRspErr_FRAME_RETRY     = 4,        /**< Ethernet frame will be re-sent (timeout, frame loss) */
    eRspErr_RETRY_FAIL      = 5,        /**< all retry mechanism fails to re-sent acyclic frames */
    eRspErr_FOREIGN_SRC_MAC = 6,        /**< Frame with MAC from other Master received */
    eRspErr_NON_ECAT_FRAME  = 7,        /**< Non EtherCAT frame received */
    eRspErr_CRC             = 8,        /**< Ethernet frame with CRC error received */
    /* Borland C++ datatype alignment correction */
    eRspErr_BCppDummy   = 0xFFFFFFFF
} EC_T_FRAME_RSPERR_TYPE;

#define EcFrameRspErrText(eInitCmdErr) \
    ((eInitCmdErr)==eRspErr_UNDEFINED?"Undefined": \
     ((eInitCmdErr)==eRspErr_NO_RESPONSE?"No response": \
      ((eInitCmdErr)==eRspErr_WRONG_IDX?"Wrong index": \
       ((eInitCmdErr)==eRspErr_UNEXPECTED?"Unexpected": \
        ((eInitCmdErr)==eRspErr_FRAME_RETRY?"Frame retry": \
         ((eInitCmdErr)==eRspErr_RETRY_FAIL?"Retry fail": \
          ((eInitCmdErr)==eRspErr_FOREIGN_SRC_MAC?"Foreign source MAC": \
           ((eInitCmdErr)==eRspErr_NON_ECAT_FRAME?"Non EtherCAT frame": \
             "Unknown error" \
    ))))))))

typedef struct _EC_T_FRAME_RSPERR_DESC
{
    EC_T_BOOL               bIsCyclicFrame;         /**< Indicates whether the lost frame was a cyclic frame */
    EC_T_FRAME_RSPERR_TYPE  EErrorType;             /**< Frame response error type */
    EC_T_BYTE               byEcCmdHeaderIdxSet;    /**< Expected IDX value, this value is valid only for acyclic frames in case EErrorType is not equal to eRspErr_UNEXPECTED */
    EC_T_BYTE               byEcCmdHeaderIdxAct;    /**< Actually received IDX value, this value is only valid for acyclic frames in case of EErrorType is equal to: eRspErr_WRONG_IDX and eRspErr_UNEXPECTED */
    EC_T_WORD               wCycFrameNum;           /**< Number of the lost cyclic frame from the ENI */
    EC_T_DWORD              dwTaskId;               /**< Cyclic Task Id from the ENI. Only valid if bIsCyclicFrame is set */
} EC_PACKED(4) EC_T_FRAME_RSPERR_DESC;

/* EtherCAT init command response error descriptor */
typedef enum _EC_T_INITCMD_ERR_TYPE
{
    eInitCmdErr_NO_ERROR        = 0,    /**< No error */
    eInitCmdErr_NO_RESPONSE     = 1,    /**< No Ethernet frame received (timeout) */
    eInitCmdErr_VALIDATION_ERR  = 2,    /**< Validation error (invalid slave command response) */
    eInitCmdErr_FAILED          = 3,    /**< Init commands failed (state could not be reached) */
    eInitCmdErr_NOT_PRESENT     = 4,    /**< Slave not present on the bus */
    eInitCmdErr_ALSTATUS_ERROR  = 5,    /**< Error in AL Status Register */
    eInitCmdErr_MBXSLAVE_ERROR  = 6,    /**< Error at Mailbox Init Command */
    eInitCmdErr_PDI_WATCHDOG    = 7,    /**< PDI watchdog has been detected */

    /* Borland C++ datatype alignment correction */
    eInitCmdErr_BCppDummy       = 0xFFFFFFFF
} EC_T_INITCMD_ERR_TYPE;

#define EcInitCmdErrText(eInitCmdErr) \
    ((eInitCmdErr)==eInitCmdErr_NO_ERROR?"No error": \
     ((eInitCmdErr)==eInitCmdErr_VALIDATION_ERR?"Validation failed": \
      ((eInitCmdErr)==eInitCmdErr_NO_RESPONSE?"No response": \
       ((eInitCmdErr)==eInitCmdErr_FAILED?"Failed": \
        ((eInitCmdErr)==eInitCmdErr_NOT_PRESENT?"Slave not present": \
         ((eInitCmdErr)==eInitCmdErr_NOT_PRESENT?"Slave not present": \
          ((eInitCmdErr)==eInitCmdErr_ALSTATUS_ERROR?"AL Status error": \
           ((eInitCmdErr)==eInitCmdErr_MBXSLAVE_ERROR?"Mailbox error": \
            ((eInitCmdErr)==eInitCmdErr_PDI_WATCHDOG?"PDI watchdog expired": \
             "Unknown error" \
    )))))))))

typedef struct _EC_T_INITCMD_ERR_DESC
{
    EC_T_SLAVE_PROP       SlaveProp;                              /**< Slave properties */
    EC_T_CHAR             achStateChangeName[MAX_SHORT_STRLEN];   /**< State change description when the error occurred */
    EC_T_INITCMD_ERR_TYPE EErrorType;                             /**< Init command error type */
    EC_T_CHAR             szComment[MAX_STD_STRLEN];              /**< Comment (ENI) */
} EC_PACKED(4) EC_T_INITCMD_ERR_DESC;

/* EtherCAT Slave error status info descriptor */
typedef struct _EC_T_SLAVE_ERROR_INFO_DESC
{
    EC_T_SLAVE_PROP SlaveProp;          /**< Slave properties */
    EC_T_WORD       wStatus;            /**< Slave Status (AL Status) */
    EC_T_WORD       wStatusCode;        /**< Error status code (AL STATUS CODE) */
} EC_PACKED(4) EC_T_SLAVE_ERROR_INFO_DESC;

/* Slaves error status descriptor */
#ifndef MAX_SLAVES_ERROR_NTFY_ENTRIES
#define MAX_SLAVES_ERROR_NTFY_ENTRIES 128
#endif
typedef struct _EC_T_SLAVES_ERROR_DESC_ENTRY
{
    EC_T_WORD wStationAddress;        /**< Slave station address */
    EC_T_WORD wStatus;                /**< Slave status (AL Status) */
    EC_T_WORD wStatusCode;            /**< Slave status code (AL Control Status) */
    EC_T_WORD wRes;
} EC_PACKED(4) EC_T_SLAVES_ERROR_DESC_ENTRY;
typedef struct _EC_T_SLAVES_ERROR_DESC
{
    EC_T_WORD wCount;                /**< Number of slave errors */
    EC_T_WORD wRes;
    EC_T_SLAVES_ERROR_DESC_ENTRY SlaveError[MAX_SLAVES_ERROR_NTFY_ENTRIES]; /**< Slave error descriptions */
} EC_PACKED(4) EC_T_SLAVES_ERROR_DESC;

/* EtherCAT Sdo abort */
typedef struct _EC_T_MBOX_SDO_ABORT_DESC
{
    EC_T_SLAVE_PROP SlaveProp;          /**< Slave properties */
    EC_T_DWORD      dwErrorCode;        /**< Error code */
    EC_T_WORD       wObjIndex;          /**< SDO object index */
    EC_T_BYTE       bySubIndex;         /**< SDO object sub index */
} EC_PACKED(4) EC_T_MBOX_SDO_ABORT_DESC;

/* EtherCAT FoE error */
typedef struct _EC_T_MBOX_FOE_ABORT_DESC
{
    EC_T_SLAVE_PROP SlaveProp;                      /**< Slave properties */
    EC_T_DWORD      dwErrorCode;                    /**< Error code */
    EC_T_CHAR       achErrorString[MAX_STD_STRLEN]; /**< FoE error string */
} EC_PACKED(4) EC_T_MBOX_FOE_ABORT_DESC;

/* Invalid mailbox data received error */
typedef struct _EC_T_MBXRCV_INVALID_DATA_DESC
{
    EC_T_SLAVE_PROP SlaveProp;          /**< Slave properties */
} EC_PACKED(4) EC_T_MBXRCV_INVALID_DATA_DESC;

/* PDI Watchdog expired */
typedef struct _EC_T_PDIWATCHDOG_DESC
{
    EC_T_SLAVE_PROP SlaveProp;          /**< Slave properties */
} EC_PACKED(4) EC_T_PDIWATCHDOG_DESC;

/* Slave not supported */
typedef struct _EC_T_SLAVE_NOTSUPPORTED_DESC
{
    EC_T_SLAVE_PROP SlaveProp;          /**< Slave properties */
} EC_PACKED(4) EC_T_SLAVE_NOTSUPPORTED_DESC;

/* Slave in unexpected state descriptor */
typedef struct _EC_T_SLAVE_UNEXPECTED_STATE_DESC
{
    EC_T_SLAVE_PROP SlaveProp;          /**< Slave properties */
    EC_T_STATE      curState;           /**< Current state */
    EC_T_STATE      expState;           /**< Expected state */
} EC_PACKED(4) EC_T_SLAVE_UNEXPECTED_STATE_DESC;

/* Slaves in unexpected state descriptor */
#ifndef MAX_SLAVES_UNEXPECTED_STATE_NTFY_ENTRIES
#define MAX_SLAVES_UNEXPECTED_STATE_NTFY_ENTRIES 128
#endif
typedef struct _EC_T_SLAVES_UNEXPECTED_STATE_DESC_ENTRY
{
    EC_T_WORD  wStationAddress;         /**< Slave station address */
    EC_T_STATE curState;                /**< Current state */
    EC_T_STATE expState;                /**< Expected state */
} EC_PACKED(4) EC_T_SLAVES_UNEXPECTED_STATE_DESC_ENTRY;
typedef struct _EC_T_SLAVES_UNEXPECTED_STATE_DESC
{
    EC_T_WORD wCount;                   /**< Number of unexpected slave state changes */
    EC_T_WORD wRes;
    EC_T_SLAVES_UNEXPECTED_STATE_DESC_ENTRY SlaveStates[MAX_SLAVES_UNEXPECTED_STATE_NTFY_ENTRIES];  /**< Slave state change descriptions */
} EC_PACKED(4) EC_T_SLAVES_UNEXPECTED_STATE_DESC;

/* Slave EEPROM checksum error */
typedef struct _EC_T_EEPROM_CHECKSUM_ERROR_DESC
{
    EC_T_SLAVE_PROP SlaveProp;          /**< Slave properties */
} EC_PACKED(4) EC_T_EEPROM_CHECKSUM_ERROR_DESC;

/* Redundancy break/fixed notification */
typedef struct _EC_T_RED_CHANGE_DESC
{
    EC_T_WORD       wNumOfSlavesMain;    /**< Number of Slaves on Main Line */
    EC_T_WORD       wNumOfSlavesRed;     /**< Number of Slaves on Red Line  */
} EC_PACKED(4) EC_T_RED_CHANGE_DESC;

typedef struct _EC_T_JUNCTION_RED_CHANGE_DESC
{
    EC_T_SLAVE_PROP SlaveProp;          /**< Slave properties */
    EC_T_BOOL       bLineBreak;         /**< EC_TRUE for line break, EC_FALSE for line fixed */
    EC_T_WORD       wPort;              /**< Port */
} EC_PACKED(4) EC_T_JUNCTION_RED_CHANGE_DESC;

/* Distributed clocks (DC) Reference Clock Presence */
typedef struct _EC_T_REFCLOCK_PRESENCE_NTFY_DESC
{
    EC_T_BOOL  bPresent;                /**< [in] Reference clock present */
    EC_T_SLAVE_PROP SlaveProp;          /**< [in] Slave properties */
} EC_PACKED(4) EC_T_REFCLOCK_PRESENCE_NTFY_DESC;

/* Distributed clocks (DC) */
typedef struct _EC_T_DC_SYNC_NTFY_DESC
{
    EC_T_DWORD      IsInSync;           /**< [in] EC_TRUE : Wire or'ed deviation value meets limit requirements. EC_FALSE: Wire or'ed deviation value does not meet limit requirements.The limit is set by ecatDcConfigure() */
    EC_T_DWORD      IsNegative;         /**< [in] EC_TRUE : deviation value is negative EC_FALSE: deviation value is positive */
    EC_T_DWORD      dwDeviation;        /**< [in] Wire or'ed deviation value [ns] in case of in sync */
    EC_T_SLAVE_PROP SlaveProp;          /**< [in] Slave properties in case of out of sync */
} EC_PACKED(4) EC_T_DC_SYNC_NTFY_DESC;

/* Distributed clocks master sync (DCM) */
typedef struct _EC_T_DCM_SYNC_NTFY_DESC
{
    EC_T_DWORD      IsInSync;           /**< [in] EC_TRUE as long as time of master and reference clock are in sync. False if the InSyncLimit from the bus shift configuration is exceeded */
    EC_T_INT        nCtlErrorNsecCur;   /**< [in] Current difference [ns] between set value and actual value of controller */
    EC_T_INT        nCtlErrorNsecAvg;   /**< [in] Average difference [ns] between set value and actual value of controller */
    EC_T_INT        nCtlErrorNsecMax;   /**< [in] Maximum difference [ns] between set value and actual value of controller */
} EC_PACKED(4) EC_T_DCM_SYNC_NTFY_DESC;

/* Distributed clocks master external sync (DCX) */
typedef struct _EC_T_DCX_SYNC_NTFY_DESC
{
    EC_T_DWORD      IsInSync;           /**< EC_TRUE if external(other Ethercat segment) and internal reference clock are in sync respectivley */
    EC_T_INT        nCtlErrorNsecCur;   /**< Current DCX controller error [ns] */
    EC_T_INT        nCtlErrorNsecAvg;   /**< Average DCX controller error [ns] */
    EC_T_INT        nCtlErrorNsecMax;   /**< Maximum DCX controller error [ns] */
    EC_T_INT64      nTimeStampDiff;     /**< Difference between external and internal time stamp [ns] */
    EC_T_DWORD      dwErrorCode;        /**< Dcx external clock error code */
} EC_PACKED(4) EC_T_DCX_SYNC_NTFY_DESC;

/* Slave state change descriptor */
typedef struct _EC_T_SLAVE_STATECHANGED_NTFY_DESC
{
    EC_T_SLAVE_PROP SlaveProp;          /**< Slave properties */
    EC_T_STATE      newState;           /**< New slave state */
} EC_PACKED(4) EC_T_SLAVE_STATECHANGED_NTFY_DESC;
#include EC_PACKED_INCLUDESTOP/*(4)*/

#include EC_PACKED_INCLUDESTART(2)
/* Slaves state change descriptor */
#ifndef MAX_SLAVES_STATECHANGED_NTFY_ENTRIES
#define MAX_SLAVES_STATECHANGED_NTFY_ENTRIES 128
#endif
typedef struct _EC_T_SLAVES_STATECHANGED_NTFY_DESC_ENTRY
{
    EC_T_WORD  wStationAddress;     /**< Slave station address */
    EC_T_BYTE  byState;             /**< New slave state */
} EC_PACKED(2) EC_T_SLAVES_STATECHANGED_NTFY_DESC_ENTRY;

typedef struct _EC_T_SLAVES_STATECHANGED_NTFY_DESC
{
    EC_T_WORD wCount;                                                                               /**< Number of slave state changes */
    EC_T_SLAVES_STATECHANGED_NTFY_DESC_ENTRY SlaveStates[MAX_SLAVES_STATECHANGED_NTFY_ENTRIES];     /**< Slave state changed descriptor */
} EC_PACKED(2) EC_T_SLAVES_STATECHANGED_NTFY_DESC;
#define SIZEOF_EC_T_SLAVES_STATECHANGED_NTFY_DESC(wCount) (sizeof(EC_T_WORD)+wCount*sizeof(EC_T_SLAVES_STATECHANGED_NTFY_DESC_ENTRY))
#include EC_PACKED_INCLUDESTOP/*(2)*/

#include EC_PACKED_INCLUDESTART(4)

typedef struct _EC_T_FRAMELOSS_AFTER_SLAVE_NTFY_DESC
{
    /* Location of frameloss */
    EC_T_SLAVE_PROP SlaveProp;  /**< slave properties */
    EC_T_WORD       wPort;      /**< port */
} EC_PACKED(4) EC_T_FRAMELOSS_AFTER_SLAVE_NTFY_DESC;

typedef struct _EC_T_BAD_CONNECTION_NTFY_DESC
{
    EC_T_SLAVE_PROP SlavePropParent; /**< slave properties of parent slave */
    EC_T_WORD       wPortAtParent;   /**< port at parent slave */
    EC_T_SLAVE_PROP SlavePropChild;  /**< slave properties of child slave */
    EC_T_WORD       wPortAtChild;    /**< port at child slave */
} EC_PACKED(4) EC_T_BAD_CONNECTION_NTFY_DESC;

typedef struct _EC_T_COMMUNICATION_TIMEOUT_NTFY_DESC
{
    EC_T_BOOL       bMainTapPortIn;         /**< EC_TRUE: Timeout occurred at the input port of the Ethernet TAP for the EtherCAT main line */
    EC_T_BOOL       bMainTapPortOut;        /**< EC_TRUE: Timeout occurred at the output port of the Ethernet TAP for the EtherCAT main line */
} EC_PACKED(4) EC_T_COMMUNICATION_TIMEOUT_NTFY_DESC;

typedef struct _EC_T_TAP_LINK_STATUS_NTFY_DESC
{
    EC_T_BOOL    bLinkConnected;            /**< Link status of EC-Monitor - Ethernet Tap connection */
} EC_PACKED(4) EC_T_TAP_LINK_STATUS_NTFY_DESC;

typedef struct _EC_T_DC_BURSTCONFIG_REQ
{
    EC_T_DWORD  dwTotalBurstLength;
    EC_T_DWORD  dwBurstBulk;
} EC_PACKED(4) EC_T_DC_BURSTCONFIG_REQ;

typedef struct _EC_T_DC_SHIFTSYSTIME_DESC
{
    EC_T_DWORD      dwCyclesToShift;    /**< Amount of cycles to apply shift */
    EC_T_INT        nShiftTime;         /**< Shift Time [ns] (signed) */
} EC_PACKED(4) EC_T_DC_SHIFTSYSTIME_DESC;

typedef  struct _EC_T_DC_STARTCYCSAFETY_DESC    /**< Start SYNC Cyc Safety [ns] (64Bit)*/
{
    EC_T_DWORD      dwStartCycSafetyLo;
    EC_T_DWORD      dwStartCycSafetyHi;
}   EC_T_DC_STARTCYCSAFETY_DESC;

typedef struct _EC_T_SB_STATUS_NTFY_DESC
{
    EC_T_DWORD      dwResultCode;       /**< [in] EC_E_NOERROR: success EC_E_NOTREADY: no bus scan executed EC_E_BUSCONFIG_MISMATCH: bus configuration mismatch Result of scanbus */
    EC_T_DWORD      dwSlaveCount;       /**< [in] number of slaves connected to the bus */
} EC_PACKED(4) EC_T_SB_STATUS_NTFY_DESC;

typedef struct _EC_T_SB_MISMATCH_DESC
{
    /* Location of mismatch */
    EC_T_WORD           wPrevFixedAddress;      /**< [in] Previous slave station address */
    EC_T_WORD           wPrevPort;              /**< [in] Previous slave station address */
    EC_T_WORD           wPrevAIncAddress;       /**< [in] Previous slave auto-increment address */

    /* Unexpected bus slave */
    EC_T_WORD           wBusAIncAddress;        /**< [in] Unexpected slave (bus) auto-inc address */
    EC_T_DWORD          dwBusVendorId;          /**< [in] Unexpected slave (bus) vendor ID */
    EC_T_DWORD          dwBusProdCode;          /**< [in] Unexpected slave (bus) product code */
    EC_T_DWORD          dwBusRevisionNo;        /**< [in] Unexpected slave (bus) revision number */
    EC_T_DWORD          dwBusSerialNo;          /**< [in] Unexpected slave (bus) serial number */
    EC_T_WORD           wBusFixedAddress;       /**< [in] Unexpected slave (bus) station address */
    EC_T_WORD           wIdentificationVal;     /**< [in] last identification value read from slave according to the last used identification method */

    /* Missing config slave */
    EC_T_WORD           wCfgFixedAddress;       /**< [in] Missing slave (config) station Address */
    EC_T_WORD           wCfgAIncAddress;        /**< [in] Missing slave (config) Auto-Increment Address */
    EC_T_DWORD          dwCfgVendorId;          /**< [in] Missing slave (config) Vendor ID */
    EC_T_DWORD          dwCfgProdCode;          /**< [in] Missing slave (config) Product code */
    EC_T_DWORD          dwCfgRevisionNo;        /**< [in] Missing slave (config) Revision Number */
    EC_T_DWORD          dwCfgSerialNo;          /**< [in] Missing slave (config) Serial Number */

    /* Hot connect */
    EC_T_BOOL           bIdentValidationError;  /**< [in] Hotconnect Identification command sent to slave but failed */
    EC_T_WORD           oIdentCmdHdr[5];        /**< [in] Last HotConnect Identification command header (if bIdentValidationError) */
    EC_T_DWORD          dwCmdData;              /**< [in] First DWORD of Data portion of last identification command */
    EC_T_DWORD          dwCmdVMask;             /**< [in] First DWORD of Validation mask of last identification command */
    EC_T_DWORD          dwCmdVData;             /**< [in] First DWORD of Validation data of last identification command */
} EC_PACKED(4) EC_T_SB_MISMATCH_DESC;

typedef struct _EC_T_LINE_CROSSED_DESC
{
    EC_T_SLAVE_PROP     SlaveProp;       /**< slave properties */
    EC_T_WORD           wInputPort;      /**< port where frame was received */
} EC_PACKED(4) EC_T_LINE_CROSSED_DESC;

typedef struct _EC_T_HC_DETECTALLGROUP_NTFY_DESC
{
    EC_T_DWORD  dwResultCode;       /**< Result of Group detection */
    EC_T_DWORD  dwGroupCount;       /**< Number of Groups */
    EC_T_DWORD  dwGroupsPresent;    /**< Number of connected groups */
    EC_T_DWORD  dwGroupMask;        /**< Bitmask of first 32   Groups 1 = present, 0 = absent */
    EC_T_DWORD  adwGroupMask[100];  /**< Bitmask of first 3200 Groups */
} EC_PACKED(4) EC_T_HC_DETECTALLGROUP_NTFY_DESC;

typedef struct _EC_T_RAWCMDRESPONSE_NTFY_DESC
{
    EC_T_DWORD  dwInvokeId;         /**< [in] Invoke Id from callee. Only lower 16 bits are relevant */
    EC_T_DWORD  dwResult;           /**< [in] EC_E_NOERROR on success, error code otherwise */
    EC_T_DWORD  dwWkc;              /**< [in] Received working counter */
    EC_T_DWORD  dwCmdIdx;           /**< [in] Command Index Field */
    EC_T_DWORD  dwAddr;             /**< [in] Address Field */
    EC_T_DWORD  dwLength;           /**< [in] Length of data portion (11 relevant bits) */
    EC_T_BYTE*  pbyData;            /**< [in] Pointer to data portion within a PDU. The callback function has to store the data into application memory, the data pointer will be invalid after returning from the callback */
} EC_PACKED(4) EC_T_RAWCMDRESPONSE_NTFY_DESC;

typedef struct _EC_T_TX_PDO_NTFY_DESC
{
    EC_T_DWORD  wPhysAddr;          /* station address */
    EC_T_DWORD  dwNumber;           /* PDO number */
    EC_T_DWORD  wLen;               /* PDO size */
    EC_T_BYTE*  pbyData;
} EC_PACKED(4) EC_T_TX_PDO_NTFY_DESC;

typedef struct _EC_T_SLAVE_LINKMSG_DESC
{
    EC_T_DWORD  dwSlaveId;          /* Slave Id */
    EC_T_BOOL   bEnableLogging;     /* EC_TRUE=> Enable LinkMsgs, EC_FALSE=> Disable */
} EC_PACKED(4) EC_T_SLAVE_LINKMSG_DESC;


/* EtherCAT state change */
typedef struct _EC_T_STATECHANGE
{
    EC_T_STATE    oldState;         /**< old operational state */
    EC_T_STATE    newState;         /**< new operational state */
} EC_PACKED(4) EC_T_STATECHANGE;

typedef struct _EC_T_HC_SLAVE_CHANGE_DESC
{
    EC_T_SLAVE_PROP     SlaveProp;                          /* Slave properties */
} EC_PACKED(4) EC_T_HC_SLAVE_CHANGE_DESC;

#include EC_PACKED_INCLUDESTOP/*(4)*/

#include EC_PACKED_INCLUDESTART(8)
typedef struct _EC_T_SLAVEREGISTER_TRANSFER_NTFY_DESC
{
    EC_T_DWORD  dwTferId;           /**< Transfer ID. For every new slave register transfer a unique ID has to be assigned. This ID can be used after completion to identify the transfer */
    EC_T_DWORD  dwResult;           /**< Result of Slave register transfer */
    EC_T_BOOL   bRead;              /**< EC_TRUE: Read register, EC_FALSE: Write register transfer */
    EC_T_WORD   wFixedAddr;         /**< Station address of slave */
    EC_T_WORD   wRegisterOffset;    /**< Register offset */
    EC_T_WORD   wLen;               /**< Length of slave register transfer */
    EC_T_BYTE*  pbyData;            /**< Pointer to the data read */
    EC_T_WORD   wWkc;               /**< Received working counter */
} EC_PACKED(8) EC_T_SLAVEREGISTER_TRANSFER_NTFY_DESC;
#include EC_PACKED_INCLUDESTOP/*(8)*/

typedef enum _EC_T_EEPROM_OPERATION_TYPE
{
    eEEPRomOp_Unknown = 0,          /**< Unknown EEPROM operation, only for internal use */
    eEEPRomOp_Assign  = 1,          /**< Assign slave EEPROM operation, used by emAssignSlaveEEPRomReq */
    eEEPRomOp_Active  = 2,          /**< Active slave EEPROM operation, used by emActiveSlaveEEPRomReq */
    eEEPRomOp_Read    = 3,          /**< Read slave EEPRom operation, used by emReadSlaveEEPRomReq */
    eEEPRomOp_Write   = 4,          /**< Write slave EEPRom operation, used by emWriteSlaveEEPRomReq */
    eEEPRomOp_Reload  = 5,          /**< Reload slave EEPRom operation, used by emReloadSlaveEEPRomReq */
    eEEPRomOp_Reset   = 6,          /**< Reset slave EEPRom operation, used by emResetSlaveController */

    /* Borland C++ datatype alignment correction */
    eEEPRomOp_BCppDummy = 0xFFFFFFFF
} EC_T_EEPROM_OPERATION_TYPE;

#include EC_PACKED_INCLUDESTART(8)
typedef struct _EC_T_EEPROM_OPERATION_NTFY_DESC
{
    EC_T_DWORD                  dwTferId;       /**< Transfer ID. For every new EEPROM operation a unique ID has to be assigned. This ID can be used after completion to identify the transfer */
    EC_T_EEPROM_OPERATION_TYPE  eType;          /**< Type of EEPROM operation */
    EC_T_DWORD                  dwResult;       /**< Result of EEPROM operation */
    EC_T_SLAVE_PROP             SlaveProp;      /**< Slave properties */

    union _EC_T_EEPROM_OPERATION_NTFY_DESC_RESULT
    {
        struct _EC_T_EEPROM_OPERATION_NTFY_DESC_RESULT_READ
        {
            EC_T_WORD           wEEPRomStartOffset;         /**< Start address of EEPRom operation. Given by API */
            EC_T_WORD*          pwData;                     /**< Pointer to WORD array contains the data. Given by API */
            EC_T_DWORD          dwReadLen;                  /**< Number of Words to be read. Given by API */
            EC_T_DWORD          dwNumOutData;               /**< Number of Words actually read from EEPRom */
        } EC_PACKED(8) Read;
        struct _EC_T_EEPROM_OPERATION_NTFY_DESC_RESULT_WRITE
        {
            EC_T_WORD           wEEPRomStartOffset;         /**< Start address of EEPRom operation. Given by API */
            EC_T_WORD*          pwData;                     /**< Pointer to WORD array contains the data. Given by API */
            EC_T_DWORD          dwWriteLen;                 /**< Number of Words to be written. Given by API */
        } EC_PACKED(8) Write;
        struct _EC_T_EEPROM_OPERATION_NTFY_DESC_RESULT_ACTIVE
        {
            EC_T_BOOL           bSlavePDIAccessActive;      /**<  EC_TRUE: EEPROM active by PDI application, EC_FALSE: EEPROM not active */
        } EC_PACKED(8) Active;
        struct
        {
            EC_T_DWORD          dwReserved;
        } EC_PACKED(8) Assign;
        struct
        {
            EC_T_DWORD          dwReserved;
        } EC_PACKED(8) Reload;
    } EC_PACKED(8) uResult;
} EC_PACKED(8) EC_T_EEPROM_OPERATION_NTFY_DESC;

typedef struct _EC_T_PORT_OPERATION_NTFY_DESC
{
    EC_T_DWORD          dwTferId;                           /**< Transfer ID. For every new port operation a unique ID has to be assigned. This ID can be used after completion to identify the transfer */
    EC_T_DWORD          dwResult;                           /**< Result of request */
    EC_T_SLAVE_PROP     SlaveProp;                          /**< Slave properties */
    EC_T_WORD           wPortStateOld;                      /**< Old state of the slave ports  */
    EC_T_WORD           wPortStateNew;                      /**< New state of the slave ports */
} EC_PACKED(8) EC_T_PORT_OPERATION_NTFY_DESC;

typedef struct _EC_T_SLAVE_IDENTIFICATION_NTFY_DESC
{
    EC_T_DWORD          dwTferId;                           /**< Transfer ID. For every new port operation a unique ID has to be assigned. This ID can be used after completion to identify the transfer */
    EC_T_DWORD          dwResult;                           /**< Result of request */
    EC_T_SLAVE_PROP     SlaveProp;                          /**< Slave properties */
    EC_T_WORD           wAdo;                               /**< Slave address offset used for identification. Given by API */
    EC_T_WORD           wValue;                             /**< Slave identification value. Given by API */
} EC_PACKED(8) EC_T_SLAVE_IDENTIFICATION_NTFY_DESC;
#include EC_PACKED_INCLUDESTOP/*(8)*/

#include EC_PACKED_INCLUDESTART(4)
typedef struct _EC_T_RELEASE_FORCED_PROCESSDATA_NTFY_DESC
{
    EC_T_BOOL           bOutput;                            /**< EC_TRUE: Output Bits, EC_FALSE: Input Bits */
    EC_T_DWORD          dwOffset;                           /**< Offeset of the forced Bits */
    EC_T_WORD           wBitLength;                         /**< Bit length */
} EC_PACKED(4) EC_T_RELEASE_FORCED_PROCESSDATA_NTFY_DESC;
#include EC_PACKED_INCLUDESTOP/*(8)*/

#include EC_PACKED_INCLUDESTART(2)
/* Slaves presence descriptor */
#ifndef MAX_SLAVES_PRESENCE_NTFY_ENTRIES
#define MAX_SLAVES_PRESENCE_NTFY_ENTRIES 128
#endif
typedef struct _EC_T_SLAVE_PRESENCE_NTFY_DESC
{
    EC_T_WORD  wStationAddress;                             /**< Slave station address */
    EC_T_BYTE  bPresent;                                    /**< EC_TRUE: present , EC_FALSE: absent */
} EC_PACKED(2) EC_T_SLAVE_PRESENCE_NTFY_DESC;

typedef struct _EC_T_SLAVES_PRESENCE_NTFY_DESC
{
    EC_T_WORD wCount;                                       /**< Number of slave presence noftifications */
    EC_T_SLAVE_PRESENCE_NTFY_DESC SlavePresence[MAX_SLAVES_PRESENCE_NTFY_ENTRIES];  /**< slave presence descriptions */
} EC_PACKED(2) EC_T_SLAVES_PRESENCE_NTFY_DESC;
#define SIZEOF_EC_T_SLAVES_PRESENCE_NTFY_DESC(wCount) (sizeof(EC_T_WORD)+wCount*sizeof(EC_T_SLAVE_PRESENCE_NTFY_DESC))
#include EC_PACKED_INCLUDESTOP/*(2)*/

#include EC_PACKED_INCLUDESTART(4)
/* EtherCAT notification descriptor */
typedef struct _EC_T_NOTIFICATION_DESC
{
    union _EC_T_NOTIFICATION_PARM
    {
        EC_T_DWORD                                     StatusCode;                 /* Generic status code only notification */
        EC_T_REFCLOCK_PRESENCE_NTFY_DESC               RefClockPresenceNtfyDesc;   /* DC Reference Clock Presence Notification descriptor */
        EC_T_DC_SYNC_NTFY_DESC                         SyncNtfyDesc;               /* DC Master / Slave Sync Notification descriptor */
        EC_T_RAWCMDRESPONSE_NTFY_DESC                  RawCmdRespNtfyDesc;         /* Queue Raw Cmd Response notification descriptor */
        EC_T_SB_STATUS_NTFY_DESC                       ScanBusNtfyDesc;            /* Scanbus Result notification descriptor */
        EC_T_SB_MISMATCH_DESC                          ScanBusMismatch;            /* Scan Bus Mismatch notification descriptor */
        EC_T_STATECHANGE                               StatusChngNtfyDesc;         /* Master EtherCAT State changed notification */
#if (defined INCLUDE_COE_PDO_SUPPORT)
        EC_T_TX_PDO_NTFY_DESC                          TxPdoNtfyDesc;              /* TxPDO transfer notification */
#endif
#if (defined INCLUDE_HOTCONNECT)
        EC_T_HC_DETECTALLGROUP_NTFY_DESC               HCDetAllGrps;               /* HC Group Detection */
        EC_T_HC_SLAVE_CHANGE_DESC                      HCSlvChgDesc;               /* HotConnect Slave State Change. Obsolete, see SlavePresenceDesc. */
#endif
        EC_T_SLAVE_PRESENCE_NTFY_DESC                  SlavePresenceDesc;          /* Slave (dis-)appeared */
        EC_T_SLAVES_PRESENCE_NTFY_DESC                 SlavesPresenceDesc;         /* Slaves (dis-)appeared */
        EC_T_LINE_CROSSED_DESC                         CrossedLineDesc;            /* Line crossed */
        EC_T_DCM_SYNC_NTFY_DESC                        DcmInSyncDesc;
        EC_T_DCX_SYNC_NTFY_DESC                        DcxInSyncDesc;
        EC_T_SLAVE_STATECHANGED_NTFY_DESC              SlaveStateChangedDesc;      /* Slave finished successfully state transition descriptor */
        EC_T_SLAVES_STATECHANGED_NTFY_DESC             SlavesStateChangedDesc;     /* Slaves finished successfully state transition descriptor */
        EC_T_SLAVEREGISTER_TRANSFER_NTFY_DESC          SlaveRegisterTransferDesc;  /* Slave register read/write notification descriptor */
        EC_T_EEPROM_OPERATION_NTFY_DESC                EEPRomOperationDesc;        /* EEProm operation notification descriptor */
        EC_T_PORT_OPERATION_NTFY_DESC                  PortOperationDesc;          /* Port operation notification descriptor */
        EC_T_SLAVE_IDENTIFICATION_NTFY_DESC            SlaveIdentificationDesc;    /* Slave read identification notification descriptor */
        EC_T_RELEASE_FORCED_PROCESSDATA_NTFY_DESC      ReleaseForcedProcessData;   /* Release forced process data */
    } EC_PACKED(4) desc;
} EC_PACKED(4) EC_T_NOTIFICATION_DESC;

/* S2S Mailbox Error */
typedef struct _EC_T_S2SMBX_ERROR_DESC
{
    EC_T_SLAVE_PROP SlaveProp;                      /* slave properties of requesting slave */
    EC_T_WORD       wTargetFixedAddress;            /* fixed address of the target slave */
    EC_T_DWORD      dwErrorCode;                    /**< error code EC_E_ */
} EC_PACKED(4) EC_T_S2SMBX_ERROR_DESC;

/* EtherCAT error notification descriptor */
typedef struct _EC_T_ERROR_NOTIFICATION_DESC
{
    EC_T_DWORD  dwNotifyErrorCode;                  /**< Error ID (same value as the notification code) */
    EC_T_CHAR   achErrorInfo[MAX_ERRINFO_STRLEN];   /**< Additional error string (may be empty) */
    union _EC_T_ERROR_NOTIFICATION_PARM
    {
        EC_T_WKCERR_DESC                        WkcErrDesc;                 /**< WKC error descriptor */
        EC_T_FRAME_RSPERR_DESC                  FrameRspErrDesc;            /**< Frame response error descriptor */
        EC_T_INITCMD_ERR_DESC                   InitCmdErrDesc;             /**< Master/Slave init command error descriptor */
        EC_T_SLAVE_ERROR_INFO_DESC              SlaveErrInfoDesc;           /**< Slave Error Info Descriptor */
        EC_T_SLAVES_ERROR_DESC                  SlavesErrDesc;              /**< Slaves Error Descriptor */
        EC_T_MBOX_SDO_ABORT_DESC                SdoAbortDesc;               /**< SDO Abort */
        EC_T_RED_CHANGE_DESC                    RedChangeDesc;              /**< Redundancy Descriptor */
        EC_T_MBOX_FOE_ABORT_DESC                FoeErrorDesc;               /**< FoE error code and string */
        EC_T_MBXRCV_INVALID_DATA_DESC           MbxRcvInvalidDataDesc;      /**< Invalid mailbox data received descriptor */
        EC_T_PDIWATCHDOG_DESC                   PdiWatchdogDesc;            /**< PDI Watchodg expired */
        EC_T_SLAVE_NOTSUPPORTED_DESC            SlaveNotSupportedDesc;      /**< Slave not supported */
        EC_T_SLAVE_UNEXPECTED_STATE_DESC        SlaveUnexpectedStateDesc;   /**< Slave in unexpected state */
        EC_T_SLAVES_UNEXPECTED_STATE_DESC       SlavesUnexpectedStateDesc;  /**< Slaves in unexpected state */
        EC_T_EEPROM_CHECKSUM_ERROR_DESC         EEPROMChecksumErrorDesc;    /**< EEPROM checksum error */
        EC_T_JUNCTION_RED_CHANGE_DESC           JunctionRedChangeDesc;      /**< Junction redundancy change descriptor */
        EC_T_FRAMELOSS_AFTER_SLAVE_NTFY_DESC    FramelossAfterSlaveDesc;    /**< Frameloss after Slave descriptor */
        EC_T_S2SMBX_ERROR_DESC                  S2SMbxErrorDesc;            /**< S2S Mailbox Error descriptor */
        EC_T_BAD_CONNECTION_NTFY_DESC           BadConnectionDesc;          /**< Bad connection descriptor */
        EC_T_COMMUNICATION_TIMEOUT_NTFY_DESC    CommunicationTimeoutDesc;   /**< Communication timeout descriptor */
        EC_T_TAP_LINK_STATUS_NTFY_DESC          TapLinkStatusDesc;          /**< Tap link status */
    } EC_PACKED(4) desc;
} EC_PACKED(4) EC_T_ERROR_NOTIFICATION_DESC;

#define SIZEOF_EC_T_ERROR_NOTIFICATION_HEADER (sizeof(EC_T_DWORD)/*dwNotifyErrorCode*/+8/*achErrorInfo*/)

#define SIZEOF_EC_T_ERROR_NOTIFICATION_WKCERR                       (SIZEOF_EC_T_ERROR_NOTIFICATION_HEADER+sizeof(EC_T_WKCERR_DESC))
#define SIZEOF_EC_T_ERROR_NOTIFICATION_FRAME_RSPERR                 (SIZEOF_EC_T_ERROR_NOTIFICATION_HEADER+sizeof(EC_T_FRAME_RSPERR_DESC))
#define SIZEOF_EC_T_ERROR_NOTIFICATION_SLAVE_INITCMD_RESPONSE_ERROR (SIZEOF_EC_T_ERROR_NOTIFICATION_HEADER+sizeof(EC_T_INITCMD_ERR_DESC))
#define SIZEOF_EC_T_ERROR_NOTIFICATION_SLAVE_ERROR_INFO             (SIZEOF_EC_T_ERROR_NOTIFICATION_HEADER+sizeof(EC_T_SLAVE_ERROR_INFO_DESC))
#define SIZEOF_EC_T_ERROR_NOTIFICATION_MBOX_SDO_ABORT               (SIZEOF_EC_T_ERROR_NOTIFICATION_HEADER+sizeof(EC_T_MBOX_SDO_ABORT_DESC))
#define SIZEOF_EC_T_ERROR_NOTIFICATION_RED_CHANGE                   (SIZEOF_EC_T_ERROR_NOTIFICATION_HEADER+sizeof(EC_T_RED_CHANGE_DESC))
#define SIZEOF_EC_T_ERROR_NOTIFICATION_MBOX_FOE_ABORT               (SIZEOF_EC_T_ERROR_NOTIFICATION_HEADER+sizeof(EC_T_MBOX_FOE_ABORT_DESC))
#define SIZEOF_EC_T_ERROR_NOTIFICATION_MBXRCV_INVALID_DATA          (SIZEOF_EC_T_ERROR_NOTIFICATION_HEADER+sizeof(EC_T_MBXRCV_INVALID_DATA_DESC))
#define SIZEOF_EC_T_ERROR_NOTIFICATION_PDIWATCHDOG                  (SIZEOF_EC_T_ERROR_NOTIFICATION_HEADER+sizeof(EC_T_PDIWATCHDOG_DESC))
#define SIZEOF_EC_T_ERROR_NOTIFICATION_SLAVE_NOTSUPPORTED           (SIZEOF_EC_T_ERROR_NOTIFICATION_HEADER+sizeof(EC_T_SLAVE_NOTSUPPORTED_DESC))
#define SIZEOF_EC_T_ERROR_NOTIFICATION_SLAVE_UNEXPECTED_STATE       (SIZEOF_EC_T_ERROR_NOTIFICATION_HEADER+sizeof(EC_T_SLAVE_UNEXPECTED_STATE_DESC))
#define SIZEOF_EC_T_ERROR_NOTIFICATION_EEPROM_CHECKSUM_ERROR        (SIZEOF_EC_T_ERROR_NOTIFICATION_HEADER+sizeof(EC_T_EEPROM_CHECKSUM_ERROR_DESC))
#define SIZEOF_EC_T_ERROR_NOTIFICATION_JUNCTION_RED_CHANGE          (SIZEOF_EC_T_ERROR_NOTIFICATION_HEADER+sizeof(EC_T_JUNCTION_RED_CHANGE_DESC))
#define SIZEOF_EC_T_ERROR_NOTIFICATION_FRAMELOSS_AFTER_SLAVE        (SIZEOF_EC_T_ERROR_NOTIFICATION_HEADER+sizeof(EC_T_FRAMELOSS_AFTER_SLAVE_NTFY_DESC))
#define SIZEOF_EC_T_ERROR_NOTIFICATION_S2SMBX_ERROR                 (SIZEOF_EC_T_ERROR_NOTIFICATION_HEADER+sizeof(EC_T_S2SMBX_ERROR_DESC))
#define SIZEOF_EC_T_ERROR_NOTIFICATION_BAD_CONNECTION               (SIZEOF_EC_T_ERROR_NOTIFICATION_HEADER+sizeof(EC_T_BAD_CONNECTION_NTFY_DESC))
#define SIZEOF_EC_T_ERROR_NOTIFICATION_COMMUNICATION_TIMEOUT        (SIZEOF_EC_T_ERROR_NOTIFICATION_HEADER+sizeof(EC_T_COMMUNICATION_TIMEOUT_NTFY_DESC))
#define SIZEOF_EC_T_ERROR_NOTIFICATION_TAP_LINK_STATUS              (SIZEOF_EC_T_ERROR_NOTIFICATION_HEADER+sizeof(EC_T_TAP_LINK_STATUS_NTFY_DESC))

#define SIZEOF_EC_T_ERROR_NOTIFICATION_SLAVES_UNEXPECTED_STATE_DESC(wCount) \
    (SIZEOF_EC_T_ERROR_NOTIFICATION_HEADER+sizeof(EC_T_WORD)/*wCount*/+sizeof(EC_T_WORD)/*wRes*/+wCount*sizeof(EC_T_SLAVES_UNEXPECTED_STATE_DESC_ENTRY))

#define SIZEOF_EC_T_ERROR_NOTIFICATION_SLAVES_ERROR_DESC(wCount) \
    (SIZEOF_EC_T_ERROR_NOTIFICATION_HEADER+sizeof(EC_T_WORD)/*wCount*/+sizeof(EC_T_WORD)/*wRes*/+wCount*sizeof(EC_T_SLAVES_ERROR_DESC_ENTRY))

typedef enum _EC_T_CNF_TYPE
{
    eCnfType_Unknown            = 0,
    eCnfType_Filename           = 1,                /**< pbyCnfData: ENI filename to read */
    eCnfType_Data               = 2,                /**< pbyCnfData: ENI data */
    eCnfType_Datadiag           = 3,                /**< pbyCnfData: ENI data for diagnosis */
    eCnfType_GenPreopENI        = 4,                /**< Generate ENI based on bus-scan result to get into PREOP state */
    eCnfType_GenPreopENIWithCRC = 5,                /**< same as eCnfType_GenPreopENI with CRC protection */
    eCnfType_GenOpENI           = 6,                /**< Generate ENI based on bus-scan result to get into OP state */
    eCnfType_None               = 7,                /**< Reset configuration */
    eCnfType_ConfigData         = 8,                /**< pbyCnfData: Binary structured configuration */

    /* Borland C++ datatype alignment correction */
    eCnfType_BCppDummy   = 0xFFFFFFFF
} EC_T_CNF_TYPE;

#define TASKID_COMPLETE_PD      0xFFFFFFFF
/**
 * \typedef EC_T_PFMEMREQ
 * \param pvContext   [in] Context pointer. This pointer is used as parameter when the callback function is called
 * \param dwTaskId    [in] Task id of cyclic data transfer. If TASKID_COMPLETE_PD is given, the function must return a complete output process data buffer which contains valid data for all cyclic tasks.
 * \param ppbyPDData  [out] Pointer to the process data buffer to be used. If set to EC_NULL, the corresponding fixed buffer from EC_T_MEMPROV_DESC is used. The provided buffer size must correspond to the caller context.
 */
typedef EC_T_VOID (*EC_T_PFMEMREQ)(EC_T_PVOID pvContext, EC_T_DWORD dwTaskId, EC_T_PBYTE* ppbyPDData);
/**
 * \typedef EC_T_PFMEMREL
 * \param pvContext   [in] Context pointer. This pointer is used as parameter when the callback function is called
 * \param dwTaskId    [in] Task id of cyclic data transfer.
 */
typedef EC_T_VOID (*EC_T_PFMEMREL)(EC_T_PVOID pvContext, EC_T_DWORD dwTaskId);

/* Descriptor for EC_IOCTL_REGISTER_PDMEMORYPROVIDER */
typedef struct _EC_T_MEMPROV_DESC
{
    EC_T_PVOID      pvContext;                      /**< Context pointer. This pointer is used every time when one of the callback functions (e.g. pfPDOutReadRequest) is called */
    EC_T_PBYTE      pbyPDOutData;                   /**< Pointer to the fixed output process data buffer (values transferred from the master to the slaves).
                                                         A value of EC_NULL may be given in case the pointer will be provided later when function EC_T_MEMPROV_DESC.pfPDOutDataReadRequest is called */
    EC_T_DWORD      dwPDOutDataLength;              /**< Length of the output process data buffer */
    EC_T_PBYTE      pbyPDInData;                    /**< Pointer to the fixed input process data buffer (values transferred from the slaves to the master).
                                                         A value of EC_NULL may be given in case the pointer will be provided later when function EC_T_MEMPROV_DESC.pfPDInDataWriteRequest is called */
    EC_T_DWORD      dwPDInDataLength;               /**< Length of the output process data buffer */
    EC_T_PFMEMREQ   pfPDOutDataReadRequest;         /**< This function will be called cyclically within the process data transfer cycle prior to read data from the output process data buffer.
                                                         If EC_NULL is set, the fixed buffer EC_T_MEMPROV_DESC.pbyPDOutData is used. */
    EC_T_PFMEMREL   pfPDOutDataReadRelease;         /**< This function will be called cyclically within the process data transfer cycle after all data were read from the output process data buffer. */

    EC_T_PFMEMREQ   pfPDOutDataWriteRequest;        /**< This function will be called cyclically within the process data transfer cycle prior to write new data into the output process data buffer.
                                                         If EC_NULL is set, the fixed buffer EC_T_MEMPROV_DESC.pbyPDOutData is used. */
    EC_T_PFMEMREL   pfPDOutDataWriteRelease;        /**< This function will be called cyclically within the process data transfer cycle after all data were written into the output process data buffer. */

    EC_T_PFMEMREQ   pfPDInDataWriteRequest;         /**< This function will be called cyclically within the process data transfer cycle prior to write new data into the input process data buffer.
                                                         If EC_NULL is set, the fixed buffer EC_T_MEMPROV_DESC.pbyPDInData is used. */
    EC_T_PFMEMREL   pfPDInDataWriteRelease;         /**< This function will be called cyclically within the process data transfer cycle after all data were written into the input process data buffer. */

    EC_T_PBYTE      pbyMasterRedPDOutData;          /**< Pointer to the MasterRed output process data buffer (ACTIVE to INACTIVE) */
    EC_T_DWORD      dwMasterRedPDOutDataLength;     /**< Length of the MasterRed output process data buffer */
    EC_T_PBYTE      pbyMasterRedPDInData;           /**< Pointer to the default input process data buffer (INACTIVE to ACTIVE) */
    EC_T_DWORD      dwMasterRedPDInDataLength;      /**< Length of the input  process data buffer */
    EC_T_PFMEMREQ   pfMasterRedPDOutReadRequest;    /**< This function will be called within the process data transfer cycle prior to read data. */
    EC_T_PFMEMREL   pfMasterRedPDOutReadRelease;    /**< This function will be called after all data was read  from output process data buffer. */
    EC_T_PFMEMREQ   pfMasterRedPDOutWriteRequest;   /**< This function will be called within the process data transfer cycle prior to read data. */
    EC_T_PFMEMREL   pfMasterRedPDOutWriteRelease;   /**< This function will be called after all data was read from output process data buffer. */
    EC_T_PFMEMREQ   pfMasterRedPDInWriteRequest;    /**< This function will be called within the process data transfer cycle prior to write data. */
    EC_T_PFMEMREL   pfMasterRedPDInWriteRelease;    /**< This function will be called after all data was written to input process data buffer. */
    EC_T_PFMEMREQ   pfMasterRedPDInReadRequest;     /**< This function will be called within the process data transfer cycle prior to write data. */
    EC_T_PFMEMREL   pfMasterRedPDInReadRelease;     /**< This function will be called after all data was written to input process data buffer. */
} EC_PACKED(4) EC_T_MEMPROV_DESC, *EC_PT_MEMPROV_DESC;

typedef struct _EC_T_SB_SLAVEINFO_DESC
{
    EC_T_DWORD  dwScanBusStatus;                    /**< [out] Scan bus status (determined in the latest scan bus) emNotify EC_NOTIFY_SB_STATUS */
    EC_T_DWORD  dwVendorId;                         /**< [out] Vendor Identification stored in the EEPROM at offset 0x0008 */
    EC_T_DWORD  dwProductCode;                      /**< [out] Product Code stored in the EEPROM at offset 0x000A */
    EC_T_DWORD  dwRevisionNumber;                   /**< [out] Revision number stored in the EEPROM at offset 0x000C (Not read by default!) */
    EC_T_DWORD  dwSerialNumber;                     /**< [out] Serial number stored in the EEPROM at offset 0x000E (Not read by default!) */
} EC_PACKED(4) EC_T_SB_SLAVEINFO_DESC, *EC_PT_SB_SLAVEINFO_DESC;

typedef enum _EC_T_eEEPENTRY
{
    eEEP_VendorId           = ESC_SII_REG_VENDORID,             /**< 0x0008, Checked by scan bus */
    eEEP_ProductCode        = ESC_SII_REG_PRODUCTCODE,          /**< 0x000A, Checked by scan bus */
    eEEP_RevisionNumber     = ESC_SII_REG_REVISIONNUMBER,       /**< 0x000C, Checked by init command */
    eEEP_SerialNumber       = ESC_SII_REG_SERIALNUMBER,         /**< 0x000E, Checked by init command */
    eEEP_BootRcvMbx         = ESC_SII_REG_BOOT_RECV_MBX_OFFSET,
    eEEP_BootSndMbx         = ESC_SII_REG_BOOT_SEND_MBX_OFFSET,
    eEEP_StdRcvMbx          = ESC_SII_REG_STD_RECV_MBX_OFFSET,
    eEEP_StdSndMbx          = ESC_SII_REG_STD_SEND_MBX_OFFSET,
    eEEP_MbxProtocol        = ESC_SII_REG_MBX_PROTOCOL,
    eEEP_AliasAddress       = ESC_SII_REG_ALIASADDRESS,

    /* Borland C++ datatype alignment correction */
    eEEP_BCppDummy          = 0xFFFFFFFF
} EC_T_eEEPENTRY;

#define BT_CHECK_EEPENTRY_VENDORID      EC_TRUE
#define BT_CHECK_EEPENTRY_PRODUCTCODE   EC_TRUE
#define BT_CHECK_EEPENTRY_REVISIONNO    EC_TRUE
#define BT_CHECK_EEPENTRY_SERIALNO      EC_TRUE
#define BT_CHECK_EEPENTRY_MBXPROTOCOL   EC_TRUE

typedef struct _EC_T_SCANBUS_PROP_DESC
{
    EC_T_eEEPENTRY  eEEPROMEntry;           /**< [in] EEPROM entry (slave property) to add */
    EC_T_DWORD      dwVerify;               /**< [in] if set to EC_TRUE the actual slave property (stored in the EEPROM) will be compared with the appropriate value in the XML configuration file */
} EC_PACKED(4) EC_T_SCANBUS_PROP_DESC, *EC_PT_SCANBUS_PROP_DESC;

typedef enum _EC_T_eSBSlaveInfoType
{
    sbsit_unknown       = 0,
    sbsit_bustopology   = 1,                /**< info from bus */
    sbsit_configuration = 2,                /**< info from XML configuration */

    /* Borland C++ datatype alignment correction */
    sbsit_BCppDummy     = 0xFFFFFFFF
} EC_T_eSBSlaveInfoType;

typedef struct _EC_T_SB_SLAVEINFO_EEP_REQ_DESC
{
    EC_T_eSBSlaveInfoType       eSbSlaveInfoType;       /**< [in] Selection whether to use Auto-Increment address of Bus or XML Configuration */
    EC_T_DWORD                  wAutoIncAddress;        /**< [in] Auto-Increment address of the slave */
    EC_T_eEEPENTRY              eEEPROMEntry;           /**< [in] EEPROM entry to read (only valid if entry was selected by emIoControl EC_IOCTL_SB_SET_BUSCNF_VERIFY_PROP) */
} EC_T_SB_SLAVEINFO_EEP_REQ_DESC;

typedef struct _EC_T_SB_SLAVEINFO_EEP_RES_DESC
{
    EC_T_DWORD                  dwScanBusStatus;        /**< [out] Scan bus status (determined in the latest scan bus) emNotify EC_NOTIFY_SB_STATUS */
    EC_T_eEEPENTRY              eEEPROMEntry;           /**< [out] Select EEPROM Entry description from Request */
    EC_T_DWORD                  dwEEPROMValue;          /**< [out] EEPROM entry value */
} EC_PACKED(4) EC_T_SB_SLAVEINFO_EEP_RES_DESC;

typedef enum _EC_T_eINFOENTRY
{
    eie_unknown         =  0,       /** nothing / invalid */

    eie_pdoffs_in       =  1,       /** config: get process data offset of Input data (in Bits) */
    eie_pdsize_in       =  2,       /** config: get process data size of Input Data (in Bits) */
    eie_pdoffs_out      =  3,       /** config: get process data offset of Output data (in Bits) */
    eie_pdsize_out      =  4,       /** config: get process data size of Output Data (in Bits) */

    eie_phys_address    =  5,       /** bus: get slave phys Address */
    eie_portstate       =  6,       /** bus: get port link state (DL_STATUS, needed e.g. for topology detection */
    eie_dcsupport       =  7,       /** bus: does slave support DC */
    eie_dc64support     =  8,       /** bus: does slave support 64 Bit DC */
    eie_alias_address   =  9,       /** bus: get slave alias address */

    eie_cfgphy_address  = 10,       /** config: get slave phys Address from config file */
    eie_device_name     = 11,       /** config: get slave name from configuration */
    eie_ismailbox_slave = 12,       /** config: get whether slave support mailboxes */

    eie_pdoffs_in2      = 21,       /** config: get process data offset of Input data (section 2) (in Bits) */
    eie_pdsize_in2      = 22,       /** config: get process data size of Input Data (section 2) (in Bits) */
    eie_pdoffs_out2     = 23,       /** config: get process data offset of Output data (section 2) (in Bits) */
    eie_pdsize_out2     = 24,       /** config: get process data size of Output Data (section 2) (in Bits) */
    eie_pdoffs_in3      = 31,       /** config: get process data offset of Input data (section 3) (in Bits) */
    eie_pdsize_in3      = 32,       /** config: get process data size of Input Data (section 3) (in Bits) */
    eie_pdoffs_out3     = 33,       /** config: get process data offset of Output data (section 3) (in Bits) */
    eie_pdsize_out3     = 34,       /** config: get process data size of Output Data (section 3) (in Bits) */
    eie_pdoffs_in4      = 41,       /** config: get process data offset of Input data (section 4) (in Bits) */
    eie_pdsize_in4      = 42,       /** config: get process data size of Input Data (section 4) (in Bits) */
    eie_pdoffs_out4     = 43,       /** config: get process data offset of Output data (section 4) (in Bits) */
    eie_pdsize_out4     = 44,       /** config: get process data size of Output Data (section 4) (in Bits) */

    eie_mbx_outsize     = 45,       /** get out mailbox 1 size */
    eie_mbx_insize      = 46,       /** get in mailbox 1 size */
    eie_mbx_outsize2    = 47,       /** get out mailbox 2 size */
    eie_mbx_insize2     = 48,       /** get in mailbox 2 size */

    eie_isoptional      = 49,       /** is slave optional */
    eie_ispresent       = 50,       /** is slave present on bus */

    eie_esctype         = 51,       /** Type of ESC controller */
    /* Borland C++ datatype alignment correction */
    eie_BCppDummy       = 0xFFFFFFFF
} EC_T_eINFOENTRY;

typedef struct _EC_T_SB_SLAVEINFO_REQ_DESC
{
    EC_T_eINFOENTRY eInfoEntry;         /**< [in] Info Entry to read */
    EC_T_WORD       wAutoIncAddress;    /**< [in] Auto-Increment address of the slave */
} EC_PACKED(4) EC_T_SB_SLAVEINFO_REQ_DESC;

typedef struct _EC_T_SB_SLAVEINFO_RES_DESC
{
    EC_T_eINFOENTRY eInfoEntry;         /**< [out] Info entry read */
    EC_T_DWORD      dwInfoLength;       /**< [in, out] Length of Info Field (buffer, actually read length) */
    EC_T_PBYTE      pbyInfo;            /**< [out] Pointer to Info (-1 if no info found in XML file) */
} EC_PACKED(4) EC_T_SB_SLAVEINFO_RES_DESC;

typedef struct _EC_T_DCL_ENABLE_DESC
{
    EC_T_DWORD  dwVerifySyncLatchConfiguration;
    EC_T_DWORD  dwDCLInitTimeout;
} EC_PACKED(4) EC_T_DCL_ENABLE_DESC;


/* descriptor for EC_IOCTL_GET_CYCLIC_CONFIG_INFO call */
typedef struct _EC_T_CYC_CONFIG_DESC
{
    EC_T_DWORD      dwNumCycEntries;    /**< [out] Total number of cyclic entries */
    EC_T_DWORD      dwTaskId;           /**< [out] Task id of selected cyclic entry */
    EC_T_DWORD      dwPriority;         /**< [out] Priority of selected cyclic entry */
    EC_T_DWORD      dwCycleTime;        /**< [out] Cycle time of selected cyclic entry */
} EC_PACKED(4) EC_T_CYC_CONFIG_DESC;

typedef EC_T_DWORD (*EC_PF_TIMESTAMP)(              EC_T_PVOID      pCallerData,
                                                    EC_T_DWORD*     pdwHostTimeLo   );

/* Structure carrying instantaneous values for Master Sync */
typedef struct _EC_T_ACTUALVAL
{
    EC_T_DWORD  dwBeginUpdateCnt;               /**< out call counter incremented each time structure
                                                 *      update is started */
    EC_T_DWORD  dwEndUpdateCnt;                 /**< out call counter incremented each time structure
                                                 *      update is finished */
    EC_T_DWORD  dwBusSyncFrameSendTimeLo;       /**< out Host stamped time (result from pfTimeStamp) LSDW */
    EC_T_DWORD  dwBusTimeHi;                    /**< out Bus time (result from ARMW) MSDW */
    EC_T_DWORD  dwBusTimeLo;                    /**< out Bus time (result from ARMW) LSDW */
    EC_T_DWORD  dwBusSyncFramePostSendTimeLo;   /**< out Host stamped time (result from pfTimeStamp) LSDW */
    EC_T_DWORD  dwSyncPulseGridOffsetHi;        /**< out Sync Pulse Offset (initial grid offset) */
    EC_T_DWORD  dwSyncPulseGridOffsetLo;        /**< out Sync Pulse Offset (initial grid offset) */
    EC_T_BOOL   bDcmCtlInSync;                  /**< in  DCM controller has synchronized with DC ref.clock */
    EC_T_DWORD  dwSyncPeriodLength;             /**< out Bus cycle time [ns] */
    EC_T_DWORD  dwRes;                          /**< out reserved */
    EC_T_DWORD  dwLastTimeStampResult;          /**< out Last result of call to registered callback
                                                 *      function EC_PF_TIMESTAMP */
    EC_T_BOOL   bSlavesInSync;                  /**< out Slaves are in sync = EC_TRUE; out of sync = EC_FALSE; */
    EC_T_BOOL   bResetRequest;                  /**< out If EC_TRUE, master request DCM controller reset (e.g. if reference clock disappears) */
} EC_PACKED(4) EC_T_ACTUALVAL;

/* Controller client register parameters */
typedef struct _EC_T_REGISTER_TSPARMS
{
    EC_T_VOID*  pCallerData;                    /**< in  used by all callback functions */
    EC_PF_TIMESTAMP pfTimeStamp;                /**< in  timestamp callback function pointer */
    EC_T_DWORD  dwUpdateMultiplier;             /**< in  Interval multiplier */
    EC_T_DWORD  dwEnableHardRealtime;           /**< in  enable hard real time in link layer */
    EC_T_DWORD  aReserved[10];                  /**< in  reserved for future use */
} EC_PACKED(4) EC_T_REGISTER_TSPARMS;

/* Controller client register result */
typedef struct _EC_T_REGISTER_TSRESULTS
{
    EC_T_DWORD          dwHardRealtimeEnabled;  /**< out Hard real time is used */
    EC_T_ACTUALVAL*     pActualValues;          /**< out pointer to data carrying current data */
} EC_PACKED(4) EC_T_REGISTER_TSRESULTS;

typedef struct _EC_T_SLVSTATISTICS_DESC
{
    EC_T_BYTE       abyInvalidFrameCnt[ESC_PORT_COUNT]; /**< [out] Invalid Frame Counters per Slave Port */
    EC_T_BYTE       abyRxErrorCnt[ESC_PORT_COUNT];      /**< [out] RX Error Counters per Slave Port */
    EC_T_BYTE       abyFwdRxErrorCnt[ESC_PORT_COUNT];   /**< [out] Forwarded RX Error Counters per Slave Port */
    EC_T_BYTE       byProcessingUnitErrorCnt;           /**< [out] Processing Unit Error Counter */
    EC_T_BYTE       byPdiErrorCnt;                      /**< [out] PDI Error Counter */
    EC_T_WORD       wAlStatusCode;                      /**< [out] AL Status Code */
    EC_T_BYTE       abyLostLinkCnt[ESC_PORT_COUNT];     /**< [out] Lost Link Counters per Slave Port */

    EC_T_UINT64     qwReadTime;                         /**< [out] Timestamp of the last read [ns] */
    EC_T_UINT64     qwChangeTime;                       /**< [out] Timestamp of the last counter change [ns] */
} EC_PACKED(4) EC_T_SLVSTATISTICS_DESC;

#define IGNORE_PREV_PORT_FLAG       ((EC_T_WORD)(0x8000))
typedef struct _EC_T_SLAVE_PORT_DESC
{
    EC_T_WORD       wSlaveAddress;
    EC_T_WORD       wPortNumber;
} EC_PACKED(4) EC_T_SLAVE_PORT_DESC;

typedef enum _EC_T_EHOTCONNECTMODE
{
    echm_unknown        = 0x0000,
    echm_manual_preop   = 0x0001,
    echm_automatic      = 0x0002,
    echm_fullmanual     = 0x0004,
    echm_manual_noreset = 0x0008,
    echm_borderclose    = 0x0010,

    echm_BCppDummy =   0xFFFFFFFF
} EC_T_EHOTCONNECTMODE;
#define echm_automan_mask (0x000f)
#define echm_manual echm_manual_preop

#define HotConnectModeText(eMode)                   \
    ((eMode)==echm_unknown?"unknown":               \
     ((eMode)==echm_manual?"manual":                \
      ((eMode)==echm_automatic?"automatic":         \
       ((eMode)==echm_fullmanual?"fullmanual":      \
        ((eMode)==echm_borderclose?"borderclose":   \
          "INVALID EC_T_EHOTCONNECTMODE")))))

typedef enum _EC_T_JUNCTION_REDUNDANCY_MODE
{
    eJunctionRedundancyMode_Disabled  = 0,
    eJunctionRedundancyMode_Automatic = 1,
    eJunctionRedundancyMode_Strict    = 2,

    eJunctionRedundancyMode_BCppDummy =   0xFFFFFFFF
} EC_T_JUNCTION_REDUNDANCY_MODE;

#define JunctionRedundancyModeText(eMode)                     \
    ((eMode)==eJunctionRedundancyMode_Disabled?"Disabled":    \
     ((eMode)==eJunctionRedundancyMode_Automatic?"Automatic": \
      ((eMode)==eJunctionRedundancyMode_Strict?"Strict":"Unknown")))

/* callback pointer for RX frame */
typedef EC_T_VOID (*EC_T_PF_RXFRAME_CB)(EC_T_VOID*);

/* ecatGetCfgSlaveInfo */
typedef struct _EC_T_CFG_SLAVE_INFO
{
    EC_T_DWORD                  dwSlaveId;                              /**< [out] The slave's ID to bind bus slave and config slave information */
    EC_T_CHAR                   abyDeviceName[ECAT_DEVICE_NAMESIZE];    /**< [out] The slave's configured name (80 Byte) (from ENI file) */
    EC_T_DWORD                  dwHCGroupIdx;                           /**< [out] Index of the hot connect group, 0 for mandatory */
    EC_T_BOOL                   bIsPresent;                             /**< [out] Slave is currently present on bus */
    EC_T_BOOL                   bIsHCGroupPresent;                      /**< [out] Slave's hot connect group is currently present on bus */

    EC_T_DWORD                  dwVendorId;                             /**< [out] Vendor identification (from ENI file) */
    EC_T_DWORD                  dwProductCode;                          /**< [out] Product code (from ENI file) */
    EC_T_DWORD                  dwRevisionNumber;                       /**< [out] Revision number (from ENI file) */
    EC_T_DWORD                  dwSerialNumber;                         /**< [out] Serial number (from ENI file) */

    EC_T_WORD                   wStationAddress;                        /**< [out] The slave's station address (from ENI file) */
    EC_T_WORD                   wAutoIncAddress;                        /**< [out] The slave's auto increment address (from ENI file) */

    EC_T_DWORD                  dwPdOffsIn;             /**< [out] Process input data bit offset (from ENI file) */
    EC_T_DWORD                  dwPdSizeIn;             /**< [out] Process input data bit size (from ENI file) */
    EC_T_DWORD                  dwPdOffsOut;            /**< [out] Process output data bit offset (from ENI file) */
    EC_T_DWORD                  dwPdSizeOut;            /**< [out] Process output data bit size (from ENI file) */

    EC_T_DWORD                  dwPdOffsIn2;            /**< [out] 2nd sync unit process input data bit offset (from ENI file) */
    EC_T_DWORD                  dwPdSizeIn2;            /**< [out] 2nd sync unit process input data bit size (from ENI file) */
    EC_T_DWORD                  dwPdOffsOut2;           /**< [out] 2nd sync unit process output data bit offset (from ENI file) */
    EC_T_DWORD                  dwPdSizeOut2;           /**< [out] 2nd sync unit process output data bit size (from ENI file) */

    EC_T_DWORD                  dwPdOffsIn3;            /**< [out] 3rd sync unit process input data bit offset (from ENI file) */
    EC_T_DWORD                  dwPdSizeIn3;            /**< [out] 3rd sync unit process input data bit size (from ENI file) */
    EC_T_DWORD                  dwPdOffsOut3;           /**< [out] 3rd sync unit process output data bit offset (from ENI file) */
    EC_T_DWORD                  dwPdSizeOut3;           /**< [out] 3rd sync unit process output data bit size (from ENI file) */

    EC_T_DWORD                  dwPdOffsIn4;            /**< [out] 4th sync unit process input data bit offset (from ENI file) */
    EC_T_DWORD                  dwPdSizeIn4;            /**< [out] 4th sync unit process input data bit size (from ENI file) */
    EC_T_DWORD                  dwPdOffsOut4;           /**< [out] 4th sync unit process output data bit offset (from ENI file) */
    EC_T_DWORD                  dwPdSizeOut4;           /**< [out] 4th sync unit process output data bit size (from ENI file) */

    EC_T_DWORD                  dwMbxSupportedProtocols;/**< [out] Mailbox protocols supported by the slave (from ENI file). Combination of EC_MBX_PROTOCOL_ flags */
    EC_T_DWORD                  dwMbxOutSize;           /**< [out] Mailbox output byte size (from ENI file) */
    EC_T_DWORD                  dwMbxInSize;            /**< [out] Mailbox input byte size (from ENI file) */

    EC_T_DWORD                  dwMbxOutSize2;          /**< [out] Bootstrap mailbox output byte size (from ENI file) */
    EC_T_DWORD                  dwMbxInSize2;           /**< [out] Bootstrap mailbox input byte size (from ENI file) */

    EC_T_BOOL                   bDcSupport;             /**< [out] Slave supports DC (from ENI file) */

    EC_T_WORD                   wNumProcessVarsInp;     /**< [out] Number of input process data variables (from ENI file) */
    EC_T_WORD                   wNumProcessVarsOutp;    /**< [out] Number of output process data variables (from ENI file) */

    EC_T_WORD                   wPrevStationAddress;    /**< [out] Station address of the previous slave (from ENI file) */
    EC_T_WORD                   wPrevPort;              /**< [out] Connected port of the previous slave (from ENI file) */

    EC_T_WORD                   wIdentifyAdo;           /**< [out] ADO used for identification command (from ENI file) */
    EC_T_WORD                   wIdentifyData;          /**< [out] Identification value to be validated (from ENI file) */
    EC_T_BYTE                   byPortDescriptor;       /**< [out] Port descriptor (ESC register 0x0007) (from ENI file) */

    EC_T_BYTE                   abyReserved[3];
    EC_T_WORD                   wWkcStateDiagOffsIn[EC_CFG_SLAVE_PD_SECTIONS];  /**< [out] Offset of WkcState bit in diagnosis image (ENI: ProcessData/Recv[1..4]/BitStart) WkcState bit values: 0 = Data Valid, 1 = Data invalid */
    EC_T_WORD                   wWkcStateDiagOffsOut[EC_CFG_SLAVE_PD_SECTIONS]; /**< [out] Offset of WkcState bit in diagnosis image (ENI: ProcessData/Send[1..4]/BitStart) WkcState bit values: 0 = Data Valid, 1 = Data invalid */

    EC_T_WORD                   awMasterSyncUnitIn[EC_CFG_SLAVE_PD_SECTIONS];   /**< [out] Sync Unit (ENI: ProcessData/TxPdo[1..4]@Su) */
    EC_T_WORD                   awMasterSyncUnitOut[EC_CFG_SLAVE_PD_SECTIONS];  /**< [out] Sync Unit (ENI: ProcessData/RxPdo[1..4]@Su) */

    EC_T_BOOL                   bDisabled;              /**< [out] Slave disabled by API (emSetSlaveDisabled / emSetSlavesDisabled). */
    EC_T_BOOL                   bDisconnected;          /**< [out] Slave disconnected by API (emSetSlaveDisconnected / emSetSlavesDisconnected). */
    EC_T_BOOL                   bExtended;              /**< [out] Slave generated by emConfigExtend */

    EC_T_DWORD                  adwReserved[13];
} EC_PACKED(4) EC_T_CFG_SLAVE_INFO;

typedef struct _EC_T_PROFILE_CHANNEL_INFO
{
    EC_T_WORD                   wProfileNo;                             /**< [out] ProfileNo: "low word of CoE object 0x1000" */
    EC_T_WORD                   wAddInfo;                               /**< [out] AddInfo : "high word of CoE object 0x1000" */
    EC_T_CHAR                   szDisplayName[ECAT_DEVICE_NAMESIZE];    /**< [out] Display name */
} EC_PACKED(4) EC_T_PROFILE_CHANNEL_INFO;

/* ecatGetCfgSlaveEoeInfo, ENI: Mailbox/EoE/InitCmds/InitCmd/Data */
typedef struct _EC_T_CFG_SLAVE_EOE_INFO
{
    EC_T_DWORD                  dwSlaveId;              /**< [out] Slave ID */
    EC_T_BOOL                   bMacAddr;               /**< [out] Indicates whether the MAC address could be read and is valid */
    EC_T_BYTE                   abyMacAddr[6];          /**< [out] MAC address */
    EC_T_BOOL                   bIpAddr;                /**< [out] Indicates whether the IP address could be read and is valid */
    EC_T_BYTE                   abyIpAddr[4];           /**< [out] IP address */
    EC_T_BOOL                   bSubnetMask;            /**< [out] Indicates whether the subnet mask could be read and is valid */
    EC_T_BYTE                   abySubnetMask[4];       /**< [out] Subnet mask */
    EC_T_BOOL                   bDefaultGateway;        /**< [out] Indicates whether the default gateway could be read and is valid */
    EC_T_BYTE                   abyDefaultGateway[4];   /**< [out] Default gateway */
    EC_T_BOOL                   bDnsServer;             /**< [out] Indicates whether the DNS server could be read and is valid */
    EC_T_BYTE                   abyDnsServer[4];        /**< [out] DNS server */
    EC_T_BOOL                   bDnsName;               /**< [out] Indicates whether the DNS name could be read and is valid */
    EC_T_CHAR                   szDnsName[32];          /**< [out] DNS name */
} EC_PACKED(4) EC_T_CFG_SLAVE_EOE_INFO;

/* ecatGetBusSlaveInfo */
/** \defgroup EC_LINECROSSED_FLAGS Linecrossed flags
@{ */
#define EC_LINECROSSED_NOT_CONNECTED_PORTA          ((EC_T_WORD)0x00000001)
#define EC_LINECROSSED_UNEXPECTED_INPUT_PORT        ((EC_T_WORD)0x00000002)
#define EC_LINECROSSED_UNEXPECTED_JUNCTION_RED      ((EC_T_WORD)0x00000004)
#define EC_LINECROSSED_UNRESOLVED_PORT_CONNECTION   ((EC_T_WORD)0x00000008)
#define EC_LINECROSSED_HIDDEN_SLAVE_CONNECTED       ((EC_T_WORD)0x00000010)
#define EC_LINECROSSED_PHYSIC_MISMATCH              ((EC_T_WORD)0x00000020)
#define EC_LINECROSSED_INVALID_PORT_CONNECTION      ((EC_T_WORD)0x00000040)
/**@}*/
typedef struct _EC_T_BUS_SLAVE_INFO
{
    EC_T_DWORD                  dwSlaveId;                          /**< [out] The slave's ID to bind bus slave and config slave information */
    EC_T_DWORD                  adwPortSlaveIds[ESC_PORT_COUNT];    /**< [out] The slave's ID of the slaves connected to ports. See port slave ID's */
    EC_T_WORD                   wPortState;                         /**< [out] Port link state. Format: wwww xxxx yyyy zzzz (each nibble : port 3210)\n
                                                                               wwww : Signal detected 1=yes, 0=no\n     xxxx : Loop closed 1=yes, 0=no\n
                                                                               yyyy : Link established 1=yes, 0=no\n    zzzz : Slave connected 1=yes, 0=no (zzzz = logical result of w,x,y) */
    EC_T_WORD                   wAutoIncAddress;                    /**< [out] The slave's auto increment address */
    EC_T_BOOL                   bDcSupport;                         /**< [out] Slave supports DC (Bus Topology Scan) */
    EC_T_BOOL                   bDc64Support;                       /**< [out] Slave supports 64 Bit DC (Bus Topology Scan) */

    EC_T_DWORD                  dwVendorId;                         /**< [out] Vendor Identification stored in the EEPROM at offset 0x0008 */
    EC_T_DWORD                  dwProductCode;                      /**< [out] Product Code stored in the EEPROM at offset 0x000A */
    EC_T_DWORD                  dwRevisionNumber;                   /**< [out] Revision number stored in the EEPROM at offset 0x000C */
    EC_T_DWORD                  dwSerialNumber;                     /**< [out] Serial number stored in the EEPROM at offset 0x000E */

    EC_T_BYTE                   byESCType;                          /**< [out] Type of ESC (Value of slave ESC register 0x0000) */
    EC_T_BYTE                   byESCRevision;                      /**< [out] Revision number of ESC (Value of slave ESC register 0x0001) */
    EC_T_WORD                   wESCBuild;                          /**< [out] Build number of ESC (Value of slave ESC register 0x0002) */
    EC_T_BYTE                   byPortDescriptor;                   /**< [out] Port descriptor (Value of slave ESC register 0x0007) */
    EC_T_BYTE                   byReserved;
    EC_T_WORD                   wFeaturesSupported;                 /**< [out] Features supported (Value of slave ESC register 0x0008) */
    EC_T_WORD                   wStationAddress;                    /**< [out] The slave's station address (Value of slave ESC register 0x0010) */
    EC_T_WORD                   wAliasAddress;                      /**< [out] The slave's alias address (Value of slave ESC register 0x0012) */
    EC_T_WORD                   wAlStatus;                          /**< [out] AL status (Value of slave ESC register 0x0130) */
    EC_T_WORD                   wAlStatusCode;                      /**< [out] AL status code. (Value of slave ESC register 0x0134 during last error acknowledge). This value is reset after a slave state change */
    EC_T_DWORD                  dwSystemTimeDifference;             /**< [out] System time difference. (Value of slave ESC register 0x092C) */
    EC_T_WORD                   wMbxSupportedProtocols;             /**< [out] Supported Mailbox Protocols stored in the EEPROM at offset 0x001C */
    EC_T_WORD                   wDlStatus;                          /**< [out] DL status (Value of slave ESC register 0x0110) */
    EC_T_WORD                   wPrevPort;                          /**< [out] Connected port of the previous slave */
    EC_T_WORD                   wIdentifyData;                      /**< [out] Last read identification value see EC_T_CFG_SLAVE_INFO.wIdentifyAdo */
    EC_T_BOOL                   bLineCrossed;                       /**< [out] Line crossed was detetected at this slave */
    EC_T_DWORD                  dwSlaveDelay;                       /**< [out] Delay behind slave in ns. This value is only valid if a DC configuration is used */
    EC_T_DWORD                  dwPropagDelay;                      /**< [out] Propagation delay in ns. ESC register 0x0928,This value is only valid if a DC configuration is used */
    EC_T_BOOL                   bIsRefClock;                        /**< [out] Slave is reference clock */
    EC_T_BOOL                   bIsDeviceEmulation;                 /**< [out] Slave without Firmware. ESC register 0x0141, enabled by EEPROM offset 0x0000.8. */
    EC_T_WORD                   wLineCrossedFlags;                  /**< [out] Combination of EC_LINECROSSED_ flags */
    EC_T_WORD                   wReserved;
    EC_T_DWORD                  adwReserved[15];

} EC_PACKED(4) EC_T_BUS_SLAVE_INFO;

/* EC_IOCTL_SB_GET_BUS_SLAVE_PORTS_INFO */
typedef struct _EC_T_BUS_SLAVE_PORTS_INFO
{
    EC_T_BYTE   byPortDescriptor;               /* ESC:0x0007 */
    EC_T_WORD   wDlStatus;                      /* ESC:0x0110 */
    EC_T_DWORD  adwRecvTime[ESC_PORT_COUNT];    /* ESC:0x0900 - 0x090C */
    EC_T_UINT64 qwRecvTimeProcessingUnit;       /* ESC:0x0918 */
} EC_PACKED(4) EC_T_BUS_SLAVE_PORTS_INFO;
#include EC_PACKED_INCLUDESTOP/*(4)*/

#include EC_PACKED_API_INCLUDESTART
typedef struct _EC_T_TRACE_DATA_INFO
{
    EC_T_BYTE*      pbyData;    /**< [out] Process data output buffer, containing trace data */
    EC_T_DWORD      dwOffset;   /**< [out] Trace data offset in bytes */
    EC_T_WORD       wSize;      /**< [out] Trace data size in bytes */
} EC_PACKED_API EC_T_TRACE_DATA_INFO;
#include EC_PACKED_INCLUDESTOP/*(8)*/

#include EC_PACKED_INCLUDESTART(4)
/* See also Object 0x2002  Bus Diagnosis Object */
typedef struct _EC_T_BUS_DIAGNOSIS_INFO
{
    EC_T_DWORD dwCRC32ConfigCheckSum;   /**< CRC32 checksum of the loaded configuration */
    EC_T_DWORD dwNumSlavesFound;        /**< Number of slaves connected */
    EC_T_DWORD dwNumDCSlavesFound;      /**< Number of slaves with DC enabled connected */
    EC_T_DWORD dwNumCfgSlaves;          /**< Number of slaves in ENI */
    EC_T_DWORD dwNumMbxSlaves;          /**< Number of slaves in ENI with mailbox support */

    EC_T_DWORD dwTXFrames;              /**< Number of frames sent */
    EC_T_DWORD dwRXFrames;              /**< Number of frames received */
    EC_T_DWORD dwLostFrames;            /**< Number of lost frames*/

    EC_T_DWORD dwCyclicFrames;          /**< Number of cyclic frames sent */
    EC_T_DWORD dwCyclicDatagrams;       /**< Number of cyclic datagrams / EtherCAT commands sent*/
    EC_T_DWORD dwAcyclicFrames;         /**< Number of acyclic frames sent */
    EC_T_DWORD dwAcyclicDatagrams;      /**< Number of acyclic datagrams / EtherCAT commands sent*/
    EC_T_DWORD dwClearCounters;         /**< Clear frame / datagram counter bit field */
    EC_T_DWORD dwCyclicLostFrames;      /**< Number of cyclic lost frames */
    EC_T_DWORD dwAcyclicLostFrames;     /**< Number of acyclic lost frames */
    EC_T_DWORD dwRes[2];
} EC_PACKED(4) EC_T_BUS_DIAGNOSIS_INFO;

/* See also Object 0x2003  Redundancy Diagnosis Object */
typedef struct _EC_T_REDUNDANCY_DIAGNOSIS_INFO
{
    EC_T_BOOL  bRedEnabled;             /**<  Cable Redundancy Enabled */
    EC_T_DWORD dwMainSlaveCnt;          /**<  Main Line Slave Count */
    EC_T_DWORD dwRedSlaveCnt;           /**<  Red Line Slave Count */
    EC_T_BOOL  bLineBreakDetected;      /**<  Line Break Detected */
    EC_T_DWORD dwRes[4];
} EC_PACKED(4) EC_T_REDUNDANCY_DIAGNOSIS_INFO;

/* See also Object 0x2006  Mailbox Statistics Object */
typedef struct _EC_T_STATISTIC
{
    EC_T_DWORD dwTotal;                         /**< Total */
    EC_T_DWORD dwLast;                          /**< Last */
} EC_PACKED(4) EC_T_STATISTIC;
typedef struct _EC_T_STATISTIC_TRANSFER
{
    EC_T_STATISTIC Cnt;                         /**< Number of transfers */
    EC_T_STATISTIC Bytes;                       /**< Number of bytes transferred */
} EC_PACKED(4) EC_T_STATISTIC_TRANSFER;
typedef struct _EC_T_STATISTIC_TRANSFER_DUPLEX
{
    EC_T_STATISTIC_TRANSFER Read;               /**< Number of read transfers */
    EC_T_STATISTIC_TRANSFER Write;              /**< Number of write transfers*/
} EC_PACKED(4) EC_T_STATISTIC_TRANSFER_DUPLEX;
typedef struct _EC_T_MAILBOX_STATISTICS
{
    EC_T_STATISTIC_TRANSFER_DUPLEX Aoe;         /**< AoE mailbox transfer statistics */
    EC_T_STATISTIC_TRANSFER_DUPLEX Coe;         /**< CoE mailbox transfer statistics */
    EC_T_STATISTIC_TRANSFER_DUPLEX Eoe;         /**< EoE mailbox transfer statistics */
    EC_T_STATISTIC_TRANSFER_DUPLEX Foe;         /**< FoE mailbox transfer statistics */
    EC_T_STATISTIC_TRANSFER_DUPLEX Soe;         /**< SoE mailbox transfer statistics */
    EC_T_STATISTIC_TRANSFER_DUPLEX Voe;         /**< VoE mailbox transfer statistics */
    EC_T_STATISTIC_TRANSFER_DUPLEX RawMbx;      /**< Raw mailbox transfer statistics */
    EC_T_STATISTIC_TRANSFER_DUPLEX aRes; /* reserved */
} EC_PACKED(4) EC_T_MAILBOX_STATISTICS;

/* ecatGetMasterInfo */
typedef struct _EC_T_MASTER_INFO
{
    EC_T_DWORD              dwMasterVersion;                /**< Master version */
    EC_T_BUS_DIAGNOSIS_INFO BusDiagnosisInfo;               /**< Bus diagnostics */
    EC_T_MAILBOX_STATISTICS MailboxStatistics;              /**< Mailbox statistics */
    EC_T_REDUNDANCY_DIAGNOSIS_INFO RedundancyDiagnosisInfo; /**< Redundancy diagnosis info */
    EC_T_DWORD              dwMasterStateSummary;           /**< Master state summary */
    EC_T_DWORD              adwReserved[23];
} EC_PACKED(4) EC_T_MASTER_INFO;

#define ATEMRAS_ACCESS_LEVEL_ALLOW_ALL  1   /* all functions calls allowed, i.e. change of master state as well */
#define ATEMRAS_ACCESS_LEVEL_READWRITE  2   /* functions with parameter change, i.e. set or download */
#define ATEMRAS_ACCESS_LEVEL_READONLY   3   /* functions with no parameter change, i.e. get or upload */
#define ATEMRAS_ACCESS_LEVEL_BLOCK_ALL  4   /* no functions calls allowed */
#define ATEMRAS_ACCESS_LEVEL_EXCLUDED   ((EC_T_DWORD)-1)

/* textual description of HotConnect Mode */
#define RasAccessLevelText(eLevel)                   \
    ((eLevel)==ATEMRAS_ACCESS_LEVEL_ALLOW_ALL?"allow all":     \
     ((eLevel)==ATEMRAS_ACCESS_LEVEL_READWRITE?"read/write":   \
      ((eLevel)==ATEMRAS_ACCESS_LEVEL_READONLY?"read only":    \
       ((eLevel)==ATEMRAS_ACCESS_LEVEL_BLOCK_ALL?"block all":  \
        "INVALID ATEMRAS_ACCESS_LEVEL_..."))))

typedef struct _EC_T_RAS_CONNECTION_INFO
{
    EC_T_DWORD      dwAccessControlActive;
    EC_T_DWORD      dwAccessLevel;
    EC_T_DWORD      adwReserved[2];
} EC_PACKED(4) EC_T_RAS_CONNECTION_INFO;

#include EC_PACKED_INCLUDESTOP/*(4)*/

#include EC_PACKED_API_INCLUDESTART
typedef struct _EC_T_MSU_INFO
{
    EC_T_WORD       wMsuId;                         /**< [out] master sync unit ID */
    EC_T_DWORD      dwBitOffsIn;                    /**< [out] input bit offset of master sync unit in process data image */
    EC_T_DWORD      dwBitSizeIn;                    /**< [out] input bit size of master sync unit */
    EC_T_DWORD      dwBitOffsOut;                   /**< [out] output bit offset of master sync unit in process data image */
    EC_T_DWORD      dwBitSizeOut;                   /**< [out] output bit size of master sync unit */
    EC_T_WORD       wWkcStateDiagOffsIn;            /**< [out] Offset of WkcState bit in diagnosis image WkcState bit values: 0 = Data Valid, 1 = Data invalid */
    EC_T_WORD       wWkcStateDiagOffsOut;           /**< [out] Offset of WkcState bit in diagnosis image WkcState bit values: 0 = Data Valid, 1 = Data invalid */

    EC_T_DWORD      adwReserved[16];                /**< reserved */
} EC_PACKED_API  EC_T_MSU_INFO;
#include EC_PACKED_INCLUDESTOP

#include EC_PACKED_API_INCLUDESTART
typedef struct _EC_T_TOPOLOGY_CHANGED_DELAYS
{
    EC_T_DWORD      dwSlavePort;                    /**< [in] Delay before opening slave port after link connection detected */
    EC_T_DWORD      dwMainLine;                     /**< [in] Delay before sending frames at main line after link connection detected */
    EC_T_DWORD      dwRedLine;                      /**< [in] Delay before sending frames at red line after link connection detected */

    EC_T_DWORD      adwReserved[5];                 /**< reserved */
} EC_PACKED_API  EC_T_TOPOLOGY_CHANGED_DELAYS;
#include EC_PACKED_INCLUDESTOP

#include EC_PACKED_API_INCLUDESTART
typedef struct _EC_T_LINK_CONNECTED_INFO
{
    EC_T_BOOL       bConnected;                     /**< [out] MAIN or RED link detected */
    EC_T_BOOL       bSendEnabled;                   /**< [out] send enabled on MAIN or RED */
    EC_T_BOOL       bMainConnected;                 /**< [out] MAIN link detected */
    EC_T_BOOL       bMainMasked;                    /**< [out] MAIN link not used for sending, because topology changed delay not elapsed yet */
    EC_T_BOOL       bRedConnected;                  /**< [out] RED  link detected */
    EC_T_BOOL       bRedMasked;                     /**< [out] RED  link not used for sending, because topology changed delay not elapsed yet */
} EC_PACKED_API EC_T_LINK_CONNECTED_INFO;
#include EC_PACKED_INCLUDESTOP

#define MSU_ID_ALL_INFO_ENTRIES         ((EC_T_WORD)0xFFFF)

/**
 * \typedef EC_PF_NOTIFY
 * \param dwCode   [in] Notification code.
 * \param pParms   [in] Notification code depending data.
 */
typedef EC_T_DWORD (*EC_PF_NOTIFY)(EC_T_DWORD dwCode, EC_T_NOTIFYPARMS* pParms);

/* EtherCAT ioctl parameters */
#include EC_PACKED_API_INCLUDESTART
typedef struct _EC_T_IOCTLPARMS
{
    EC_T_BYTE*      pbyInBuf;                       /**< [in] Pointer to control input parameter. */
    EC_T_DWORD      dwInBufSize;                    /**< [in] Size of the input buffer provided at pbyInBuf in bytes */
    EC_T_BYTE*      pbyOutBuf;                      /**< [out] Pointer to control output buffer where the results will be copied into */
    EC_T_DWORD      dwOutBufSize;                   /**< [in] Size of the output buffer provided at pbyOutBuf in bytes */
    EC_T_DWORD*     pdwNumOutData;                  /**< [out] Pointer to EC_T_DWORD. Amount of bytes written to the output buffer */
} EC_PACKED_API EC_T_IOCTLPARMS;
#include EC_PACKED_INCLUDESTOP

/* Client register parameters */
typedef struct _EC_T_REGISTERPARMS
{
    EC_T_VOID*      pCallerData;                    /**< [in] used by all callback functions */
    EC_PF_NOTIFY    pfnNotify;                      /**< [in] notify callback function pointer */
}/*EC_PACKED */EC_T_REGISTERPARMS; /* Packed function pointer causes error with VxWorks DIAB compiler */

                                   /* Client register result */
#include EC_PACKED_INCLUDESTART(1)
typedef struct _EC_T_REGISTERRESULTS
{
    EC_T_DWORD      dwClntId;                       /**< [out] Client ID */
    EC_T_BYTE*      pbyPDIn;                        /**< [out] Pointer to process data input memory */
    EC_T_DWORD      dwPDInSize;                     /**< [out] Size of process data input memory (in bytes) */
    EC_T_BYTE*      pbyPDOut;                       /**< [out] Pointer to process data output memory */
    EC_T_DWORD      dwPDOutSize;                    /**< [out] Size of process data output memory (in bytes) */
} EC_PACKED(1) EC_T_REGISTERRESULTS;
#include EC_PACKED_INCLUDESTOP

/* Msg logging */
typedef EC_T_BOOL (*EC_T_PFLOGMSG_CB)(const EC_T_CHAR* szFormat, EC_T_VALIST vaArgs);

/**
\defgroup EC_LOG_FRAME_FLAGS
@{
*/
#define EC_LOG_FRAME_FLAG_MASTERSTATE_MASK 0xFFFF   /**< Bit 0 to 15: Master state mask */
#define EC_LOG_FRAME_FLAG_ACYC_FRAME       (1<<16)  /**< Bit 16 (0x00010000): 0=cyclic frame, 1=acyclic frame */
#define EC_LOG_FRAME_FLAG_DBG_FRAME        (1<<17)  /**< Bit 17 (0x00020000): 0=EtherCAT frame, 1=debug frame */
#define EC_LOG_FRAME_FLAG_RED_FRAME        (1<<18)  /**< Bit 18 (0x00040000): 0=main frame, 1=red frame */
#define EC_LOG_FRAME_FLAG_RX_FRAME         (1<<19)  /**< Bit 19 (0x00080000): 0=TX frame, 1=RX frame */
#define EC_LOG_FRAME_FLAG_MASTER_RED_FRAME (1<<20)  /**< Bit 20 (0x00100000): 0=slave frame, 1=MasterMaster frame */
/**@}*/

/**
 * \typedef EC_T_PFLOGFRAME_CB
 * \param pvContext   [in] Context pointer. This pointer is used as parameter when the callback function is called
 * \param dwLogFlags  [in] Frame logging flags, EC_LOG_FRAME_FLAG_...
 * \param dwFrameSize [in] Size of frame in bytes
 * \param pbyFrame    [in] Pointer to frame data
 * \note The master discards the frame if the callback function modifies the Ethernet frame type at byte offset 12.
 */
typedef EC_T_VOID (*EC_T_PFLOGFRAME_CB)(EC_T_VOID* pvContext, EC_T_DWORD dwLogFlags, EC_T_DWORD dwFrameSize, EC_T_BYTE* pbyFrame);

/* Master Redundancy initialization parameters, see ecatInitMaster */
#include EC_PACKED_API_INCLUDESTART
typedef struct _EC_T_MASTER_RED_PARMS
{
    EC_T_BOOL   bEnabled;                      /**< [in] set to EC_TRUE if using Master Redundancy */
    EC_T_WORD   wMasterPdOutSize;              /**< [in] ACTIVE to INACTIVE Master Process Data (in bytes) */
    EC_T_WORD   wMasterPdInSize;               /**< [in] INACTIVE to ACTIVE Master Process Data (in bytes) */
    EC_T_DWORD  dwMaxAcycFramesPerCycle;       /**< [in] maximum acyclic Master Red frames sent per cycle */
    EC_T_BOOL   bUpdateSlavePdOut;             /**< [in] set to EC_TRUE to update Slave OUTPUT Proces Data Image at INACTIVE Master (from CSF) */
    EC_T_BOOL   bUpdateSlavePdIn;              /**< [in] set to EC_TRUE to update Slave INPUT Proces Data Image at INACTIVE Master (from CMF) */
} EC_PACKED_API EC_T_MASTER_RED_PARMS;
#include EC_PACKED_INCLUDESTOP

/* Distributed clocks */
#include EC_PACKED_API_INCLUDESTART
typedef struct _EC_T_DC_SYNCSO_CB_PARM
{
    EC_T_VOID*              pvContext;              /**< [in] Calling environment context */
    EC_T_DWORD              dwTimeStamp;            /**< [in] Pre-Send Timestamp */
    EC_T_DWORD              dwPostTimeStamp;        /**< [in] Post-Send Timestamp */
    EC_T_DWORD              dwBusTimeLo;            /**< [in] Bus Time Lower 32 Bit */
    EC_T_DWORD              dwBusTimeHi;            /**< [in] Bus Time Upper 32 Bit */
    EC_T_DWORD              dwTimeStampResult;      /**< [in] Result of stamp */

    EC_T_DWORD              dwSyncPeriodLength;     /**< [in] Time between two consecutive SYNC0 signals of clock master slave [ns]. */

    EC_T_DWORD              dwStartSyncTimeLo;      /**< [out] Sync Start Time Lower 32 Bit */
    EC_T_DWORD              dwStartSyncTimeHi;      /**< [out] Sync Start Time Upper 32 Bit */

} EC_PACKED_API  EC_T_DC_SYNCSO_CB_PARM, *EC_PT_DC_SYNCSO_CB_PARM;
#include EC_PACKED_INCLUDESTOP

typedef EC_T_DWORD (*EC_T_PFSYNCSO_CB)(EC_T_DC_SYNCSO_CB_PARM* pParm);

#include EC_PACKED_API_INCLUDESTART
typedef struct _EC_T_DC_SYNCSO_REGDESC
{
    EC_T_DC_SYNCSO_CB_PARM*     pCallbackParm;      /**< [in] Callback Parameter */
    EC_T_PFSYNCSO_CB            pfnCallback;        /**< [in] Callback Function */
} EC_PACKED_API  EC_T_DC_SYNCSO_REGDESC, *EC_PT_DC_SYNCSO_REGDESC;
#include EC_PACKED_INCLUDESTOP

#include EC_PACKED_API_INCLUDESTART
typedef struct _EC_T_DC_CONFIGURE
{
    EC_T_DWORD                  dwClntId;                       /**< [in] Reserved */
    EC_T_DWORD                  dwTimeout;                      /**< [in] Timeout [ms] for the DC initialization in which time offsets and propagation delays are evaluated. */
    EC_T_DWORD                  dwDevLimit;                     /**< [in] Maximum permissible deviation of the individual slave clock and the DC reference clock. The maximum deviation is determined by wire or'ed the deviations of the individual slave clocks with one another.
                                                                          The check against the limit is only active if "Sync Window Monitoring" is set in the configuration tool (EC Engineer), which generates a BRD command to read the slave register 0x092C in every cycle.
                                                                          The limit is calculated as follows:\n 2^n - 1 ns, e.g. a dwDevLimit of 4 corresponds to 14 ns.\n A value of 0 disables the "Sync Window Monitoring" */
    EC_T_DWORD                  dwSettleTime;                   /**< [in] Settle time [ms]. At the beginning of the synchronization the slave clocks oscillate strongly.
                                                                          To prevent multiple in-sync and out-of-sync notifications from being generated, a settling time can be set in which no notifications are generated. */
    EC_T_DWORD                  dwTotalBurstLength;             /**< [in] Overall amount of burst frames sent. Default 10000. */
    EC_T_DWORD                  dwBurstBulk;                    /**< [in] Amount of burst frames per cycle during initialization burst. Default 12. */
    EC_T_BOOL                   bBulkInLinkLayer;               /**< [in] If EC_TRUE, bulk is realized by link layer, otherwise by master. The MAC needs to support the frame repeating function.
                                                                          In this case the link layer will repeat the DC burst frames itself,reduing the hardware accesses of the master to the MAC.  */
    EC_T_BOOL                   bAcycDistributionDisabled;      /**< [in] If EC_TRUE, acyclic distribution is disabled */
    EC_T_DWORD                  dwDcStartTimeGrid;              /**< [in] Time grid [ns] to align DC start time. With the help of the grid, several EtherCAT networks can be synchronized without a random shift value between the SYNC signals. */
    EC_T_BOOL                   bDcInitBeforeSlaveStateChange;  /**< [in] If EC_TRUE, DC is initialized before slaves state change to PREOP */
    EC_T_DWORD                  dwReserved[4];                  /**< [in/out] Reserved */
} EC_PACKED_API  EC_T_DC_CONFIGURE, *EC_PT_DC_CONFIGURE;
#include EC_PACKED_INCLUDESTOP

typedef enum _EC_T_DCM_MODE
{
    eDcmMode_Off               = 0,     /**< DCM disabled */
    eDcmMode_BusShift          = 1,     /**< DCM BusShift mode */
    eDcmMode_MasterShift       = 2,     /**< DCM MasterShift mode */
    eDcmMode_LinkLayerRefClock = 3,     /**< DCM LinkLayer Ref Clock mode*/
    eDcmMode_MasterRefClock    = 4,     /**< DCM Master Ref Clock mode */
    eDcmMode_Dcx               = 5,     /**< DCM DCX External synchronization mode */
    eDcmMode_MasterShiftByApp  = 6,     /**< DCM MasterShift controlled by application mode */

    /* Borland C++ datatype alignment correction */
    eDcmMode_BCppDummy      = 0xFFFFFFFF
} EC_T_DCM_MODE;

#include EC_PACKED_API_INCLUDESTART
/**
 * \typedef EC_PF_DC_STARTTIME_CB
 * \brief EC-Master requests DC start time for every single slave from a given callback DcStartTimeCallbackDesc with slave station address as input parameter.
 *        The slave specific DC start time value will be passed directly to the slave without modifications by master.
 *        This means no other values like nCtlSetVal will be added. Shift value configured in ENI will still be applied.
 * \param pvContext       [in]  Context pointer. It is used as parameter when the callback function is called.
 * \param wSlaveFixedAddr [in]  Slave fixed address.
 * \param pqwDcStartTime  [out] DC start time for specific slave.
 * \return EC_E_NOERROR or error code
 */
typedef EC_T_DWORD (*EC_PF_DC_STARTTIME_CB)(EC_T_VOID* pvContext, EC_T_WORD wSlaveFixedAddr, EC_T_UINT64* pqwDcStartTime);
typedef struct _EC_T_DC_STARTTIME_CB_DESC
{
    EC_T_VOID*                  pvContext;          /**< [in] Context pointer. It is used as parameter when the callback function is called */
    EC_PF_DC_STARTTIME_CB       pfnCallback;        /**< [in] Dc start time callback function pointer. If not null, DC start time calculated by application, otherwise by master */
} EC_PACKED_API EC_T_DC_STARTTIME_CB_DESC;
#include EC_PACKED_INCLUDESTOP

#include EC_PACKED_API_INCLUDESTART
typedef struct _EC_T_DCM_CONFIG_BUSSHIFT
{
    EC_T_INT                    nCtlSetVal;                 /**< [in] Controller set value [ns]. This is the time distance between the cyclic frame send time and the DC base on bus (SYNC0 if shift is zero). */
    EC_T_INT                    nCtlGain;                   /**< [in] Proportional gain in ppt (part per thousand). Default is value 2. A value of 0 let the current setting unmodified. */
    EC_T_INT                    nCtlDriftErrorGain;         /**< [in] Multiplier for drift error. Default value is 3. A value of 0 let the current setting unmodified */
    EC_T_INT                    nMaxValidVal;               /**< [in] Error inputs above this value are considered invalid. If error input prediction is valid then the difference between the error input and the expected value is taken. Default value is 3000. A value of 0 let the current setting unmodified */
    EC_T_BOOL                   bLogEnabled;                /**< [in] If set to EC_TRUE, logging information are generated and can be get calling emDcmGetLog */
    EC_T_DWORD                  dwInSyncLimit;              /**< [in] Limit [ns] for InSync monitoring. Default value is 4000. A value of 0 let the current setting unmodified */
    EC_T_DWORD                  dwInSyncSettleTime;         /**< [in] Settle time [ms] for InSync monitoring. Default value is 1500. A value of 0 let the current setting unmodified */
    EC_T_BOOL                   bCtlOff;                    /**< [in] If set to EC_TRUE, control loop is disabled. Combined with bLogEnabled, it makes possible to analyze the natural drift between the stack cycle and the reference clock */
    EC_T_BOOL                   bUseDcLoopCtlStdValues;     /**< [in] If set to EC_TRUE, the values of ESC DC time loop control register 0x930 and 0x934 are not changed by master. This could increase the time it takes to get the InSync. Use only if there are a problems with the reference clock to get InSync */
    EC_T_DWORD                  dwInSyncStartDelayCycle;    /**< [in] Delay time [ms] before InSync monitoring start */
} EC_PACKED_API  EC_T_DCM_CONFIG_BUSSHIFT, *EC_PT_DCM_CONFIG_BUSSHIFT;
#include EC_PACKED_INCLUDESTOP

#include EC_PACKED_API_INCLUDESTART
typedef struct _EC_T_DCM_CONFIG_MASTERSHIFT
{
    EC_T_INT                    nCtlSetVal;                 /**< [in] Controller set value [ns]. This is the time distance between the cyclic frame send time and the DC base on bus (SYNC0 if shift is zero) */
    EC_T_INT                    nCtlGain;                   /**< [in] Proportional gain in ppt (part per thousand). Default is value 2. A value of 0 let the current setting unmodified */
    EC_T_INT                    nCtlDriftErrorGain;         /**< [in] Multiplier for drift error. Default value is 3. A value of 0 let the current setting unmodified */
    EC_T_INT                    nMaxValidVal;               /**< [in] Error inputs above this value are considered invalid. If error input prediction is valid then the difference between the error input and the expected value is taken. Default value is 3000. A value of 0 let the current setting unmodified */
    EC_T_BOOL                   bLogEnabled;                /**< [in] If set to EC_TRUE, logging information are generated and can be get calling emDcmGetLog */
    EC_T_DWORD                  dwInSyncLimit;              /**< [in] Limit [ns] for InSync monitoring. Default value is 4000. A value of 0 let the current setting unmodified */
    EC_T_DWORD                  dwInSyncSettleTime;         /**< [in] Settle time [ms] for InSync monitoring. Default value is 1500. A value of 0 let the current setting unmodified */
    EC_T_BOOL                   bCtlOff;                    /**< [in] If set to EC_TRUE, control loop is disabled. Combined with bLogEnabled, it makes possible to analyze the natural drift between the stack cycle and the reference clock. Also it provides reading of current adjustment value using emDcmGetAdjust function */
    EC_T_DWORD                  dwInSyncStartDelayCycle;    /**< [in] Delay time [ms] before InSync monitoring start */
    EC_PACKED_API_MEMBER \
        EC_T_DC_STARTTIME_CB_DESC   DcStartTimeCallbackDesc;    /**< [in] If not null, DC start time calculated by application, otherwise by master. See also EC_T_DC_STARTTIME_CB_DESC. Shift value configured in ENI will still be applied */
} EC_PACKED_API  EC_T_DCM_CONFIG_MASTERSHIFT, *EC_PT_DCM_CONFIG_MASTERSHIFT;
#include EC_PACKED_INCLUDESTOP

#include EC_PACKED_API_INCLUDESTART
typedef struct _EC_T_DCM_CONFIG_LINKLAYERREFCLOCK
{
    EC_T_INT                    nCtlSetVal;                     /**< [in] Controller set value [ns]. This is the time distance between the cyclic frame send time and the DC base on bus (SYNC0 if shift is zero) */
    EC_T_BOOL                   bLogEnabled;                    /**< [in] If set to EC_TRUE, logging information are generated and can be get calling emDcmGetLog */
   EC_PACKED_API_MEMBER \
        EC_T_DC_STARTTIME_CB_DESC   DcStartTimeCallbackDesc;    /**< [in] If not null, DC start time calculated by application, otherwise by master. See also EC_T_DC_STARTTIME_CB_DESC. Shift value configured in ENI will still be applied. */
} EC_PACKED_API  EC_T_DCM_CONFIG_LINKLAYERREFCLOCK, *EC_PT_DCM_CONFIG_LINKLAYERREFCLOCK;
#include EC_PACKED_INCLUDESTOP

#include EC_PACKED_API_INCLUDESTART
typedef struct _EC_T_DCM_CONFIG_MASTERREFCLOCK
{
    EC_T_INT                    nCtlSetVal;              /**< [in] Controller set value [ns]. This is the time distance between the cyclic frame send time and the DC base on bus (SYNC0 if shift is zero) */
    EC_T_BOOL                   bLogEnabled;             /**< [in] If set to EC_TRUE, logging information are generated and can be get calling emDcmGetLog */
    EC_T_DWORD                  dwInSyncLimit;           /**< [in] Limit [ns] for InSync monitoring. Default value is 4000. A value of 0 let the current setting unmodified */
    EC_T_DWORD                  dwInSyncSettleTime;      /**< [in] Settle time [ms] for InSync monitoring. Default value is 1500. A value of 0 let the current setting unmodified */
    EC_T_DWORD                  dwInSyncStartDelayCycle; /**< [in] Delay time [ms] before InSync monitoring start */
} EC_PACKED_API  EC_T_DCM_CONFIG_MASTERREFCLOCK, *EC_PT_DCM_CONFIG_MASTERREFCLOCK;
#include EC_PACKED_INCLUDESTOP

#include EC_PACKED_API_INCLUDESTART
typedef struct _EC_T_DCM_CONFIG_DCX
{
    EC_T_DCM_CONFIG_MASTERSHIFT MasterShift;
    EC_T_INT                    nCtlSetVal;              /**< [in] controller set val [ns] */
    EC_T_INT                    nCtlGain;                /**< [in] controller gain */
    EC_T_INT                    nCtlDriftErrorGain;      /**< [in] controller drift error gain */
    EC_T_INT                    nMaxValidVal;            /**< [in] max valid input value */
    EC_T_BOOL                   bLogEnabled;             /**< [in] EC_TRUE if logging information should be generated */
    EC_T_DWORD                  dwInSyncLimit;           /**< [in] limit [ns] for InSync monitoring */
    EC_T_DWORD                  dwInSyncSettleTime;      /**< [in] settle time [ms] for InSync monitoring */
    EC_T_BOOL                   bCtlOff;                 /**< [in] EC_TRUE if controller should not adjust the reference clock (for diagnostic, or self adjust using emDcmGetAdjust()) */
    EC_T_WORD                   wExtClockFixedAddr;      /**< [in] Fixed address of external clock slave (publishing PDO 0x10F4) (optional if ENI is generated by EcEngineer) */
    EC_T_DWORD                  dwExtClockTimeout;       /**< [in] Wait timeout for external clock slave */
    EC_T_DWORD                  dwInSyncStartDelayCycle; /**< [in] delay time [ms] before InSync monitoring start */
    EC_T_DWORD                  dwMaxErrCompensableOnExtClockReconnect; /**< [in] max error that should be compensated after a reconnect of the external clock. Sync restart if error exceeds this limit */
} EC_PACKED_API  EC_T_DCM_CONFIG_DCX, *EC_PT_DCM_CONFIG_DCX;
#include EC_PACKED_INCLUDESTOP

#include EC_PACKED_API_INCLUDESTART
typedef struct _EC_T_DCM_CONFIG
{
    EC_T_DCM_MODE               eMode;                       /**< [in] DCM mode */
    union {
        EC_T_DCM_CONFIG_BUSSHIFT          BusShift;          /**< [in] BusShift configuration. Valid if eMode is set to eDcmMode_BusShift          */
        EC_T_DCM_CONFIG_MASTERSHIFT       MasterShift;       /**< [in] MasterShift configuration. Valid if eMode is set to eDcmMode_MasterShift       */
        EC_T_DCM_CONFIG_LINKLAYERREFCLOCK LinkLayerRefClock; /**< [in] LinkLayerRefClock configuration. Valid if eMode is set to eDcmMode_LinkLayerRefClock */
        EC_T_DCM_CONFIG_MASTERREFCLOCK    MasterRefClock;    /**< [in] MasterRefClock configuration. Valid if eMode is set to eDcmMode_MasterRefClock    */
        EC_T_DCM_CONFIG_DCX               Dcx;               /**< [in] DCX configuration. Valid if eMode is set to eDcmMode_Dcx               */
        EC_T_DWORD                        adwReserved[32];
    } EC_PACKED_API u;
} EC_PACKED_API  EC_T_DCM_CONFIG, *EC_PT_DCM_CONFIG;
#include EC_PACKED_INCLUDESTOP

#include EC_PACKED_API_INCLUDESTART
typedef struct _EC_T_DCM_LOG
{
    EC_T_DWORD                  dwMsecCounter;          /**< [out] Current MsecCounter */
    EC_T_INT                    nCtlSetVal;             /**< [out] Configured controller set val [ns] */
    EC_T_UINT64                 qwBusTime;              /**< [out] Current BusTime */
    EC_T_INT                    nCtlErrorNsec;          /**< [out] Current controller error [ns] */
    EC_T_INT                    nDrift;                 /**< [out] Current calculated drift [ppm] */
    EC_T_DWORD                  dwErrorCode;            /**< [out] Last returned error code  by controller */
    EC_T_BOOL                   bDcmInSync;             /**< [out] EC_TRUE if DCM is in sync, EC_FALSE if out of sync */
    EC_T_BOOL                   bDcInSync;              /**< [out] EC_TRUE if DC is in sync, EC_FALSE if out of sync */
    EC_T_UINT64                 qwDcStartTime;          /**< [out] Last used DC StartTime */
    EC_T_INT                    nSystemTimeDifference;  /**< [out] Last read System Time Difference (ESC register 0x092C) */
} EC_PACKED_API  EC_T_DCM_LOG, *EC_PT_DCM_LOG;
#include EC_PACKED_INCLUDESTOP

/* ecatGetSlaveInfo */
#include EC_PACKED_INCLUDESTART(1)
typedef struct _EC_T_GET_SLAVE_INFO
{
    EC_T_DWORD                  dwScanBusStatus;    /* 0x00 */  /**< Status during last Bus Scan */

    EC_T_DWORD                  dwVendorId;         /* 0x01 */  /**< Vendor Identification */
    EC_T_DWORD                  dwProductCode;      /* 0x02 */  /**< Product Code */
    EC_T_DWORD                  dwRevisionNumber;   /* 0x03 */  /**< Revision Number */
    EC_T_DWORD                  dwSerialNumber;     /* 0x04 */  /**< Serial Number */

    EC_T_WORD                   wPortState;         /* 0x05 */  /**< [out] port link state (SB Instance)*/
    EC_T_WORD                   wReserved;                      /**< Res */

    EC_T_BOOL                   bDcSupport;         /* 0x06 */  /**< [out] slave does support DC*/
    EC_T_BOOL                   bDc64Support;       /* 0x07 */  /**< [out] slave does support 64Bit DC*/

    EC_T_WORD                   wAliasAddress;      /* 0x08 */  /**< [out] slave alias address*/
    EC_T_WORD                   wPhysAddress;                   /**< [out] slave station address*/

    EC_T_DWORD                  dwPdOffsIn;         /* 0x09 */  /**< [out] process data offset of Input Data (in bits)*/
    EC_T_DWORD                  dwPdSizeIn;         /* 0x0A */  /**< [out] process data size of Input Data (in bits)*/
    EC_T_DWORD                  dwPdOffsOut;        /* 0x0B */  /**< [out] process data offset of Output Data (in bits)*/
    EC_T_DWORD                  dwPdSizeOut;        /* 0x0C */  /**< [out] process data size of Output Data*/
    EC_T_DWORD                  dwPdOffsIn2;        /* 0x0D */  /**< [out] process data offset of Input data (in bits)*/
    EC_T_DWORD                  dwPdSizeIn2;        /* 0x0E */  /**< [out] process data size of Input Data (in bits)*/
    EC_T_DWORD                  dwPdOffsOut2;       /* 0x0F */  /**< [out] process data offset of Output Data (in bits)*/
    EC_T_DWORD                  dwPdSizeOut2;       /* 0x10 */  /**< [out] process data size of Output Data*/
    EC_T_DWORD                  dwPdOffsIn3;        /* 0x11 */  /**< [out] process data offset of Input Data (in bits)*/
    EC_T_DWORD                  dwPdSizeIn3;        /* 0x12 */  /**< [out] process data size of Input Data (in bits)*/
    EC_T_DWORD                  dwPdOffsOut3;       /* 0x13 */  /**< [out] process data offset of Output Data (in bits)*/
    EC_T_DWORD                  dwPdSizeOut3;       /* 0x14 */  /**< [out] process data size of Output Data*/
    EC_T_DWORD                  dwPdOffsIn4;        /* 0x15 */  /**< [out] process data offset of Input Data (in bits)*/
    EC_T_DWORD                  dwPdSizeIn4;        /* 0x16 */  /**< [out] process data size of Input Data (in bits)*/
    EC_T_DWORD                  dwPdOffsOut4;       /* 0x17 */  /**< [out] process data offset of Output Data (in bits)*/
    EC_T_DWORD                  dwPdSizeOut4;       /* 0x18 */  /**< [out] process data size of Output Data*/

    EC_T_WORD                   wCfgPhyAddress;     /* 0x19 */  /**< [out] slave configured station address*/
    EC_T_WORD                   wReserved2;                     /**< reserved */

    EC_T_CHAR                   abyDeviceName[ECAT_DEVICE_NAMESIZE];
    /* 0x1A */  /**< [out] slave name of configuration*/
    EC_T_BOOL                   bIsMailboxSlave;    /* 0x2E */  /**< [out] whether slave support mailboxes*/
    EC_T_DWORD                  dwMbxOutSize;       /* 0x2F */  /**< [out] mailbox 1 output size*/
    EC_T_DWORD                  dwMbxInSize;        /* 0x30 */  /**< [out] mailbox 1 input size*/
    EC_T_DWORD                  dwMbxOutSize2;      /* 0x31 */  /**< [out] mailbox 2 output size*/
    EC_T_DWORD                  dwMbxInSize2;       /* 0x32 */  /**< [out] mailbox 2 input size*/

    EC_T_DWORD                  dwErrorCode;        /* 0x33 */  /**< [out] last return code*/
    EC_T_DWORD                  dwSBErrorCode;      /* 0x34 */  /**< [out] last return value from SB*/

    EC_T_BYTE                   byPortDescriptor;   /* 0x35 */  /**< [out] Port Descriptor (ESC register 0x0007) */
    EC_T_BYTE                   byESCType;                      /**< [out] ESC Node Type */
    EC_T_WORD                   wSupportedMbxProtocols;         /**< [out] supported mailbox protocols: AoE, EoE, CoE, FoE, SoE */

    EC_T_WORD                   wAlStatusValue;     /* 0x36 */  /**< [out] AL Status Register Value (ESC register 0x0130) */
    EC_T_WORD                   wAlStatusCode;                  /**< [out] AL Status Code (ESC register 0x0134) */

    EC_T_BOOL                   bIsOptional;        /* 0x37 */  /**< [out] slave is in an optional hot connect group */
    EC_T_BOOL                   bIsPresent;         /* 0x38 */  /**< [out] slave is currently present on bus */

    EC_T_WORD                   wNumProcessVarsInp; /* 0x39 */  /**< [out] number of output process data variables*/
    EC_T_WORD                   wNumProcessVarsOutp;            /**< [out] number of input process data variables */

    EC_T_DWORD                  dwSlaveId;          /* 0x3A */  /**< [out] slave ID */
    EC_T_BOOL                   bIsHCGroupPresent;  /* 0x3B */  /**< [out] the hot connect group of the slave is present */

    EC_T_DWORD                  aPortSlaveIds[ESC_PORT_COUNT];  /* 0x3C */  /**< [out] slave IDs connected to ports */

    EC_T_DWORD                  dwSystemTimeDifference;         /* 0x40   < out System time difference (ESC register 0x092C) */

    EC_T_DWORD                  adwReserved[18];    /* 0x41 */  /**< [out] Reserved*/

} EC_PACKED(1)  EC_T_GET_SLAVE_INFO, *EC_PT_GET_SLAVE_INFO; /* Size = 0x53 * DWORD */
#include EC_PACKED_INCLUDESTOP

                                                            /* emGetSlaveInpVarInfo / emGetSlaveOutpVarInfo*/
#include EC_PACKED_INCLUDESTART(1)
typedef struct _EC_T_PROCESS_VAR_INFO
{
    EC_T_CHAR                   szName[MAX_PROCESS_VAR_NAME_LEN];   /**< [out] Name of the found process variable */
    EC_T_WORD                   wDataType;                          /**< [out] Data type of the found process variable (according to ETG.1000, section 5). See also EcCommon.h, DEFTYPE_BOOLEAN */
    EC_T_WORD                   wFixedAddr;                         /**< [out] Station address of the slave that is owner of this variable */
    EC_T_INT                    nBitSize;                           /**< [out] Size in bit of the found process variable */
    EC_T_INT                    nBitOffs;                           /**< [out] Bit offset in the process data image */
    EC_T_BOOL                   bIsInputData;                       /**< [out] Determines whether the found process variable is an input variable or an output variable */
} EC_PACKED(1) EC_T_PROCESS_VAR_INFO, *EC_PT_PROCESS_VAR_INFO;
#include EC_PACKED_INCLUDESTOP

#include EC_PACKED_INCLUDESTART(1)
typedef struct _EC_T_PROCESS_VAR_INFO_EX
{
    EC_T_CHAR                   szName[MAX_PROCESS_VAR_NAME_LEN_EX];    /**< [out] Name of the found process variable */
    EC_T_WORD                   wDataType;                              /**< [out] Data type of the found process variable */
    EC_T_WORD                   wFixedAddr;                             /**< [out] Station address of the slave that is owner of this variable */
    EC_T_INT                    nBitSize;                               /**< [out] Size in bit of the found process variable */
    EC_T_INT                    nBitOffs;                               /**< [out] Bit offset in the process data image */
    EC_T_BOOL                   bIsInputData;                           /**< [out] Determines whether the found process variable is an input variable or an output variable */
    EC_T_WORD                   wIndex;                                 /**< [out] Object index */
    EC_T_WORD                   wSubIndex;                              /**< [out] Object sub index */
    EC_T_WORD                   wPdoIndex;                              /**< [out] Index of PDO (process data object) */
    EC_T_WORD                   wWkcStateDiagOffs;                      /**< [out] Bit offset in the diagnostic image (emGetDiagnosisImagePtr) */
    EC_T_WORD                   wMasterSyncUnit;                        /**< [out] Master Sync Unit (ENI: RxPdo[1..4]@Su, TxPdo[1..4]@Su) */
    EC_T_WORD                   wRes1;
    EC_T_DWORD                  dwRes1;
} EC_PACKED(1) EC_T_PROCESS_VAR_INFO_EX, *EC_PT_PROCESS_VAR_INFO_EX;
#include EC_PACKED_INCLUDESTOP

/* EtherCAT CoE OD list type values */
typedef enum _EC_T_COE_ODLIST_TYPE
{
    eODListType_Lengths     = 0,                    /**< Lengths of each list type */
    eODListType_ALL         = 1,                    /**< List contains all objects */
    eODListType_RxPdoMap    = 2,                    /**< List with PDO mappable objects */
    eODListType_TxPdoMap    = 3,                    /**< List with objects that can be changed */
    eODListType_StoredFRepl = 4,                    /**< Only stored for a device replacement objects */
    eODListType_StartupParm = 5,                    /**< Only startup parameter objects */

    /* Borland C++ datatype alignment correction */
    eODListType_BCppDummy   = 0xFFFFFFFF
} EC_T_COE_ODLIST_TYPE;
#define CoeOdListTypeText(EType)                                \
    ((EType)==eODListType_Lengths?"Lengths":                    \
     ((EType)==eODListType_ALL?"All":                           \
      ((EType)==eODListType_RxPdoMap?"RxPDO":                   \
       ((EType)==eODListType_TxPdoMap?"TxPDO":                  \
        ((EType)==eODListType_StoredFRepl?"Device Replacement": \
         ((EType)==eODListType_StartupParm?"Startup Param":     \
         "Unknown"                                              \
    ))))))


/* EtherCAT CoE OD list */
#include EC_PACKED_INCLUDESTART(1)
typedef struct _EC_T_COE_ODLIST
{
    EC_T_COE_ODLIST_TYPE    eOdListType;            /**< list type */
    EC_T_WORD               wLen;                   /**< amount of object IDs */
    EC_T_WORD               wStationAddress;        /**< Station address of the slave */
    EC_T_WORD*              pwOdList;               /**< array containing object IDs */
} EC_PACKED(1) EC_T_COE_ODLIST;
#include EC_PACKED_INCLUDESTOP

/* EtherCAT CoE Object description */
#include EC_PACKED_INCLUDESTART(1)
typedef struct _EC_T_COE_OBDESC
{
    EC_T_WORD   wObIndex;                           /**< Index in the object dictionary */
    EC_T_WORD   wDataType;                          /**< Data type of the object */
    EC_T_BYTE   byObjCode;                          /**< Object code, see Table 62, ETG.1000 section 6 */
    EC_T_BYTE   byObjCategory;                      /**< Object category */
    EC_T_BYTE   byMaxNumSubIndex;                   /**< Maximum sub index number */
    EC_T_BYTE   byReserve;
    EC_T_WORD   wObNameLen;                         /**< Length of the object name */
    EC_T_WORD   wStationAddress;                    /**< Station address of the slave */
    EC_T_CHAR*  pchObName;                          /**< Object name (not NULL terminated!) */
} EC_PACKED(1) EC_T_COE_OBDESC;
#include EC_PACKED_INCLUDESTOP

/* EtherCAT CoE Object Entry description */
#include EC_PACKED_INCLUDESTART(1)
typedef struct _EC_T_COE_ENTRYDESC
{
    EC_T_WORD   wObIndex;                           /**< Index in the object dictionary */
    EC_T_BYTE   byObSubIndex;                       /**< Sub index in the object dictionary */
    EC_T_BYTE   byValueInfo;                        /**< Bit mask which information is included in pbyData. See \ref EC_COE_ENTRY_VALUEINFO "Value info flags" */
    EC_T_WORD   wDataType;                          /**< Object data type according to ETG.1000 */
    EC_T_WORD   wBitLen;                            /**< Object size (number of bits) */
    EC_T_BYTE   byObAccess;                         /**< Access rights. See \ref EC_COE_ENTRY_OBJACCESS "Object access flags" */
    EC_T_BYTE   byReserved[3];
    EC_T_BOOL   bRxPdoMapping;                      /**< Object is mappable in a RxPDO */
    EC_T_BOOL   bTxPdoMapping;                      /**< Object is mappable in a TxPDO */
    EC_T_BOOL   bObCanBeUsedForBackup;              /**< Object can be used for backup */
    EC_T_BOOL   bObCanBeUsedForSettings;            /**< Object can be used for settings */
    EC_T_WORD   wStationAddress;                    /**< Station address of the slave */
    EC_T_WORD   wDataLen;                           /**< Size of the remaining object data */
    EC_T_BYTE*  pbyData;                            /**< Remaining object data:
                                                        dwUnitType, pbyDefaultValue, pbyMinValue, pbyMaxValue, pbyDescription\n
                                                        (see ETG.1000.5 and ETG.1000.6)*/
} EC_PACKED(1) EC_T_COE_ENTRYDESC;
#include EC_PACKED_INCLUDESTOP

/* EtherCAT CoE emergency request */
#define EC_COE_EMERGENCY_DATASIZE 5
#include EC_PACKED_INCLUDESTART(1)
typedef struct _EC_T_COE_EMERGENCY
{
    EC_T_WORD   wErrorCode;                             /**< Error code according to EtherCAT specification */
    EC_T_BYTE   byErrorRegister;                        /**< Error register */
    EC_T_BYTE   abyData[EC_COE_EMERGENCY_DATASIZE];     /**< Error data */
    EC_T_WORD   wStationAddress;                        /**< Slave node address of the faulty slave*/
} EC_PACKED(1) EC_T_COE_EMERGENCY;
#include EC_PACKED_INCLUDESTOP

#include EC_PACKED_INCLUDESTART(1)
typedef struct _EC_T_MBX_DATA_COE_INITCMD
{
    EC_T_SLAVE_PROP SlaveProp;                          /**< Slave properties */
    EC_T_DWORD      dwHandle;                           /**< Handle passed by EC_IOCTL_ADD_COE_INITCMD, otherwise zero */
    EC_T_WORD       wTransition;                        /**< Transition, e.g. ECAT_INITCMD_I_P */
    EC_T_CHAR       szComment[MAX_STD_STRLEN];          /**< Comment (ENI) */
    EC_T_DWORD      dwErrorCode;                        /**< InitCmd result */
    EC_T_BOOL       bFixed;                             /**< Fixed flag (ENI) */
    EC_T_BYTE       byCcs;                              /**< Client command specifier (read or write access) */
    EC_T_BOOL       bCompleteAccess;                    /**< Complete access */
    EC_T_WORD       wIndex;                             /**< Object Index */
    EC_T_BYTE       bySubIndex;                         /**< Object SubIndex */
    EC_T_DWORD      dwDataLen;                          /**< InitCmd data length */
    EC_T_BYTE*      pbyData;                            /**< InitCmd data */
} EC_PACKED(1) EC_T_MBX_DATA_COE_INITCMD;
#include EC_PACKED_INCLUDESTOP

#include EC_PACKED_INCLUDESTART(8)
typedef struct _EC_T_MBX_DATA_COE
{
    EC_T_WORD       wStationAddress;                    /**< Station address of the slave */
    EC_T_WORD       wIndex;                             /**< Object index */
    EC_T_BYTE       bySubIndex;                         /**< Object subindex */
    EC_T_BOOL       bCompleteAccess;                    /**< Complete access */
} EC_PACKED(8)  EC_T_MBX_DATA_COE;
#include EC_PACKED_INCLUDESTOP

#include EC_PACKED_INCLUDESTART(1)
typedef struct _EC_T_MBX_DATA_FOE
{
    EC_T_DWORD  dwTransferredBytes;                         /**< [out] amount of transferred bytes */
    EC_T_DWORD  dwRequestedBytes;                           /**< [out] amount of bytes to be provided by application */

    EC_T_DWORD  dwBusyDone;                                 /**< [out] If slave is busy: 0 ... dwBusyEntire */
    EC_T_DWORD  dwBusyEntire;                               /**< [out] If dwBusyEntire > 0: Slave is busy */
#define EC_FOE_BUSY_COMMENT_SIZE    32
    EC_T_CHAR   szBusyComment[EC_FOE_BUSY_COMMENT_SIZE];    /**< [out] Busy Comment from slave */
    EC_T_DWORD  dwFileSize;                                 /**< [out] File size */
    EC_T_WORD   wStationAddress;                            /**< [out] Station address of the slave */
} EC_PACKED(1) EC_T_MBX_DATA_FOE;
#include EC_PACKED_INCLUDESTOP

#include EC_PACKED_INCLUDESTART(1)
typedef struct _EC_T_MBX_DATA_FOE_REQ
{
    EC_T_WORD   wStationAddress;                            /**< [out] Station address of the slave */
    EC_T_DWORD  dwPassword;                                 /**< [out] FoE read/write request password */
    EC_T_CHAR   szFileName[EC_MAX_FILE_NAME_SIZE];          /**< [out] Name of the file to be read/write */
} EC_PACKED(1) EC_T_MBX_DATA_FOE_REQ;
#include EC_PACKED_INCLUDESTOP

#include EC_PACKED_INCLUDESTART(1)
typedef struct _EC_T_MBX_DATA_SOE
{
    EC_T_BYTE   byElementFlags;
} EC_PACKED(1) EC_T_MBX_DATA_SOE;
#include EC_PACKED_INCLUDESTOP

/* EtherCAT SoE notification */
#define EC_SOE_NOTIFICATION_DATASIZE 5
#include EC_PACKED_INCLUDESTART(1)
typedef struct _EC_T_SOE_NOTIFICATION
{
    EC_T_WORD   wHeader;                                /**< SoE Header */
    EC_T_WORD   wIdn;                                   /**< IDN number */
    EC_T_BYTE   abyData[EC_SOE_NOTIFICATION_DATASIZE];  /**< Error data */
    EC_T_WORD   wStationAddress;                        /**< Station address of the slave */
} EC_PACKED(1) EC_T_SOE_NOTIFICATION;
#include EC_PACKED_INCLUDESTOP

/* EtherCAT SoE emergency request */
#define EC_SOE_EMERGENCY_DATASIZE 5
#include EC_PACKED_INCLUDESTART(1)
typedef struct _EC_T_SOE_EMERGENCY
{
    EC_T_WORD   wHeader;                                /**< SoE Header */
    EC_T_BYTE   abyData[EC_SOE_EMERGENCY_DATASIZE];     /**< Emergency error data */
    EC_T_WORD   wStationAddress;                        /**< Station address of the slave initiated the emergency request */
} EC_PACKED(1) EC_T_SOE_EMERGENCY;
#include EC_PACKED_INCLUDESTOP

/* AoE NetID */
#include EC_PACKED_INCLUDESTART(1)
typedef struct _EC_T_AOE_NETID
{
    EC_T_BYTE   aby[6];      /**< AoE net id */
} EC_PACKED(1) EC_T_AOE_NETID;
#define EC_T_AOE_NETID_SIZE         (6)
#include EC_PACKED_INCLUDESTOP

/* AoE mailbox response error codes */
#include EC_PACKED_INCLUDESTART(1)
typedef struct _EC_T_AOE_CMD_RESPONSE
{
    EC_T_DWORD   dwErrorCode;   /**< AoE response error code */
    EC_T_DWORD   dwCmdResult;   /**< AoE command result code */
    EC_T_DWORD   dwRsvd;
} EC_PACKED(1) EC_T_AOE_CMD_RESPONSE;
#include EC_PACKED_INCLUDESTOP

/* EtherCAT mailbox data */
#include EC_PACKED_INCLUDESTART(1)
typedef union _EC_T_MBX_DATA
{
    EC_T_AOE_CMD_RESPONSE   AoE_Response;               /**< AoE */
    EC_T_MBX_DATA_COE       CoE;                        /**< CoE */
    EC_T_COE_ODLIST         CoE_ODList;                 /**< CoE Object Dictionary list */
    EC_T_COE_OBDESC         CoE_ObDesc;                 /**< CoE object description */
    EC_T_COE_ENTRYDESC      CoE_EntryDesc;              /**< CoE entry description */
    EC_T_COE_EMERGENCY      CoE_Emergency;              /**< CoE emergency data */
    EC_T_MBX_DATA_COE_INITCMD CoE_InitCmd;              /**< CoE InitCmd */
    EC_T_MBX_DATA_FOE       FoE;                        /**< FoE */
    EC_T_MBX_DATA_FOE_REQ   FoE_Request;                /**< FoE request */
    EC_T_MBX_DATA_SOE       SoE;                        /**< SoE */
    EC_T_SOE_NOTIFICATION   SoE_Notification;           /**< SoE notification request */
    EC_T_SOE_EMERGENCY      SoE_Emergency;              /**< SoE emergency request */
} EC_PACKED(1) EC_T_MBX_DATA;
#include EC_PACKED_INCLUDESTOP

/* EtherCAT mailbox type values */
typedef enum _EC_T_MBXTFER_TYPE
{
    eMbxTferType_COE_SDO_DOWNLOAD   = 0,                /**< CoE SDO download */
    eMbxTferType_COE_SDO_UPLOAD     = 1,                /**< CoE SDO upload */
    eMbxTferType_COE_GETODLIST      = 2,                /**< CoE Get object dictionary list */
    eMbxTferType_COE_GETOBDESC      = 3,                /**< CoE Get object description */
    eMbxTferType_COE_GETENTRYDESC   = 4,                /**< CoE Get object entry description */
    eMbxTferType_COE_EMERGENCY      = 5,                /**< CoE emergency request */
    eMbxTferType_COE_RX_PDO         = 6,                /**< CoE RxPDO */
    eMbxTferType_FOE_FILE_UPLOAD    = 7,                /**< FoE upload */
    eMbxTferType_FOE_FILE_DOWNLOAD  = 8,                /**< FoE download */
    eMbxTferType_SOE_READREQUEST    = 9,                /**< SoE read request */
    eMbxTferType_SOE_READRESPONSE   = 10,               /**< SoE read response */
    eMbxTferType_SOE_WRITEREQUEST   = 11,               /**< SoE write request */
    eMbxTferType_SOE_WRITERESPONSE  = 12,               /**< SoE write response */
    eMbxTferType_SOE_NOTIFICATION   = 13,               /**< SoE notification */
    eMbxTferType_SOE_EMERGENCY      = 14,               /**< SoE emergency */
    eMbxTferType_VOE_MBX_READ       = 15,               /**< VoE read */
    eMbxTferType_VOE_MBX_WRITE      = 16,               /**< VoE write */
    eMbxTferType_AOE_READ           = 17,               /**< AoE read */
    eMbxTferType_AOE_WRITE          = 18,               /**< AoE write */
    eMbxTferType_AOE_READWRITE      = 19,               /**< AoE read/write */
    eMbxTferType_AOE_WRITECONTROL   = 20,               /**< AoE write control */
    eMbxTferType_RAWMBX             = 21,               /**< Raw mbx */
    eMbxTferType_FOE_SEG_DOWNLOAD   = 22,               /**< FoE segmented download */
    eMbxTferType_FOE_SEG_UPLOAD     = 23,               /**< FoE segmented upload */
    eMbxTferType_S2SMBX             = 24,               /**< S2S mbx */
    eMbxTferType_FOE_UPLOAD_REQ     = 25,               /**< FoE upload request */
    eMbxTferType_FOE_DOWNLOAD_REQ   = 26,               /**< FoE download request */

    /* Borland C++ datatype alignment correction */
    eMbxTferType_BCppDummy          = 0xFFFFFFFF
} EC_T_MBXTFER_TYPE;

static EC_INLINESTART const EC_T_CHAR* MbxTferTypeText(EC_T_MBXTFER_TYPE EType)
{
    switch (EType)
    {
    case eMbxTferType_COE_SDO_DOWNLOAD:     return "CoE SDO download";
    case eMbxTferType_COE_SDO_UPLOAD:       return "CoE SDO upload";
    case eMbxTferType_COE_GETODLIST:        return "CoE Get object dictionary list";
    case eMbxTferType_COE_GETOBDESC:        return "CoE Get object description";
    case eMbxTferType_COE_GETENTRYDESC:     return "CoE Get object entry description";
    case eMbxTferType_COE_EMERGENCY:        return "CoE emergency request";
    case eMbxTferType_COE_RX_PDO:           return "CoE RxPDO";
    case eMbxTferType_FOE_FILE_UPLOAD:      return "FoE upload";
    case eMbxTferType_FOE_FILE_DOWNLOAD:    return "FoE download";
    case eMbxTferType_SOE_READREQUEST:      return "SoE read request";
    case eMbxTferType_SOE_READRESPONSE:     return "SoE read response";
    case eMbxTferType_SOE_WRITEREQUEST:     return "SoE write request";
    case eMbxTferType_SOE_WRITERESPONSE:    return "SoE write response";
    case eMbxTferType_SOE_NOTIFICATION:     return "SoE notification";
    case eMbxTferType_SOE_EMERGENCY:        return "SoE emergency";
    case eMbxTferType_VOE_MBX_READ:         return "VoE read";
    case eMbxTferType_VOE_MBX_WRITE:        return "VoE write";
    case eMbxTferType_AOE_READ:             return "AoE read";
    case eMbxTferType_AOE_WRITE:            return "AoE write";
    case eMbxTferType_AOE_READWRITE:        return "AoE read/write";
    case eMbxTferType_AOE_WRITECONTROL:     return "AoE write control";
    case eMbxTferType_RAWMBX:               return "Raw mbx";
    case eMbxTferType_FOE_SEG_DOWNLOAD:     return "FoE segmented download";
    case eMbxTferType_FOE_SEG_UPLOAD:       return "FoE segmented upload";
    case eMbxTferType_S2SMBX:               return "S2S mbx";
    case eMbxTferType_FOE_UPLOAD_REQ:       return "FoE upload request";
    case eMbxTferType_FOE_DOWNLOAD_REQ:     return "FoE download request";
    case eMbxTferType_BCppDummy:            return "Dummy EC_T_MBXTFER_TYPE";
    }
    return "Unknown EC_T_MBXTFER_TYPE";
} EC_INLINESTOP

/* EtherCAT mailbox transfer descriptor */
#include EC_PACKED_INCLUDESTART(1)
typedef struct _EC_T_MBXTFER_DESC
{
    EC_T_DWORD          dwMaxDataLen;               /**< Maximum amount of data bytes that shall be transferred using this object. A mailbox transfer type without data transfer will ignore this parameter */
    EC_T_BYTE*          pbyMbxTferDescData;         /**< Pointer to byte stream carrying in and out data of mailbox content */
} EC_PACKED(1) EC_T_MBXTFER_DESC;
#include EC_PACKED_INCLUDESTOP

/* EtherCAT mailbox status values */
typedef enum _EC_T_MBXTFER_STATUS
{
    eMbxTferStatus_Idle                     = 0,    /**< Mailbox transfer object not in use */
    eMbxTferStatus_Pend                     = 1,    /**< Mailbox transfer in process */
    eMbxTferStatus_TferDone                 = 2,    /**< Mailbox transfer completed */
    eMbxTferStatus_TferReqError             = 3,    /**< Mailbox transfer request error */
    eMbxTferStatus_TferWaitingForContinue   = 4,    /**< Mailbox transfer waiting for continue, object owned by application */

    /* Borland C++ datatype alignment correction */
    eMbxTferStatus_BCppDummy    = 0xFFFFFFFF
} EC_T_MBXTFER_STATUS;
#define MbxTferStatusText(EStatus)                                                  \
    ((EStatus)==eMbxTferStatus_Idle?"Idle":                                         \
     ((EStatus)==eMbxTferStatus_Pend?"Pend":                                        \
      ((EStatus)==eMbxTferStatus_TferDone?"TferDone":                               \
       ((EStatus)==eMbxTferStatus_TferReqError?"TferReqError":                      \
        ((EStatus)==eMbxTferStatus_TferWaitingForContinue?"TferWaitingForContinue": \
          "Unknown")))))

/* EtherCAT mailbox transfer object */
#include EC_PACKED_INCLUDESTART(1)
typedef struct _EC_T_MBXTFER
{
    EC_T_DWORD          dwClntId;                   /**< [] Client ID */
    EC_T_MBXTFER_DESC   MbxTferDesc;                /**< [out] Mailbox transfer descriptor. All elements of pMbxTferDesc will be stored here */
    EC_T_MBXTFER_TYPE   eMbxTferType;               /**< [] This type information is written to the Mailbox Transfer Object by the last call to a mailbox command function. It may be used as an information, and is required to fan out consecutive notifications. This value is only valid until next mailbox relevant API call, where this value may be overwritten */
    EC_T_DWORD          dwDataLen;                  /**< [] Amount of data bytes for the next mailbox transfer. If the mailbox transfer does not transfer data from or to the slave this parameter will be ignored. This element has to be set to an appropriate value every time prior to initiate a new request. When the transfer is completed (emNotify) this value will contain the amount of data that was actually transferred */
    EC_T_BYTE*          pbyMbxTferData;             /**< [in/out] Pointer to data. In case of a download transfer the client has to store the data in this location. In case of an upload transfer this element points to the received data. Access to data that was uploaded from a slave is only valid within the notification function because the buffer will be re-used by the master "this data has to be copied into a separate buffer in case it has to be used later by the client */
    EC_T_MBXTFER_STATUS eTferStatus;                /**< [out] Transfer state. After a new transfer object is created the state will be set to eMbxTferStatus_Idle */
    EC_T_DWORD          dwErrorCode;                /**< [out] Error code of a mailbox transfer that was terminated with error */
    EC_T_DWORD          dwTferId;                   /**< [] Transfer ID. For every new mailbox transfer a unique ID has to be assigned. This ID can be used after mailbox transfer completion to identify the transfer */
    EC_T_MBX_DATA       MbxData;                    /**< [] Mailbox data. This element contains mailbox transfer data, e.g. the CoE object dictionary list. */
} EC_PACKED(1) EC_T_MBXTFER;
#include EC_PACKED_INCLUDESTOP

/* Supported EtherCAT commands for the ecatTferSingleRawCmd() function */
typedef enum _EC_T_RAWCMD
{
    eRawCmd_APRD    = EC_CMD_TYPE_APRD,             /**< Auto-Increment physical read */
    eRawCmd_APWR    = EC_CMD_TYPE_APWR,             /**< Auto-Increment physical write */
    eRawCmd_APRW    = EC_CMD_TYPE_APRW,             /**< Auto-Increment physical read/write */
    eRawCmd_BRD     = EC_CMD_TYPE_BRD,              /**< Broadcast (wire-or'ed) read */
    eRawCmd_BWR     = EC_CMD_TYPE_BWR,              /**< Broadcast write */
    eRawCmd_BRW     = EC_CMD_TYPE_BRW,              /**< Broadcast read/write */
    eRawCmd_LRD     = EC_CMD_TYPE_LRD,              /**< Logical read */
    eRawCmd_LWR     = EC_CMD_TYPE_LWR,              /**< Logical write */
    eRawCmd_LRW     = EC_CMD_TYPE_LRW,              /**< Logical read/write */
    eRawCmd_ARMW    = EC_CMD_TYPE_ARMW,             /**< Auto-increment physical read, multiple write */
    eRawCmd_FPRD    = EC_CMD_TYPE_FPRD,             /**< Fixed address physical read */
    eRawCmd_FPWR    = EC_CMD_TYPE_FPWR,             /**< Fixed address physical write */
    eRawCmd_FPRW    = EC_CMD_TYPE_FPRW,             /**< Fixed address physical read/write */

                                                    /* Borland C++ datatype alignment correction */
                                                    eRawCmd_BCppDummy   = 0xFFFFFFFF
} EC_T_RAWCMD;

/* Descriptor for EC_IOCTL_GET_PDMEMORYSIZE call */
#include EC_PACKED_INCLUDESTART(1)
typedef struct _EC_T_MEMREQ_DESC
{
    EC_T_DWORD  dwPDOutSize;                        /* Size of the output process data image */
    EC_T_DWORD  dwPDInSize;                         /* Size of the input  process data image */
} EC_PACKED(1) EC_T_MEMREQ_DESC, *EC_PT_MEMREQ_DESC;
#include EC_PACKED_INCLUDESTOP

/* Descriptor for EC_IOCTL_SET_MASTER_DEFAULT_TIMEOUTS */
#include EC_PACKED_INCLUDESTART(1)
typedef struct _EC_T_MASTERDEFAULTTIMEOUTS_DESC
{
    EC_T_DWORD  dwMasterStateChange;                /**< Default state change timeout [ms], applied if emSetMasterState called with EC_NOWAIT. */
    EC_T_DWORD  dwInitCmdRetry;                     /**< Timeout [ms] between retry sending an init-command. */
    EC_T_DWORD  dwMbxCmd;                           /**< Timeout [ms] between retry sending an mailbox command */
    EC_T_DWORD  dwMbxPolling;                       /**< Mailbox polling cycle [ms] */
    EC_T_DWORD  dwDcmInSync;                        /**< Timeout [ms] to wait for DCM InSync in state change PREOP to SAFEOP */
    EC_T_WORD   wInitCmd;                           /**< Timeout [ms] to InitCmds if not specified in ENI */
    EC_T_WORD   wReserved;
    EC_T_DWORD  dwSlaveIdentification;              /**< Timeout [ms] to wait for the reading of the slave identification */
    EC_T_DWORD  dwReserved[9];
} EC_PACKED(1) EC_T_MASTERDEFAULTTIMEOUTS_DESC;
#include EC_PACKED_INCLUDESTOP

/* Descriptor for EC_IOCTL_HC_CONFIGURETIMEOUTS call */
#include EC_PACKED_INCLUDESTART(1)
typedef struct _EC_T_HC_CONFIGURETIMEOUTS_DESC
{
    EC_T_DWORD  dwDetectionTimeout;                 /* [in] Timeout for Group detection */
    EC_T_DWORD  dwTopologyChangeTimeout;            /* [in] Topology Change Timeout (usually larger than dwDetectionTimeout) */
    EC_T_DWORD  dwRsvd[2];
} EC_PACKED(1) EC_T_HC_CONFIGURETIMEOUTS_DESC;
#include EC_PACKED_INCLUDESTOP

/* EtherCAT user controlled execution job */
typedef enum _EC_T_USER_JOB
{
    eUsrJob_Undefined                       = 0,
    eUsrJob_ProcessAllRxFrames              = 1,    /**< Receive frames and process all received data. Polling mode only. */
    eUsrJob_SendAllCycFrames                = 2,    /**< Send all cyclic frames */
    eUsrJob_RunMcSm                         = 3,    /**< obsolete */
    eUsrJob_MasterTimer                     = 4,    /**< Run internal master and slave state machines for generic management */
    eUsrJob_FlushQueuedCmds                 = 5,    /**< obsolete */
    eUsrJob_SendAcycFrames                  = 6,    /**< Send acyclic frames */
    eUsrJob_SendCycFramesByTaskId           = 7,    /**< Send cyclic frames related to a specific task id (TaskId entry in the XML file) */
    eUsrJob_MasterTimerMinimal              = 8,    /**< Run minimal master timer routine: no state change possible  */
    eUsrJob_ProcessRxFramesByTaskId         = 9,    /**< Receive frames and process received data related to a specific task id (TaskId entry in the XML file) */
    eUsrJob_ProcessAcycRxFrames             = 10,   /**< Receive frames and process received data related to acyclic frames */
    eUsrJob_SwitchEoeFrames                 = 11,   /**< Switch queued EoE frames (see EC_IOCTL_SET_EOE_DEFFERED_SWITCHING_ENABLED) */
    eUsrJob_StartTask                       = 12,   /**< Start new task */
    eUsrJob_StopTask                        = 13,   /**< Stop currently running task */

    eUsrJob_StampSendAllCycFrames           = 22,   /**< obsolete */
    eUsrJob_StampSendCycFramesByTaskId      = 27,   /**< obsolete */

    eUsrJob_SimulatorTimer                  = 32,   /**< Run EC-Simulator timer routine (generic management) */
    eUsrJob_MonitorTimer                    = 33,   /**< Run EC-Monitor timer routine (generic management) */

    /* Borland C++ datatype alignment correction */
    eUsrJob_BCppDummy                       = 0xFFFFFFFF
} EC_T_USER_JOB;

#define EC_USER_JOB_COUNT 32

/* Performance measurement */
/**
 * \typedef EC_PF_PERF_MEAS_GETCOUNTERTICKS
 * \param pvContext[in] Context pointer
 */
typedef EC_T_UINT64(EC_FNCALL *EC_PF_PERF_MEAS_GETCOUNTERTICKS) (EC_T_VOID* pvContext);

/* call emPerfMeas... Api for all benchmarks */
#define EC_PERF_MEAS_ALL 0xFFFFFFFF

#include EC_PACKED_API_INCLUDESTART
typedef struct _EC_T_PERF_MEAS_INFO
{
    EC_T_CHAR     szName[MAX_STD_STRLEN]; /**< Name of the benchmark */
    EC_T_UINT64   qwFrequency;            /**< Frequency in Hz used by the timer */
    EC_T_USER_JOB eUserJob;               /**< UserJob associated with the benchmark */
    EC_T_DWORD    dwBinCountHistogram;    /**< length of Histogram Bins */
    EC_T_DWORD    dwFlags;                /**< Flags associated with the benchmark (See EC_T_PERF_MEAS_FLAG...) */
    EC_T_DWORD    dwReserved[4];
} EC_PACKED_API EC_T_PERF_MEAS_INFO;
#include EC_PACKED_INCLUDESTOP

#include EC_PACKED_API_INCLUDESTART
typedef struct _EC_T_PERF_MEAS_HISTOGRAM
{
    EC_T_DWORD* aBins;           /**< Histogram Bins:\n
                                      The first bin is used for times below `dwMinTicks`.
                                      The last  bin is used for times above `dwMaxTicks`.
                                      All other times are stored in `dwBinCount - 2` bins of equal size. */
    EC_T_DWORD  dwBinCount;      /**< length of aBins */
    EC_T_UINT64 qwMinTicks;      /**< results below qwMinTicks are stored in the first bin */
    EC_T_UINT64 qwMaxTicks;      /**< results above qwMaxTicks are stored in the last bin */
    EC_T_DWORD  dwReserved[4];
} EC_PACKED_API EC_T_PERF_MEAS_HISTOGRAM;
#include EC_PACKED_INCLUDESTOP

#include EC_PACKED_API_INCLUDESTART
typedef struct _EC_T_PERF_MEAS_VAL
{
    EC_T_UINT64      qwCurrTicks; /**< [ticks] */
    EC_T_UINT64      qwMinTicks;  /**< [ticks] */
    EC_T_UINT64      qwMaxTicks;  /**< [ticks] */
    EC_T_UINT64      qwAvgTicks;  /**< [ticks] */
    EC_T_DWORD       dwReserved[4];
} EC_PACKED_API EC_T_PERF_MEAS_VAL;
#include EC_PACKED_INCLUDESTOP

#include EC_PACKED_API_INCLUDESTART
typedef struct _EC_T_PERF_MEAS_COUNTER_PARMS
{
    EC_PF_PERF_MEAS_GETCOUNTERTICKS pfGetCounterTicks;        /**< [in]  Function returning the current counter ticks */
    EC_T_VOID*                      pvGetCounterTicksContext; /**< [in]  Context passed into GetCounterTicks */
    EC_T_UINT64                     qwFrequency;              /**< [in]  Frequency in Hz used by the timer in GetCounterTicks */
} EC_PACKED_API EC_T_PERF_MEAS_COUNTER_PARMS;
#include EC_PACKED_INCLUDESTOP

#include EC_PACKED_API_INCLUDESTART
typedef struct _EC_T_PERF_MEAS_HISTOGRAM_PARMS
{
    EC_T_DWORD  dwBinCount;  /**< [in]  amount of bins to use for the histogram. */
    EC_T_UINT64 qwMinTicks;  /**< [in]  results below qwMinTicks are stored in the first bin */
    EC_T_UINT64 qwMaxTicks;  /**< [in]  results above qwMaxTicks are stored in the last bin */
} EC_PACKED_API  EC_T_PERF_MEAS_HISTOGRAM_PARMS;
#include EC_PACKED_INCLUDESTOP

#include EC_PACKED_API_INCLUDESTART
typedef struct _EC_T_PERF_MEAS_INFO_PARMS
{
    EC_T_CHAR   szName[MAX_STD_STRLEN]; /**< [in]  performance counter name */
    EC_T_DWORD  dwFlags;                /**< [in]  Flags associated with the benchmark (See EC_T_PERF_MEAS_FLAG...) */
    EC_T_DWORD  dwReserved[4];
} EC_PACKED_API  EC_T_PERF_MEAS_INFO_PARMS;
#include EC_PACKED_INCLUDESTOP

/*********************************************************************/
/** \defgroup EC_T_PERF_MEAS_FLAG Flags for benchmarks
@{*/
/**
 * distance benchmarks are used to measure the time between the cycle start and the benchmark start.
 * This can be helpful when visualizing the benchmarks inside a cycle
 */
#define EC_T_PERF_MEAS_FLAG_OFFSET   ((EC_T_WORD)((EC_T_DWORD)1 << 0))
/**
 * Changes the default of qwMinTicks/qwMaxTicks selected when passing
 * qwMinTicks=qwMinTicks=0 from `0 - cycle time` to `0.5 * cycle time - 1.5 * cycle time`
 */
#define EC_T_PERF_MEAS_FLAG_LONG_TIMER ((EC_T_WORD)((EC_T_DWORD)1 << 1))
/**@}*/

#include EC_PACKED_API_INCLUDESTART
typedef struct _EC_T_PERF_MEAS_INTERNAL_PARMS
{
    EC_T_BOOL                       bEnabled;        /**< [in]  enable/disable internal performance counters. */
    EC_T_PERF_MEAS_COUNTER_PARMS    CounterParms;    /**< [in]  Timer function settings. When not provided OsMeasGetCounterTicks is used  */
    EC_T_PERF_MEAS_HISTOGRAM_PARMS  HistogramParms;  /**< [in]  Histogram settings. When not provided the histogram is disabled. */
} EC_PACKED_API  EC_T_PERF_MEAS_INTERNAL_PARMS;
#include EC_PACKED_INCLUDESTOP

#include EC_PACKED_API_INCLUDESTART
typedef struct _EC_T_PERF_MEAS_APP_PARMS
{
    EC_T_DWORD                      dwNumMeas;      /**< [in]  Number of performance counters to create */
    EC_T_PERF_MEAS_INFO_PARMS*      aPerfMeasInfos; /**< [in] PerfMeasInfos associated with the corresponding benchmark */
    EC_T_PERF_MEAS_COUNTER_PARMS    CounterParms;   /**< [in]  Timer function settings. When not provided OsMeasGetCounterTicks is used  */
    EC_T_PERF_MEAS_HISTOGRAM_PARMS  HistogramParms; /**< [in]  Histogram settings. When not provided the histogram is disabled. */
    EC_T_DWORD                      dwReserved[4];
} EC_PACKED_API  EC_T_PERF_MEAS_APP_PARMS;
#include EC_PACKED_INCLUDESTOP

/**
 * \typedef EC_PF_CYCFRAME_RECV
 * \param dwTaskId     [in] Task id of the received cyclic frame.
 * \param pvContext    [in] Context pointer. This pointer is used as parameter every time when the callback function is called.
 */
typedef EC_T_VOID (*EC_PF_CYCFRAME_RECV)(EC_T_DWORD dwTaskId, EC_T_VOID* pvContext);
#include EC_PACKED_INCLUDESTART(1)
typedef struct _EC_T_CYCFRAME_RX_CBDESC
{
    EC_T_VOID*                  pCallbackContext;   /**< [in]  Context pointer. This pointer is used as parameter every time when the callback function is called */
    EC_PF_CYCFRAME_RECV         pfnCallback;        /**< [in]  This function will be called after the cyclic frame is received, if there is more than one cyclic frame after the last frame. The application has to assure that these functions will not block. */
} EC_PACKED(1)  EC_T_CYCFRAME_RX_CBDESC;
#include EC_PACKED_INCLUDESTOP

#include EC_PACKED_API_INCLUDESTART
typedef union _EC_T_USER_JOB_PARMS
{
    EC_T_BOOL       bAllCycFramesProcessed;     /**< [out] Indicates whether all previously initiated cyclic frames are received and processed within this call */
    EC_T_DWORD      dwNumFramesSent;            /**< [out] Indicates number of frames send within this call */
    EC_T_DWORD      dwTaskIdToSend;             /**< [in]  Task ID of the cycle whose frames are to be sent (obsolete use SendCycFramesByTaskId instead) */
    struct _SEND_CYCFRAME_BY_TASKID
    {
        EC_T_DWORD  dwTaskId;                   /**< [in]  Task ID of the cycle whose frames are to be sent */
    } EC_PACKED_API SendCycFramesByTaskId;
    struct _PROCESS_RXFRAME_BY_TASKID
    {
        EC_T_BOOL   bCycFramesProcessed;        /**< [out] Indicates whether all previously initiated cyclic frames of a specific cyclic task are received and processed */
        EC_T_DWORD  dwTaskId;                   /**< [in]  Task ID of the cycle whose frames are to be processed */
    } EC_PACKED_API ProcessRxFramesByTaskId;
    struct _SWITCH_EOE_FRAMES
    {
        EC_T_DWORD  dwMaxPortsToProcess;        /**< [in]  Maximum number of EoE ports to be processed */
        EC_T_DWORD  dwNumFramesProcessed;       /**< [out] Number of frames processed */
    } EC_PACKED_API SwitchEoeFrames;
    struct _START_TASK
    {
        EC_T_DWORD  dwTaskId;                   /**< [in]  Task ID of the task to start */
    } EC_PACKED_API StartTask;
    struct _STOP_TASK
    {
        EC_T_DWORD  dwTaskId;                   /**< [in]  Task ID of the task to stop */
    } EC_PACKED_API StopTask;
} EC_PACKED_API EC_T_USER_JOB_PARMS;
#include EC_PACKED_INCLUDESTOP

typedef enum _EC_T_COE_INITCMD_CALLBACK_RESULT
{
    eCoeInitCmdCallbackResult_Skip = 0,
    eCoeInitCmdCallbackResult_Send = 1,
    eCoeInitCmdCallbackResult_Busy = 2,

    /* Borland C++ datatype alignment correction */
    eCoeInitCmdCallbackResult_BCppDummy = 0xFFFFFFFF
} EC_T_COE_INITCMD_CALLBACK_RESULT;

typedef EC_T_COE_INITCMD_CALLBACK_RESULT (*EC_PF_COE_INITCMD_CALLBACK)(EC_T_VOID* pvParm, EC_T_DWORD dwSlaveId, EC_T_DWORD dwHandle, EC_T_WORD wIndex, EC_T_BYTE bySubIndex, EC_T_DWORD wDataLen, EC_T_BYTE* pbyData);

#include EC_PACKED_API_INCLUDESTART
typedef struct _EC_T_ADD_COE_INITCMD_DESC_ENTRY
{
    EC_T_DWORD      dwHandle;                           /**< 32Bit handle passed to registered pfnCallback */
    EC_T_WORD       wTransition;                        /**< transition e.g. ECAT_INITCMD_I_P */
    EC_T_CHAR       szComment[MAX_STD_STRLEN];          /**< comment (ENI) */
    EC_T_WORD       wTimeout;                           /**< timeout */
    EC_T_BOOL       bIgnoreFailure;                     /**< continue to process InitCmd on error */
    EC_T_BYTE       byCcs;                              /**< client command specifier (read or write access) */
    EC_T_BOOL       bCompleteAccess;                    /**< complete access */
    EC_T_WORD       wIndex;                             /**< object index */
    EC_T_BYTE       bySubIndex;                         /**< object subindex */
    EC_T_DWORD      wDataLen;                           /**< length of the data */
    EC_T_BYTE*      pbyData;
} EC_PACKED_API  EC_T_ADD_COE_INITCMD_DESC_ENTRY;
#include EC_PACKED_INCLUDESTOP

#include EC_PACKED_API_INCLUDESTART
typedef struct _EC_T_ADD_COE_INITCMD_DESC
{
    EC_T_DWORD dwSlaveId;
    EC_T_WORD  wCount;
    EC_T_ADD_COE_INITCMD_DESC_ENTRY* pbCoeInitCmds;
    EC_T_VOID* pvCallbackParm;
    EC_PF_COE_INITCMD_CALLBACK pfnCallback;
} EC_PACKED_API  EC_T_ADD_COE_INITCMD_DESC;
#include EC_PACKED_INCLUDESTOP

typedef enum _EC_T_CYCFRAME_LAYOUT
{
    eCycFrameLayout_STANDARD    = 0,    /**< Layout according ENI with command add/reordering, no relationship to PD */
    eCycFrameLayout_DYNAMIC     = 1,    /**< Layout is dynamically modified to send as less as possible cyclic frames and commands */
    eCycFrameLayout_FIXED       = 2,    /**< Layout strictly match ENI, frame buffers and PD area overlapped */
    eCycFrameLayout_IN_DMA      = 3,    /**< Layout strictly match ENI, frame buffers and PD area overlapped, frame buffers in DMA */

    eCycFrameLayout_BCppDummy   = 0xFFFFFFFF
} EC_T_CYCFRAME_LAYOUT;

/* pass through server states */
typedef enum _EC_PTS_STATE
{
    ePtsStateNone                  = 0x0000,
    ePtsStateNotRunning            = 0x0001,
    ePtsStateRunningDisabled       = 0x0002,
    ePtsStateRunningEnabled        = 0x0003,

    /* Borland C++ datatype alignment correction */
    ePtsStateDummy                 = 0xFFFFFFFF
} EC_PTS_STATE;


/* Start parameter for the Pass-Through-Server */
#include EC_PACKED_INCLUDESTART(1)
typedef struct _EC_T_PTS_SRV_START_PARMS
{
    EC_T_IPADDR oIpAddr;
    EC_T_DWORD  dwPtsThreadPriority;
    EC_T_WORD   wPort;
    EC_T_WORD   wReserved;
} EC_PACKED(1)  EC_T_PTS_SRV_START_PARMS;
#include EC_PACKED_INCLUDESTOP

/** \defgroup EC_SET_NOTIFICATION_ENABLED
@{ */
#define EC_NOTIFICATION_DISABLED    (0)             /**< Disable notification */
#define EC_NOTIFICATION_ENABLED     (1)             /**< Enable notification */
#define EC_NOTIFICATION_DEFAULT     (2)             /**< Reset notification to default */
#define EC_ALL_NOTIFICATIONS        (0xffffffff)    /**< Notification code to change all notifications */
/**@}*/
#include EC_PACKED_INCLUDESTART(1)
typedef struct _EC_T_SET_NOTIFICATION_ENABLED_PARMS
{
    EC_T_DWORD dwClientId;          /**< [in] Client ID, 0: Master */
    EC_T_DWORD dwCode;              /**< [in] Notification code */
    EC_T_DWORD dwEnabled;           /**< [in] Enable, disable or reset to default notification */
} EC_PACKED(1) EC_T_SET_NOTIFICATION_ENABLED_PARMS;
#include EC_PACKED_INCLUDESTOP

#include EC_PACKED_INCLUDESTART(1)
typedef struct _EC_T_GET_NOTIFICATION_ENABLED_PARMS
{
    EC_T_DWORD dwClientId;          /**< [in] Client ID, 0: Master */
    EC_T_DWORD dwCode;              /**< [in] Notification code */
} EC_PACKED(1) EC_T_GET_NOTIFICATION_ENABLED_PARMS;
#include EC_PACKED_INCLUDESTOP

#include EC_PACKED_API_INCLUDESTART
typedef struct _EC_T_CLEAR_MASTER_INFO_COUNTERS_PARMS
{
    EC_T_DWORD  dwClearBusDiagnosisCounters;        /**< [in] Bit 0..7: Clear corresponding Counter ID:
                                                        - Bit 0: Clear all Counters
                                                        - Bit 1: Clear Tx Frame Counter
                                                        - Bit 2: Clear Rx Frame Counter
                                                        - Bit 3: Clear Lost Frame Counter
                                                        - Bit 4: Clear Cyclic Frame Counter
                                                        - Bit 5: Clear Cyclic Datagram Counter
                                                        - Bit 6: Clear Acyclic Frame Counter
                                                        - Bit 7: Clear Acyclic DataGram Counter 
                                                        - Bit 8: Clear Cyclic Lost Frame Counter 
                                                        - Bit 9: Clear Acyclic Lost Frame Counter */
    EC_T_UINT64 qwMailboxStatisticsClearCounters;   /**< [in]  Bit 0..56: Clear corresponding Counter ID.
                                                        - Bit 0..7: Clear AoE statistics
                                                            - Bit 0: Total Read Transfer Count
                                                            - Bit 1: Read Transfer Count Last Second
                                                            - Bit 2: Total Bytes Read
                                                            - Bit 3: Bytes Read Last Second
                                                            - Bit 4: Total Write Transfer Count
                                                            - Bit 5: Write Transfer Count Last Second
                                                            - Bit 6: Total Bytes Write
                                                            - Bit 7: Bytes Write Last Second
                                                        - Bit 8..15: Clear CoE statistics (same ordering as Bit 0..7, AoE)
                                                        - Bit 16..23: Clear EoE statistics (same ordering as Bit 0..7, AoE)
                                                        - Bit 24..31: Clear FoE statistics (same ordering as Bit 0..7, AoE)
                                                        - Bit 32..39: Clear SoE statistics (same ordering as Bit 0..7, AoE)
                                                        - Bit 40..47: Clear VoE statistics (same ordering as Bit 0..7, AoE)
                                                        - Bit 48..55: Clear RawMbx statistics (same ordering as Bit 0..7, AoE)*/
    EC_T_DWORD  dwReserved[6];
} EC_PACKED_API EC_T_CLEAR_MASTER_INFO_COUNTERS_PARMS;
#include EC_PACKED_INCLUDESTOP

#include EC_PACKED_INCLUDESTART(1)
typedef struct _EC_T_MASTEROD_OBJECT_PARMS
{
    EC_T_WORD  wIndex;      /**< Object's index, e.g. 0x1018 */
    EC_T_BYTE  bySubindex;  /**< Object's sub-index, e.g. 1 */
    EC_T_BYTE  byReserved;
    EC_T_BYTE* pbyData;     /**< Pointer to object's data to be written */
    EC_T_DWORD dwLength;    /**< Data length to be written */
    EC_T_DWORD dwReserved;
} EC_PACKED(1) EC_T_MASTEROD_OBJECT_PARMS;
#include EC_PACKED_INCLUDESTOP

/* Start parameters for ADS Adapter */
#define ATEM_ADS_ADAPTER_START_PARMS_SIGNATURE_PATTERN                    0xDF300000
#define ATEM_ADS_ADAPTER_START_PARMS_SIGNATURE (  ATEM_ADS_ADAPTER_START_PARMS_SIGNATURE_PATTERN       \
                         | (ATECAT_VERS_MAJ         << 16) \
                         | (ATECAT_VERS_MIN         << 12) \
                         | (ATECAT_VERS_SERVICEPACK <<  8) \
                         | (ATECAT_VERS_BUILD       <<  0) \
                         )
#include EC_PACKED_INCLUDESTART(1)
typedef struct _EC_T_ADS_ADAPTER_START_PARMS
{
    EC_T_DWORD      dwSignature;         /**< [in]   Set to ATEM_ADS_ADAPTER_START_PARMS_SIGNATURE */
    EC_T_DWORD      dwSize;              /**< [in]   Set to sizeof(EC_T_ADS_ADAPTER_START_PARMS) */
    EC_T_LOG_PARMS  LogParms;
    EC_T_CPUSET     cpuAffinityMask;
    EC_T_DWORD      dwThreadPriority;
    EC_T_AOE_NETID  targetNetID;
    EC_T_WORD       targetPort;
    EC_T_WORD       wReserved;
} EC_PACKED(1) EC_T_ADS_ADAPTER_START_PARMS;
#include EC_PACKED_INCLUDESTOP

#include EC_PACKED_INCLUDESTART(1)
typedef struct _EC_T_CYCLIC_MASTER_RED_FRAME_CMD_DESC
{
    EC_T_DWORD      dwMasterInstanceId; /* EC-Master Instance ID */
    EC_T_WORD       wType;              /* See EC_T_CMF_CMD_TYPE_... */
    EC_T_WORD       wDataOffset;
    EC_T_WORD       wCmdFlags;          /* See EC_T_CMF_CMD_FLAG... */
    EC_T_WORD       wReserved;
} EC_PACKED(1) EC_T_CYCLIC_MASTER_RED_FRAME_CMD_DESC;
#include EC_PACKED_INCLUDESTOP
#define EC_T_CMF_CMD_TYPE_UNKNOWN                        0
#define EC_T_CMF_CMD_TYPE_SLAVE_PD                       1
#define EC_T_CMF_CMD_TYPE_MASTER_PD                      2
#define EC_T_CMF_CMD_TYPE_SLAVE_STATES                   3
#define EC_T_CMF_CMD_TYPE_DIAG_IMAGE                     4
#define EC_T_CMF_CMD_TYPE_BRD_ALSTATUS                   5

#define EC_T_CMF_CMD_FLAG_LAST_CYC_FRAME            0x0001
#define EC_T_CMF_CMD_FLAG_DEVICE_FLAGS_MAIN_MASK    0x0F00 /* see EC_T_CMF_CMD_DEVICE_FLAG_... */
#define EC_T_CMF_CMD_FLAG_DEVICE_FLAGS_MAIN_OFFSET       8
#define EC_T_CMF_CMD_FLAG_DEVICE_FLAGS_RED_MASK     0xF000 /* see EC_T_CMF_CMD_DEVICE_FLAG_... */
#define EC_T_CMF_CMD_FLAG_DEVICE_FLAGS_RED_OFFSET       12

/* EC_T_CMF_CMD_FLAG_DEVICE_FLAGS_... */
#define EC_T_CMF_CMD_DEVICE_FLAG_LINK                  0x1 /* link detected */
#define EC_T_CMF_CMD_DEVICE_FLAG_SEND_ENABLED          0x2 /* send enabled (topo change delay elapsed or frame received) */
#define EC_T_CMF_CMD_DEVICE_FLAG_SLAVE_DATA_RX         0x4 /* slave data received last cycle */
#define EC_T_CMF_CMD_DEVICE_FLAG_MASTERRED_DATA_RX     0x8 /* foreign master red data received last cycle */

typedef enum _EC_T_SLAVE_SELECTION
{
    eSlaveSelectionSingle,              /**< [in] Select only one slave */
    eSlaveSelectionTopoFollowers,       /**< [in] Select slave and his topological followers */
    eSlaveSelectionMasterSyncUnit,      /**< [in] Select slave and his topological followers */

    /* Borland C++ datatype alignment correction */
    eSlaveSelectionDummy = 0xFFFFFFFF
} EC_T_SLAVE_SELECTION;

#include EC_PACKED_API_INCLUDESTART
typedef struct _EC_T_SET_MAILBOX_POLLING_CYCLES_DESC
{
    EC_T_DWORD      dwSlaveId;          /**< [in] Slave Id */
    EC_T_WORD       wCycles;            /**< [in] Number of cycles between polling [ms] */
} EC_PACKED_API  EC_T_SET_MAILBOX_POLLING_CYCLES_DESC;
#include EC_PACKED_INCLUDESTOP

#include EC_PACKED_API_INCLUDESTART
typedef struct _EC_IOCTL_SET_CONFIGDATA_MEMORY_POOL_DESC
{
    EC_T_BYTE*      pbyStart;           /**< [in] Start of the memory pool */
    EC_T_DWORD      dwSize;             /**< [in] Size of the memory pool in bytes */
} EC_PACKED_API  EC_IOCTL_SET_CONFIGDATA_MEMORY_POOL_DESC;
#include EC_PACKED_INCLUDESTOP

#include EC_PACKED_API_INCLUDESTART
typedef struct _EC_T_SELFTESTSCAN_PARMS
{
    EC_T_DWORD  dwSize;                  /**< [in] Set to sizeof(EC_T_SELFTESTSCAN_PARMS) */
    EC_T_DWORD  dwTimeout;               /**< [in] Timeout [ms], 0 or EC_NOWAIT defaults to 500ms*/
    EC_T_DWORD  dwFrameCount;            /**< [in] Total number of frames sent during the self-test. Default value is 1500. A value of 0 let the current setting unmodified. */
    EC_T_DWORD  dwFrameSizeMin;          /**< [in] Min frame size [bytes]. Default value is 60. A value of 0 let the current setting unmodified. */
    EC_T_DWORD  dwFrameSizeMax;          /**< [in] Max frame size [bytes]. Default value is 1514. A value of 0 let the current setting unmodified. */
    EC_T_DWORD  dwFrameSizeStep;         /**< [in] Size [bytes] by which the frame increases or decreases continuously during the self-test. Default value is 1. A value of 0 let the current setting unmodified. */
    EC_T_BOOL   bDetectBadConnections;   /**< [in] Execute the bad connection detection after self-test */
    EC_T_UINT64 qwFrameRoundtripTimeAvg; /**< [out] Roundtrip time average [us]. Time taken from sending to receiving the frame (master application level). */
    EC_T_UINT64 qwFrameRoundtripTimeMin; /**< [out] Roundtrip time minimum [us]. Time taken from sending to receiving the frame (master application level). */
    EC_T_UINT64 qwFrameRoundtripTimeMax; /**< [out] Roundtrip time maximum [us]. Time taken from sending to receiving the frame (master application level). */
} EC_T_SELFTESTSCAN_PARMS;
#include EC_PACKED_INCLUDESTOP
/*-COMPILER SETTINGS---------------------------------------------------------*/
#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* INC_ECINTERFACECOMMON */

/*-END OF SOURCE FILE--------------------------------------------------------*/
