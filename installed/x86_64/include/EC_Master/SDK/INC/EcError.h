/*-----------------------------------------------------------------------------
 * EcError.h
 * Copyright                acontis technologies GmbH, Ravensburg, Germany
 * Response                 Stefan Zintgraf
 * Description              EtherCAT Master error definitions
 *---------------------------------------------------------------------------*/

#ifndef INC_ECERROR
#define INC_ECERROR 1

/*-DEFINES-------------------------------------------------------------------*/

/*********************************************************************

  Do not change these offset!
  Insert new codes in the free areas

   Errorcode                     Text-ID
    Start   |      End    | Start   | End    |  Size  |  Description
-----------------------------------------------------------------------
0x9811.0000 | 0x9811.017D | 0x0000  | 0x017F |  0x180 | EC_E_...: Master Error Code
            |             |         |        |        |
0x9811.0180 | 0x9811.01BF | 0x0180  | 0x01BF |  0x040 | EMRAS_E...: RAS Error Code
            |             |         |        |        |
0x9812.01C0 | 0x9812.01FF | 0x01C0  | 0x01FF |  0x040 | DCM_E_...: DCM Error Code
            |             |         |        |        |
    -       |      -      | 0x0200  | 0x02FF |  0x100 | Application framework
            |             |         |        |        |
    -       |      -      | 0x0300  | 0x03FF |  0x100 | AL Status Codes
            |             |         |        |        |
0x9813.0000 | 0x9813.FFFF | 0x0000  | 0xFFFF | 0xFFFF | EC_E_...: Master Error Code

*********************************************************************/

/*********************************************************************/
/** \defgroup EC_ERROR_CODES_GENERIC Generic Error Codes
@{*/
#define       EC_E_NOERROR                      ((EC_T_DWORD)0x00000000) /**< 0x00000000: No Error */
#define EC_SZTXT_E_NOERROR                      "No Error"

#define       EC_E_ERROR                        ((EC_T_DWORD)0x98110000) /**< 0x98110000: Unspecific Error */
#define EC_SZTXT_E_ERROR                        "Unspecific Error"

#define       EMRAS_E_ERROR                     ((EC_T_DWORD)0x98110180) /**< 0x98110180: Unspecific RAS Error */
#define EMRAS_SZTXT_E_ERROR                     "Unspecific RAS Error"
/**@}*/ /* EC_ERROR_CODES_GENERIC */

#define       EC_TEXTBASE                       ((EC_T_WORD)0x0200)
#define EC_SZTXT_TEXTBASE                       "Unknown Text (Base)"

/* AL Status Code according to ETG.1020 Protocol Enhancements */
#define       EC_ALSTATUSCODEBASE               ((EC_T_WORD)0x0300)
#define EC_SZTXT_ALSTATUSCODEBASE               "AL Status Code No Error"


/*********************************************************************/
/* Master Error Strings                                              */
/*********************************************************************/
/*********************************************************************/
/** \defgroup EC_ERROR_CODES_GENERIC Generic Error Codes
@{*/
#define       EC_E_NOTSUPPORTED                 ((EC_T_DWORD)0x98110001) /**< 0x98110001: APP: Feature not supported (e.g. function or property not available) */
#define EC_SZTXT_E_NOTSUPPORTED                 "ERROR: Feature not supported"

#define       EC_E_INVALIDINDEX                 ((EC_T_DWORD)0x98110002) /**< 0x98110002: APP: Invalid index (e.g. CoE: invalid SDO index) */
#define EC_SZTXT_E_INVALIDINDEX                 "ERROR: Invalid index"

#define       EC_E_INVALIDOFFSET                ((EC_T_DWORD)0x98110003) /**< 0x98110003: ISW: Invalid offset (e.g. invalid offset while accessing Process Data Image) */
#define EC_SZTXT_E_INVALIDOFFSET                "ERROR: Invalid offset"

#define       EC_E_CANCEL                       ((EC_T_DWORD)0x98110004) /**< 0x98110004: APP: Cancel (e.g. master should abort current mbx transfer) */
#define EC_SZTXT_E_CANCEL                       "ERROR: Cancel"

#define       EC_E_INVALIDSIZE                  ((EC_T_DWORD)0x98110005) /**< 0x98110005: APP: Invalid size */
#define EC_SZTXT_E_INVALIDSIZE                  "ERROR: Invalid size"

#define       EC_E_INVALIDDATA                  ((EC_T_DWORD)0x98110006) /**< 0x98110006: ISW: Invalid data (multiple error sources) */
#define EC_SZTXT_E_INVALIDDATA                  "ERROR: Invalid data"

#define       EC_E_NOTREADY                     ((EC_T_DWORD)0x98110007) /**< 0x98110007: ISW: Not ready (multiple error sources) */
#define EC_SZTXT_E_NOTREADY                     "ERROR: Not ready"

#define       EC_E_BUSY                         ((EC_T_DWORD)0x98110008) /**< 0x98110008: APP: Busy (e.g. stack is busy currently and not available to process the API request. The function may be called again later) */
#define EC_SZTXT_E_BUSY                         "ERROR: Busy"

#define       EC_E_ACYC_FRM_FREEQ_EMPTY         ((EC_T_DWORD)0x98110009) /**< 0x98110009: ISW: Cannot queue acyclic ecat command (Acyclic command queue is full. Possible solution: Increase of configuration value dwMaxQueuedEthFrames) */
#define EC_SZTXT_E_ACYC_FRM_FREEQ_EMPTY         "ERROR: Cannot queue acyclic EtherCAT command (MasterConfig.dwMaxQueuedEthFrames)"

#define       EC_E_NOMEMORY                     ((EC_T_DWORD)0x9811000A) /**< 0x9811000A: CFG: No memory left (e.g. memory full / fragmented)) */
#define EC_SZTXT_E_NOMEMORY                     "ERROR: No memory left"

#define       EC_E_INVALIDPARM                  ((EC_T_DWORD)0x9811000B) /**< 0x9811000B: APP: Invalid parameter (e.g. API function called with erroneous parameter set) */
#define EC_SZTXT_E_INVALIDPARM                  "ERROR: Invalid parameter"

#define       EC_E_NOTFOUND                     ((EC_T_DWORD)0x9811000C) /**< 0x9811000C: APP: Not found (e.g. Network Information File ENI not found or API called with invalid slave ID) */
#define EC_SZTXT_E_NOTFOUND                     "ERROR: Not found"

#define       EC_E_DUPLICATE                    ((EC_T_DWORD)0x9811000D) /**< 0x9811000D: ISW: Duplicated fixed address detected (handled internally) */
#define EC_SZTXT_E_DUPLICATE                    "ERROR: Duplicate"

#define       EC_E_INVALIDSTATE                 ((EC_T_DWORD)0x9811000E) /**< 0x9811000E: ISW: Invalid state (master not initialized or not configured) */
#define EC_SZTXT_E_INVALIDSTATE                 "ERROR: Invalid state"

#define       EC_E_TIMER_LIST_FULL              ((EC_T_DWORD)0x9811000F) /**< 0x9811000F: ISW: Cannot add slave to timer list (slave timer list full) */
#define EC_SZTXT_E_TIMER_LIST_FULL              "ERROR: Cannot add slave to timer list"

#define       EC_E_TIMEOUT                      ((EC_T_DWORD)0x98110010) /**< 0x98110010: Timeout */
#define EC_SZTXT_E_TIMEOUT                      "ERROR: Time-out"

#define       EC_E_OPENFAILED                   ((EC_T_DWORD)0x98110011) /**< 0x98110011: ISW: Open failed */
#define EC_SZTXT_E_OPENFAILED                   "ERROR: Open failed"

#define       EC_E_SENDFAILED                   ((EC_T_DWORD)0x98110012) /**< 0x98110012: LLA: Frame send failed */
#define EC_SZTXT_E_SENDFAILED                   "ERROR: Send failed"

#define       EC_E_INSERTMAILBOX                ((EC_T_DWORD)0x98110013) /**< 0x98110013: CFG: Insert Mailbox error (internal limit MAX_QUEUED_COE_CMDS: 20) */
#define EC_SZTXT_E_INSERTMAILBOX                "ERROR: Insert mailbox error"

#define       EC_E_INVALIDCMD                   ((EC_T_DWORD)0x98110014) /**< 0x98110014: ISW: Invalid Command (Unknown mailbox command code) */
#define EC_SZTXT_E_INVALIDCMD                   "ERROR: Invalid mailbox command"

#define       EC_E_UNKNOWN_MBX_PROTOCOL         ((EC_T_DWORD)0x98110015) /**< 0x98110015: ISW: Unknown Mailbox Protocol Command (Unknown Mailbox protocol or mailbox command with unknown protocol association) */
#define EC_SZTXT_E_UNKNOWN_MBX_PROTOCOL         "ERROR: Unknown mailbox protocol command"

#define       EC_E_ACCESSDENIED                 ((EC_T_DWORD)0x98110016) /**< 0x98110016: ISW: Access Denied (e.g. master internal software error) */
#define EC_SZTXT_E_ACCESSDENIED                 "ERROR: Access denied"

#define       EC_E_IDENTIFICATIONFAILED         ((EC_T_DWORD)0x98110017) /**< 0x98110017: ENI: Identification failed (e.g. identification command failed) */
#define EC_SZTXT_E_IDENTIFICATIONFAILED         "ERROR: Identification failed"

#define       EC_E_LOCK_CREATE_FAILED           ((EC_T_DWORD)0x98110018) /**< 0x98110018: SYS: Create lock failed (e.g. OsCreateLockTyped failed) */
#define EC_SZTXT_E_LOCK_CREATE_FAILED           "ERROR: Create lock failed"

#define       EC_E_PRODKEY_INVALID              ((EC_T_DWORD)0x9811001A) /**< 0x9811001A: CFG: Product Key Invalid (e.g. application using protected version of stack, which stops operation after 60 minutes if license not provided) */
#define EC_SZTXT_E_PRODKEY_INVALID              "ERROR: Invalid product key"

#define       EC_E_WRONG_FORMAT                 ((EC_T_DWORD)0x9811001B) /**< 0x9811001B: ENI: Wrong configuration format (e.g. Network information file empty or malformed) */
#define EC_SZTXT_E_WRONG_FORMAT                 "ERROR: Wrong format of Configuration file"

#define       EC_E_FEATURE_DISABLED             ((EC_T_DWORD)0x9811001C) /**< 0x9811001C: APP: Feature disabled (e.g. Application tried to perform a missing or disabled API function) */
#define EC_SZTXT_E_FEATURE_DISABLED             "ERROR: Feature disabled"

#define       EC_E_SHADOW_MEMORY                ((EC_T_DWORD)0x9811001D) /**< 0x9811001D: Shadow memory requested in wrong mode */
#define EC_SZTXT_E_SHADOW_MEMORY                "ERROR: Shadow memory requested in wrong mode"

#define       EC_E_BUSCONFIG_MISMATCH           ((EC_T_DWORD)0x9811001E) /**< 0x9811001E: ENI: Bus Config Mismatch (e.g. Network information file and currently connected bus topology does not match) */
#define EC_SZTXT_E_BUSCONFIG_MISMATCH           "Bus configuration mismatch"

#define       EC_E_CONFIGDATAREAD               ((EC_T_DWORD)0x9811001F) /**< 0x9811001F: ENI: Error reading config file (e.g. Network information file could not be read) */
#define EC_SZTXT_E_CONFIGDATAREAD               "ERROR: Error in reading config file"

#define       EC_E_ENI_NO_SAFEOP_OP_SUPPORT     ((EC_T_DWORD)0x98110020) /**< 0x98110020: Configuration doesn't support SAFEOP and OP requested state */
#define EC_SZTXT_E_ENI_NO_SAFEOP_OP_SUPPORT     "ERROR: Configuration doesn't support SAFEOP and OP requested state"

#define       EC_E_XML_CYCCMDS_MISSING          ((EC_T_DWORD)0x98110021) /**< 0x98110021: ENI: Cyclic commands are missing (e.g. Network information file does not contain cyclic commands) */
#define EC_SZTXT_E_XML_CYCCMDS_MISSING          "ERROR: Cyclic commands missing"

#define       EC_E_XML_ALSTATUS_READ_MISSING    ((EC_T_DWORD)0x98110022) /**< 0x98110022: ENI: AL_STATUS register read missing in XML file for at least one state (e.g. Read of AL Status register is missing in cyclic part of given network information file) */
#define EC_SZTXT_E_XML_ALSTATUS_READ_MISSING    "ERROR: AL_STATUS register read missing in XML file for at least one state"

#define       EC_E_MCSM_FATAL_ERROR             ((EC_T_DWORD)0x98110023) /**< 0x98110023: ISW: Fatal internal McSm (master control state machine is in an undefined state) */
#define EC_SZTXT_E_MCSM_FATAL_ERROR             "ERROR: Fatal internal McSm"

#define       EC_E_SLAVE_ERROR                  ((EC_T_DWORD)0x98110024) /**< 0x98110024: SLV: Slave error (e.g. A slave error was detected. See also EC_NOTIFY_STATUS_SLAVE_ERROR and EC_NOTIFY_SLAVE_ERROR_STATUS_INFO) */
#define EC_SZTXT_E_SLAVE_ERROR                  "ERROR: Slave error"

#define       EC_E_FRAME_LOST                   ((EC_T_DWORD)0x98110025) /**< 0x98110025: SLV: Frame lost, IDX mismatch (EtherCAT frame(s) lost on bus, means the response was not received. In case this error shows frequently a problem with the wiring could be the cause) */
#define EC_SZTXT_E_FRAME_LOST                   "ERROR: Frame lost, IDX mismatch"

#define       EC_E_CMD_MISSING                  ((EC_T_DWORD)0x98110026) /**< 0x98110026: SLV: At least one EtherCAT command is missing in the received frame (e.g. received EtherCAT frame incomplete) */
#define EC_SZTXT_E_CMD_MISSING                  "ERROR: At least one EtherCAT command missing in received frame"

#define       EC_E_CYCCMD_WKC_ERROR             ((EC_T_DWORD)0x98110027) /**< 0x98110027: Cyclic command WKC error */
#define EC_SZTXT_E_CYCCMD_WKC_ERROR             "ERROR: Cyclic command WKC error"

#define       EC_E_INVALID_DCL_MODE             ((EC_T_DWORD)0x98110028) /**< 0x98110028: APP: IOCTL EC_IOCTL_DC_LATCH_REQ_LTIMVALS invalid in DCL auto read mode (this function cannot be used if DC Latching is running in mode "Auto Read") */
#define EC_SZTXT_E_INVALID_DCL_MODE             "ERROR: IOCTL EC_IOCTL_DC_LATCH_REQ_LTIMVALS not possible in DC Latching auto read mode"

#define       EC_E_AI_ADDRESS                   ((EC_T_DWORD)0x98110029) /**< 0x98110029: SLV: Auto increment address increment mismatch (e.g. Network information file and bus topology doesn't match any more. Error shows only, if a already recognized slave isn't present any more) */
#define EC_SZTXT_E_AI_ADDRESS                   "ERROR: Auto increment address - increment mismatch (slave missing)"

#define       EC_E_INVALID_SLAVE_STATE          ((EC_T_DWORD)0x9811002A) /**< 0x9811002A: APP: Slave in invalid state, e.g. not in OP (API not callable in this state) (mailbox commands are not allowed in current slave state) */
#define EC_SZTXT_E_INVALID_SLAVE_STATE          "ERROR: Slave in invalid state, e.g. not in OP (API not callable in this state)"

#define       EC_E_SLAVE_NOT_ADDRESSABLE        ((EC_T_DWORD)0x9811002B) /**< 0x9811002B: SLV: Station address lost (or slave missing) - FPRD to AL_STATUS failed (e.g. Slave had a powercycle) */
#define EC_SZTXT_E_SLAVE_NOT_ADDRESSABLE        "ERROR: Station address lost or slave missing - FPRD to AL_STATUS failed"

#define       EC_E_CYC_CMDS_OVERFLOW            ((EC_T_DWORD)0x9811002C) /**< 0x9811002C: ENI: Too many cyclic commands in XML configuration file (e.g. EC_T_INIT_MASTER_PARMS.dwMaxQueuedEthFrames too small) */
#define EC_SZTXT_E_CYC_CMDS_OVERFLOW            "ERROR: Too many cyclic commands in XML configuration file. (Check EC_T_INIT_MASTER_PARMS.dwMaxQueuedEthFrames)"

#define       EC_E_LINK_DISCONNECTED            ((EC_T_DWORD)0x9811002D) /**< 0x9811002D: SLV: Ethernet link cable disconnected (e.g. EtherCAT bus segment not connected to network interface) */
#define EC_SZTXT_E_LINK_DISCONNECTED            "ERROR: Ethernet link cable disconnected"

#if (defined EC_SIMULATOR) /* AtesRasClnt */
#define       EC_E_MASTERCORE_INACCESSIBLE      ((EC_T_DWORD)0x9811002E) /**< 0x9811002E: RAS: Simulator core not accessible (e.g. Connection to remote server was terminated or simulator instance has been stopped on remote side) */
#define EC_SZTXT_E_MASTERCORE_INACCESSIBLE      "ERROR: Simulator core not accessible"
#else
#define       EC_E_MASTERCORE_INACCESSIBLE      ((EC_T_DWORD)0x9811002E) /**< 0x9811002E: RAS: Master core not accessible (e.g. Connection to remote server was terminated or master instance has been stopped on remote side) */
#define EC_SZTXT_E_MASTERCORE_INACCESSIBLE      "ERROR: Master core not accessible"
#endif


#define       EC_E_COE_MBXSND_WKC_ERROR         ((EC_T_DWORD)0x9811002F) /**< 0x9811002F: SLV: CoE mbox send: working counter (e.g. CoE mailbox couldn't be read on slave, slave didn't read out mailbox since last write) */
#define EC_SZTXT_E_COE_MBXSND_WKC_ERROR         "ERROR CoE: Mailbox send: working counter"

/* obsolete */
#define       EC_E_COE_MBXRCV_WKC_ERROR         ((EC_T_DWORD)0x98110030) /**< 0x98110030: SLV: CoE mbox receive: working counter (e.g. CoE Mailbox couldn't be read from slave) */
#define EC_SZTXT_E_COE_MBXRCV_WKC_ERROR         "ERROR CoE: Mailbox receive: working counter"

#define       EC_E_NO_MBX_SUPPORT               ((EC_T_DWORD)0x98110031) /**< 0x98110031: APP: No mailbox support (e.g. Slave does not support mailbox access) */
#define EC_SZTXT_E_NO_MBX_SUPPORT               "ERROR: No mailbox support"

#define       EC_E_NO_COE_SUPPORT               ((EC_T_DWORD)0x98110032) /**< 0x98110032: ENI: CoE protocol not supported (e.g. Configuration error or slave information file doesn't match slave firmware) */
#define EC_SZTXT_E_NO_COE_SUPPORT               "ERROR CoE: Protocol not supported"

#define       EC_E_NO_EOE_SUPPORT               ((EC_T_DWORD)0x98110033) /**< 0x98110033: ENI: EoE protocol not supported (e.g. Configuration error or slave information file doesn't match slave firmware) */
#define EC_SZTXT_E_NO_EOE_SUPPORT               "ERROR EoE: Protocol not supported"

#define       EC_E_NO_FOE_SUPPORT               ((EC_T_DWORD)0x98110034) /**< 0x98110034: ENI: FoE protocol not supported (e.g. Configuration error or slave information file doesn't match slave firmware) */
#define EC_SZTXT_E_NO_FOE_SUPPORT               "ERROR FoE: Protocol not supported"

#define       EC_E_NO_SOE_SUPPORT               ((EC_T_DWORD)0x98110035) /**< 0x98110035: ENI: SoE protocol not supported (e.g. Configuration error or slave information file doesn't match slave firmware) */
#define EC_SZTXT_E_NO_SOE_SUPPORT               "ERROR SoE: Protocol not supported"

#define       EC_E_NO_VOE_SUPPORT               ((EC_T_DWORD)0x98110036) /**< 0x98110036: ENI: VoE protocol not supported (e.g. Configuration error or slave information file doesn't match slave firmware) */
#define EC_SZTXT_E_NO_VOE_SUPPORT               "ERROR VoE: Protocol not supported"

#define       EC_E_EVAL_VIOLATION               ((EC_T_DWORD)0x98110037) /**< 0x98110037: ENI: Configuration violates Evaluation limits (obsolete) */
#define EC_SZTXT_E_EVAL_VIOLATION               "ERROR: Configuration violates evaluation limits"

#define       EC_E_EVAL_EXPIRED                 ((EC_T_DWORD)0x98110038) /**< 0x98110038: CFG: Evaluation Time limit reached (e.g. Licence not provided and evaluation period (1 hour) of protected version exceeded) */
#define EC_SZTXT_E_EVAL_EXPIRED                 "ERROR: Evaluation time limit reached / slave count limit exeeded"

#define       EC_E_LICENSE_MISSING              ((EC_T_DWORD)0x98110039) /**< 0x98110039: License key invalid or missing */
#define EC_SZTXT_E_LICENSE_MISSING              "ERROR: License key invalid or missing"
/**@}*/ /* EC_ERROR_CODES_GENERIC */

/*****************************************************************************
 * CoE Abort Codes are defined in ETG.1000.6
 * Additional codes are defined in ETG.1020, V1.2.0, Table 21: CoE Abort Codes (extension)
 * See also SDO_ABORTCODE_...
 *****************************************************************************/
#define       EC_E_SDO_ABORTCODE_FIRST          ((EC_T_DWORD)0x98110040)

/*********************************************************************/
/** \defgroup EC_ERROR_CODES_COE CAN application protocol over EtherCAT (CoE) SDO Error Codes
@{*/
#define       EC_E_SDO_ABORTCODE_TOGGLE         ((EC_T_DWORD)0x98110040) /**< 0x98110040: SLV: SDO: Toggle bit not alternated (CoE abort code 0x05030000 of slave) */
#define EC_SZTXT_E_SDO_ABORTCODE_TOGGLE         "ERROR SDO: Toggle bit not alternated"

#define       EC_E_SDO_ABORTCODE_TIMEOUT        ((EC_T_DWORD)0x98110041) /**< 0x98110041: SLV: SDO: Protocol timed out (CoE abort code 0x05040000 of slave) */
#define EC_SZTXT_E_SDO_ABORTCODE_TIMEOUT        "ERROR SDO: SDO protocol time-out"

#define       EC_E_SDO_ABORTCODE_CCS_SCS        ((EC_T_DWORD)0x98110042) /**< 0x98110042: SLV: SDO: Client/server command specifier not valid or unknown (CoE abort code 0x05040001 of slave) */
#define EC_SZTXT_E_SDO_ABORTCODE_CCS_SCS        "ERROR SDO: Client/server command specifier not valid or unknown"

#define       EC_E_SDO_ABORTCODE_BLK_SIZE       ((EC_T_DWORD)0x98110043) /**< 0x98110043: SLV: SDO: Invalid block size (block mode only) (CoE abort code 0x05040002 of slave) */
#define EC_SZTXT_E_SDO_ABORTCODE_BLK_SIZE       "ERROR SDO: Invalid block size (block mode only)"

#define       EC_E_SDO_ABORTCODE_SEQNO          ((EC_T_DWORD)0x98110044) /**< 0x98110044: SLV: SDO: Invalid sequence number (block mode only) (CoE abort code 0x05040003 of slave) */
#define EC_SZTXT_E_SDO_ABORTCODE_SEQNO          "ERROR SDO: Invalid sequence number (block mode only)"

#define       EC_E_SDO_ABORTCODE_CRC            ((EC_T_DWORD)0x98110045) /**< 0x98110045: SLV: SDO: CRC error (block mode only) (CoE abort code 0x05040004 of slave) */
#define EC_SZTXT_E_SDO_ABORTCODE_CRC            "ERROR SDO: CRC error (block mode only)"

#define       EC_E_SDO_ABORTCODE_MEMORY         ((EC_T_DWORD)0x98110046) /**< 0x98110046: SLV: SDO: Out of memory (CoE abort code 0x05040005 of slave) */
#define EC_SZTXT_E_SDO_ABORTCODE_MEMORY         "ERROR SDO: Out of memory"

#define       EC_E_SDO_ABORTCODE_ACCESS         ((EC_T_DWORD)0x98110047) /**< 0x98110047: SLV: SDO: Unsupported access to an object (CoE abort code 0x06010000 of slave) */
#define EC_SZTXT_E_SDO_ABORTCODE_ACCESS         "ERROR SDO: Unsupported access to an object"

#define       EC_E_SDO_ABORTCODE_WRITEONLY      ((EC_T_DWORD)0x98110048) /**< 0x98110048: SLV: SDO: Attempt to read a write only object (CoE abort code 0x06010001 of slave) */
#define EC_SZTXT_E_SDO_ABORTCODE_WRITEONLY      "ERROR SDO: Attempt to read a write only object"

#define       EC_E_SDO_ABORTCODE_READONLY       ((EC_T_DWORD)0x98110049) /**< 0x98110049: SLV: SDO: Attempt to write a read only object (CoE abort code 0x06010002 of slave) */
#define EC_SZTXT_E_SDO_ABORTCODE_READONLY       "ERROR SDO: Attempt to write a read only object"

#define       EC_E_SDO_ABORTCODE_INDEX          ((EC_T_DWORD)0x9811004A) /**< 0x9811004A: SLV: SDO: Object does not exist in the object dictionary (CoE abort code 0x06020000 of slave) */
#define EC_SZTXT_E_SDO_ABORTCODE_INDEX          "ERROR SDO: Object does not exist in object dictionary"

#define       EC_E_SDO_ABORTCODE_PDO_MAP        ((EC_T_DWORD)0x9811004B) /**< 0x9811004B: SLV: SDO: Object cannot be mapped to the PDO (CoE abort code 0x06040041 of slave) */
#define EC_SZTXT_E_SDO_ABORTCODE_PDO_MAP        "ERROR SDO: Object cannot be mapped to PDO"

#define       EC_E_SDO_ABORTCODE_PDO_LEN        ((EC_T_DWORD)0x9811004C) /**< 0x9811004C: SLV: SDO: The number and length of the objects to be mapped would exceed PDO length (CoE abort code 0x06040042 of slave) */
#define EC_SZTXT_E_SDO_ABORTCODE_PDO_LEN        "ERROR SDO: Number and length of objects to be mapped exceed PDO length"

#define       EC_E_SDO_ABORTCODE_P_INCOMP       ((EC_T_DWORD)0x9811004D) /**< 0x9811004D: SLV: SDO: General parameter incompatibility reason (CoE abort code 0x06040043 of slave) */
#define EC_SZTXT_E_SDO_ABORTCODE_P_INCOMP       "ERROR SDO: General parameter incompatibility"

#define       EC_E_SDO_ABORTCODE_I_INCOMP       ((EC_T_DWORD)0x9811004E) /**< 0x9811004E: SLV: SDO: General internal incompatibility in the device (CoE abort code 0x06040047 of slave) */
#define EC_SZTXT_E_SDO_ABORTCODE_I_INCOMP       "ERROR SDO: General internal incompatibility in device"

#define       EC_E_SDO_ABORTCODE_HARDWARE       ((EC_T_DWORD)0x9811004F) /**< 0x9811004F: SLV: SDO: Access failed due to an hardware error (CoE abort code 0x06060000 of slave) */
#define EC_SZTXT_E_SDO_ABORTCODE_HARDWARE       "ERROR SDO: Access failed due to an hardware error"

#define       EC_E_SDO_ABORTCODE_DATA_LENGTH_NOT_MATCH      ((EC_T_DWORD)0x98110050) /**< 0x98110050: SLV: SDO: Data type does not match, length of service parameter does not match (CoE abort code 0x06070010 of slave) */
#define EC_SZTXT_E_SDO_ABORTCODE_DATA_LENGTH_NOT_MATCH      "ERROR SDO: Data type does not match, length of service parameter does not match"

#define       EC_E_SDO_ABORTCODE_DATA_LENGTH_TOO_HIGH       ((EC_T_DWORD)0x98110051) /**< 0x98110051: SLV: SDO: Data type does not match, length of service parameter too high (CoE abort code 0x06070012 of slave) */
#define EC_SZTXT_E_SDO_ABORTCODE_DATA_LENGTH_TOO_HIGH       "ERROR SDO: Data type does not match, service parameter too high"

#define       EC_E_SDO_ABORTCODE_DATA_LENGTH_TOO_LOW        ((EC_T_DWORD)0x98110052) /**< 0x98110052: SLV: SDO: Data type does not match, length of service parameter too low (CoE abort code 0x06070013 of slave) */
#define EC_SZTXT_E_SDO_ABORTCODE_DATA_LENGTH_TOO_LOW        "ERROR SDO: Data type does not match, service parameter too low"

#define       EC_E_SDO_ABORTCODE_OFFSET                     ((EC_T_DWORD)0x98110053) /**< 0x98110053: SLV: SDO: Sub-index does not exist (CoE abort code 0x06090011 of slave) */
#define EC_SZTXT_E_SDO_ABORTCODE_OFFSET                     "ERROR SDO: Sub-index does not exist"

#define       EC_E_SDO_ABORTCODE_VALUE_RANGE                 ((EC_T_DWORD)0x98110054) /**< 0x98110054: SLV: SDO: Value range of parameter exceeded (only for write access) (CoE abort code 0x06090030 of slave) */
#define EC_SZTXT_E_SDO_ABORTCODE_VALUE_RANGE                "ERROR SDO: Write access - Value range of parameter exceeded"

#define       EC_E_SDO_ABORTCODE_VALUE_TOO_HIGH              ((EC_T_DWORD)0x98110055) /**< 0x98110055: SLV: SDO: Value of parameter written too high (CoE abort code 0x06090031 of slave) */
#define EC_SZTXT_E_SDO_ABORTCODE_VALUE_TOO_HIGH             "ERROR SDO: Write access - Value of parameter written too high"

#define       EC_E_SDO_ABORTCODE_VALUE_TOO_LOW              ((EC_T_DWORD)0x98110056) /**< 0x98110056: SLV: SDO: Value of parameter written too low (CoE abort code 0x06090032 of slave) */
#define EC_SZTXT_E_SDO_ABORTCODE_VALUE_TOO_LOW              "ERROR SDO: Write access - Value of parameter written too low"

#define       EC_E_SDO_ABORTCODE_MINMAX                     ((EC_T_DWORD)0x98110057) /**< 0x98110057: SLV: SDO: Maximum value is less than minimum value (CoE abort code 0x06090036 of slave) */
#define EC_SZTXT_E_SDO_ABORTCODE_MINMAX                     "ERROR SDO: Maximum value less than minimum value"

#define       EC_E_SDO_ABORTCODE_GENERAL                    ((EC_T_DWORD)0x98110058) /**< 0x98110058: SLV: SDO: General error (CoE abort code 0x08000000 of slave) */
#define EC_SZTXT_E_SDO_ABORTCODE_GENERAL                    "ERROR SDO: General error"

#define       EC_E_SDO_ABORTCODE_TRANSFER                   ((EC_T_DWORD)0x98110059) /**< 0x98110059: SLV: SDO: Data cannot be transferred or stored to the application (CoE abort code 0x08000020 of slave) */
#define EC_SZTXT_E_SDO_ABORTCODE_TRANSFER                   "ERROR SDO: Data cannot be transferred or stored to the application"

#define       EC_E_SDO_ABORTCODE_TRANSFER_LOCAL_CONTROL      ((EC_T_DWORD)0x9811005A) /**< 0x9811005A: SLV: SDO: Data cannot be transferred or stored to the application because of local control (CoE abort code 0x08000021 of slave) */
#define EC_SZTXT_E_SDO_ABORTCODE_TRANSFER_LOCAL_CONTROL      "ERROR SDO: Data cannot be transferred or stored to the application because of local control"

#define       EC_E_SDO_ABORTCODE_TRANSFER_DEVICE_STATE      ((EC_T_DWORD)0x9811005B) /**< 0x9811005B: SLV: SDO: Data cannot be transferred or stored to the application because of the present device state (CoE abort code 0x08000022 of slave) */
#define EC_SZTXT_E_SDO_ABORTCODE_TRANSFER_DEVICE_STATE      "ERROR SDO: Data cannot be transferred or stored to the application because of the present device state"

#define       EC_E_SDO_ABORTCODE_DICTIONARY     ((EC_T_DWORD)0x9811005C) /**< 0x9811005C: SLV: SDO: Object dictionary dynamic generation fails or no object dictionary is present (e.g. object dictionary is generated from file and generation fails because of an file error) (CoE abort code 0x08000023 of slave) */
#define EC_SZTXT_E_SDO_ABORTCODE_DICTIONARY     "ERROR SDO: Dynamic generation of object dictionary failed or missing object dictionary"

#define       EC_E_SDO_ABORTCODE_UNKNOWN        ((EC_T_DWORD)0x9811005D) /**< 0x9811005D: SLV: SDO: Unknown code (Unknown CoE abort code of slave) */
#define EC_SZTXT_E_SDO_ABORTCODE_UNKNOWN        "ERROR SDO: Unknown code"

#define       EC_E_SDO_ABORTCODE_MODULE_ID_LIST_NOT_MATCH   ((EC_T_DWORD)0x9811005E) /**< 0x9811005E: Detected Module Ident List (0xF030) and Configured Module Ident list (0xF050) does not match */
#define EC_SZTXT_E_SDO_ABORTCODE_MODULE_ID_LIST_NOT_MATCH   "ERROR SDO: Detected Module Ident List (0xF030) and Configured Module Ident list (0xF050) does not match"
/**@}*/ /* EC_ERROR_CODES_COE */

#define       EC_E_SDO_ABORTCODE_LAST           ((EC_T_DWORD)0x9811005E)

/*********************************************************************/
/** \defgroup EC_ERROR_CODES_FOE File Transfer over EtherCAT (FoE) Error Codes
@{*/
#define       EC_E_FOE_ERRCODE_NOTDEFINED       ((EC_T_DWORD)0x98110060) /**< 0x98110060: SLV: ERROR FoE: not defined (FoE Error Code 0 (0x8000) of slave) */
#define EC_SZTXT_E_FOE_ERRCODE_NOTDEFINED       "ERROR FoE: Vendor specific FoE error"

#define       EC_E_FOE_ERRCODE_NOTFOUND         ((EC_T_DWORD)0x98110061) /**< 0x98110061: SLV: ERROR FoE: not found (FoE Error Code 1 (0x8001) of slave) */
#define EC_SZTXT_E_FOE_ERRCODE_NOTFOUND         "ERROR FoE: Not found"

#define       EC_E_FOE_ERRCODE_ACCESS           ((EC_T_DWORD)0x98110062) /**< 0x98110062: SLV: ERROR FoE: access denied (FoE Error Code 2 (0x8002) of slave) */
#define EC_SZTXT_E_FOE_ERRCODE_ACCESS           "ERROR FoE: Access denied"

#define       EC_E_FOE_ERRCODE_DISKFULL         ((EC_T_DWORD)0x98110063) /**< 0x98110063: SLV: ERROR FoE: disk full (FoE Error Code 3 (0x8003) of slave) */
#define EC_SZTXT_E_FOE_ERRCODE_DISKFULL         "ERROR FoE: Disk full"

#define       EC_E_FOE_ERRCODE_ILLEGAL          ((EC_T_DWORD)0x98110064) /**< 0x98110064: SLV: ERROR FoE: illegal (FoE Error Code 4 (0x8004) of slave) */
#define EC_SZTXT_E_FOE_ERRCODE_ILLEGAL          "ERROR FoE: Illegal"

#define       EC_E_FOE_ERRCODE_PACKENO          ((EC_T_DWORD)0x98110065) /**< 0x98110065: SLV: ERROR FoE: packet number wrong (FoE Error Code 5 (0x8005) of slave) */
#define EC_SZTXT_E_FOE_ERRCODE_PACKENO          "ERROR FoE: Wrong packet number"

#define       EC_E_FOE_ERRCODE_EXISTS           ((EC_T_DWORD)0x98110066) /**< 0x98110066: SLV: ERROR FoE: already exists (FoE Error Code 6 (0x8006) of slave) */
#define EC_SZTXT_E_FOE_ERRCODE_EXISTS           "ERROR FoE: Already exists"

#define       EC_E_FOE_ERRCODE_NOUSER           ((EC_T_DWORD)0x98110067) /**< 0x98110067: SLV: ERROR FoE: no user (FoE Error Code 7 (0x8007) of slave) */
#define EC_SZTXT_E_FOE_ERRCODE_NOUSER           "ERROR FoE: User missing"

#define       EC_E_FOE_ERRCODE_BOOTSTRAPONLY    ((EC_T_DWORD)0x98110068) /**< 0x98110068: SLV: ERROR FoE: bootstrap only (FoE Error Code 8 (0x8008) of slave) */
#define EC_SZTXT_E_FOE_ERRCODE_BOOTSTRAPONLY    "ERROR FoE: Bootstrap only"

#define       EC_E_FOE_ERRCODE_NOTINBOOTSTRAP   ((EC_T_DWORD)0x98110069) /**< 0x98110069: SLV: ERROR FoE: Downloaded file name is not valid in Bootstrap state (FoE Error Code 9 (0x8009) of slave) */
#define EC_SZTXT_E_FOE_ERRCODE_NOTINBOOTSTRAP   "ERROR FoE: Downloaded file name is not valid in Bootstrap state"

#define       EC_E_FOE_ERRCODE_INVALIDPASSWORD  ((EC_T_DWORD)0x9811006A) /**< 0x9811006A: SLV: ERROR FoE: no rights (FoE Error Code 10 (0x800A) of slave) */
#define EC_SZTXT_E_FOE_ERRCODE_INVALIDPASSWORD  "ERROR FoE: No rights"

#define       EC_E_FOE_ERRCODE_PROGERROR        ((EC_T_DWORD)0x9811006B) /**< 0x9811006B: SLV: ERROR FoE: program error (FoE Error Code 11 (0x800B) of slave) */
#define EC_SZTXT_E_FOE_ERRCODE_PROGERROR        "ERROR FoE: Program error"

#define       EC_E_FOE_ERRCODE_INVALID_CHECKSUM ((EC_T_DWORD)0x9811006C) /**< 0x9811006C: FoE: Wrong checksum */
#define EC_SZTXT_E_FOE_ERRCODE_INVALID_CHECKSUM "ERROR FoE: Wrong checksum"

#define       EC_E_FOE_ERRCODE_INVALID_FIRMWARE ((EC_T_DWORD)0x9811006D ) /**< 0x9811006D: SLV: ERROR FoE: Firmware does not fit for Hardware (FoE Error Code 13 (0x800D) of slave) */
#define EC_SZTXT_E_FOE_ERRCODE_INVALID_FIRMWARE "ERROR FoE: Firmware does not fit for Hardware"

#define       EC_E_FOE_ERRCODE_NO_FILE          ((EC_T_DWORD)0x9811006F) /**< 0x9811006F: SLV: ERROR FoE: No file to read (FoE Error Code 15 (0x800F) of slave) */
#define EC_SZTXT_E_FOE_ERRCODE_NO_FILE          "ERROR FoE: No file to read"
/**@}*/ /* EC_ERROR_CODES_FOE */

/*********************************************************************/
/** \defgroup EC_ERROR_CODES_GENERIC Generic Error Codes
@{*/
#define       EC_E_CFGFILENOTFOUND              ((EC_T_DWORD)0x98110070) /**< 0x98110070: CFG: Master configuration not found (e.g. path to master configuration file (XML) was wrong or the file is not available) */
#define EC_SZTXT_E_CFGFILENOTFOUND              "ERROR: Configuration not found"

#define       EC_E_EEPROMREADERROR              ((EC_T_DWORD)0x98110071) /**< 0x98110071: SLV: Command error while EEPROM upload (read slave EEPROM) */
#define EC_SZTXT_E_EEPROMREADERROR              "ERROR: Command error while EEPROM upload"

#define       EC_E_EEPROMWRITEERROR             ((EC_T_DWORD)0x98110072) /**< 0x98110072: SLV: Command error while EEPROM download (write slave EEPROM) */
#define EC_SZTXT_E_EEPROMWRITEERROR             "ERROR: Command error while EEPROM download"

#define       EC_E_XML_CYCCMDS_SIZEMISMATCH     ((EC_T_DWORD)0x98110073) /**< 0x98110073: ENI: Cyclic command wrong size (too long) (size in master configuration file (XML) does not match size of process data) */
#define EC_SZTXT_E_XML_CYCCMDS_SIZEMISMATCH     "ERROR: Cyclic command wrong size (too long)"

#define       EC_E_XML_INVALID_INP_OFF          ((EC_T_DWORD)0x98110074) /**< 0x98110074: ENI: Invalid input offset in cyc cmd, please check InputOffs */
#define EC_SZTXT_E_XML_INVALID_INP_OFF          "ERROR: Invalid input offset in cyc cmd, please check InputOffs"

#define       EC_E_XML_INVALID_OUT_OFF          ((EC_T_DWORD)0x98110075) /**< 0x98110075: ENI: Invalid output offset in cyc cmd, please check OutputOffs */
#define EC_SZTXT_E_XML_INVALID_OUT_OFF          "ERROR: Invalid output offset in cyc cmd, please check OutputOffs"

#define       EC_E_PORTCLOSE                    ((EC_T_DWORD)0x98110076) /**< 0x98110076: Port close failed */
#define EC_SZTXT_E_PORTCLOSE                    "ERROR: Port Close failed"

#define       EC_E_PORTOPEN                     ((EC_T_DWORD)0x98110077) /**< 0x98110077: Port open failed */
#define EC_SZTXT_E_PORTOPEN                     "ERROR: Port Open failed"
/**@}*/ /* EC_ERROR_CODES_GENERIC */

/*********************************************************************/
/** \defgroup EC_ERROR_CODES_SOE Servo Drive Profil over EtherCAT (SoE) Error Codes
@{*/
#define       EC_E_SOE_ERRORCODE_INVALID_ACCESS ((EC_T_DWORD)0x98110078) /**< 0x98110078: ERROR SoE: Invalid access to element 0 */
#define EC_SZTXT_E_SOE_ERRORCODE_INVALID_ACCESS "ERROR SoE: Invalid access to element 0"

#define       EC_E_SOE_ERRORCODE_NOT_EXIST      ((EC_T_DWORD)0x98110079) /**< 0x98110079: ERROR SoE: Does not exist */
#define EC_SZTXT_E_SOE_ERRORCODE_NOT_EXIST      "ERROR SoE: Does not exist"

#define       EC_E_SOE_ERRORCODE_INVL_ACC_ELEM1 ((EC_T_DWORD)0x9811007A) /**< 0x9811007A: ERROR SoE: Invalid access to element 1 */
#define EC_SZTXT_E_SOE_ERRORCODE_INVL_ACC_ELEM1 "ERROR SoE: Invalid access to element 1"

#define       EC_E_SOE_ERRORCODE_NAME_NOT_EXIST ((EC_T_DWORD)0x9811007B) /**< 0x9811007B: ERROR SoE: Name does not exist */
#define EC_SZTXT_E_SOE_ERRORCODE_NAME_NOT_EXIST "ERROR SoE: Name does not exist"

#define       EC_E_SOE_ERRORCODE_NAME_UNDERSIZE ((EC_T_DWORD)0x9811007C) /**< 0x9811007C: ERROR SoE: Name undersize in transmission */
#define EC_SZTXT_E_SOE_ERRORCODE_NAME_UNDERSIZE "ERROR SoE: Name undersize in transmission"

#define       EC_E_SOE_ERRORCODE_NAME_OVERSIZE  ((EC_T_DWORD)0x9811007D) /**< 0x9811007D: ERROR SoE: Name oversize in transmission */
#define EC_SZTXT_E_SOE_ERRORCODE_NAME_OVERSIZE  "ERROR SoE: Name oversize in transmission"

#define       EC_E_SOE_ERRORCODE_NAME_UNCHANGE  ((EC_T_DWORD)0x9811007E) /**< 0x9811007E: ERROR SoE: Name unchangeable */
#define EC_SZTXT_E_SOE_ERRORCODE_NAME_UNCHANGE  "ERROR SoE: Name unchangeable"

#define       EC_E_SOE_ERRORCODE_NAME_WR_PROT   ((EC_T_DWORD)0x9811007F) /**< 0x9811007F: ERROR SoE: Name currently write-protected */
#define EC_SZTXT_E_SOE_ERRORCODE_NAME_WR_PROT   "ERROR SoE: Name currently write-protected"

#define       EC_E_SOE_ERRORCODE_UNDERS_TRANS   ((EC_T_DWORD)0x98110080) /**< 0x98110080: ERROR SoE: Attribute undersize in transmission */
#define EC_SZTXT_E_SOE_ERRORCODE_UNDERS_TRANS   "ERROR SoE: Attribute undersize in transmission"

#define       EC_E_SOE_ERRORCODE_OVERS_TRANS    ((EC_T_DWORD)0x98110081) /**< 0x98110081: ERROR SoE: Attribute oversize in transmission */
#define EC_SZTXT_E_SOE_ERRORCODE_OVERS_TRANS    "ERROR SoE: Attribute oversize in transmission"

#define       EC_E_SOE_ERRORCODE_ATTR_UNCHANGE  ((EC_T_DWORD)0x98110082) /**< 0x98110082: ERROR SoE: Attribute unchangeable */
#define EC_SZTXT_E_SOE_ERRORCODE_ATTR_UNCHANGE  "ERROR SoE: Attribute unchangeable"

#define       EC_E_SOE_ERRORCODE_ATTR_WR_PROT   ((EC_T_DWORD)0x98110083) /**< 0x98110083: ERROR SoE: Attribute currently write-protected */
#define EC_SZTXT_E_SOE_ERRORCODE_ATTR_WR_PROT   "ERROR SoE: Attribute currently write-protected"

#define       EC_E_SOE_ERRORCODE_UNIT_NOT_EXIST ((EC_T_DWORD)0x98110084) /**< 0x98110084: ERROR SoE: Unit does not exist */
#define EC_SZTXT_E_SOE_ERRORCODE_UNIT_NOT_EXIST "ERROR SoE: Unit does not exist"

#define       EC_E_SOE_ERRORCODE_UNIT_UNDERSIZE ((EC_T_DWORD)0x98110085) /**< 0x98110085: ERROR SoE: Unit undersize in transmission */
#define EC_SZTXT_E_SOE_ERRORCODE_UNIT_UNDERSIZE "ERROR SoE: Unit undersize in transmission"

#define       EC_E_SOE_ERRORCODE_UNIT_OVERSIZE  ((EC_T_DWORD)0x98110086) /**< 0x98110086: ERROR SoE: Unit oversize in transmission */
#define EC_SZTXT_E_SOE_ERRORCODE_UNIT_OVERSIZE  "ERROR SoE: Unit oversize in transmission"

#define       EC_E_SOE_ERRORCODE_UNIT_UNCHANGE  ((EC_T_DWORD)0x98110087) /**< 0x98110087: ERROR SoE: Unit unchangeable */
#define EC_SZTXT_E_SOE_ERRORCODE_UNIT_UNCHANGE  "ERROR SoE: Unit unchangeable"

#define       EC_E_SOE_ERRORCODE_UNIT_WR_PROT   ((EC_T_DWORD)0x98110088) /**< 0x98110088: ERROR SoE: Unit currently write-protected */
#define EC_SZTXT_E_SOE_ERRORCODE_UNIT_WR_PROT   "ERROR SoE: Unit currently write-protected"

#define       EC_E_SOE_ERRORCODE_MIN_NOT_EXIST  ((EC_T_DWORD)0x98110089) /**< 0x98110089: ERROR SoE: Minimum input value does not exist */
#define EC_SZTXT_E_SOE_ERRORCODE_MIN_NOT_EXIST  "ERROR SoE: Minimum input value does not exist"

#define       EC_E_SOE_ERRORCODE_MIN_UNDERSIZE  ((EC_T_DWORD)0x9811008A) /**< 0x9811008A: ERROR SoE: Minimum input value undersize in transmission */
#define EC_SZTXT_E_SOE_ERRORCODE_MIN_UNDERSIZE  "ERROR SoE: Minimum input value undersize in transmission"

#define       EC_E_SOE_ERRORCODE_MIN_OVERSIZE   ((EC_T_DWORD)0x9811008B) /**< 0x9811008B: ERROR SoE: Minimum input value oversize in transmission */
#define EC_SZTXT_E_SOE_ERRORCODE_MIN_OVERSIZE   "ERROR SoE: Minimum input value oversize in transmission"

#define       EC_E_SOE_ERRORCODE_MIN_UNCHANGE   ((EC_T_DWORD)0x9811008C) /**< 0x9811008C: ERROR SoE: Minimum input value unchangeable */
#define EC_SZTXT_E_SOE_ERRORCODE_MIN_UNCHANGE   "ERROR SoE: Minimum input value unchangeable"

#define       EC_E_SOE_ERRORCODE_MIN_WR_PROT    ((EC_T_DWORD)0x9811008D) /**< 0x9811008D: ERROR SoE: Minimum input value currently write-protected */
#define EC_SZTXT_E_SOE_ERRORCODE_MIN_WR_PROT    "ERROR SoE: Minimum input value currently write-protected"

#define       EC_E_SOE_ERRORCODE_MAX_NOT_EXIST  ((EC_T_DWORD)0x9811008E) /**< 0x9811008E: ERROR SoE: Maximum input value does not exist */
#define EC_SZTXT_E_SOE_ERRORCODE_MAX_NOT_EXIST  "ERROR SoE: Maximum input value does not exist"

#define       EC_E_SOE_ERRORCODE_MAX_UNDERSIZE  ((EC_T_DWORD)0x9811008F) /**< 0x9811008F: ERROR SoE: Maximum input value undersize in transmission */
#define EC_SZTXT_E_SOE_ERRORCODE_MAX_UNDERSIZE  "ERROR SoE: Maximum input value undersize in transmission"

#define       EC_E_SOE_ERRORCODE_MAX_OVERSIZE   ((EC_T_DWORD)0x98110090) /**< 0x98110090: ERROR SoE: Maximum input value oversize in transmission */
#define EC_SZTXT_E_SOE_ERRORCODE_MAX_OVERSIZE   "ERROR SoE: Maximum input value oversize in transmission"

#define       EC_E_SOE_ERRORCODE_MAX_UNCHANGE   ((EC_T_DWORD)0x98110091) /**< 0x98110091: ERROR SoE: Maximum input value unchangeable */
#define EC_SZTXT_E_SOE_ERRORCODE_MAX_UNCHANGE   "ERROR SoE: Maximum input value unchangeable"

#define       EC_E_SOE_ERRORCODE_MAX_WR_PROT    ((EC_T_DWORD)0x98110092) /**< 0x98110092: ERROR SoE: Maximum input value currently write-protected */
#define EC_SZTXT_E_SOE_ERRORCODE_MAX_WR_PROT    "ERROR SoE: Maximum input value currently write-protected"

#define       EC_E_SOE_ERRORCODE_DATA_NOT_EXIST ((EC_T_DWORD)0x98110093) /**< 0x98110093: ERROR SoE: Data item does not exist */
#define EC_SZTXT_E_SOE_ERRORCODE_DATA_NOT_EXIST "ERROR SoE: Data item does not exist"

#define       EC_E_SOE_ERRORCODE_DATA_UNDERSIZE ((EC_T_DWORD)0x98110094) /**< 0x98110094: ERROR SoE: Data item undersize in transmission */
#define EC_SZTXT_E_SOE_ERRORCODE_DATA_UNDERSIZE "ERROR SoE: Data item undersize in transmission"

#define       EC_E_SOE_ERRORCODE_DATA_OVERSIZE  ((EC_T_DWORD)0x98110095) /**< 0x98110095: ERROR SoE: Data item oversize in transmission */
#define EC_SZTXT_E_SOE_ERRORCODE_DATA_OVERSIZE  "ERROR SoE: Data item oversize in transmission"

#define       EC_E_SOE_ERRORCODE_DATA_UNCHANGE  ((EC_T_DWORD)0x98110096) /**< 0x98110096: ERROR SoE: Data item unchangeable */
#define EC_SZTXT_E_SOE_ERRORCODE_DATA_UNCHANGE  "ERROR SoE: Data item unchangeable"

#define       EC_E_SOE_ERRORCODE_DATA_WR_PROT   ((EC_T_DWORD)0x98110097) /**< 0x98110097: ERROR SoE: Data item currently write-protected */
#define EC_SZTXT_E_SOE_ERRORCODE_DATA_WR_PROT   "ERROR SoE: Data item currently write-protected"

#define       EC_E_SOE_ERRORCODE_DATA_MIN_LIMIT ((EC_T_DWORD)0x98110098) /**< 0x98110098: ERROR SoE: Data item less than minimum input value limit */
#define EC_SZTXT_E_SOE_ERRORCODE_DATA_MIN_LIMIT "ERROR SoE: Data item less than minimum input value limit"

#define       EC_E_SOE_ERRORCODE_DATA_MAX_LIMIT ((EC_T_DWORD)0x98110099) /**< 0x98110099: ERROR SoE: Data item exceeds maximum input value limit */
#define EC_SZTXT_E_SOE_ERRORCODE_DATA_MAX_LIMIT "ERROR SoE: Data item exceeds maximum input value limit"

#define       EC_E_SOE_ERRORCODE_DATA_INCOR     ((EC_T_DWORD)0x9811009A) /**< 0x9811009A: ERROR SoE: Data item incorrect */
#define EC_SZTXT_E_SOE_ERRORCODE_DATA_INCOR     "ERROR SoE: Data item incorrect"

#define       EC_E_SOE_ERRORCODE_PASWD_PROT     ((EC_T_DWORD)0x9811009B) /**< 0x9811009B: ERROR SoE: Data item protected by password */
#define EC_SZTXT_E_SOE_ERRORCODE_PASWD_PROT     "ERROR SoE: Data item protected by password"

#define       EC_E_SOE_ERRORCODE_TEMP_UNCHANGE  ((EC_T_DWORD)0x9811009C) /**< 0x9811009C: ERROR SoE: Data item temporary unchangeable (in AT or MDT) */
#define EC_SZTXT_E_SOE_ERRORCODE_TEMP_UNCHANGE  "ERROR SoE: Data item temporary unchangeable (in AT or MDT)"

#define       EC_E_SOE_ERRORCODE_INVL_INDIRECT  ((EC_T_DWORD)0x9811009D) /**< 0x9811009D: ERROR SoE: Invalid indirect */
#define EC_SZTXT_E_SOE_ERRORCODE_INVL_INDIRECT  "ERROR SoE: Invalid indirect"

#define       EC_E_SOE_ERRORCODE_TEMP_UNCHANGE1 ((EC_T_DWORD)0x9811009E) /**< 0x9811009E: ERROR SoE: Data item temporary unchangeable (parameter or opmode) */
#define EC_SZTXT_E_SOE_ERRORCODE_TEMP_UNCHANGE1 "ERROR SoE: Data item temporary unchangeable (parameter or opmode)"

#define       EC_E_SOE_ERRORCODE_ALREADY_ACTIVE ((EC_T_DWORD)0x9811009F) /**< 0x9811009F: ERROR SoE: Command already active */
#define EC_SZTXT_E_SOE_ERRORCODE_ALREADY_ACTIVE "ERROR SoE: Command already active"

#define       EC_E_SOE_ERRORCODE_NOT_INTERRUPT  ((EC_T_DWORD)0x98110100) /**< 0x98110100: ERROR SoE: Command not interruptable */
#define EC_SZTXT_E_SOE_ERRORCODE_NOT_INTERRUPT  "ERROR SoE: Command not interruptable"

#define       EC_E_SOE_ERRORCODE_CMD_NOT_AVAIL  ((EC_T_DWORD)0x98110101) /**< 0x98110101: ERROR SoE: Command not available (in this phase) */
#define EC_SZTXT_E_SOE_ERRORCODE_CMD_NOT_AVAIL  "ERROR SoE: Command not available (in this phase)"

#define       EC_E_SOE_ERRORCODE_CMD_NOT_AVAIL1 ((EC_T_DWORD)0x98110102) /**< 0x98110102: ERROR SoE: Command not available (invalid parameter) */
#define EC_SZTXT_E_SOE_ERRORCODE_CMD_NOT_AVAIL1 "ERROR SoE: Command not available (invalid parameter)"

#define       EC_E_SOE_ERRORCODE_DRIVE_NO       ((EC_T_DWORD)0x98110103) /**< 0x98110103: ERROR SoE: Response drive number not identical with requested drive number */
#define EC_SZTXT_E_SOE_ERRORCODE_DRIVE_NO       "ERROR SoE: Response drive number not identical with requested drive number"

#define       EC_E_SOE_ERRORCODE_IDN            ((EC_T_DWORD)0x98110104) /**< 0x98110104: ERROR SoE: Response IDN not identical with requested IDN */
#define EC_SZTXT_E_SOE_ERRORCODE_IDN            "ERROR SoE: Response IDN not identical with requested IDN"

#define       EC_E_SOE_ERRORCODE_FRAGMENT_LOST  ((EC_T_DWORD)0x98110105) /**< 0x98110105: ERROR SoE: At least one fragment lost */
#define EC_SZTXT_E_SOE_ERRORCODE_FRAGMENT_LOST  "ERROR SoE: At least one fragment lost"

#define       EC_E_SOE_ERRORCODE_BUFFER_FULL    ((EC_T_DWORD)0x98110106) /**< 0x98110106: ERROR SoE: RX buffer full (ecat call with to small data-buffer) */
#define EC_SZTXT_E_SOE_ERRORCODE_BUFFER_FULL    "ERROR SoE: RX buffer full (ecat call with to small data-buffer)"

#define       EC_E_SOE_ERRORCODE_NO_DATA        ((EC_T_DWORD)0x98110107) /**< 0x98110107: ERROR SoE: No data state */
#define EC_SZTXT_E_SOE_ERRORCODE_NO_DATA        "ERROR SoE: No data state"

#define       EC_E_SOE_ERRORCODE_NO_DEFAULT_VALUE  ((EC_T_DWORD)0x98110108) /**< 0x98110108: ERROR SoE: No default value */
#define EC_SZTXT_E_SOE_ERRORCODE_NO_DEFAULT_VALUE  "ERROR SoE: No default value"

#define       EC_E_SOE_ERRORCODE_DEFAULT_LONG   ((EC_T_DWORD)0x98110109) /**< 0x98110109: ERROR SoE: Default value transmission too long */
#define EC_SZTXT_E_SOE_ERRORCODE_DEFAULT_LONG   "ERROR SoE: Default value transmission too long"

#define       EC_E_SOE_ERRORCODE_DEFAULT_WP     ((EC_T_DWORD)0x9811010A) /**< 0x9811010A: ERROR SoE: Default value cannot be changed, read only */
#define EC_SZTXT_E_SOE_ERRORCODE_DEFAULT_WP     "ERROR SoE: Default value cannot be changed, read only"

#define       EC_E_SOE_ERRORCODE_INVL_DRIVE_NO  ((EC_T_DWORD)0x9811010B) /**< 0x9811010B: ERROR SoE: Invalid drive number */
#define EC_SZTXT_E_SOE_ERRORCODE_INVL_DRIVE_NO  "ERROR SoE: Invalid drive number"

#define       EC_E_SOE_ERRORCODE_GENERAL_ERROR  ((EC_T_DWORD)0x9811010C) /**< 0x9811010C: ERROR SoE: General error */
#define EC_SZTXT_E_SOE_ERRORCODE_GENERAL_ERROR  "ERROR SoE: General error"

#define       EC_E_SOE_ERRCODE_NO_ELEM_ADR      ((EC_T_DWORD)0x9811010D) /**< 0x9811010D: ERROR SoE: No element addressed */
#define EC_SZTXT_E_SOE_ERRCODE_NO_ELEM_ADR      "ERROR SoE: No element addressed"
/**@}*/ /* EC_ERROR_CODES_SOE */

/*********************************************************************/
/** \defgroup EC_ERROR_CODES_GENERIC Generic Error Codes
@{*/
#define       EC_E_SLAVE_NOT_PRESENT            ((EC_T_DWORD)0x9811010E) /**< 0x9811010E: APP / SLV: command not executed (slave not present on bus) (e.g. slave disappeared or was never present) */
#define EC_SZTXT_E_SLAVE_NOT_PRESENT            "Command not executed (slave not present on bus)"
/**@}*/ /* EC_ERROR_CODES_GENERIC */

/*********************************************************************/
/** \defgroup EC_ERROR_CODES_FOE File Transfer over EtherCAT (FoE) Error Codes
@{*/
#define       EC_E_NO_FOE_SUPPORT_BS            ((EC_T_DWORD)0x9811010F) /**< 0x9811010F: APP: ERROR FoE: Protocol not supported in boot strap (e.g. Application requested FoE in Bootstrap although slave does not support this) */
#define EC_SZTXT_E_NO_FOE_SUPPORT_BS            "ERROR FoE: Protocol not supported in boot strap"
/**@}*/ /* EC_ERROR_CODES_FOE */

/*********************************************************************/
/** \defgroup EC_ERROR_CODES_GENERIC Generic Error Codes
@{*/
#define       EC_E_EEPROMRELOADERROR            ((EC_T_DWORD)0x98110110) /**< 0x98110110: Command error while EEPROM reload */
#define EC_SZTXT_E_EEPROMRELOADERROR            "ERROR: command error while EEPROM reload"

#define       EC_E_SLAVECTRLRESETERROR          ((EC_T_DWORD)0x98110111) /**< 0x98110111: Command error while Reset Slave Controller */
#define EC_SZTXT_E_SLAVECTRLRESETERROR          "ERROR: command error while Reset Slave Controller"

#define       EC_E_SYSDRIVERMISSING             ((EC_T_DWORD)0x98110112) /**< 0x98110112: SYS: Cannot open system driver (e.g. system driver was not loaded) */
#define EC_SZTXT_E_SYSDRIVERMISSING             "ERROR: Cannot open system driver"

#define       EC_E_BUSCONFIG_TOPOCHANGE         ((EC_T_DWORD)0x9811011E) /**< 0x9811011E: Bus configuration not detected, Topology changed (e.g. Topology changed while scanning bus) */
#define EC_SZTXT_E_BUSCONFIG_TOPOCHANGE         "Bus configuration not detected, Topology changed"

#define       EC_E_EOE_MBX_WKC_ERROR            ((EC_T_DWORD)0x9811011F) /**< 0x9811011F: EoE: Mailbox receive: working counter */
#define EC_SZTXT_E_EOE_MBX_WKC_ERROR            "ERROR EoE: Mailbox receive: working counter"

#define       EC_E_FOE_MBX_WKC_ERROR            ((EC_T_DWORD)0x98110120) /**< 0x98110120: FoE: Mailbox receive: working counter */
#define EC_SZTXT_E_FOE_MBX_WKC_ERROR            "ERROR FoE: Mailbox receive: working counter"

#define       EC_E_SOE_MBX_WKC_ERROR            ((EC_T_DWORD)0x98110121) /**< 0x98110121: SoE: mailbox receive: working counter */
#define EC_SZTXT_E_SOE_MBX_WKC_ERROR            "ERROR SoE: mailbox receive: working counter"

#define       EC_E_AOE_MBX_WKC_ERROR            ((EC_T_DWORD)0x98110122) /**< 0x98110122: AoE: Mailbox receive: working counter */
#define EC_SZTXT_E_AOE_MBX_WKC_ERROR            "ERROR AoE: Mailbox receive: working counter"

#define       EC_E_VOE_MBX_WKC_ERROR            ((EC_T_DWORD)0x98110123) /**< 0x98110123: SLV: VoE mailbox send: working counter (VoE mailbox couldn't be written) */
#define EC_SZTXT_E_VOE_MBX_WKC_ERROR            "ERROR VoE: Mailbox receive: working counter"

#define       EC_E_EEPROMASSIGNERROR            ((EC_T_DWORD)0x98110124) /**< 0x98110124: SLV: EEPROM assignment failed */
#define EC_SZTXT_E_EEPROMASSIGNERROR            "ERROR: EEPROM assignment failed"

#define       EC_E_MBX_ERROR_TYPE               ((EC_T_DWORD)0x98110125) /**< 0x98110125: SLV: Unknown mailbox error code received in mailbox */
#define EC_SZTXT_E_MBX_ERROR_TYPE               "ERROR: Error mailbox received"

#define       EC_E_REDLINEBREAK                 ((EC_T_DWORD)0x98110126) /**< 0x98110126: SLV: Redundancy line break (e.g. cable break between slaves or between master and first slave) */
#define EC_SZTXT_E_REDLINEBREAK                 "ERROR: Redundancy line break"

#define       EC_E_XML_INVALID_CMD_WITH_RED     ((EC_T_DWORD)0x98110127) /**< 0x98110127: ENI: Invalid EtherCAT cmd in cyclic frame with redundancy (e.g. BRW commands are not allowed with redundancy) */
#define EC_SZTXT_E_XML_INVALID_CMD_WITH_RED     "ERROR: Invalid EtherCAT cmd in cyclic frame with redundancy"

#define       EC_E_XML_PREV_PORT_MISSING        ((EC_T_DWORD)0x98110128) /**< 0x98110128: ENI: <PreviousPort>-tag is missing (e.g. if the auto increment address is not the first slave on the bus we check if a previous port tag OR a hot connect tag is available) */
#define EC_SZTXT_E_XML_PREV_PORT_MISSING        "ERROR: <PreviousPort>-tag missing"

#define       EC_E_XML_DC_CYCCMDS_MISSING       ((EC_T_DWORD)0x98110129) /**< 0x98110129: DC enabled and DC cyclic commands missing (e.g. access to 0x0900) */
#define EC_SZTXT_E_XML_DC_CYCCMDS_MISSING       "ERROR: DC enabled and DC cyclic commands missing (e.g. access to 0x0900)"

#define       EC_E_DLSTATUS_IRQ_TOPOCHANGED     ((EC_T_DWORD)0x98110130) /**< 0x98110130: SLV: Data link (DL) status interrupt because of changed topology (automatically handled by master) */
#define EC_SZTXT_E_DLSTATUS_IRQ_TOPOCHANGED     "ERROR: DL Status Interrupt because of changed Topology"

#define       EC_E_PTS_IS_NOT_RUNNING           ((EC_T_DWORD)0x98110131) /**< 0x98110131: PTS: Pass Through Server is not running (Pass-Through-Server was tried to be enabled/disabled or stopped without being started) */
#define EC_SZTXT_E_PTS_IS_NOT_RUNNING           "ERROR: Pass Through Server not running"

#define       EC_E_PTS_IS_RUNNING               ((EC_T_DWORD)0x98110132) /**< 0x98110132: PTS: Pass Through Server is running (obsolete, replaced by EC_E_ADS_IS_RUNNING) */
#define       EC_E_ADS_IS_RUNNING               ((EC_T_DWORD)0x98110132) /**< 0x98110132: PTS: ADS adapter (Pass Through Server) is running (API call conflicts with ADS state (running)) */
#define EC_SZTXT_E_PTS_IS_RUNNING               "ERROR: ADS adapter (Pass Through Server) running"

#define       EC_E_PTS_THREAD_CREATE_FAILED     ((EC_T_DWORD)0x98110133) /**< 0x98110133: PTS: Could not start the Pass Through Server */
#define EC_SZTXT_E_PTS_THREAD_CREATE_FAILED     "ERROR: Could not start Pass Through Server"

#define       EC_E_PTS_SOCK_BIND_FAILED         ((EC_T_DWORD)0x98110134) /**< 0x98110134: PTS: The Pass Through Server could not bind the IP address with a socket (e.g. Possibly because the IPaddress (and Port) is already in use or the IP-address does not exist) */
#define EC_SZTXT_E_PTS_SOCK_BIND_FAILED         "ERROR: Pass Through Server could not bind IP address with socket"

#define       EC_E_PTS_NOT_ENABLED              ((EC_T_DWORD)0x98110135) /**< 0x98110135: PTS: The Pass Through Server is running but not enabled */
#define EC_SZTXT_E_PTS_NOT_ENABLED              "Pass Through Server running but not enabled"

#define       EC_E_PTS_LL_MODE_NOT_SUPPORTED    ((EC_T_DWORD)0x98110136) /**< 0x98110136: PTS: The Link Layer mode is not supported by the Pass Through Server (e.g. The Master is running in interrupt mode but the Pass-Through-Server only supports polling mode) */
#define EC_SZTXT_E_PTS_LL_MODE_NOT_SUPPORTED    "ERROR: LinkLayer mode not supported by Pass Through Server"

#define       EC_E_VOE_NO_MBX_RECEIVED          ((EC_T_DWORD)0x98110137) /**< 0x98110137: SLV: No VoE mailbox received yet from specific slave */
#define EC_SZTXT_E_VOE_NO_MBX_RECEIVED          "ERROR VoE: No VoE mailbox received"

#define       EC_E_DC_REF_CLOCK_SYNC_OUT_UNIT_DISABLED ((EC_T_DWORD)0x98110138) /**< 0x98110138: DC (time loop control) unit of reference clock disabled */
#define EC_SZTXT_E_DC_REF_CLOCK_SYNC_OUT_UNIT_DISABLED "ERROR: DC (time loop control) unit of reference clock disabled"

#define       EC_E_DC_REF_CLOCK_NOT_FOUND       ((EC_T_DWORD)0x98110139) /**< 0x98110139: SLV: Reference clock not found! May happen if reference clock is removed from network. */
#define EC_SZTXT_E_DC_REF_CLOCK_NOT_FOUND       "ERROR: Reference clock not found"

#define       EC_E_MBX_CMD_WKC_ERROR            ((EC_T_DWORD)0x9811013B) /**< 0x9811013B: SLV: Mailbox command working counter error (e.g. Mbx Init Cmd Retry Count exceeded) */
#define EC_SZTXT_E_MBX_CMD_WKC_ERROR            "ERROR: Mailbox command working counter error"

#define       EC_E_NO_AOE_SUPPORT               ((EC_T_DWORD)0x9811013C) /**< 0x9811013C: APP / SLV: AoE: Protocol not supported (e.g. Application calls AoE-API although not implemented at slave) */
#define EC_SZTXT_E_NO_AOE_SUPPORT               "ERROR AoE: Protocol not supported"

#define       EC_E_AOE_INV_RESPONSE_SIZE        ((EC_T_DWORD)0x9811013D) /**< 0x9811013D: AoE: Invalid AoE response received */
#define EC_SZTXT_E_AOE_INV_RESPONSE_SIZE        "ERROR AoE: Invalid AoE response received"

#define       EC_E_AOE_ERROR                    ((EC_T_DWORD)0x9811013E) /**< 0x9811013E: AoE: Common AoE device error */
#define EC_SZTXT_E_AOE_ERROR                    "ERROR AoE: Common AoE device error"

#define       EC_E_AOE_SRVNOTSUPP               ((EC_T_DWORD)0x9811013F) /**< 0x9811013F: AoE: Service not supported by server */
#define EC_SZTXT_E_AOE_SRVNOTSUPP               "ERROR AoE: Service not supported by server"

#define       EC_E_AOE_INVALIDGRP               ((EC_T_DWORD)0x98110140) /**< 0x98110140: AoE: Invalid index group */
#define EC_SZTXT_E_AOE_INVALIDGRP               "ERROR AoE: Invalid index group"

#define       EC_E_AOE_INVALIDOFFSET            ((EC_T_DWORD)0x98110141) /**< 0x98110141: AoE: Invalid index offset */
#define EC_SZTXT_E_AOE_INVALIDOFFSET            "ERROR AoE: Invalid index offset"

#define       EC_E_AOE_INVALIDACCESS            ((EC_T_DWORD)0x98110142) /**< 0x98110142: AoE: Reading/writing not permitted */
#define EC_SZTXT_E_AOE_INVALIDACCESS            "ERROR AoE: Reading/writing not permitted"

#define       EC_E_AOE_INVALIDSIZE              ((EC_T_DWORD)0x98110143) /**< 0x98110143: AoE: Parameter size not correct */
#define EC_SZTXT_E_AOE_INVALIDSIZE              "ERROR AoE: Parameter size not correct"

#define       EC_E_AOE_INVALIDDATA              ((EC_T_DWORD)0x98110144) /**< 0x98110144: AoE: Invalid parameter value(s) */
#define EC_SZTXT_E_AOE_INVALIDDATA              "ERROR AoE: Invalid parameter value(s)"

#define       EC_E_AOE_NOTREADY                 ((EC_T_DWORD)0x98110145) /**< 0x98110145: AoE: Device not in a ready state */
#define EC_SZTXT_E_AOE_NOTREADY                 "ERROR AoE: Device not in a ready state"

#define       EC_E_AOE_BUSY                     ((EC_T_DWORD)0x98110146) /**< 0x98110146: AoE: Device busy */
#define EC_SZTXT_E_AOE_BUSY                     "ERROR AoE: Device busy"

#define       EC_E_AOE_INVALIDCONTEXT           ((EC_T_DWORD)0x98110147) /**< 0x98110147: AoE: Invalid context */
#define EC_SZTXT_E_AOE_INVALIDCONTEXT           "ERROR AoE: Invalid context"

#define       EC_E_AOE_NOMEMORY                 ((EC_T_DWORD)0x98110148) /**< 0x98110148: AoE: Out of memory */
#define EC_SZTXT_E_AOE_NOMEMORY                 "ERROR AoE: Out of memory"

#define       EC_E_AOE_INVALIDPARM              ((EC_T_DWORD)0x98110149) /**< 0x98110149: AoE: Invalid parameter value(s) */
#define EC_SZTXT_E_AOE_INVALIDPARM              "ERROR AoE: Invalid parameter value(s)"

#define       EC_E_AOE_NOTFOUND                 ((EC_T_DWORD)0x9811014A) /**< 0x9811014A: AoE: Not found */
#define EC_SZTXT_E_AOE_NOTFOUND                 "ERROR AoE: Not found"

#define       EC_E_AOE_SYNTAX                   ((EC_T_DWORD)0x9811014B) /**< 0x9811014B: AoE: Syntax error in comand or file */
#define EC_SZTXT_E_AOE_SYNTAX                   "ERROR AoE: Syntax error in comand or file"

#define       EC_E_AOE_INCOMPATIBLE             ((EC_T_DWORD)0x9811014C) /**< 0x9811014C: AoE: Objects do not match */
#define EC_SZTXT_E_AOE_INCOMPATIBLE             "ERROR AoE: Objects do not match"

#define       EC_E_AOE_EXISTS                   ((EC_T_DWORD)0x9811014D) /**< 0x9811014D: AoE: Object already exists */
#define EC_SZTXT_E_AOE_EXISTS                   "ERROR AoE: Object already exists"

#define       EC_E_AOE_SYMBOLNOTFOUND           ((EC_T_DWORD)0x9811014E) /**< 0x9811014E: AoE: Symbol not found */
#define EC_SZTXT_E_AOE_SYMBOLNOTFOUND           "ERROR AoE: Symbol not found"

#define       EC_E_AOE_SYMBOLVERSIONINVALID     ((EC_T_DWORD)0x9811014F) /**< 0x9811014F: AoE: Symbol version invalid */
#define EC_SZTXT_E_AOE_SYMBOLVERSIONINVALID     "ERROR AoE: Symbol version invalid"

#define       EC_E_AOE_INVALIDSTATE             ((EC_T_DWORD)0x98110150) /**< 0x98110150: AoE: Server in invalid state */
#define EC_SZTXT_E_AOE_INVALIDSTATE             "ERROR AoE: Server in invalid state"

#define       EC_E_AOE_TRANSMODENOTSUPP         ((EC_T_DWORD)0x98110151) /**< 0x98110151: AoE: AdsTransMode not supported */
#define EC_SZTXT_E_AOE_TRANSMODENOTSUPP         "ERROR AoE: AdsTransMode not supported"

#define       EC_E_AOE_NOTIFYHNDINVALID         ((EC_T_DWORD)0x98110152) /**< 0x98110152: AoE: Notification handle invalid */
#define EC_SZTXT_E_AOE_NOTIFYHNDINVALID         "ERROR AoE: Notification handle invalid"

#define       EC_E_AOE_CLIENTUNKNOWN            ((EC_T_DWORD)0x98110153) /**< 0x98110153: AoE: Notification client not registered */
#define EC_SZTXT_E_AOE_CLIENTUNKNOWN            "ERROR AoE: Notification client not registered"

#define       EC_E_AOE_NOMOREHDLS               ((EC_T_DWORD)0x98110154) /**< 0x98110154: AoE: No more notification handles */
#define EC_SZTXT_E_AOE_NOMOREHDLS               "ERROR AoE: No more notification handles"

#define       EC_E_AOE_INVALIDWATCHSIZE         ((EC_T_DWORD)0x98110155) /**< 0x98110155: AoE: Size for watch to big */
#define EC_SZTXT_E_AOE_INVALIDWATCHSIZE         "ERROR AoE: Size for watch to big"

#define       EC_E_AOE_NOTINIT                  ((EC_T_DWORD)0x98110156) /**< 0x98110156: AoE: Device not initialized */
#define EC_SZTXT_E_AOE_NOTINIT                  "ERROR AoE: Device not initialized"

#define       EC_E_AOE_TIMEOUT                  ((EC_T_DWORD)0x98110157) /**< 0x98110157: AoE: Device has a timeout */
#define EC_SZTXT_E_AOE_TIMEOUT                  "ERROR AoE: Device has a timeout"

#define       EC_E_AOE_NOINTERFACE              ((EC_T_DWORD)0x98110158) /**< 0x98110158: AoE: Query interface failed */
#define EC_SZTXT_E_AOE_NOINTERFACE              "ERROR AoE: Query interface failed"

#define       EC_E_AOE_INVALIDINTERFACE         ((EC_T_DWORD)0x98110159) /**< 0x98110159: AoE: Wrong interface required */
#define EC_SZTXT_E_AOE_INVALIDINTERFACE         "ERROR AoE: Wrong interface required"

#define       EC_E_AOE_INVALIDCLSID             ((EC_T_DWORD)0x9811015A) /**< 0x9811015A: AoE: Class ID invalid */
#define EC_SZTXT_E_AOE_INVALIDCLSID             "ERROR AoE: Class ID invalid"

#define       EC_E_AOE_INVALIDOBJID             ((EC_T_DWORD)0x9811015B) /**< 0x9811015B: AoE: Object ID invalid */
#define EC_SZTXT_E_AOE_INVALIDOBJID             "ERROR AoE: Object ID invalid"

#define       EC_E_AOE_PENDING                  ((EC_T_DWORD)0x9811015C) /**< 0x9811015C: AoE: Request pending */
#define EC_SZTXT_E_AOE_PENDING                  "ERROR AoE: Request pending"

#define       EC_E_AOE_ABORTED                  ((EC_T_DWORD)0x9811015D) /**< 0x9811015D: AoE: Request aborted */
#define EC_SZTXT_E_AOE_ABORTED                  "ERROR AoE: Request aborted"

#define       EC_E_AOE_WARNING                  ((EC_T_DWORD)0x9811015E) /**< 0x9811015E: AoE: Signal warning */
#define EC_SZTXT_E_AOE_WARNING                  "ERROR AoE: Signal warning"

#define       EC_E_AOE_INVALIDARRAYIDX          ((EC_T_DWORD)0x9811015F) /**< 0x9811015F: AoE: Invalid array index */
#define EC_SZTXT_E_AOE_INVALIDARRAYIDX          "ERROR AoE: Invalid array index"

#define       EC_E_AOE_SYMBOLNOTACTIVE          ((EC_T_DWORD)0x98110160) /**< 0x98110160: AoE: Symbol not active -> release handle and try again */
#define EC_SZTXT_E_AOE_SYMBOLNOTACTIVE          "ERROR AoE: Symbol not active -> release handle and try again"

#define       EC_E_AOE_ACCESSDENIED             ((EC_T_DWORD)0x98110161) /**< 0x98110161: AoE: Access denied */
#define EC_SZTXT_E_AOE_ACCESSDENIED             "ERROR AoE: Access denied"

#define       EC_E_AOE_INTERNAL                 ((EC_T_DWORD)0x98110162) /**< 0x98110162: AoE: Internal error */
#define EC_SZTXT_E_AOE_INTERNAL                 "ERROR AoE: Internal error"

#define       EC_E_AOE_TARGET_PORT_NOT_FOUND    ((EC_T_DWORD)0x98110163) /**< 0x98110163: AoE: Target port not found */
#define EC_SZTXT_E_AOE_TARGET_PORT_NOT_FOUND    "ERROR AoE: Target port not found"

#define       EC_E_AOE_TARGET_MACHINE_NOT_FOUND ((EC_T_DWORD)0x98110164) /**< 0x98110164: AoE: Target machine not found */
#define EC_SZTXT_E_AOE_TARGET_MACHINE_NOT_FOUND "ERROR AoE: Target machine not found"

#define       EC_E_AOE_UNKNOWN_CMD_ID           ((EC_T_DWORD)0x98110165) /**< 0x98110165: AoE: Unknown command ID */
#define EC_SZTXT_E_AOE_UNKNOWN_CMD_ID           "ERROR AoE: Unknown command ID"

#define       EC_E_AOE_PORT_NOT_CONNECTED       ((EC_T_DWORD)0x98110166) /**< 0x98110166: AoE: Port not connected */
#define EC_SZTXT_E_AOE_PORT_NOT_CONNECTED       "ERROR AoE: Port not connected"

#define       EC_E_AOE_INVALID_AMS_LENGTH       ((EC_T_DWORD)0x98110167) /**< 0x98110167: AoE: Invalid AMS length */
#define EC_SZTXT_E_AOE_INVALID_AMS_LENGTH       "ERROR AoE: Invalid AMS length"

#define       EC_E_AOE_INVALID_AMS_ID           ((EC_T_DWORD)0x98110168) /**< 0x98110168: AoE: invalid AMS Net ID */
#define EC_SZTXT_E_AOE_INVALID_AMS_ID           "ERROR AoE: invalid AMS Net ID"

#define       EC_E_AOE_PORT_DISABLED            ((EC_T_DWORD)0x98110169) /**< 0x98110169: AoE: Port disabled */
#define EC_SZTXT_E_AOE_PORT_DISABLED            "ERROR AoE: Port disabled"

#define       EC_E_AOE_PORT_CONNECTED           ((EC_T_DWORD)0x9811016A) /**< 0x9811016A: AoE: Port already connected */
#define EC_SZTXT_E_AOE_PORT_CONNECTED           "ERROR AoE: Port already connected"

#define       EC_E_AOE_INVALID_AMS_PORT         ((EC_T_DWORD)0x9811016B) /**< 0x9811016B: AoE: Invalid AMS port */
#define EC_SZTXT_E_AOE_INVALID_AMS_PORT         "ERROR AoE: Invalid AMS port"

#define       EC_E_AOE_NO_MEMORY                ((EC_T_DWORD)0x9811016C) /**< 0x9811016C: AoE: No memory */
#define EC_SZTXT_E_AOE_NO_MEMORY                "ERROR AoE: No memory"

#define       EC_E_AOE_VENDOR_SPECIFIC          ((EC_T_DWORD)0x9811016D) /**< 0x9811016D: AoE: Vendor specific AoE device error */
#define EC_SZTXT_E_AOE_VENDOR_SPECIFIC          "ERROR AoE: Vendor specific AoE device error"

#define       EC_E_XML_AOE_NETID_INVALID        ((EC_T_DWORD)0x9811016E) /**< 0x9811016E: ENI: AoE: Invalid NetID (e.g. Error from Configuration Tool) */
#define EC_SZTXT_E_XML_AOE_NETID_INVALID        "ERROR AoE: Invalid NetID"

#define       EC_E_MAX_BUS_SLAVES_EXCEEDED      ((EC_T_DWORD)0x9811016F) /**< 0x9811016F: CFG: Error: Maximum number of bus slave has been exceeded (The maximum number of pre-allocated bus slave objects are to small. The maximum number can be adjusted by the master initialization parameter EC_T_INITMASTERPARMS.dwMaxBusSlaves) */
#define EC_SZTXT_E_MAX_BUS_SLAVES_EXCEEDED      "ERROR: Maximum number of bus slave has been exceeded"

#define       EC_E_MBXERR_SYNTAX                ((EC_T_DWORD)0x98110170) /**< 0x98110170: SLV: Mailbox error: Syntax of 6 octet Mailbox header is wrong (Slave error mailbox return value: 0x01) */
#define EC_SZTXT_E_MBXERR_SYNTAX                "ERROR Mailbox: Syntax of 6 octet Mailbox header wrong"

#define       EC_E_MBXERR_UNSUPPORTEDPROTOCOL   ((EC_T_DWORD)0x98110171) /**< 0x98110171: SLV: Mailbox error: The Mailbox protocol is not supported (Slave error mailbox return value: 0x02) */
#define EC_SZTXT_E_MBXERR_UNSUPPORTEDPROTOCOL   "ERROR Mailbox: Mailbox protocol not supported"

#define       EC_E_MBXERR_INVALIDCHANNEL        ((EC_T_DWORD)0x98110172) /**< 0x98110172: SLV: Mailbox error: Field contains wrong value (Slave error mailbox return value: 0x03) */
#define EC_SZTXT_E_MBXERR_INVALIDCHANNEL        "ERROR Mailbox: Field contains wrong value"

#define       EC_E_MBXERR_SERVICENOTSUPPORTED   ((EC_T_DWORD)0x98110173) /**< 0x98110173: SLV: Mailbox error: The mailbox protocol header of the mailbox protocol is wrong (Slave error mailbox return value: 0x04) */
#define EC_SZTXT_E_MBXERR_SERVICENOTSUPPORTED   "ERROR Mailbox: service in Mailbox protocol not supported"

#define       EC_E_MBXERR_INVALIDHEADER         ((EC_T_DWORD)0x98110174) /**< 0x98110174: SLV: Mailbox error: The mailbox protocol header of the mailbox protocol is wrong (Slave error mailbox return value: 0x05) */
#define EC_SZTXT_E_MBXERR_INVALIDHEADER         "ERROR Mailbox: mailbox protocol header of mailbox protocol wrong"

#define       EC_E_MBXERR_SIZETOOSHORT          ((EC_T_DWORD)0x98110175) /**< 0x98110175: SLV: Mailbox error: Length of received mailbox data is too short (Slave error mailbox return value: 0x06) */
#define EC_SZTXT_E_MBXERR_SIZETOOSHORT          "ERROR Mailbox: Length of received mailbox data too short"

#define       EC_E_MBXERR_NOMOREMEMORY          ((EC_T_DWORD)0x98110176) /**< 0x98110176: SLV: Mailbox error: Mailbox protocol can not be processed because of limited ressources (Slave error mailbox return value: 0x07) */
#define EC_SZTXT_E_MBXERR_NOMOREMEMORY          "ERROR Mailbox: Mailbox protocol can not be processed because of limited ressources"

#define       EC_E_MBXERR_INVALIDSIZE           ((EC_T_DWORD)0x98110177) /**< 0x98110177: SLV: Mailbox error: The length of data is inconsistent (Slave error mailbox return value: 0x08) */
#define EC_SZTXT_E_MBXERR_INVALIDSIZE           "ERROR Mailbox: length of data inconsistent"

#define       EC_E_DC_SLAVES_BEFORE_REF_CLOCK   ((EC_T_DWORD)0x98110178) /**< 0x98110178: ENI: Slaves with DC configured present on bus before reference clock (e.g. The first DC Slave was not configured as potential reference clock) */
#define EC_SZTXT_E_DC_SLAVES_BEFORE_REF_CLOCK   "ERROR: Slaves with DC configured present on bus before reference clock"

#define       EC_E_DATA_TYPE_CONVERSION_FAILED  ((EC_T_DWORD)0x98110179) /**< 0x98110179: Data type conversion failed */
#define EC_SZTXT_E_DATA_TYPE_CONVERSION_FAILED  "ERROR: Data type conversion failed"
/**@}*/ /* EC_ERROR_CODES_GENERIC */

/*********************************************************************/
/** \defgroup EC_ERROR_CODES_FOE File Transfer over EtherCAT (FoE) Error Codes
@{*/
#define       EC_E_FOE_ERRCODE_MAX_FILE_SIZE    ((EC_T_DWORD)0x9811017A) /**< 0x9811017A: APP: ERROR FoE: File is bigger than max file size (e.g. Slave returned more data than the buffer provided by application can store.) */
#define EC_SZTXT_E_FOE_ERRCODE_MAX_FILE_SIZE    "ERROR FoE: File bigger than max file size"
/**@}*/ /* EC_ERROR_CODES_FOE */

/*********************************************************************/
/** \defgroup EC_ERROR_CODES_GENERIC Generic Error Codes
@{*/
#define       EC_E_LINE_CROSSED                 ((EC_T_DWORD)0x9811017B) /**< 0x9811017B: Line crossed (cabling wrong) */
#define EC_SZTXT_E_LINE_CROSSED                 "ERROR: Line crossed"

#define       EC_E_LINE_CROSSED_SLAVE_INFO      ((EC_T_DWORD)0x9811017C) /**< 0x9811017C: Line crossed at slave (obsolete) */
#define EC_SZTXT_E_LINE_CROSSED_SLAVE_INFO      "ERROR: Line crossed at slave %s, EtherCAT auto-increment address=%d, station address=%d. Error at port %d."
/**@}*/ /* EC_ERROR_CODES_GENERIC */

/*********************************************************************/
/** \defgroup EC_ERROR_CODES_RAS Remote API Error Codes
@{*/
#define       EC_E_SOCKET_DISCONNECTED          ((EC_T_DWORD)0x9811017D) /**< 0x9811017D: RAS: Socket disconnected (e.g. IP connection terminated or lost) */
#define EC_SZTXT_E_SOCKET_DISCONNECTED          "ERROR: Socket disconnected"
/**@}*/ /* EC_ERROR_CODES_RAS */

/*********************************************************************/
/** \defgroup EC_ERROR_CODES_GENERIC Generic Error Codes
@{*/
#define       EC_E_ADO_NOT_SUPPORTED            ((EC_T_DWORD)0x9811017E) /**< 0x9811017E: SLV: ADO for slave identification not supported (e.g. Request ID mechanism (ADO 0x134) not supported by slave) */
#define EC_SZTXT_E_ADO_NOT_SUPPORTED            "ERROR: Ado not supported"

#define       EC_E_FRAMELOSS_AFTER_SLAVE        ((EC_T_DWORD)0x9811017F) /**< 0x9811017F: Frameloss after Slave (opening port destroys communication) */
#define EC_SZTXT_E_FRAMELOSS_AFTER_SLAVE        "ERROR: Frameloss after Slave"
/**@}*/ /* EC_ERROR_CODES_GENERIC */

#define       EC_E_ERROR_LAST                   ((EC_T_DWORD)0x9811017F)
/* see next block EC_E_ERROR2...EC_E_ERROR_LAST2  */

/*********************************************************************/
/** \defgroup EC_ERROR_CODES_DCM DCM Error Codes
@{*/
#define       DCM_E_ERROR                       ((EC_T_DWORD)0x981201C0)    /**< 0x981201C0: Unspecific DCM Error */
#define DCM_SZTXT_E_ERROR                       "Unspecific DCM Error"

#define       DCM_E_NOTINITIALIZED              ((EC_T_DWORD)0x981201C1)    /**< 0x981201C1: Not initialized  */
#define DCM_SZTXT_E_NOTINITIALIZED              "ERROR: init function not called or not successful"

#define       DCM_E_MAX_CTL_ERROR_EXCEED        ((EC_T_DWORD)0x981201C2)    /**< 0x981201C2: DCM controller - synchronisation out of limit */
#define DCM_SZTXT_E_MAX_CTL_ERROR_EXCEED        "ERROR: controller error - synchronisation out of limit"

#define       DCM_E_NOMEMORY                    ((EC_T_DWORD)0x981201C3)    /**< 0x981201C3: Not enough memory */
#define DCM_SZTXT_E_NOMEMORY                    "ERROR: not enough memory"

#define       DCM_E_INVALID_HWLAYER             ((EC_T_DWORD)0x981201C4)    /**< 0x981201C4: Hardware layer - (BSP) invalid */
#define DCM_SZTXT_E_INVALID_HWLAYER             "ERROR: hardware layer - (BSP) invalid"

#define       DCM_E_TIMER_MODIFY_ERROR          ((EC_T_DWORD)0x981201C5)    /**< 0x981201C5: Hardware layer - error modifying timer */
#define DCM_SZTXT_E_TIMER_MODIFY_ERROR          "ERROR: hardware layer - error modifying timer"

#define       DCM_E_TIMER_NOT_RUNNING           ((EC_T_DWORD)0x981201C6)    /**< 0x981201C6: Hardware layer - timer not running */
#define DCM_SZTXT_E_TIMER_NOT_RUNNING           "ERROR: hardware layer - timer not running"

#define       DCM_E_WRONG_CPU                   ((EC_T_DWORD)0x981201C7)    /**< 0x981201C7: Hardware layer - function called on wrong CPU */
#define DCM_SZTXT_E_WRONG_CPU                   "ERROR: hardware layer - function called on wrong CPU"

#define       DCM_E_INVALID_SYNC_PERIOD         ((EC_T_DWORD)0x981201C8)    /**< 0x981201C8: Invalid DC sync period length (invalid clock master?) */
#define DCM_SZTXT_E_INVALID_SYNC_PERIOD         "ERROR: invalid DC sync period length (invalid clock master?)"

#define       DCM_E_INVALID_SETVAL              ((EC_T_DWORD)0x981201C9)    /**< 0x981201C9: DCM controller SetVal to small */
#define DCM_SZTXT_E_INVALID_SETVAL              "ERROR: Error DCM Controller SetVal to small"

#define       DCM_E_DRIFT_TO_HIGH               ((EC_T_DWORD)0x981201CA)    /**< 0x981201CA: DCM controller - Drift between local timer and ref clock to high */
#define DCM_SZTXT_E_DRIFT_TO_HIGH               "ERROR: Error DCM Controller - Drift between local timer and ref clock to high"

#define       DCM_E_BUS_CYCLE_WRONG             ((EC_T_DWORD)0x981201CB)    /**< 0x981201CB: DCM controller - Bus cycle time (dwBusCycleTimeUsec) doesn't match real cycle */
#define DCM_SZTXT_E_BUS_CYCLE_WRONG              "ERROR: Error DCM Controller - Bus cycle time (dwBusCycleTimeUsec) doesn't match real cycle"

#define       DCX_E_NO_EXT_CLOCK                ((EC_T_DWORD)0x981201CC)    /**< 0x981201CC: DCX controller - No external synchronization clock found */
#define DCX_SZTXT_E_NO_EXT_CLOCK                 "ERROR: Error DCX Controller - No external synchronization clock found"

#define       DCM_E_INVALID_DATA                ((EC_T_DWORD)0x981201CD)    /**< 0x981201CD: DCM controller - Invalid data */
#define DCM_SZTXT_E_INVALID_DATA                 "ERROR: Error DCM Controller - Invalid data"
/**@}*/ /* EC_ERROR_CODES_DCM */
#define       DCM_E_ERROR_LAST                  ((EC_T_DWORD)0x981201FF)


#define       EC_E_ERROR2                       ((EC_T_DWORD)0x98130000)

/*********************************************************************/
/** \defgroup EC_ERROR_CODES_FOE File Transfer over EtherCAT (FoE) Error Codes
@{*/
#define       EC_E_FOE_ERRCODE_FILE_HEAD_MISSING  ((EC_T_DWORD)0x98130001) /**< 0x98130001: SLV: ERROR FoE: File header does not exist (FoE Error Code 16 (0x8010) of slave) */
#define EC_SZTXT_E_FOE_ERRCODE_FILE_HEAD_MISSING "ERROR FoE: File header does not exist"

#define       EC_E_FOE_ERRCODE_FLASH_PROBLEM     ((EC_T_DWORD)0x98130002) /**< 0x98130002: SLV: ERROR FoE: Flash problem (FoE Error Code 17 (0x8011) of slave) */
#define EC_SZTXT_E_FOE_ERRCODE_FLASH_PROBLEM     "ERROR FoE: Flash problem"

#define       EC_E_FOE_ERRCODE_FILE_INCOMPATIBLE  ((EC_T_DWORD)0x98130003) /**< 0x98130003: SLV: ERROR FoE: File incompatible (FoE Error Code 18 (0x8012) of slave) */
#define EC_SZTXT_E_FOE_ERRCODE_FILE_INCOMPATIBLE "ERROR FoE: File incompatible"
/**@}*/ /* EC_ERROR_CODES_FOE */

/*********************************************************************/
/** \defgroup EC_ERROR_CODES_COE CAN application protocol over EtherCAT (CoE) SDO Error Codes
@{*/
#define       EC_E_SDO_ABORTCODE_SI_NOT_WRITTEN  ((EC_T_DWORD)0x98130004) /**< 0x98130004: SLV: SDO: Sub Index cannot be written, SI0 must be 0 for write access (CoE abort code 0x06010003 of slave) */
#define EC_SZTXT_E_SDO_ABORTCODE_SI_NOT_WRITTEN  "ERROR SDO: Subindex cannot be written, SI0 must be 0 for write access"

#define       EC_E_SDO_ABORTCODE_CA_TYPE_MISM    ((EC_T_DWORD)0x98130005) /**< 0x98130005: SLV: SDO: Complete access not supported for objects of cvariable length suach as ENUM object types (CoE abort code 0x06010004 of slave) */
#define EC_SZTXT_E_SDO_ABORTCODE_CA_TYPE_MISM    "ERROR SDO: Complete access not supported for objects of variable length such as ENUM object types"

#define       EC_E_SDO_ABORTCODE_OBJ_TOO_BIG     ((EC_T_DWORD)0x98130006) /**< 0x98130006: SLV: SDO: Object length exceeds mailbox size (CoE abort code 0x06010005 of slave) */
#define EC_SZTXT_E_SDO_ABORTCODE_OBJ_TOO_BIG     "ERROR SDO: Object length exceeds mailbox size"

#define       EC_E_SDO_ABORTCODE_PDO_MAPPED      ((EC_T_DWORD)0x98130007) /**< 0x98130007: SLV: SDO: Object mapped to RxPDO, SDO Download blocked (CoE abort code 0x06010006 of slave) */
#define EC_SZTXT_E_SDO_ABORTCODE_PDO_MAPPED      "ERROR SDO: Object mapped to RxPDO, SDO Download blocked"
/**@}*/ /* EC_ERROR_CODES_COE */

/*********************************************************************/
/** \defgroup EC_ERROR_CODES_GENERIC Generic Error Codes
@{*/
#define       EC_E_OEM_SIGNATURE_MISMATCH        ((EC_T_DWORD)0x98130008) /**< 0x98130008: ENI, OEM: Manufacturer signature mismatch */
#define EC_SZTXT_E_OEM_SIGNATURE_MISMATCH        "ERROR manufacturer signature mismatch"

#define       EC_E_ENI_ENCRYPTION_WRONG_VERSION  ((EC_T_DWORD)0x98130009) /**< 0x98130009: ENI, OEM: ENI encryption algorithm version not supported */
#define EC_SZTXT_E_ENI_ENCRYPTION_WRONG_VERSION  "ERROR ENI encryption algorithm version not supported"

#define       EC_E_ENI_ENCRYPTED                 ((EC_T_DWORD)0x9813000A) /**< 0x9813000A: OEM: Loading encrypted ENI needs OEM key */
#define EC_SZTXT_E_ENI_ENCRYPTED                 "ERROR loading encrypted ENI needs OEM key set (ecatSetOemKey)"

#define       EC_E_OEM_KEY_MISMATCH              ((EC_T_DWORD)0x9813000B) /**< 0x9813000B: RAS, APP: OEM key mismatch */
#define EC_SZTXT_E_OEM_KEY_MISMATCH              "ERROR: OEM key mismatch"

#define       EC_E_OEM_KEY_MISSING               ((EC_T_DWORD)0x9813000C) /**< 0x9813000C: APP: OEM key access needs OEM key set (e.g. Application must call esSetOemKey (HiL) or set EC_T_LINK_PARMS_SIMULATOR::qwOemKey (SiL)) */
#define EC_SZTXT_E_OEM_KEY_MISSING               "ERROR: OEM key access needs OEM key set (ecatSetOemKey)"
/**@}*/ /* EC_ERROR_CODES_GENERIC */

/*********************************************************************/
/** \defgroup EC_ERROR_CODES_AOE ADS over EtherCAT (AoE) Error Codes
@{*/
#define       EC_E_AOE_NO_RTIME                  ((EC_T_DWORD)0x9813000D) /**< 0x9813000D: AoE: No Rtime */
#define EC_SZTXT_E_AOE_NO_RTIME                  "ERROR AoE: No Rtime"

#define       EC_E_AOE_LOCKED_MEMORY             ((EC_T_DWORD)0x9813000E) /**< 0x9813000E: AoE: Allocation locked memory */
#define EC_SZTXT_E_AOE_LOCKED_MEMORY             "ERROR AoE: Allocation locked memory"

#define       EC_E_AOE_MAILBOX                   ((EC_T_DWORD)0x9813000F) /**< 0x9813000F: AoE: Insert mailbox error */
#define EC_SZTXT_E_AOE_MAILBOX                   "ERROR AoE: Insert mailbox error"

#define       EC_E_AOE_WRONG_HMSG                ((EC_T_DWORD)0x98130010) /**< 0x98130010: AoE: Wrong receive HMSG */
#define EC_SZTXT_E_AOE_WRONG_HMSG                "ERROR AoE: Wrong receive HMSG"

#define       EC_E_AOE_BAD_TASK_ID               ((EC_T_DWORD)0x98130011) /**< 0x98130011: AoE: Bad task ID */
#define EC_SZTXT_E_AOE_BAD_TASK_ID               "ERROR AoE: Bad task ID"

#define       EC_E_AOE_NO_IO                     ((EC_T_DWORD)0x98130012) /**< 0x98130012: AoE: No IO */
#define EC_SZTXT_E_AOE_NO_IO                     "ERROR AoE: No IO"

#define       EC_E_AOE_UNKNOWN_AMS_COMMAND       ((EC_T_DWORD)0x98130013) /**< 0x98130013: AoE: Unknown ADS command */
#define EC_SZTXT_E_AOE_UNKNOWN_AMS_COMMAND       "ERROR AoE: Unknown ADS command"

#define       EC_E_AOE_WIN32                     ((EC_T_DWORD)0x98130014) /**< 0x98130014: AoE: Win 32 error */
#define EC_SZTXT_E_AOE_WIN32                     "ERROR AoE: Win 32 error"

#define       EC_E_AOE_LOW_INSTALL_LEVEL         ((EC_T_DWORD)0x98130015) /**< 0x98130015: AoE: Low installation level */
#define EC_SZTXT_E_AOE_LOW_INSTALL_LEVEL         "ERROR AoE: Low installation level"

#define       EC_E_AOE_NO_DEBUG                  ((EC_T_DWORD)0x98130016) /**< 0x98130016: AoE: No debug available */
#define EC_SZTXT_E_AOE_NO_DEBUG                  "ERROR AoE: No debug available"

#define       EC_E_AOE_AMS_SYNC_WIN32            ((EC_T_DWORD)0x98130017) /**< 0x98130017: AoE: Sync Win 32 error */
#define EC_SZTXT_E_AOE_AMS_SYNC_WIN32            "ERROR AoE: Sync Win 32 error"

#define       EC_E_AOE_AMS_SYNC_TIMEOUT          ((EC_T_DWORD)0x98130018) /**< 0x98130018: AoE: Sync Timeout */
#define EC_SZTXT_E_AOE_AMS_SYNC_TIMEOUT          "ERROR AoE: Sync Timeout"

#define       EC_E_AOE_AMS_SYNC_AMS              ((EC_T_DWORD)0x98130019) /**< 0x98130019: AoE: Sync AMS error */
#define EC_SZTXT_E_AOE_AMS_SYNC_AMS              "ERROR AoE: Sync AMS error"

#define       EC_E_AOE_AMS_SYNC_NO_INDEX_MAP     ((EC_T_DWORD)0x9813001A) /**< 0x9813001A: AoE: Sync no index map */
#define EC_SZTXT_E_AOE_AMS_SYNC_NO_INDEX_MAP     "ERROR AoE: Sync no index map"

#define       EC_E_AOE_TCP_SEND                  ((EC_T_DWORD)0x9813001B) /**< 0x9813001B: AoE: TCP send error */
#define EC_SZTXT_E_AOE_TCP_SEND                  "ERROR AoE: TCP send error"

#define       EC_E_AOE_HOST_UNREACHABLE          ((EC_T_DWORD)0x9813001C) /**< 0x9813001C: AoE: Host unreachable */
#define EC_SZTXT_E_AOE_HOST_UNREACHABLE          "ERROR AoE: Host unreachable"

#define       EC_E_AOE_INVALIDAMSFRAGMENT        ((EC_T_DWORD)0x9813001D) /**< 0x9813001D: AoE: Invalid AMS fragment */
#define EC_SZTXT_E_AOE_INVALIDAMSFRAGMENT        "ERROR AoE: Invalid AMS fragment"

#define       EC_E_AOE_NO_LOCKED_MEMORY          ((EC_T_DWORD)0x9813001E) /**< 0x9813001E: AoE: No allocation locked memory */
#define EC_SZTXT_E_AOE_NO_LOCKED_MEMORY          "ERROR AoE: No allocation locked memory"

#define       EC_E_AOE_MAILBOX_FULL              ((EC_T_DWORD)0x9813001F) /**< 0x9813001F: AoE: Mailbox full */
#define EC_SZTXT_E_AOE_MAILBOX_FULL              "ERROR AoE: Mailbox full"
/**@}*/ /* EC_ERROR_CODES_AOE */

/*********************************************************************/
/** \defgroup EC_ERROR_CODES_GENERIC Generic Error Codes
@{*/
#define       EC_E_S2SMBX_NOT_CONFIGURED         ((EC_T_DWORD)0x98130020) /**< 0x98130020: S2S: Not Configured */
#define EC_SZTXT_E_S2SMBX_NOT_CONFIGURED         "ERROR S2S: Not Configured"

#define       EC_E_S2SMBX_NO_MEMORY              ((EC_T_DWORD)0x98130021) /**< 0x98130021: S2S: No Memory */
#define EC_SZTXT_E_S2SMBX_NO_MEMORY              "ERROR S2S: No Memory"

#define       EC_E_S2SMBX_NO_DESCRIPTOR          ((EC_T_DWORD)0x98130022) /**< 0x98130022: S2S: No Descriptor */
#define EC_SZTXT_E_S2SMBX_NO_DESCRIPTOR          "ERROR S2S: No Descriptor"

#define       EC_E_S2SMBX_DEST_SLAVE_NOT_FOUND   ((EC_T_DWORD)0x98130023) /**< 0x98130023: S2S: Destination Slave not found */
#define EC_SZTXT_E_S2SMBX_DEST_SLAVE_NOT_FOUND   "ERROR S2S: Destination Slave not found"

#define       EC_E_MASTER_RED_STATE_INACTIVE     ((EC_T_DWORD)0x98130024) /**< 0x98130024: APP: Master Redundancy State is INACTIVE (e.g. API not allowed in current Master Redundancy State) */
#define EC_SZTXT_E_MASTER_RED_STATE_INACTIVE     "ERROR: Master Redundancy State is INACTIVE"

#define       EC_E_MASTER_RED_STATE_ACTIVE       ((EC_T_DWORD)0x98130025) /**< 0x98130025: APP: Master Redundancy State is ACTIVE (e.g. API not allowed in current Master Redundancy State) */
#define EC_SZTXT_E_MASTER_RED_STATE_ACTIVE       "ERROR: Master Redundancy State is ACTIVE"

#define       EC_E_JUNCTION_RED_LINE_BREAK       ((EC_T_DWORD)0x98130026) /**< 0x98130026: Junction redundancy line break */
#define EC_SZTXT_E_JUNCTION_RED_LINE_BREAK       "ERROR: Junction redundancy line break"

#define       EC_E_VALIDATION_ERROR              ((EC_T_DWORD)0x98130027) /**< 0x98130027: Validation error (validation data mismatch) */
#define EC_SZTXT_E_VALIDATION_ERROR              "ERROR: Validation error (validation data mismatch)"

#define       EC_E_TIMEOUT_WAITING_FOR_DC        ((EC_T_DWORD)0x98130028) /**< 0x98130028: Timeout waiting for DC */
#define EC_SZTXT_E_TIMEOUT_WAITING_FOR_DC        "ERROR: Timeout waiting for DC"

#define       EC_E_TIMEOUT_WAITING_FOR_DCM       ((EC_T_DWORD)0x98130029) /**< 0x98130029: Timeout waiting for DCM */
#define EC_SZTXT_E_TIMEOUT_WAITING_FOR_DCM       "ERROR: Timeout waiting for DCM"

#define       EC_E_SIGNATURE_MISMATCH            ((EC_T_DWORD)0x98130030) /**< 0x98130030: Signature mismatch */
#define EC_SZTXT_E_SIGNATURE_MISMATCH            "ERROR: Signature mismatch"

#define       EC_E_PDIWATCHDOG                    ((EC_T_DWORD)0x98130031) /**< 0x98130031: PDI watchdog expired */
#define EC_SZTXT_E_PDIWATCHDOG                   "ERROR: PDI watchdog expired"

#define       EC_E_BAD_CONNECTION                 ((EC_T_DWORD)0x98130032) /**< 0x98130032: Bad connection */
#define EC_SZTXT_E_BAD_CONNECTION                "ERROR: Bad connection"

#define       EC_E_XML_INCONSISTENT              ((EC_T_DWORD)0x98130033) /**< 0x98130033: ENI: Inconsistent content */
#define EC_SZTXT_E_XML_INCONSISTENT              "ERROR: Inconsistent content"
/**@}*/ /* EC_ERROR_CODES_GENERIC */

#define       EC_E_ERROR_LAST2                   ((EC_T_DWORD)0x9813FFFF)


/*********************************************************************/
/* RAS Error Codes                                                   */
/*********************************************************************/

#define EMRAS_SZTXT_EMRAS_E_ERROR                                   "RAS Error"

/*********************************************************************/
/** \defgroup EC_ERROR_CODES_RAS Remote API Error Codes
@{*/
#define       EMRAS_E_INVALIDCOOKIE                                 ((EC_T_DWORD)0x98110181) /**< 0x98110181: RAS: Invalid Cookie (e.g.obsolete) */
#define EMRAS_SZTXT_E_INVALIDCOOKIE                                 "ERROR: Invalid cookie"

#define       EMRAS_E_MULSRVDISMULCON                               ((EC_T_DWORD)0x98110183) /**< 0x98110183: RAS: Connect 2nd server denied because Multi Server support is disabled (obsolete) */
#define EMRAS_SZTXT_E_MULSRVDISMULCON                               "ERROR: Connecting 2nd server denied because Multi Server support disabled"

#define       EMRAS_E_LOGONCANCELLED                                ((EC_T_DWORD)0x98110184) /**< 0x98110184: RAS: Logon canceled (Serverside connection reject while opening a client connection.) */
#define EMRAS_SZTXT_E_LOGONCANCELLED                                "ERROR: Logon cancelled"

#define       EMRAS_E_INVALIDVERSION                                ((EC_T_DWORD)0x98110186) /**< 0x98110186: RAS: Invalid Version (Connection reject because of using mismatching protocol versions on client and server side) */
#define EMRAS_SZTXT_E_INVALIDVERSION                                "ERROR: Invalid version"

#define       EMRAS_E_INVALIDACCESSCONFIG                           ((EC_T_DWORD)0x98110187) /**< 0x98110187: RAS: Access configuration is invalid (e.g. SPoC access configuration invalid) */
#define EMRAS_SZTXT_E_INVALIDACCESSCONFIG                           "ERROR: Access configuration invalid"

#define       EMRAS_E_ACCESSLESS                                    ((EC_T_DWORD)0x98110188) /**< 0x98110188: RAS: No access to this call at this accesslevel (e.g. a higher SPoC accesslevel is needed to use the called Remote API function) */
#define EMRAS_SZTXT_E_ACCESSLESS                                    "ERROR: No access to this call at this accesslevel"

#define       EMRAS_E_INVALIDDATARECEIVED                           ((EC_T_DWORD)0x98110189) /**< 0x98110189: RAS: Invalid data received (communication corrupted) */
#define EMRAS_SZTXT_E_INVALIDDATARECEIVED                           "ERROR: Invalid data received"

#define       EMRAS_EVT_SERVERSTOPPED                               ((EC_T_DWORD)0x98110191) /**< 0x98110191: RAS: Server stopped (e.g. connection dropped because of Remote API Server stop) */
#define EMRAS_SZTXT_EVT_SERVERSTOPPED                               "Server stopped"

#define       EMRAS_EVT_WDEXPIRED                                   ((EC_T_DWORD)0x98110192) /**< 0x98110192: RAS: Watchdog expired (e.g. connection dropped because of missing keep-alive messages) */
#define EMRAS_SZTXT_EVT_WDEXPIRED                                   "Watchdog expired"

#define       EMRAS_EVT_RECONEXPIRED                                ((EC_T_DWORD)0x98110193) /**< 0x98110193: RAS: Reconnect expired (obsolete) */
#define EMRAS_SZTXT_EVT_RECONEXPIRED                                "Reconnect expired"

#define       EMRAS_EVT_CLIENTLOGON                                 ((EC_T_DWORD)0x98110194) /**< 0x98110194: RAS Server: Client logged on */
#define EMRAS_SZTXT_EVT_CLIENTLOGON                                 "Client logged on"

#define       EMRAS_EVT_RECONNECT                                   ((EC_T_DWORD)0x98110195) /**< 0x98110195: RAS: obsolete */
#define EMRAS_SZTXT_EVT_RECONNECT                                   "Client reconnected"

#define       EMRAS_EVT_SOCKCHANGE                                  ((EC_T_DWORD)0x98110196) /**< 0x98110196: RAS: Socket exchanged after reconnect (obsolete) */
#define EMRAS_SZTXT_EVT_SOCKCHANGE                                  "Socket exchanged after reconnect"

#define       EMRAS_EVT_CLNTDISC                                    ((EC_T_DWORD)0x98110197) /**< 0x98110197: RAS: Client disconnect */
#define EMRAS_SZTXT_EVT_CLNTDISC                                    "Client disconnected"

#define       EMRAS_E_ACCESS_NOT_FOUND                              ((EC_T_DWORD)0x98110198) /**< 0x98110198: RAS: Access not configured for this call (e.g. SPoC access configuration missing) */
#define EMRAS_SZTXT_E_ACCESS_NOT_FOUND                              "Access level to this call not configured"

/**@}*/ /* EC_ERROR_CODES_RAS */

#define       EMRAS_E_ERROR_LAST                                    ((EC_T_DWORD)0x981101BF)


/*********************************************************************/
/* Application Framework Strings                                     */
/*********************************************************************/

#define       EC_TXT_MASTER_STATE_CHANGE                            ((EC_T_WORD)(EC_TEXTBASE+0x0001))
#define EC_SZTXT_TXT_MASTER_STATE_CHANGE                            "Master state changed from <%s> to <%s>"

#define       EC_TXT_SB_RESULT_OK                                   ((EC_T_WORD)(EC_TEXTBASE+0x0002))
#define EC_SZTXT_TXT_SB_RESULT_OK                                   "Bus scan successful - %d slaves found"

#define       EC_TXT_SB_RESULT_ERROR                                ((EC_T_WORD)(EC_TEXTBASE+0x0003))
#define EC_SZTXT_TXT_SB_RESULT_ERROR                                "Bus scan error '%s (0x%x)', %d slaves found"

#define       EC_TXT_DC_STATUS                                      ((EC_T_WORD)(EC_TEXTBASE+0x0004))
#define EC_SZTXT_TXT_DC_STATUS                                      "Distributed clocks - status %s (0x%x)"

#define       EC_TXT_DCL_STATUS                                     ((EC_T_WORD)(EC_TEXTBASE+0x0005))
#define EC_SZTXT_TXT_DCL_STATUS                                     "Distributed clocks - latching status %s (0x%x)"

#define       EC_TXT_COE_SDO_DNLD_ERROR                             ((EC_T_WORD)(EC_TEXTBASE+0x0006))
#define EC_SZTXT_TXT_COE_SDO_DNLD_ERROR                             "CoE - SDO download failure, EtherCAT address=%d - Index=0x%x SubIndex=0x%x CompleteAccess=%d, statVal=%d, errCode=0x%x (%s)"

#define       EC_TXT_COE_SDO_UPLD_ERROR                             ((EC_T_WORD)(EC_TEXTBASE+0x0007))
#define EC_SZTXT_TXT_COE_SDO_UPLD_ERROR                             "CoE - SDO upload failure, EtherCAT address=%d - Index=0x%x SubIndex=0x%x CompleteAccess=%d, statVal=%d, errCode=0x%x (%s)"

#define       EC_TXT_COE_GETODL_ERROR                               ((EC_T_WORD)(EC_TEXTBASE+0x0008))
#define EC_SZTXT_TXT_COE_GETODL_ERROR                               "CoE - OD-list upload failure, statVal=%d, errCode=0x%x (%s)"

#define       EC_TXT_COE_GETOBDESC_ERROR                            ((EC_T_WORD)(EC_TEXTBASE+0x0009))
#define EC_SZTXT_TXT_COE_GETOBDESC_ERROR                            "CoE - object description upload failure, statVal=%d, errCode=0x%x (%s)"

#define       EC_TXT_COE_GETENTRYDESC_ERROR                         ((EC_T_WORD)(EC_TEXTBASE+0x000A))
#define EC_SZTXT_TXT_COE_GETENTRYDESC_ERROR                         "CoE - object entry description upload failure, statVal=%d, errCode=0x%x (%s)"

#define       EC_TXT_COE_EMRG_TFER_ERROR                            ((EC_T_WORD)(EC_TEXTBASE+0x000B))
#define EC_SZTXT_TXT_COE_EMRG_TFER_ERROR                            "CoE - emergency transfer failure, statVal=%d, errCode=0x%x (%s)"

#define       EC_TXT_COE_EMRG                                       ((EC_T_WORD)(EC_TEXTBASE+0x000C))
#define EC_SZTXT_TXT_COE_EMRG                                       "CoE - emergency request, id =0x%x, len=%d ==> slave address=%d, ErrCode=0x%x, ErrReg=0x%x, data: '%02x %02x %02x %02x %02x'"

#define       EC_TXT_CYCCMD_WKC_ERROR                               ((EC_T_WORD)(EC_TEXTBASE+0x000D))
#define EC_SZTXT_TXT_CYCCMD_WKC_ERROR                               "Cyclic command WKC error on %s - Address: 0x%x - WKC act/set=%d/%d"

#define       EC_TXT_MASTINITCMD_WKC_ERROR                          ((EC_T_WORD)(EC_TEXTBASE+0x000E))
#define EC_SZTXT_TXT_MASTINITCMD_WKC_ERROR                          "Master init command working counter error - Command: %s - Logical/Physical address: 0x%x, WKC act/set=%d/%d"

#define       EC_TXT_SLVINITCMD_WKC_ERROR                           ((EC_T_WORD)(EC_TEXTBASE+0x000F))
#define EC_SZTXT_TXT_SLVINITCMD_WKC_ERROR                           "Slave init command working counter error - Slave %s: - EtherCAT address=%d - Command: %s - Logical/Physical address: 0x%x, WKC act/set=%d/%d"

#define       EC_TXT_EOEMBXRCV_WKC_ERROR                            ((EC_T_WORD)(EC_TEXTBASE+0x0010))
#define EC_SZTXT_TXT_EOEMBXRCV_WKC_ERROR                            "EoE mbox receive working counter error - Slave %s: - EtherCAT address=%d - Command: %s - Logical/Physical address: 0x%x, WKC act/set=%d/%d"

#define       EC_TXT_COEMBXRCV_WKC_ERROR                            ((EC_T_WORD)(EC_TEXTBASE+0x0011))
#define EC_SZTXT_TXT_COEMBXRCV_WKC_ERROR                            "CoE mbox receive working counter error - Slave %s: - EtherCAT address=%d - Command: %s - Logical/Physical address: 0x%x, WKC act/set=%d/%d"

#define       EC_TXT_FOEMBXRCV_WKC_ERROR                            ((EC_T_WORD)(EC_TEXTBASE+0x0012))
#define EC_SZTXT_TXT_FOEMBXRCV_WKC_ERROR                            "FoE mbox receive working counter error - Slave %s: - EtherCAT address=%d - Command: %s - Logical/Physical address: 0x%x, WKC act/set=%d/%d"

#define       EC_TXT_SOEMBXRCV_WKC_ERROR                            ((EC_T_WORD)(EC_TEXTBASE+0x0013))
#define EC_SZTXT_TXT_SOEMBXRCV_WKC_ERROR                            "SoE mbox receive working counter error - Slave %s: - EtherCAT address=%d - Command: %s - Logical/Physical address: 0x%x, WKC act/set=%d/%d"

#define       EC_TXT_EOEMBXSND_WKC_ERROR                            ((EC_T_WORD)(EC_TEXTBASE+0x0014))
#define EC_SZTXT_TXT_EOEMBXSND_WKC_ERROR                            "EoE mbox send working counter error - Slave %s: - EtherCAT address=%d - Command: %s - Logical/Physical address: 0x%x, WKC act/set=%d/%d"

#define       EC_TXT_COEMBXSND_WKC_ERROR                            ((EC_T_WORD)(EC_TEXTBASE+0x0015))
#define EC_SZTXT_TXT_COEMBXSND_WKC_ERROR                            "CoE mbox send working counter error - Slave %s: - EtherCAT address=%d - Command: %s - Logical/Physical address: 0x%x, WKC act/set=%d/%d"

#define       EC_TXT_FOEMBXSND_WKC_ERROR                            ((EC_T_WORD)(EC_TEXTBASE+0x0016))
#define EC_SZTXT_TXT_FOEMBXSND_WKC_ERROR                            "FoE mbox send working counter error - Slave %s: - EtherCAT address=%d - Command: %s - Logical/Physical address: 0x%x, WKC act/set=%d/%d"

#define       EC_TXT_SOEMBXSND_WKC_ERROR                            ((EC_T_WORD)(EC_TEXTBASE+0x0017))
#define EC_SZTXT_TXT_SOEMBXSND_WKC_ERROR                            "SoE mbox send working counter error - Slave %s: - EtherCAT address=%d - Command: %s - Logical/Physical address: 0x%x, WKC act/set=%d/%d"

#define       EC_TXT_FRMRESP_RETRY                                  ((EC_T_WORD)(EC_TEXTBASE+0x0018))
#define EC_SZTXT_TXT_FRMRESP_RETRY                                  "Retry sending a %s frame due to %s"

#define       EC_TXT_FRMRESP_NORETRY                                ((EC_T_WORD)(EC_TEXTBASE+0x0019))
#define EC_SZTXT_TXT_FRMRESP_NORETRY                                "%s response on %s Ethernet frame"

#define       EC_TXT_ADDERRINFO                                     ((EC_T_WORD)(EC_TEXTBASE+0x001A))
#define EC_SZTXT_TXT_ADDERRINFO                                     "Additional error information: %s"

#define       EC_TXT_CMDIDXACTVAL                                   ((EC_T_WORD)(EC_TEXTBASE+0x001B))
#define EC_SZTXT_TXT_CMDIDXACTVAL                                   "EtherCAT command IDX act value=0x%x"

#define       EC_TXT_CMDIDXSETVAL                                   ((EC_T_WORD)(EC_TEXTBASE+0x001C))
#define EC_SZTXT_TXT_CMDIDXSETVAL                                   "EtherCAT command IDX set value=0x%x"

#define       EC_TXT_SLVINITCMDRSPERR_NR                            ((EC_T_WORD)(EC_TEXTBASE+0x001D))
#define EC_SZTXT_TXT_SLVINITCMDRSPERR_NR                            "Slave init command response error - Slave %s: - EtherCAT address=%d - Current State change of slave=\"%s\" No Response"

#define       EC_TXT_SLVINITCMDRSPERR_VE                            ((EC_T_WORD)(EC_TEXTBASE+0x001E))
#define EC_SZTXT_TXT_SLVINITCMDRSPERR_VE                            "Slave init command response error - Slave %s: - EtherCAT address=%d - Current State change of slave=\"%s\" Validation error"

#define       EC_TXT_SLVINITCMDRSPERR_FLD                           ((EC_T_WORD)(EC_TEXTBASE+0x001F))
#define EC_SZTXT_TXT_SLVINITCMDRSPERR_FLD                           "Slave init command response error - Slave %s: - EtherCAT address=%d - Current State change of slave=\"%s\" target state could not be reached"

#define       EC_TXT_MASTINITCMDRSPERR_NR                           ((EC_T_WORD)(EC_TEXTBASE+0x0020))
#define EC_SZTXT_TXT_MASTINITCMDRSPERR_NR                           "Master init command response error - Current State change of master=\"%s\" No Response. Anything connected?"

#define       EC_TXT_MASTINITCMDRSPERR_VE                           ((EC_T_WORD)(EC_TEXTBASE+0x0021))
#define EC_SZTXT_TXT_MASTINITCMDRSPERR_VE                           "Master init command response error - Current State change of master=\"%s\" Validation error. Correct slaves connected?"

#define       EC_TXT_CMD_MISSING                                    ((EC_T_WORD)(EC_TEXTBASE+0x0022))
#define EC_SZTXT_TXT_CMD_MISSING                                    "Missing EtherCAT %s command in Ethernet frame - Slave %s: - EtherCAT address=%d - Index of missing command in Ethernet frame=%d"

#define       EC_TXT_MBSLV_INITCMDTO                                ((EC_T_WORD)(EC_TEXTBASE+0x0023))
#define EC_SZTXT_TXT_MBSLV_INITCMDTO                                "\"Mailbox init command\" time-out - Slave %s: - EtherCAT address=%d - Current State change of slave=\"%s\""

#define       EC_TXT_NOT_ALL_DEVS_OP                                ((EC_T_WORD)(EC_TEXTBASE+0x0024))
#define EC_SZTXT_TXT_NOT_ALL_DEVS_OP                                "Not all EtherCAT slave devices in operational state"

#define       EC_TXT_CABLE_CONNECTED                                ((EC_T_WORD)(EC_TEXTBASE+0x0025))
#define EC_SZTXT_TXT_CABLE_CONNECTED                                "Ethernet cable connected"

#define       EC_TXT_CABLE_NOT_CONNECTED                            ((EC_T_WORD)(EC_TEXTBASE+0x0026))
#define EC_SZTXT_TXT_CABLE_NOT_CONNECTED                            "Ethernet cable not connected"

#define       EC_TXT_CYCCMD_TIMEOUT                                 ((EC_T_WORD)(EC_TEXTBASE+0x0027))
#define EC_SZTXT_TXT_CYCCMD_TIMEOUT                                 "Cyclic command time-out: Time between sending cyclic commands too long"

#define       EC_TXT_REDLINEBREAK                                   ((EC_T_WORD)(EC_TEXTBASE+0x0028))
#define EC_SZTXT_TXT_REDLINEBREAK                                   "ERROR: Redundancy Line break %d slaves on main and %d slaves on red"

#define       EC_TXT_SLVERR_DETECTED                                ((EC_T_WORD)(EC_TEXTBASE+0x0029))
#define EC_SZTXT_TXT_SLVERR_DETECTED                                "ERROR: At least one slave in error status"

#define       EC_TXT_SLVERR_INFO                                    ((EC_T_WORD)(EC_TEXTBASE+0x002A))
#define EC_SZTXT_TXT_SLVERR_INFO                                    "Slave error \"%s\": - EtherCAT address=%d - State <%s%s>(0x%x), control status <%s>(0x%x)"

#define       EC_TXT_SLV_NOT_ADDRABLE                               ((EC_T_WORD)(EC_TEXTBASE+0x002B))
#define EC_SZTXT_TXT_SLV_NOT_ADDRABLE                               "Unable to address slave %s: - EtherCAT address=%d"

#define       EC_TXT_MBSLV_SDO_ABORT                                ((EC_T_WORD)(EC_TEXTBASE+0x002C))
#define EC_SZTXT_TXT_MBSLV_SDO_ABORT                                "SDO abort - Slave %s: - EtherCAT address=%d - %s (0x%x) - Index=0x%x SubIndex=0x%x"

#define       EC_TXT_DCSLVSYNC_INSYNC                               ((EC_T_WORD)(EC_TEXTBASE+0x002D))
#define EC_SZTXT_TXT_DCSLVSYNC_INSYNC                               "DC slaves \"in-sync\" - Deviation: 0x%x"

#define       EC_TXT_DCSLVSYNC_OUTOFSYNC                            ((EC_T_WORD)(EC_TEXTBASE+0x002E))
#define EC_SZTXT_TXT_DCSLVSYNC_OUTOFSYNC                            "DC slaves \"out-of-sync\" - Deviation: 0x%x"

#define       EC_TXT_DCL_SINGLE_LATCH                               ((EC_T_WORD)(EC_TEXTBASE+0x002F))
#define EC_SZTXT_TXT_DCL_SINGLE_LATCH                               "DC single latch on slave 0x%x Ident 0x%x"

#define       EC_TXT_FOE_UPLD_ERROR                                 ((EC_T_WORD)(EC_TEXTBASE+0x0030))
#define EC_SZTXT_TXT_FOE_UPLD_ERROR                                 "MbxTferRcv - file upload failure, statVal=%d, errCode=0x%x (%s)"

#define       EC_TXT_FOE_DNLD_ERROR                                 ((EC_T_WORD)(EC_TEXTBASE+0x0031))
#define EC_SZTXT_TXT_FOE_DNLD_ERROR                                 "MbxTferRcv - file download failure, statVal=%d, errCode=0x%x (%s)"

#define       EC_TXT_CLNTREGDROP                                    ((EC_T_WORD)(EC_TEXTBASE+0x0032))
#define EC_SZTXT_TXT_CLNTREGDROP                                    "Client registration lost"

#define       EC_TXT_REDLINEFIXED                                   ((EC_T_WORD)(EC_TEXTBASE+0x0033))
#define EC_SZTXT_TXT_REDLINEFIXED                                   "Redundancy - Line fixed"

#define       EC_TXT_SOEMBX_WRITE_ERROR                             ((EC_T_WORD)(EC_TEXTBASE+0x0034))
#define EC_SZTXT_TXT_SOEMBX_WRITE_ERROR                             "SoE init command mbox write error - Slave %s: - EtherCAT address=%d - Current State change of slave=\"%s\""

#define       EC_TXT_DCM_INSYNC                                     ((EC_T_WORD)(EC_TEXTBASE+0x0035))
#define EC_SZTXT_TXT_DCM_INSYNC                                     "DCM in sync (cur/avg/max) [ns]: %7d/%7d/%7d"

#define       EC_TXT_DCM_OUTOFSYNC                                  ((EC_T_WORD)(EC_TEXTBASE+0x0036))
#define EC_SZTXT_TXT_DCM_OUTOFSYNC                                  "DCM out of sync (cur/avg/max) [ns]: %7d/%7d/%7d"

#define       EC_TXT_SLAVE_STATECHANGED                             ((EC_T_WORD)(EC_TEXTBASE+0x0037))
#define EC_SZTXT_TXT_SLAVE_STATECHANGED                             "Slave %d: state changed to <%s>"

#define       EC_TXT_DCX_INSYNC                                     ((EC_T_WORD)(EC_TEXTBASE+0x0038))
#define EC_SZTXT_TXT_DCX_INSYNC                                     "DCX in sync (cur/avg/max) [ns]: %7d/%7d/%7d, TimeStamp= %7d"

#define       EC_TXT_DCX_OUTOFSYNC                                  ((EC_T_WORD)(EC_TEXTBASE+0x0039))
#define EC_SZTXT_TXT_DCX_OUTOFSYNC                                  "DCX out of sync (cur/avg/max) [ns]: %7d/%7d/%7d, TimeStamp= %7d"

#define       EC_TXT_FRAME_TYPE_CYCLIC                              ((EC_T_WORD)(EC_TEXTBASE+0x003A))
#define EC_SZTXT_TXT_FRAME_TYPE_CYCLIC                              "cyclic"

#define       EC_TXT_FRAME_TYPE_ACYCLIC                             ((EC_T_WORD)(EC_TEXTBASE+0x003B))
#define EC_SZTXT_TXT_FRAME_TYPE_ACYCLIC                             "acyclic"

#define       EC_TXT_FRAME_RESPONSE_ERRTYPE_NO                      ((EC_T_WORD)(EC_TEXTBASE+0x003C))
#define EC_SZTXT_TXT_FRAME_RESPONSE_ERRTYPE_NO                      "No"

#define       EC_TXT_FRAME_RESPONSE_ERRTYPE_WRONG                   ((EC_T_WORD)(EC_TEXTBASE+0x003D))
#define EC_SZTXT_TXT_FRAME_RESPONSE_ERRTYPE_WRONG                   "Wrong"

#define       EC_TXT_FRAME_RESPONSE_ERRTYPE_UNEXPECTED              ((EC_T_WORD)(EC_TEXTBASE+0x003E))
#define EC_SZTXT_TXT_FRAME_RESPONSE_ERRTYPE_UNEXPECTED              "Unexpected"

#define       EC_TXT_FRAME_RESPONSE_ERRTYPE_RETRY_FAIL              ((EC_T_WORD)(EC_TEXTBASE+0x003F))
#define EC_SZTXT_TXT_FRAME_RESPONSE_ERRTYPE_RETRY_FAIL              "Retry failed. No"

#define       EC_TXT_DEVICE_STATE_BASE                              ((EC_T_WORD)(EC_TEXTBASE+0x0040))
#define       EC_TXT_DEVICE_STATE_UNKNOWN                           ((EC_T_WORD)(EC_TEXTBASE+0x0040))
#define EC_SZTXT_TXT_DEVICE_STATE_UNKNOWN                           "UNKNOWN"

#define       EC_TXT_DEVICE_STATE_INIT                              ((EC_T_WORD)(EC_TEXTBASE+0x0041))
#define EC_SZTXT_TXT_DEVICE_STATE_INIT                              "INIT"

#define       EC_TXT_DEVICE_STATE_PREOP                             ((EC_T_WORD)(EC_TEXTBASE+0x0042))
#define EC_SZTXT_TXT_DEVICE_STATE_PREOP                             "PREOP"

#define       EC_TXT_DEVICE_STATE_BOOTSTRAP                         ((EC_T_WORD)(EC_TEXTBASE+0x0043))
#define EC_SZTXT_TXT_DEVICE_STATE_BOOTSTRAP                         "BOOTSTRAP"

#define       EC_TXT_DEVICE_STATE_SAFEOP                            ((EC_T_WORD)(EC_TEXTBASE+0x0044))
#define EC_SZTXT_TXT_DEVICE_STATE_SAFEOP                            "SAFEOP"

#define       EC_TXT_DEVICE_STATE_OP                                ((EC_T_WORD)(EC_TEXTBASE+0x0048))
#define EC_SZTXT_TXT_DEVICE_STATE_OP                                "OP"

#define       EC_TXT_HC_DETAGRESULT_OK                              ((EC_T_WORD)(EC_TEXTBASE+0x0050))
#define EC_SZTXT_TXT_HC_DETAGRESULT_OK                              "Hot Connect detect all groups successful - %d groups found, %d groups present, Mask: 0x%x"

#define       EC_TXT_HC_DETAGRESULT_ERROR                           ((EC_T_WORD)(EC_TEXTBASE+0x0051))
#define EC_SZTXT_TXT_HC_DETAGRESULT_ERROR                           "Hot Connect detect all groups error '%s (0x%x)', %d groups found"

#define       EC_TXT_SLAVE_ABSENT                                   ((EC_T_WORD)(EC_TEXTBASE+0x0052))
#define EC_SZTXT_TXT_SLAVE_ABSENT                                   "Slave with station address %d absent"

#define       EC_TXT_SLAVE_PRESENT                                  ((EC_T_WORD)(EC_TEXTBASE+0x0053))
#define EC_SZTXT_TXT_SLAVE_PRESENT                                  "Slave with station address %d present"

#define       EC_TXT_HC_TOPOCHGDONE                                 ((EC_T_WORD)(EC_TEXTBASE+0x0054))
#define EC_SZTXT_TXT_HC_TOPOCHGDONE                                 "Topology Change Performed %s (0x%x)"

#define       EC_TXT_HC_BORDERCLOSE                                 ((EC_T_WORD)(EC_TEXTBASE+0x0055))
#define EC_SZTXT_TXT_HC_BORDERCLOSE                                 "Border Close Performed on Port %x:%x (%s) %s (0x%x)"

#define       EC_TXT_SOE_WRITE_ERROR                                ((EC_T_WORD)(EC_TEXTBASE+0x0056))
#define EC_SZTXT_TXT_SOE_WRITE_ERROR                                "MbxTferRcv - Write IDN failure, statVal=%d, errCode=0x%x (%s)"

#define       EC_TXT_SOE_READ_ERROR                                 ((EC_T_WORD)(EC_TEXTBASE+0x0057))
#define EC_SZTXT_TXT_SOE_READ_ERROR                                 "MbxTferRcv - Read IDN failure, statVal=%d, errCode=0x%x (%s)"

#define       EC_TXT_SOE_EMRG_TFER_ERROR                            ((EC_T_WORD)(EC_TEXTBASE+0x0058))
#define EC_SZTXT_TXT_SOE_EMRG_TFER_ERROR                            "SoE - emergency transfer failure, statVal=%d, errCode=0x%x (%s)"

#define       EC_TXT_SOE_EMRG                                       ((EC_T_WORD)(EC_TEXTBASE+0x0059))
#define EC_SZTXT_TXT_SOE_EMRG                                       "SoE - emergency request, id =0x%x, len=%d ==> slave address=%d, ErrCode=0x%x, data: '%02x %02x %02x %02x %02x'"

#define       EC_TXT_SOE_NOTIFIC_TFER_ERROR                         ((EC_T_WORD)(EC_TEXTBASE+0x005a))
#define EC_SZTXT_TXT_SOE_NOTIFIC_TFER_ERROR                         "SoE - notification transfer failure, statVal=%d, errCode=0x%x (%s)"

#define       EC_TXT_SOE_NOTIFICATION                               ((EC_T_WORD)(EC_TEXTBASE+0x005b))
#define EC_SZTXT_TXT_SOE_NOTIFICATION                               "SoE - notification request, id =0x%x, len=%d ==> slave address=%d, Header=0x%x, IDN=%d, data: '%02x %02x %02x %02x %02x'"

#define       EC_TXT_MBSLV_FOE_ABORT                                ((EC_T_WORD)(EC_TEXTBASE+0x005C))
#define EC_SZTXT_TXT_MBSLV_FOE_ABORT                                "FoE abort - Slave %s: - EtherCAT address=%d - %s (0x%x)"

#define       EC_TXT_MBXRCV_INVALID_DATA                            ((EC_T_WORD)(EC_TEXTBASE+0x005D))
#define EC_SZTXT_TXT_MBXRCV_INVALID_DATA                            "Invalid mailbox data received - Slave %s: - EtherCAT address=%d"

#define       EC_TXT_PDIWATCHDOG                                    ((EC_T_WORD)(EC_TEXTBASE+0x005E))
#define EC_SZTXT_TXT_PDIWATCHDOG                                    "PDI Watchdog expired - Slave %s: - EtherCAT address=%d"

#define       EC_TXT_SLAVE_NOTSUPPORTED                             ((EC_T_WORD)(EC_TEXTBASE+0x005F))
#define EC_SZTXT_TXT_SLAVE_NOTSUPPORTED                             "Slave not supported - Slave %s: - EtherCAT address=%d"

#define       EC_TXT_SLAVE_UNEXPECTED_STATE                         ((EC_T_WORD)(EC_TEXTBASE+0x0060))
#define EC_SZTXT_TXT_SLAVE_UNEXPECTED_STATE                         "Unexpected state - Slave %s: - EtherCAT address=%d - Current state=<%s> - Expected=<%s>"

#define       EC_TXT_ALL_DEVS_OP                                    ((EC_T_WORD)(EC_TEXTBASE+0x0061))
#define EC_SZTXT_TXT_ALL_DEVS_OP                                    "All EtherCAT slave devices in operational state"

#define       EC_TXT_VOE_DNLD_ERROR                                 ((EC_T_WORD)(EC_TEXTBASE+0x0062))
#define EC_SZTXT_TXT_VOE_DNLD_ERROR                                 "VoE write failed, eTferStatus = %d, error = 0x%x (%s)"

#define       EC_TXT_VOE_UPLD_ERROR                                 ((EC_T_WORD)(EC_TEXTBASE+0x0063))
#define EC_SZTXT_TXT_VOE_UPLD_ERROR                                 "VoE read failed, eTferStatus = %d, error = 0x%x (%s)"

#define       EC_TXT_VOEMBXSND_WKC_ERROR                            ((EC_T_WORD)(EC_TEXTBASE+0x0064))
#define EC_SZTXT_TXT_VOEMBXSND_WKC_ERROR                            "VoE mbox send working counter error - Slave %s: - EtherCAT address=%d - Command: %s - Logical/Physical address: 0x%x, WKC act/set=%d/%d"

#define       EC_TXT_AOE_CMD_ERROR                                  ((EC_T_WORD)(EC_TEXTBASE+0x0065))
#define EC_SZTXT_TXT_AOE_CMD_ERROR                                  "AoE command response error - error = 0x%x (%s) - device error: 0x%x - command error: 0x%x"

#define       EC_TXT_EEPROM_CHECKSUM_ERROR                          ((EC_T_WORD)(EC_TEXTBASE+0x0066))
#define EC_SZTXT_TXT_EEPROM_CHECKSUM_ERROR                          "EEPROM checksum error - Slave %s: - EtherCAT address=%d"

#define       EC_TXT_JUNCTION_RED_CHANGE                            ((EC_T_WORD)(EC_TEXTBASE+0x0067))
#define EC_SZTXT_TXT_JUNCTION_RED_CHANGE                            "Junction redundancy change (Line break = %d) - Slave %s: - EtherCAT address=%d at port %d"

#define       EC_TXT_LINE_CROSSED                                   ((EC_T_WORD)EC_TEXTBASE+0x0068)
#define EC_SZTXT_TXT_LINE_CROSSED                                   "Line crossed at slave %s, EtherCAT auto-increment address=%d, station address=%d. Error at port %d."

#define       EC_TXT_SLAVES_UNEXPECTED_STATE                         ((EC_T_WORD)(EC_TEXTBASE+0x0069))
#define EC_SZTXT_TXT_SLAVES_UNEXPECTED_STATE                         "Unexpected state - Slave EtherCAT address=%d - Current state=<%s> - Expected=<%s>"

#define       EC_TXT_MASTER_RED_STATE_CHANGE                        ((EC_T_WORD)(EC_TEXTBASE+0x006A))
#define EC_SZTXT_TXT_MASTER_RED_STATE_CHANGE                        "Master redundancy state changed to <%s>"

#define       EC_TXT_MASTER_RED_FOREIGN_SRC_MAC                     ((EC_T_WORD)(EC_TEXTBASE+0x006B))
#define EC_SZTXT_TXT_MASTER_RED_FOREIGN_SRC_MAC                     "Foreign communication detected"

#define       EC_TXT_SLAVE_STATE_ERROR_BASE                         ((EC_T_WORD)(EC_TEXTBASE+0x0070))
#define       EC_TXT_SLAVE_STATE_ERROR_UNKNOWN                      ((EC_T_WORD)(EC_TEXTBASE+0x0070))
#define EC_SZTXT_TXT_SLAVE_STATE_ERROR_UNKNOWN                      "UNKNOWN ERROR"

#define       EC_TXT_SLAVE_STATE_ERROR_INIT                         ((EC_T_WORD)(EC_TEXTBASE+0x0071))
#define EC_SZTXT_TXT_SLAVE_STATE_ERROR_INIT                         "INIT ERROR"

#define       EC_TXT_SLAVE_STATE_ERROR_PREOP                        ((EC_T_WORD)(EC_TEXTBASE+0x0072))
#define EC_SZTXT_TXT_SLAVE_STATE_ERROR_PREOP                        "PREOP ERROR"

#define       EC_TXT_SLAVE_STATE_ERROR_BOOTSTRAP                    ((EC_T_WORD)(EC_TEXTBASE+0x0073))
#define EC_SZTXT_TXT_SLAVE_STATE_ERROR_BOOTSTRAP                    "BOOTSTRAP ERROR"

#define       EC_TXT_SLAVE_STATE_ERROR_SAFEOP                       ((EC_T_WORD)(EC_TEXTBASE+0x0074))
#define EC_SZTXT_TXT_SLAVE_STATE_ERROR_SAFEOP                       "SAFEOP ERROR"

#define       EC_TXT_SLAVE_STATE_ERROR_OP                           ((EC_T_WORD)(EC_TEXTBASE+0x0078))
#define EC_SZTXT_TXT_SLAVE_STATE_ERROR_OP                           "OP ERROR"

#define       EC_TXT_FRAMELOSS_AFTER_SLAVE                          ((EC_T_WORD)(EC_TEXTBASE+0x0079))
#define EC_SZTXT_TXT_FRAMELOSS_AFTER_SLAVE                          "Frameloss after slave %s, EtherCAT auto-increment address=%d, station address=%d, port %d."

#define       EC_TXT_BAD_CONNECTION                                 ((EC_T_WORD)(EC_TEXTBASE+0x007A))
#define EC_SZTXT_TXT_BAD_CONNECTION                                 "Bad connection between slave %s, station address=%d, port %d and slave %s, station address=%d, port %d"

#define       EC_TXT_COMMUNICATION_TIMEOUT                          ((EC_T_WORD)(EC_TEXTBASE+0x007B))
#define EC_SZTXT_TXT_COMMUNICATION_TIMEOUT                          "Communication timeout at Ethernet TAP port in=%d, port out=%d"

#define       EC_TXT_TAP_LINK_STATUS                                ((EC_T_WORD)(EC_TEXTBASE+0x007C))
#define EC_SZTXT_TXT_TAP_LINK_STATUS                                "Connection between EC-Monitor and Ethernet TAP changed, link connected=%d"

#define       EC_TXT_S2SMBX_ERROR                                   ((EC_T_WORD)(EC_TEXTBASE+0x0080))
#define EC_SZTXT_TXT_S2SMBX_ERROR                                   "Slave-to-Slave mailbox error. Slave %s, source station addr=%d, target station addr=%d, error code=0x%x"

#define       EC_TXT_SLVINITCMDRSPERR_NPR                           ((EC_T_WORD)(EC_TEXTBASE+0x0081))
#define EC_SZTXT_TXT_SLVINITCMDRSPERR_NPR                           "Slave init command response error - Slave %s: - EtherCAT address=%d - Current State change of slave=\"%s\" Slave not present"

#define       EC_TXT_SLVINITCMDRSPERR_ALS                           ((EC_T_WORD)(EC_TEXTBASE+0x0082))
#define EC_SZTXT_TXT_SLVINITCMDRSPERR_ALS                           "Slave init command response error - Slave %s: - EtherCAT address=%d - Current State change of slave=\"%s\" Error in AL Status"

#define       EC_TXT_SLVINITCMDRSPERR_MBX                           ((EC_T_WORD)(EC_TEXTBASE+0x0083))
#define EC_SZTXT_TXT_SLVINITCMDRSPERR_MBX                           "Slave init command response error - Slave %s: - EtherCAT address=%d - Current State change of slave=\"%s\" Mailbox Error"

#define       EC_TXT_SLVINITCMDRSPERR_PWD                           ((EC_T_WORD)(EC_TEXTBASE+0x0084))
#define EC_SZTXT_TXT_SLVINITCMDRSPERR_PWD                           "Slave init command response error - Slave %s: - EtherCAT address=%d - Current State change of slave=\"%s\" PDI Watchdog expired"

#define       EC_TXT_RELEASE_FORCED_PROCESSDATA                     ((EC_T_WORD)(EC_TEXTBASE+0x0085))
#define EC_SZTXT_TXT_RELEASE_FORCED_PROCESSDATA                      "Release forced %s data with bit Offset %d"

#define       EC_TEXTBASE_LAST                                      ((EC_T_WORD)0x02FF)

/**********************************************************************************
 * AL Status Code, ETG.1020, V1.2.0, Table 1: Description of AL Status Codes usage
 * See also DEVICE_STATUSCODE_..., EcStatusCodeToString, ALSTATUSCODE_ in SSC
 **********************************************************************************/
#define       EC_TXT_STATUSCODE_NOERROR                             ((EC_T_WORD)(EC_ALSTATUSCODEBASE+0x0000)) /* DEVICE_STATUSCODE_NOERROR */
#define EC_SZTXT_TXT_STATUSCODE_NOERROR                             "No error"

#define       EC_TXT_STATUSCODE_ERROR                               ((EC_T_WORD)(EC_ALSTATUSCODEBASE+0x0001)) /* DEVICE_STATUSCODE_ERROR */
#define EC_SZTXT_TXT_STATUSCODE_ERROR                               "Unspecified error"

#define       EC_TXT_STATUSCODE_NO_MEMORY                           ((EC_T_WORD)(EC_ALSTATUSCODEBASE+0x0002)) /* DEVICE_STATUSCODE_NO_MEMORY */
#define EC_SZTXT_TXT_STATUSCODE_NO_MEMORY                           "No memory"

#define       EC_TXT_STATUSCODE_INVDEVSETUP                         ((EC_T_WORD)(EC_ALSTATUSCODEBASE+0x0003)) /* DEVICE_STATUSCODE_INVALID_DEVICE_SETUP */
#define EC_SZTXT_TXT_STATUSCODE_INVDEVSETUP                         "Invalid device setup"

#define       EC_TXT_STATUSCODE_FW_MISMATCH                         ((EC_T_WORD)(EC_ALSTATUSCODEBASE+0x0006)) /* DEVICE_STATUSCODE_SII_EEPROM_INFORMATION_MISMATCH */
#define EC_SZTXT_TXT_STATUSCODE_FW_MISMATCH                         "Firmware and EEPROM do not match. Slave needs BOOT-INIT transition"

#define       EC_TXT_STATUSCODE_FW_UPDATE_ERR                       ((EC_T_WORD)(EC_ALSTATUSCODEBASE+0x0007)) /* DEVICE_STATUSCODE_FIRMWARE_UPDATE_ERROR */
#define EC_SZTXT_TXT_STATUSCODE_FW_UPDATE_ERR                       "Firmware update not successful. Old firmware still running"

#define       EC_TXT_STATUSCODE_LICENSE_ERROR                       ((EC_T_WORD)(EC_ALSTATUSCODEBASE+0x000E)) /* DEVICE_STATUSCODE_LICENSE_ERROR */
#define EC_SZTXT_TXT_STATUSCODE_LICENSE_ERROR                       "License error (HW/SW license invalid or evaluation period expired)"

#define       EC_TXT_STATUSCODE_INVREQSTATECNG                      ((EC_T_WORD)(EC_ALSTATUSCODEBASE+0x0011)) /* DEVICE_STATUSCODE_INVREQSTATECNG */
#define EC_SZTXT_TXT_STATUSCODE_INVREQSTATECNG                      "Invalid requested state change"

#define       EC_TXT_STATUSCODE_UNKREQSTATE                         ((EC_T_WORD)(EC_ALSTATUSCODEBASE+0x0012)) /* DEVICE_STATUSCODE_UNKREQSTATE */
#define EC_SZTXT_TXT_STATUSCODE_UNKREQSTATE                         "Unknown requested state"

#define       EC_TXT_STATUSCODE_BOOTSTRAPNSUPP                      ((EC_T_WORD)(EC_ALSTATUSCODEBASE+0x0013)) /* DEVICE_STATUSCODE_BOOTSTRAPNSUPP */
#define EC_SZTXT_TXT_STATUSCODE_BOOTSTRAPNSUPP                      "Bootstrap not supported"

#define       EC_TXT_STATUSCODE_NOVALIDFW                           ((EC_T_WORD)(EC_ALSTATUSCODEBASE+0x0014)) /* DEVICE_STATUSCODE_NOVALIDFW */
#define EC_SZTXT_TXT_STATUSCODE_NOVALIDFW                           "No valid firmware"

#define       EC_TXT_STATUSCODE_INVALIDMBXCNF1                      ((EC_T_WORD)(EC_ALSTATUSCODEBASE+0x0015)) /* DEVICE_STATUSCODE_INVALIDMBXCNF1 */
#define EC_SZTXT_TXT_STATUSCODE_INVALIDMBXCNF1                      "Invalid mailbox configuration in BOOT"

#define       EC_TXT_STATUSCODE_INVALIDMBXCNF2                      ((EC_T_WORD)(EC_ALSTATUSCODEBASE+0x0016)) /* DEVICE_STATUSCODE_INVALIDMBXCNF2 */
#define EC_SZTXT_TXT_STATUSCODE_INVALIDMBXCNF2                      "Invalid mailbox configuration in PREOP"

#define       EC_TXT_STATUSCODE_INVALIDSMCNF                        ((EC_T_WORD)(EC_ALSTATUSCODEBASE+0x0017)) /* DEVICE_STATUSCODE_INVALIDSMCNF */
#define EC_SZTXT_TXT_STATUSCODE_INVALIDSMCNF                        "Invalid sync manager configuration"

#define       EC_TXT_STATUSCODE_NOVALIDIN                           ((EC_T_WORD)(EC_ALSTATUSCODEBASE+0x0018)) /* DEVICE_STATUSCODE_NOVALIDIN */
#define EC_SZTXT_TXT_STATUSCODE_NOVALIDIN                           "No valid inputs available"

#define       EC_TXT_STATUSCODE_NOVALIDOUT                          ((EC_T_WORD)(EC_ALSTATUSCODEBASE+0x0019)) /* DEVICE_STATUSCODE_NOVALIDOUT */
#define EC_SZTXT_TXT_STATUSCODE_NOVALIDOUT                          "No valid outputs available"

#define       EC_TXT_STATUSCODE_SYNCERROR                           ((EC_T_WORD)(EC_ALSTATUSCODEBASE+0x001A)) /* DEVICE_STATUSCODE_SYNCERROR */
#define EC_SZTXT_TXT_STATUSCODE_SYNCERROR                           "Synchronization error"

#define       EC_TXT_STATUSCODE_SMWATCHDOG                          ((EC_T_WORD)(EC_ALSTATUSCODEBASE+0x001B)) /* DEVICE_STATUSCODE_SMWATCHDOG */
#define EC_SZTXT_TXT_STATUSCODE_SMWATCHDOG                          "Sync manager watchdog"

#define       EC_TXT_STATUSCODE_INVSMTYPES                          ((EC_T_WORD)(EC_ALSTATUSCODEBASE+0x001C)) /* DEVICE_STATUSCODE_INVSMTYPES */
#define EC_SZTXT_TXT_STATUSCODE_INVSMTYPES                          "Invalid Sync Manager Types"

#define       EC_TXT_STATUSCODE_INVOUTCONFIG                        ((EC_T_WORD)(EC_ALSTATUSCODEBASE+0x001D)) /* DEVICE_STATUSCODE_INVOUTCONFIG */
#define EC_SZTXT_TXT_STATUSCODE_INVOUTCONFIG                        "Invalid Output Configuration"

#define       EC_TXT_STATUSCODE_INVINCONFIG                         ((EC_T_WORD)(EC_ALSTATUSCODEBASE+0x001E)) /* DEVICE_STATUSCODE_INVINCONFIG */
#define EC_SZTXT_TXT_STATUSCODE_INVINCONFIG                         "Invalid Input Configuration"

#define       EC_TXT_STATUSCODE_INVWDCONFIG                         ((EC_T_WORD)(EC_ALSTATUSCODEBASE+0x001F)) /* DEVICE_STATUSCODE_INVWDCONFIG */
#define EC_SZTXT_TXT_STATUSCODE_INVWDCONFIG                         "Invalid Watchdog Configuration"

#define       EC_TXT_STATUSCODE_SLVNEEDCOLDRS                       ((EC_T_WORD)(EC_ALSTATUSCODEBASE+0x0020)) /* DEVICE_STATUSCODE_SLVNEEDCOLDRS */
#define EC_SZTXT_TXT_STATUSCODE_SLVNEEDCOLDRS                       "Slave needs cold start"

#define       EC_TXT_STATUSCODE_SLVNEEDINIT                         ((EC_T_WORD)(EC_ALSTATUSCODEBASE+0x0021)) /* DEVICE_STATUSCODE_SLVNEEDINIT */
#define EC_SZTXT_TXT_STATUSCODE_SLVNEEDINIT                         "Slave needs INIT"

#define       EC_TXT_STATUSCODE_SLVNEEDPREOP                        ((EC_T_WORD)(EC_ALSTATUSCODEBASE+0x0022)) /* DEVICE_STATUSCODE_SLVNEEDPREOP */
#define EC_SZTXT_TXT_STATUSCODE_SLVNEEDPREOP                        "Slave needs PREOP"

#define       EC_TXT_STATUSCODE_SLVNEEDSAFEOP                       ((EC_T_WORD)(EC_ALSTATUSCODEBASE+0x0023)) /* DEVICE_STATUSCODE_SLVNEEDSAFEOP */
#define EC_SZTXT_TXT_STATUSCODE_SLVNEEDSAFEOP                       "Slave needs SAFEOP"

#define       EC_TXT_STATUSCODE_INVALID_INPUT_MAPPING               ((EC_T_WORD)(EC_ALSTATUSCODEBASE+0x0024)) /* DEVICE_STATUSCODE_INVALID_INPUT_MAPPING */
#define EC_SZTXT_TXT_STATUSCODE_INVALID_INPUT_MAPPING               "Invalid Input Mapping"

#define       EC_TXT_STATUSCODE_INVALID_OUTPUT_MAPPING              ((EC_T_WORD)(EC_ALSTATUSCODEBASE+0x0025)) /* DEVICE_STATUSCODE_INVALID_OUTPUT_MAPPING */
#define EC_SZTXT_TXT_STATUSCODE_INVALID_OUTPUT_MAPPING              "Invalid Output Mapping"

#define       EC_TXT_STATUSCODE_INCONSISTENT_SETTINGS               ((EC_T_WORD)(EC_ALSTATUSCODEBASE+0x0026)) /* DEVICE_STATUSCODE_INCONSISTENT_SETTINGS */
#define EC_SZTXT_TXT_STATUSCODE_INCONSISTENT_SETTINGS               "Inconsistent Settings"

#define       EC_TXT_STATUSCODE_FREERUN_NOT_SUPPORTED               ((EC_T_WORD)(EC_ALSTATUSCODEBASE+0x0027)) /* DEVICE_STATUSCODE_FREERUN_NOT_SUPPORTED */
#define EC_SZTXT_TXT_STATUSCODE_FREERUN_NOT_SUPPORTED               "FreeRun not supported"

#define       EC_TXT_STATUSCODE_SYNCMODE_NOT_SUPPORTED              ((EC_T_WORD)(EC_ALSTATUSCODEBASE+0x0028)) /* DEVICE_STATUSCODE_SYNCMODE_NOT_SUPPORTED */
#define EC_SZTXT_TXT_STATUSCODE_SYNCMODE_NOT_SUPPORTED              "SyncMode not supported"

#define       EC_TXT_STATUSCODE_FREERUN_NEEDS_THREEBUFFER_MODE      ((EC_T_WORD)(EC_ALSTATUSCODEBASE+0x0029)) /* DEVICE_STATUSCODE_FREERUN_NEEDS_THREEBUFFER_MODE */
#define EC_SZTXT_TXT_STATUSCODE_FREERUN_NEEDS_THREEBUFFER_MODE      "FreeRun needs 3Buffer Mode"

#define       EC_TXT_STATUSCODE_BACKGROUND_WATCHDOG                 ((EC_T_WORD)(EC_ALSTATUSCODEBASE+0x002A)) /* DEVICE_STATUSCODE_BACKGROUND_WATCHDOG */
#define EC_SZTXT_TXT_STATUSCODE_BACKGROUND_WATCHDOG                 "Background Watchdog"

#define       EC_TXT_STATUSCODE_NO_VALID_INPUTS_AND_OUTPUTS         ((EC_T_WORD)(EC_ALSTATUSCODEBASE+0x002B)) /* DEVICE_STATUSCODE_NO_VALID_INPUTS_AND_OUTPUTS */
#define EC_SZTXT_TXT_STATUSCODE_NO_VALID_INPUTS_AND_OUTPUTS         "No Valid Inputs and Outputs"

#define       EC_TXT_STATUSCODE_FATAL_SYNC_ERROR                    ((EC_T_WORD)(EC_ALSTATUSCODEBASE+0x002C)) /* DEVICE_STATUSCODE_FATAL_SYNC_ERROR */
#define EC_SZTXT_TXT_STATUSCODE_FATAL_SYNC_ERROR                    "Fatal Sync Error"

#define       EC_TXT_STATUSCODE_NO_SYNC_ERROR                       ((EC_T_WORD)(EC_ALSTATUSCODEBASE+0x002D)) /* DEVICE_STATUSCODE_NO_SYNC_ERROR */
#define EC_SZTXT_TXT_STATUSCODE_NO_SYNC_ERROR                       "No Sync Error"

#define       EC_TXT_STATUSCODE_CYCLE_TOO_SHORT                     ((EC_T_WORD)(EC_ALSTATUSCODEBASE+0x002E)) /* DEVICE_STATUSCODE_CYCLE_TIME_TOO_SMALL */
#define EC_SZTXT_TXT_STATUSCODE_CYCLE_TOO_SHORT                     "EtherCAT cycle time smaller Minimum Cycle Time supported by slave"

#define       EC_TXT_STATUSCODE_INVDCSYNCCNFG                       ((EC_T_WORD)(EC_ALSTATUSCODEBASE+0x0030)) /* DEVICE_STATUSCODE_INVDCSYNCCNFG */
#define EC_SZTXT_TXT_STATUSCODE_INVDCSYNCCNFG                       "Invalid DC SYNCH Configuration"

#define       EC_TXT_STATUSCODE_INVDCLATCHCNFG                      ((EC_T_WORD)(EC_ALSTATUSCODEBASE+0x0031)) /* DEVICE_STATUSCODE_INVDCLATCHCNFG */
#define EC_SZTXT_TXT_STATUSCODE_INVDCLATCHCNFG                      "Invalid DC Latch Configuration"

#define       EC_TXT_STATUSCODE_PLLERROR                            ((EC_T_WORD)(EC_ALSTATUSCODEBASE+0x0032)) /* DEVICE_STATUSCODE_PLLERROR */
#define EC_SZTXT_TXT_STATUSCODE_PLLERROR                            "PLL Error"

#define       EC_TXT_STATUSCODE_INVDCIOERROR                        ((EC_T_WORD)(EC_ALSTATUSCODEBASE+0x0033)) /* DEVICE_STATUSCODE_INVDCIOERROR */
#define EC_SZTXT_TXT_STATUSCODE_INVDCIOERROR                        "Invalid DC IO Error"

#define       EC_TXT_STATUSCODE_INVDCTOERROR                        ((EC_T_WORD)(EC_ALSTATUSCODEBASE+0x0034)) /* DEVICE_STATUSCODE_INVDCTOERROR */
#define EC_SZTXT_TXT_STATUSCODE_INVDCTOERROR                        "Invalid DC Timeout Error"

#define       EC_TXT_STATUSCODE_DC_INVALID_SYNC_CYCLE_TIME          ((EC_T_WORD)(EC_ALSTATUSCODEBASE+0x0035)) /* DEVICE_STATUSCODE_DC_INVALID_SYNC_CYCLE_TIME */
#define EC_SZTXT_TXT_STATUSCODE_DC_INVALID_SYNC_CYCLE_TIME          "DC Invalid Sync Cycle Time"

#define       EC_TXT_STATUSCODE_DC_SYNC0_CYCLE_TIME                 ((EC_T_WORD)(EC_ALSTATUSCODEBASE+0x0036)) /* DEVICE_STATUSCODE_DC_SYNC0_CYCLE_TIME */
#define EC_SZTXT_TXT_STATUSCODE_DC_SYNC0_CYCLE_TIME                 "DC Sync0 Cycle Time"

#define       EC_TXT_STATUSCODE_DC_SYNC1_CYCLE_TIME                 ((EC_T_WORD)(EC_ALSTATUSCODEBASE+0x0037)) /* DEVICE_STATUSCODE_DC_SYNC1_CYCLE_TIME */
#define EC_SZTXT_TXT_STATUSCODE_DC_SYNC1_CYCLE_TIME                 "DC Sync1 Cycle Time"

#define       EC_TXT_STATUSCODE_MBX_AOE                             ((EC_T_WORD)(EC_ALSTATUSCODEBASE+0x0041)) /* DEVICE_STATUSCODE_MBX_AOE */
#define EC_SZTXT_TXT_STATUSCODE_MBX_AOE                             "MBX AOE"

#define       EC_TXT_STATUSCODE_MBX_EOE                             ((EC_T_WORD)(EC_ALSTATUSCODEBASE+0x0042)) /* DEVICE_STATUSCODE_MBX_EOE */
#define EC_SZTXT_TXT_STATUSCODE_MBX_EOE                             "MBX EOE"

#define       EC_TXT_STATUSCODE_MBX_COE                             ((EC_T_WORD)(EC_ALSTATUSCODEBASE+0x0043)) /* DEVICE_STATUSCODE_MBX_COE */
#define EC_SZTXT_TXT_STATUSCODE_MBX_COE                             "MBX COE"

#define       EC_TXT_STATUSCODE_MBX_FOE                             ((EC_T_WORD)(EC_ALSTATUSCODEBASE+0x0044)) /* DEVICE_STATUSCODE_MBX_FOE */
#define EC_SZTXT_TXT_STATUSCODE_MBX_FOE                             "MBX FOE"

#define       EC_TXT_STATUSCODE_MBX_SOE                             ((EC_T_WORD)(EC_ALSTATUSCODEBASE+0x0045)) /* DEVICE_STATUSCODE_MBX_SOE */
#define EC_SZTXT_TXT_STATUSCODE_MBX_SOE                             "MBX SOE"

#define       EC_TXT_STATUSCODE_MBX_VOE                             ((EC_T_WORD)(EC_ALSTATUSCODEBASE+0x004F)) /* DEVICE_STATUSCODE_MBX_VOE */
#define EC_SZTXT_TXT_STATUSCODE_MBX_VOE                             "MBX VOE"

#define       EC_TXT_STATUSCODE_EEPROM_NO_ACCESS                    ((EC_T_WORD)(EC_ALSTATUSCODEBASE+0x0050)) /* DEVICE_STATUSCODE_EEPROM_NO_ACCESS */
#define EC_SZTXT_TXT_STATUSCODE_EEPROM_NO_ACCESS                    "EEPROM no access"

#define       EC_TXT_STATUSCODE_EEPROM_ERROR                        ((EC_T_WORD)(EC_ALSTATUSCODEBASE+0x0051)) /* DEVICE_STATUSCODE_EEPROM_ERROR */
#define EC_SZTXT_TXT_STATUSCODE_EEPROM_ERROR                        "EEPROM Error"

#define       EC_TXT_STATUSCODE_EXT_HARDWARE_NOT_READY              ((EC_T_WORD)(EC_ALSTATUSCODEBASE+0x0052)) /* DEVICE_STATUSCODE_EXT_HARDWARE_NOT_READY */
#define EC_SZTXT_TXT_STATUSCODE_EXT_HARDWARE_NOT_READY              "External hardware not ready"

#define       EC_TXT_STATUSCODE_SLAVE_RESTARTED_LOCALLY             ((EC_T_WORD)(EC_ALSTATUSCODEBASE+0x0060)) /* DEVICE_STATUSCODE_SLAVE_RESTARTED_LOCALLY */
#define EC_SZTXT_TXT_STATUSCODE_SLAVE_RESTARTED_LOCALLY             "Slave restarted locally"

#define       EC_TXT_STATUSCODE_DEVICE_IDENTIFICATION_UPDATED       ((EC_T_WORD)(EC_ALSTATUSCODEBASE+0x0061)) /* DEVICE_STATUSCODE_DEVICE_IDENTIFICATION_UPDATED */
#define EC_SZTXT_TXT_STATUSCODE_DEVICE_IDENTIFICATION_UPDATED       "Device identification value updated"

#define       EC_TXT_STATUSCODE_MODULE_ID_LIST_NOT_MATCH            ((EC_T_WORD)(EC_ALSTATUSCODEBASE+0x0070)) /* DEVICE_STATUSCODE_MODULE_ID_LIST_NOT_MATCH */
#define EC_SZTXT_TXT_STATUSCODE_MODULE_ID_LIST_NOT_MATCH            "Detected Module Ident List (0xF030) and Configured Module Ident List (0xF050) does not match"

#define       EC_TXT_STATUSCODE_APPLICATION_CONTROLLER_AVAILABLE    ((EC_T_WORD)(EC_ALSTATUSCODEBASE+0x00F0)) /* DEVICE_STATUSCODE_APPLICATION_CONTROLLER_AVAILABLE */
#define EC_SZTXT_TXT_STATUSCODE_APPLICATION_CONTROLLER_AVAILABLE    "Application controller available"

#define       EC_ALSTATUSCODEBASE_LAST                              ((EC_T_WORD)0x03FF)

#endif /* INC_ECERROR */

/*-END OF SOURCE FILE--------------------------------------------------------*/
