# coding: utf-8

"""
Ctypes wrapper module for BUSMUST CAN Interface on win32/win64 systems.

Authors: busmust <busmust@126.com>, BUSMUST Co.,Ltd.
"""

# Import Standard Python Modules
# ==============================
import ctypes
import logging
import platform
from .exceptions import BmError

# Define Module Logger
# ====================
LOG = logging.getLogger(__name__)

# Vector XL API Definitions
# =========================
# Load Windows DLL
if 'Linux' in platform.system():
    DLL_NAME = 'libbmapi'
    if platform.architecture()[0] == '64bit':
        DLL_NAME += '64'
    if not 'x86' in platform.machine():
        DLL_NAME += '.arm'
    DLL_NAME += '.so'
else:
    DLL_NAME = './bmapi64.dll' if platform.architecture()[0] == '64bit' else './bmapi.dll'
import os
current_path = os.path.dirname(os.path.abspath(__file__))
library_path = os.path.join(current_path, DLL_NAME)

bmapi_dll = ctypes.cdll.LoadLibrary(library_path)

#/**
# * @def   BM_DATA_HEADER_SIZE
# * @brief Size (in bytes) of BM Data header, which contains type, routing, length and timestamp.
# */
BM_DATA_HEADER_SIZE = 8

#/**
# * @def   BM_DATA_PAYLOAD_MAX_SIZE
# * @brief Size (in bytes) of BM Data payload, which contains a concrete message in CANFD|LIN|FLEXRAY|... type.
# */
BM_DATA_PAYLOAD_MAX_SIZE = 72

#/**
# * @def   BM_DATA_MAX_SIZE
# * @brief Size (in bytes) of BM Data, which contains a header and payload.
# */
BM_DATA_MAX_SIZE = (BM_DATA_HEADER_SIZE + BM_DATA_PAYLOAD_MAX_SIZE)

#/**
# * @enum  BM_LogLevelTypeDef
# * @brief Busmust library log level, see BM_SetLogLevel() for details.
# */
BM_LOG_NONE = 0              #/**< Show nothing on debug console                                                                                 */
BM_LOG_ERR = 1               #/**< Show only ERR level messages on debug console, note this is the default level for release versions            */
BM_LOG_WRN = 2               #/**< Show ERR and WRN level messages on debug console                                                              */
BM_LOG_INF = 3               #/**< Show ERR|WRN|INF level messages on debug console                                                              */
BM_LOG_DBG = 4               #/**< Show all messages on debug console, including debug messages, note this is NOT available for release versions */


#/**
# * @enum  BM_CapabilityTypeDef
# * @brief Busmust Device capability flags, retrieved when enumerating devices using BM_Enumerate().
# */
BM_NONE_CAP = 0x0000        #/**< No capability */
BM_NONE_CAP = 0x0000        #/**< No capability */
BM_LIN_CAP = 0x0001         #/**< The device is capable of handling LIN messages */
BM_CAN_CAP = 0x0002         #/**< The device is capable of handling CAN messages */
BM_CAN_FD_CAP = 0x0004      #/**< The device is capable of handling CANFD (and CAN) messages */
BM_FLEXRAY_CAP = 0x0008     #/**< The device is capable of handling FLEXRAY messages */
BM_MODBUS_CAP = 0x0010      #/**< The device is capable of handling MODBUS messages */
BM_ETHERNET_CAP = 0x0020    #/**< The device is capable of handling ETHERNET messages */
BM_ALL_CAP = 0xFFFF         #/**< Typically used for masking the CAP fields when programming */

#/**
# * @enum  BM_DataTypeTypeDef
# * @brief Busmust data type flags, must be given in BM_DataTypeDef.
# */
BM_UNKNOWN_DATA = 0         #/**< Unknown data type */
BM_LIN_DATA = 1             #/**< LIN message data type */
BM_CAN_FD_DATA = 2          #/**< CAN or CAN-FD message data type (check FDF flag further) */
BM_FLEXRAY_DATA = 3         #/**< Flexray message data type */
BM_MODBUS_DATA = 4          #/**< MODBUS message data type */
BM_ETHERNET_DATA = 5        #/**< Ethernet message data type */
BM_ACK_DATA = 0x8           #/**< ACK from bus, which indicates TXCMPLT event if this is BM_CAN_FD_DATA */

#/**
# * @enum  BM_StatusTypeDef
# * @brief Busmust device & operation status, most APIs would return a status code to indicate the result of an operation.
# */
BM_ERROR_OK = 0x00000                       #/**< SUCCESS: No error occurred */
BM_ERROR_XMTFULL = 0x00001                  #/**< Low level Transmit buffer is full */
BM_ERROR_OVERRUN = 0x00002                  #/**< Bus overrun (the device cannot keep up with the high bus throughput) */
BM_ERROR_BUSLIGHT = 0x00004                 #/**< CAN Bus communication is light, see ISO11898 for details */
BM_ERROR_BUSHEAVY = 0x00008                 #/**< CAN Bus communication is heavy, see ISO11898 for details */
BM_ERROR_BUSWARNING = BM_ERROR_BUSHEAVY     #/**< CAN Bus communication is in warning state, see ISO11898 for details */
BM_ERROR_BUSPASSIVE = 0x40000               #/**< CAN node is in passive state, see ISO11898 for details */
BM_ERROR_BUSTIMEOUT = 0x80000               #/**< CAN node failed to transmit message within specified time, the node might be in PASSIVE or BUSOFF state */
BM_ERROR_BUSOFF = 0x00010                   #/**< CAN bus is in BUSOFF state, see ISO11898 for details */
BM_ERROR_ANYBUSERR = (BM_ERROR_BUSWARNING | BM_ERROR_BUSLIGHT | BM_ERROR_BUSHEAVY | BM_ERROR_BUSOFF | BM_ERROR_BUSPASSIVE)  #/**< CAN bus error occurred */
BM_ERROR_QRCVEMPTY = 0x00020                #/**< Receive buffer is empty, this might NOT be an error if you use BMAPI in polling mode */
BM_ERROR_QOVERRUN = 0x00040                 #/**< BMAPI internal Q overrun */
BM_ERROR_QXMTFULL = 0x00080                 #/**< High level Transmit queue is full */
BM_ERROR_REGTEST = 0x00100                  #/**< Reserved */
BM_ERROR_NODRIVER = 0x00200                 #/**< Reserved */
BM_ERROR_HWINUSE = 0x00400                  #/**< Hardware is in use (opened by another application) */
BM_ERROR_NETINUSE = 0x00800                 #/**< Reserved */
BM_ERROR_ILLHW = 0x01400                    #/**< Hardware error or invalid hardware handle */
BM_ERROR_ILLNET = 0x01800                   #/**< Invalid bus */
BM_ERROR_ILLCLIENT = 0x01C00                #/**< Invalid client */
BM_ERROR_ILLHANDLE = (BM_ERROR_ILLHW | BM_ERROR_ILLNET | BM_ERROR_ILLCLIENT)  #/* Invalid handle*/
BM_ERROR_RESOURCE = 0x02000                 #/**< Out of resource */
BM_ERROR_ILLPARAMTYPE = 0x04000             #/**< Invalid parameter type in API call */
BM_ERROR_ILLPARAMVAL = 0x08000              #/**< Invalid parameter value in API call */
BM_ERROR_UNKNOWN = 0x10000                  #/**< Unknown error */
BM_ERROR_ILLDATA = 0x20000                  #/**< Invalid data received/transmitted */
BM_ERROR_CAUTION = 0x2000000                #/**< Reserved */
BM_ERROR_INITIALIZE = 0x4000000             #/**< The device/library is not initialized */
BM_ERROR_ILLOPERATION = 0x8000000           #/**< Invalid operation */

#/**
# * @enum  BM_CanModeTypeDef
# * @brief CAN mode IDs, used by BM_SetCanMode() to change the operation mode of CAN device.
# */
BM_CAN_OFF_MODE = 0x01                      #/**< The device is logically disconnected from CAN bus */
BM_CAN_NORMAL_MODE = 0x00                   #/**< The device is running normally (with the capability to handle CAN and CANFD messages */
BM_CAN_SLEEP_MODE = 0x01                    #/**< The device is logically disconnected from CAN bus */
BM_CAN_INTERNAL_LOOPBACK_MODE = 0x02        #/**< The device is looping back messages internally without impacting the physical CAN bus */
BM_CAN_LISTEN_ONLY_MODE = 0x03              #/**< The device is receiving messages without impacting the physical CAN bus (do not send ACKs to the bus) */
BM_CAN_CONFIGURATION_MODE = 0x04            #/**< The device is under configuration and temporarily disconnected from CAN bus, For Internal usage only */
BM_CAN_EXTERNAL_LOOPBACK_MODE = 0x05        #/**< The device is looping back messages externally, all transmitted messages are echoed by the device itself */
BM_CAN_CLASSIC_MODE = 0x06                  #/**< The device is running normally (with the capability to handle only classical CAN2.0 messages */
BM_CAN_RESTRICTED_MODE = 0x07               #/**< Reserved */

#/**
# * @enum  BM_TerminalResistorTypeDef
# * @brief Terminal resistor values, used by BM_SetTerminalResistor() to change the terminal resistor of CAN device.
# */
BM_TRESISTOR_AUTO = 0               #/**< Reserved, currently unsupported */
BM_TRESISTOR_60 = 60                #/**< Currently unsupported */
BM_TRESISTOR_120 = 120              #/**< 120Ohm */
BM_TRESISTOR_DISABLED = 0xFFFF      #/**< Disable terminal resistor */

#/**
# * @enum  BM_MessageFlagsTypeDef
# * @brief CAN Message type flags, used in BM_CanMessageTypeDef.
# */
BM_MESSAGE_FLAGS_NORMAL = 0         #/**< Normal CAN message */
BM_MESSAGE_FLAGS_IDE = 0x01         #/**< Extended CAN message */
BM_MESSAGE_FLAGS_RTR = 0x02         #/**< Remote CAN message */
BM_MESSAGE_FLAGS_BRS = 0x04         #/**< CAN-FD bitrate switching is enabled */
BM_MESSAGE_FLAGS_FDF = 0x08         #/**< CAN-FD message */
BM_MESSAGE_FLAGS_ESI = 0x10         #/**< Reserved for gateways */

#/**
# * @enum  BM_RxFilterTypeTypeDef
# * @brief CAN RX filter type IDs, used in BM_RxFilterTypeDef.
# */
BM_RXFILTER_INVALID = 0                 #/**< Invalid (unused) RX filter entry */
BM_RXFILTER_BASIC = 1                   #/**< Basic RX filter, traditional acceptance filter based on message ID mask */
BM_RXFILTER_ADVANCED = 2                #/**< Busmust advanced RX filter, check both message ID and message payload */
BM_RXFILTER_E2EPASS = 3                 #/**< Busmust E2E RX filter, accept only messages that passed E2E checking */
BM_RXFILTER_E2EFAIL = 4                 #/**< Busmust E2E RX filter, accept only messages that failed E2E checking (for debugging purpose) */

#/**
# * @enum  BM_TxTaskTypeTypeDef
# * @brief CAN TX task type IDs, used in BM_TxTaskTypeDef.
# */
BM_TXTASK_INVALID = 0               #/**< Invalid (unused) TX task entry */
BM_TXTASK_FIXED = 1                 #/**< Basic TX task, send fixed ID and fixed payload */
BM_TXTASK_INCDATA = 2               #/**< Self-increment Data TX task */
BM_TXTASK_INCID = 3                 #/**< Self-increment ID TX task */
BM_TXTASK_RANDOMDATA = 4            #/**< Random Data TX task */
BM_TXTASK_RANDOMID = 5              #/**< Random ID TX task */

#/**
# * @enum  BM_StatTypeDef
# * @brief CAN runtime statistics item IDs, used in BM_GetStat().
# */
BM_STAT_NONE = 0                    #/**< Invalid statistics item */
BM_STAT_TX_MESSAGE = 1              #/**< Number of TX messages */
BM_STAT_RX_MESSAGE = 2              #/**< Number of RX messages */
BM_STAT_TX_BYTE = 3                 #/**< Number of TX bytes */
BM_STAT_RX_BYTE = 4                 #/**< Number of RX bytes */
BM_STAT_TX_ERROR = 5                #/**< Number of TX errors */
BM_STAT_RX_ERROR = 6                #/**< Number of RX errors */

#/**
# * @enum  BM_IsotpModeTypeDef
# * @brief ISOTP operation mode, used in BM_IsotpConfigTypeDef.
# */
BM_ISOTP_NORMAL_TESTER = 0          #/**< Default mode: normal (non-extended-addressing) UDS client(tester) */
BM_ISOTP_NORMAL_ECU = 1             #/**< normal (non-extended-addressing) UDS server(ECU)                  */
BM_ISOTP_EXTENDED_TESTER = 2        #/**< Currently unsupported: extended-addressing UDS client(tester)     */
BM_ISOTP_EXTENDED_ECU = 3           #/**< Currently unsupported: extended-addressing UDS server(ECU)        */


#/**
# * @typedef BM_DataHeaderTypeDef
# * @brief   Busmust data header, each BM_DataTypeDef contains a header which indicates payload information.
# */
class BM_DataHeaderTypeDef(ctypes.Structure):
    _pack_ = 1
    _fields_ = [
        ('type', ctypes.c_uint16, 4),           #/**< Data type, see BM_DataTypeTypeDef for details. */
        ('flags', ctypes.c_uint16, 4),          #/**< Reserved flags, keep 0 */
        ('dchn', ctypes.c_uint16, 4),           #/**< Destination channel ID, starting from zero, used by TX data to indicate the hardware about the target port. */
        ('schn', ctypes.c_uint16, 4),           #/**< Source channel ID, starting from zero, used by RX data to indicate the application about the source port. */
    ]

    def BM_DataHeaderTypeDef(self, type, flags, dchn, schn):
        self.type = type & 0xF
        self.flags = type & 0xF
        self.dchn = type & 0xF
        self.schn = type & 0xF

#/**
# * @typedef BM_DataTypeDef
# * @brief   Busmust data, abstract structure which holds concrete payload messages of various types (i.e. CAN messages).
# */
class BM_DataTypeDef(ctypes.Structure):
    _pack_ = 1
    _fields_ = [
        ('header', BM_DataHeaderTypeDef),                       #/**< data header, see BM_DataHeaderTypeDef for details. */
        ('length', ctypes.c_uint16),                            #/**< length in bytes of the payload only (header excluded). */
        ('timestamp', ctypes.c_uint32),                         #/**< 32-bit device local high precision timestamp in microseconds. */
        ('payload', ctypes.c_ubyte * BM_DATA_PAYLOAD_MAX_SIZE),  #/**< buffer holding concrete message payload (i.e. a CAN message in BM_CanMessageTypeDef format). */
    ]

    def __init__(self):
        # Default as CANFD message data object
        self.header.type = BM_CAN_FD_DATA
        self.length = ctypes.sizeof(message)

    def setCanMessage(self, message):
        self.header.type = BM_CAN_FD_DATA
        self.length = ctypes.sizeof(message)
        ctypes.memmove(ctypes.addressof(self.payload), ctypes.addressof(message), ctypes.sizeof(message))

    def getCanMessage(self):
        message = BM_CanMessageTypeDef()
        ctypes.memmove(ctypes.addressof(message), ctypes.addressof(self.payload), ctypes.sizeof(message))
        return message

#/**
# * @typedef BM_MessageIdTypeDef
# * @brief   Busmust CAN Message ID.
# * @note    You could also use a uint32_t, but please take care of memory alignments.
# */
class BM_MessageIdTypeDef(ctypes.Structure):
    _pack_ = 1
    _fields_ = [
        ('SID', ctypes.c_uint32, 11),           #/**< Standard ID */
        ('EID', ctypes.c_uint32, 18),           #/**< Extended ID */
        ('SID11', ctypes.c_uint32, 1),          #/**< Reserved */
        ('unimplemented1', ctypes.c_uint32, 2), #/**< Reserved */
    ]

    def getStandardId(self):
        return self.SID

    def getExtendedId(self):
        return (self.SID << 18) | self.EID

    def setStandardId(self, id11):
        self.SID = id11
        self.EID = 0

    def setExtendedId(self, id29):
        self.SID = (id29 >> 18) & 0x7FF
        self.EID = (id29 >> 0) & 0x3FFFF

#/**
# * @typedef BM_TxMessageCtrlTypeDef
# * @brief   Busmust TX CAN Message control fields.
# * @note    The first a few fields (until FDF) are bit compatible with BM_RxMessageCtrlTypeDef.
# */
class BM_TxMessageCtrlTypeDef(ctypes.Structure):
    _pack_ = 1
    _fields_ = [
        ('DLC', ctypes.c_uint32, 4),    #/**< CAN message DLC(0-F), note this is not the message length */
        ('IDE', ctypes.c_uint32, 1),    #/**< This message is an extended CAN message */
        ('RTR', ctypes.c_uint32, 1),    #/**< This message is a remote CAN message */
        ('BRS', ctypes.c_uint32, 1),    #/**< This message requires CAN-FD bitrate switching */
        ('FDF', ctypes.c_uint32, 1),    #/**< This message is a CAN-FD CAN message */
        ('ESI', ctypes.c_uint32, 1),    #/**< Reserved for gateways */
        ('SEQ', ctypes.c_uint32, 23),   #/**< Reserved for hardware sync */
    ]

#/**
# * @typedef BM_RxMessageCtrlTypeDef
# * @brief   Busmust RX CAN Message control fields.
# * @note    The first a few fields (until FDF) are bit compatible with BM_TxMessageCtrlTypeDef.
# */
class BM_RxMessageCtrlTypeDef(ctypes.Structure):
    _pack_ = 1
    _fields_ = [
        ('DLC', ctypes.c_uint32, 4),                #/**< CAN message DLC(0-F), note this is not the message length */
        ('IDE', ctypes.c_uint32, 1),                #/**< This message is an extended CAN message */
        ('RTR', ctypes.c_uint32, 1),                #/**< This message is a remote CAN message */
        ('BRS', ctypes.c_uint32, 1),                #/**< This message requires CAN-FD bitrate switching */
        ('FDF', ctypes.c_uint32, 1),                #/**< This message is a CAN-FD CAN message */
        ('ESI', ctypes.c_uint32, 1),                #/**< Reserved for gateways */
        ('unimplemented1', ctypes.c_uint32, 2),     #/**< Reserved */
        ('FilterHit', ctypes.c_uint32, 5),          #/**< By wich RX filter the message is accepted */
        ('unimplemented2', ctypes.c_uint32, 16),    #/**< Reserved */
    ]

#/**
# * @typedef BM_MessageCtrlTypeDef
# * @brief   Busmust CAN Message control fields.
# */
class BM_MessageCtrlTypeDef(ctypes.Union):
    _pack_ = 1
    _fields_ = [
        ('tx', BM_TxMessageCtrlTypeDef),                #/**< TX control */
        ('rx', BM_RxMessageCtrlTypeDef),                #/**< RX control */
    ]

#/**
# * @typedef BM_CanMessageTypeDef
# * @brief   Busmust CAN Message concrete type, usually used as payload of BM_DataTypeDef.
# * @note    The total length of this structure is 72B, it support both classic and FD CAN messages.
# */
class BM_CanMessageTypeDef(ctypes.Structure):
    _pack_ = 1
    _fields_ = [
        ('mid', BM_MessageIdTypeDef),                   #/**< CAN message ID, see BM_MessageIdTypeDef for details. */
        ('ctrl', BM_MessageCtrlTypeDef),                #/**< CAN message control fields, whether TX or RX is taken depends on the message direction. */
        ('payload', ctypes.c_ubyte * 64),                #/**< CAN message payload */
    ]

    def __init__(self, mid=0, dlc=0, ide=0, fdf=0, brs=0, rtr=0, esi=0, payload=None):
        ctypes.Structure.__init__(self)
        self.ctrl.tx.DLC = dlc
        self.ctrl.tx.IDE = ide
        self.ctrl.tx.FDF = fdf
        self.ctrl.tx.BRS = brs
        self.ctrl.tx.RTR = rtr
        self.ctrl.tx.ESI = esi
        self.setMessageId(mid)
        copyLength = min(len(payload), ctypes.ctypes.sizeof(self.payload)) if payload is not None else 0
        if copyLength > 0:
            ctypes.memmove(ctypes.addressof(self.payload), ctypes.addressof(payload), copyLength)

    def setMessageId(self, mid):
        if self.ctrl.tx.IDE:
            self.mid.setExtendedId(mid)
        else:
            self.mid.setStandardId(mid)

    def getMessageId(self):
        if self.ctrl.tx.IDE:
            return self.mid.getExtendedId()
        else:
            return self.mid.getStandardId()

#/**
# * @typedef BM_ChannelInfoTypeDef
# * @brief   Channel information, created when enumerating devices by BM_Enumerate() and used when opening device by BM_OpenEx().
# */
class BM_ChannelInfoTypeDef(ctypes.Structure):
    _pack_ = 1
    _fields_ = [
        ('name', ctypes.c_char * 64),       #/**< Device full name, for display purpose */
        ('sn', ctypes.c_ubyte * 16),        #/**< Device SN */
        ('uid', ctypes.c_ubyte * 12),       #/**< Device UID */
        ('version', ctypes.c_ubyte * 4),    #/**< Device Firmware Version */
        ('vid', ctypes.c_ushort),           #/**< Device VID */
        ('pid', ctypes.c_ushort),           #/**< Device PID */
        ('port', ctypes.c_ushort),          #/**< Port ID (0-7) of the device, note a multi-port device is enumerated as multiple dedicated BM_ChannelInfoTypeDef entries */
        ('cap', ctypes.c_ushort),           #/**< Device Capability flags, see BM_CapabilityTypeDef for details. */
        ('reserved', ctypes.c_ubyte * 4),
    ]

#/**
# * @typedef BM_ChannelInfoListTypeDef
# * @brief   A list of channel info.
# */
class BM_ChannelInfoListTypeDef(ctypes.Structure):
    _pack_ = 1
    _fields_ = [
        ('entries', BM_ChannelInfoTypeDef * 64),       #/**< A list of channel info */
    ]


#/**
# * @typedef BM_CanStatusInfoTypedef
# * @brief   CAN channel status detailed information, retrieved by calling BM_GetCanStatus(), see ISO11898 for details.
# */
class BM_CanStatusInfoTypedef(ctypes.Structure):
    _pack_ = 1
    _fields_ = [
        ('TXBO', ctypes.c_ubyte),       #/**< The CAN channel is in BUSOFF state */
        ('reserved', ctypes.c_ubyte),   #/**< Reserved */
        ('TXBP', ctypes.c_ubyte),       #/**< The CAN channel is in TX bus passive state */
        ('RXBP', ctypes.c_ubyte),       #/**< The CAN channel is in RX bus passive state */
        ('TXWARN', ctypes.c_ubyte),     #/**< The CAN channel is in TX warn state */
        ('RXWARN', ctypes.c_ubyte),     #/**< The CAN channel is in RX warn state */
        ('TEC', ctypes.c_ubyte),        #/**< TX Bus Error counter */
        ('REC', ctypes.c_ubyte),        #/**< RX Bus Error counter */
    ]

#/**
# * @typedef BM_BitrateTypeDef
# * @brief   CAN channel bitrate configuration, used by BM_SetBitrate().
# */
class BM_BitrateTypeDef(ctypes.Structure):
    _pack_ = 1
    _fields_ = [
        ('nbitrate', ctypes.c_ushort),  #/**< Nominal bitrate in kbps, default as 500, note this is the only valid birate in CAN CLASSIC mode. */
        ('dbitrate', ctypes.c_ushort),  #/**< Data bitrate in kbps, default as 500, note this is ignored in CAN CLASSIC mode. */
        ('nsamplepos', ctypes.c_ubyte), #/**< Nominal sample position (percentage), 0-100, default as 75 */
        ('dsamplepos', ctypes.c_ubyte), #/**< Data sample position (percentage), 0-100, default as 75 */
        #/* Setting any of the fields below would override the nbitrate configuration */
        ('clockfreq', ctypes.c_ubyte),  #/**< CAN controller clock in Mhz, default as 0 */
        ('reserved', ctypes.c_ubyte),
        ('nbtr0', ctypes.c_ubyte),      #/**< Nominal BTR0 register value, note this value is calculated using clockfreq, which might not be 16MHz */
        ('nbtr1', ctypes.c_ubyte),      #/**< Nominal BTR1 register value, note this value is calculated using clockfreq, which might not be 16MHz */
        ('dbtr0', ctypes.c_ubyte),      #/**< Data BTR0 register value, note this value is calculated using clockfreq, which might not be 16MHz */
        ('dbtr1', ctypes.c_ubyte),      #/**< Data BTR1 register value, note this value is calculated using clockfreq, which might not be 16MHz */
    ]

#/**
# * @typedef BM_RxFilterTypeDef
# * @brief   CAN channel RX filter item structure, used by BM_SetRxFilter().
# * @note    The filter support masking ID, flags and payload according to its type, 
# *          in order for a message to be accepted, all the fields are masked using AND logic:
# *          (flags & filter.flags_mask == filter.flags_value) AND (ID & filter.id_mask == filter.id_value) AND (payload & filter.payload_mask == filter.payload_value)
# */
class BM_RxFilterTypeDef(ctypes.Structure):
    _pack_ = 1
    _fields_ = [
        ('type', ctypes.c_ubyte),                   #/**< Type ID of the RX filter, see BM_RxFilterTypeTypeDef for details. */
        ('unused', ctypes.c_ubyte),                 
        ('flags_mask', ctypes.c_ubyte),             #/**< CAN message control Flags masks, see BM_MessageFlagsTypeDef for details. */
        ('flags_value', ctypes.c_ubyte),            #/**< CAN message control Flags values, see BM_MessageFlagsTypeDef for details. */
        ('reserved', ctypes.c_ubyte * 4),  
        ('id_mask', ctypes.c_uint32),
        ('id_value', ctypes.c_uint32),              #/**< CAN message ID values, see BM_MessageIdTypeDef for details. */
        ('payload_mask', ctypes.c_ubyte * 8),       #/**< CAN message payload masks, for CAN-FD messages, only the first 8 bytes are checked. */
        ('payload_value', ctypes.c_ubyte * 8),      #/**< CAN message payload values, for CAN-FD messages, only the first 8 bytes are checked. */
    ]

#/**
# * @typedef BM_RxFilterListTypeDef
# * @brief   A list of RX filters.
# */
class BM_RxFilterListTypeDef(ctypes.Structure):
    _pack_ = 1
    _fields_ = [
        ('entries', BM_RxFilterTypeDef * 2),                   #/**< A list of filters */
    ]

class BM_TxTaskIncDataPatternTypeDef(ctypes.Structure):
    _pack_ = 1
    _fields_ = [
        ('startbit', ctypes.c_uint16),              #/**< Start bit of data increment, currently only 8-bit aligned value is accepted */
        ('nbits', ctypes.c_ubyte),                  #/**< Number of bits of data increment, currently only 32 is accepted */
        ('format', ctypes.c_ubyte),                 #/**< 0x80=Intel, 0x00=Motorola */
        ('min', ctypes.c_uint32),                   #/**< Minimum value of the Increment range */
        ('max', ctypes.c_uint32),                   #/**< Maximum value of the Increment range */
        ('step', ctypes.c_uint32),                  #/**< Step of the Increment range */
    ]

class BM_TxTaskIncIdPatternTypeDef(ctypes.Structure):
    _pack_ = 1
    _fields_ = [
        ('min', ctypes.c_uint32),                   #/**< Minimum value of the Increment range */
        ('max', ctypes.c_uint32),                   #/**< Maximum value of the Increment range */
        ('step', ctypes.c_uint32),                  #/**< Step of the Increment range */
    ]

class BM_TxTaskRndDataPatternTypeDef(ctypes.Structure):
    _pack_ = 1
    _fields_ = [
        ('startbit', ctypes.c_uint16),              #/**< Start bit of data Random, currently only 8-bit aligned value is accepted */
        ('nbits', ctypes.c_ubyte),                  #/**< Number of bits of data Random, currently only 32 is accepted */
        ('format', ctypes.c_ubyte),                 #/**< 0x80=Intel, 0x00=Motorola */
        ('min', ctypes.c_uint32),                   #/**< Minimum value of the Random range */
        ('max', ctypes.c_uint32),                   #/**< Maximum value of the Random range */
        ('seed', ctypes.c_uint32),                  #/**< Seed of the Random range */
    ]

class BM_TxTaskRndIdPatternTypeDef(ctypes.Structure):
    _pack_ = 1
    _fields_ = [
        ('min', ctypes.c_uint32),                   #/**< Minimum value of the Increment range */
        ('max', ctypes.c_uint32),                   #/**< Maximum value of the Increment range */
        ('seed', ctypes.c_uint32),                  #/**< Seed of the Random range */
    ]

class BM_TxTaskPatternTypeDef(ctypes.Union):
    _pack_ = 1
    _fields_ = [
        ('incdata', BM_TxTaskIncIdPatternTypeDef),  #/**< INCDATA pattern data */
        ('incid', BM_TxTaskIncIdPatternTypeDef),    #/**< INCID pattern data */
        ('rnddata', BM_TxTaskRndDataPatternTypeDef),#/**< RNDDATA pattern data */
        ('rndid', BM_TxTaskRndIdPatternTypeDef),    #/**< RNDID pattern data */
        ('unused',  ctypes.c_ubyte * 48),           #/**< Reserved */
    ]

#/**
# * @typedef BM_TxTaskTypeDef
# * @brief   CAN channel TX task item structure, used by BM_SetTxTask().
# * @note    Once the CAN device is armed with TX tasks, it will try to parse the TX task and send CAN messages automatically.
# *          The difference with a software triggered CAN message in BusMaster is that 
# *          hardware triggered CAN messages are more precise in time and could reach a higher throughput.
# */
class BM_TxTaskTypeDef(ctypes.Structure):
    _pack_ = 1
    _fields_ = [
        ('type', ctypes.c_ubyte),               #/**< Type ID of the TX task, see BM_TxTaskTypeTypeDef for details. */
        ('unused', ctypes.c_ubyte),             #/**< Reserved */
        ('flags', ctypes.c_ubyte),              #/**< CAN message control Flags, see BM_MessageFlagsTypeDef for details. */
        ('length', ctypes.c_ubyte),             #/**< Length of payload in bytes (not DLC) */
        ('e2e', ctypes.c_ubyte),                #/**< Index of E2E (in E2E table), currently unsupported */
        ('reserved', ctypes.c_ubyte),           #/**< Reserved */
        ('cycle', ctypes.c_uint16),             #/**< ms delay between rounds */
        ('nrounds', ctypes.c_uint16),           #/**< num of cycles */
        ('nmessages', ctypes.c_uint16),         #/**< messages per round */
        ('id', ctypes.c_uint32),                #/**< CAN message arbitration id, see BM_MessageIdTypeDef for details. */
        ('pattern', BM_TxTaskPatternTypeDef),   #/**< TX task pattern data */
        ('payload', ctypes.c_ubyte * 64),       #/**< Default payload data, note this is also the template payload of the unchanged part in a volatile TX task */
    ]

#/**
# * @typedef BM_IsotpStatusTypeDef
# * @brief   ISOTP status report, used by ISOTP operation callback function.
# */
class BM_IsotpStatusTypeDef(ctypes.Structure):
    _pack_ = 1
    _fields_ = [
        ('version', ctypes.c_uint8),            #/**< Currently always 0x01 */
        ('flowcontrol', ctypes.c_uint8),        #/**< Current flow control status, 0=continue, 1=wait, 2=overflow, ff=timeout, */
        ('stmin', ctypes.c_uint8),              #/**< Current flow control status, i.e. 30 00 00 */
        ('blocksize', ctypes.c_uint8),          #/**< Current flow control status, i.e. 30 00 00 */
        ('ntransferredbytes', ctypes.c_uint32), #/**< Number of transferred bytes by now. */
        ('ntotalbytes', ctypes.c_uint32),       #/**< Number of total bytes indicated by ISOTP FF or SF. */
        ('timestamp', ctypes.c_uint32),         #/**< Current timestamp reported by device. */
        ('reserved', ctypes.c_uint32 * 4),      #/**< Reserved for future. */
    ]

#/**
# * @typedef BM_IsotpCallbackHandle
# * @brief   Pointer to a Callback function when ISOTP transaction progress updates, normally this would occur at least once per FC frame.
# * @param[in] status              Current ISOTP status reported by ISOTP context.
# * @param[in] userarg             Arbitrary user argument passed by BM_IsotpConfigTypeDef.
# * @return                        Currently not used, please return 0 for further compatibility.
# */
#typedef uint8_t (*BM_IsotpCallbackHandle)(const BM_IsotpStatusTypeDef* status, uintptr_t userarg);
BM_IsotpCallbackHandle = ctypes.CFUNCTYPE(ctypes.POINTER(BM_IsotpStatusTypeDef), ctypes.c_void_p)

class BM_IsotpTimeoutConfigTypeDef(ctypes.Structure):
    _pack_ = 1
    _fields_ = [
        ('a', ctypes.c_uint16),                 #/**< A timeout in milliseconds: =N_As if writing as tester or reading as ECU, otherwise =N_Ar */
        ('b', ctypes.c_uint16),                 #/**< B timeout in milliseconds: =N_Bs if writing as tester or reading as ECU, otherwise =N_Br */
        ('c', ctypes.c_uint16),                 #/**< C timeout in milliseconds: =N_Cs if writing as tester or reading as ECU, otherwise =N_Cr */
    ]

class BM_IsotpFlowcontrolConfigTypeDef(ctypes.Structure):
    _pack_ = 1
    _fields_ = [
        ('stmin', ctypes.c_uint8),              #/**< STmin raw value (0x00-0x7F or 0xF1-0xF9) if Busmust device is acting as UDS server */
        ('blockSize', ctypes.c_uint8),          #/**< Blocksize if can card is acting as UDS server, 0 means no further FC is needed     */
        ('fcFrameLength', ctypes.c_uint8),      #/**< Flow control frame length in bytes                                                 */
        ('reserved', ctypes.c_uint8),           
    ]

#/**
# * @typedef BM_IsotpConfigTypeDe
# * @brief   ISOTP Protocol (See ISO15765-2 for details) configuration, used by BM_ConfigIsotp().
# */
class BM_IsotpConfigTypeDef(ctypes.Structure):
    _pack_ = 1
    _fields_ = [
        ('version', ctypes.c_ubyte),                            #/**< Currently must be set to 0x01                                                      */
        ('mode', ctypes.c_ubyte),                               #/**< Currently only 0 is supported: normal (non-extended-addressing) UDS client(tester) */
        ('testerTimeout', BM_IsotpTimeoutConfigTypeDef),        #/**< Tester(UDS Client) Timeout configuration                                           */
        ('ecuTimeout', BM_IsotpTimeoutConfigTypeDef),           #/**< ECU(UDS Server) Timeout configuration                                              */
        ('flowcontrol', BM_IsotpFlowcontrolConfigTypeDef),      #/**< Current flow control status, i.e. 30 00 00                                         */
        ('extendedAddress', ctypes.c_uint8),                    #/**< UDS Address in Extended Addressing mode                                            */
        ('paddingEnabled', ctypes.c_uint8),                     #/**< Enable padding for unused payload bytes/                                           */
        ('paddingValue', ctypes.c_uint8),                       #/**< Padding byte value (i.e. 0xCC) for unused payload bytes                            */
        ('longPduEnabled', ctypes.c_uint8),                     #/**< Enable long PDU (>4095), note if CAN message DLC>8, long PDU is enabled by default */
        ('padding', ctypes.c_uint8 * 2),                        #/**< Reserved for future                                                                */
        ('callbackFunc', BM_IsotpCallbackHandle),               #/**< Callback function when any progress is made, used typically by GUI to show progress bar */
        ('callbackUserarg', ctypes.c_void_p),                   #/**< Callback userarg when any progress is made, used typically by GUI to show progress bar  */
        ('testerDataTemplate', BM_DataTypeDef),                 #/**< All tester messages will be formatted/checked using this template, configure CAN message ID and IDE/FDF flags here  */
        ('ecuDataTemplate', BM_DataTypeDef),                    #/**< All ECU messages will be formatted/checked using this template, configure CAN message ID and IDE/FDF flags here */
    ]

    def __init__(self):
        ctypes.Structure.__init__(self)
        self.version = 1

BM_StatusTypeDef = ctypes.c_int
BM_CanModeTypeDef = ctypes.c_int
BM_TerminalResistorTypeDef = ctypes.c_int
BM_INVALID_CHANNELHANDLE = ctypes.c_void_p()
BM_ChannelHandle = ctypes.c_void_p
BM_NotificationHandle = ctypes.c_void_p

#/**
# * @typedef BM_AutosetCallbackHandle
# * @brief   Pointer to a Callback function when AUTOSET status is updates, indicating a bitrate option has passed or failed.
# * @param[in] bitrate      The bitrate option value which has passed or failed.
# * @param[in] tres         The terminal resistor option value which has passed or failed.
# * @param[in] nrxmessages  Number of received messages while listening to the bus using bitrate and tres.
# * @param[in] userarg      Arbitrary user argument passed by BM_Autoset().
# */
#typedef void (*BM_AutosetCallbackHandle)(const BM_BitrateTypeDef* bitrate, BM_TerminalResistorTypeDef tres, int nrxmessages, uintptr_t userarg);
BM_AutosetCallbackHandle = ctypes.CFUNCTYPE(ctypes.POINTER(BM_BitrateTypeDef), BM_TerminalResistorTypeDef, ctypes.c_int, ctypes.c_void_p)

def check_status(result, function, arguments):
    if result > 0:
        buf = ctypes.create_string_buffer(256)
        BM_GetErrorText(result, buf, len(buf), 0)
        raise BmError(result, buf.value.decode(), function.__name__)
    return result

#/**
# * @brief  Initialize BMAPI library, this function shall be called before any other API calls and shall only be called once.
# * @return Operation status, see BM_StatusTypeDef for details.
# */
#BMAPI BM_StatusTypeDef BM_Init(void);
BM_Init = bmapi_dll.BM_Init
BM_Init.argtypes = []
BM_Init.restype = BM_StatusTypeDef
BM_Init.errcheck = check_status

#/**
# * @brief  Un-initialize BMAPI library, this function shall be called after any other API calls and shall only be called once.
# * @return Operation status, see BM_StatusTypeDef for details.
# */
#BMAPI BM_StatusTypeDef BM_UnInit(void);
BM_UnInit = bmapi_dll.BM_UnInit
BM_UnInit.argtypes = []
BM_UnInit.restype = BM_StatusTypeDef
BM_UnInit.errcheck = check_status

#/**
# * @brief        Enumerate all connected Busmust device.
# * @param[out]   channelinfos  An array of BM_ChannelInfoTypeDef structure which holds info of all the enumerated Busmust devices.
# * @param[inout] nchannels     Number of device channels available, which is also the number of valid entries in channelinfos, 
# *                             this param must be initialized with the maximum length of the channelinfos array when calling this function.
# * @return       Operation status, see BM_StatusTypeDef for details.
# */
#BMAPI BM_StatusTypeDef BM_Enumerate(BM_ChannelInfoTypeDef channelinfos[], int* nchannels);
BM_Enumerate = bmapi_dll.BM_Enumerate
BM_Enumerate.argtypes = [ctypes.POINTER(BM_ChannelInfoListTypeDef), ctypes.POINTER(ctypes.c_int)]
BM_Enumerate.restype = BM_StatusTypeDef
BM_Enumerate.errcheck = check_status

#/**
# * @brief Start AUTOSET sequence, for a BM-USB-CAN(FD) device, the AUTOSET sequence will detect the correct bitrate and terminal resistor.
# * @param[in]  channelinfo  Info of the device channel to run AUTOSET on, usually the info is filled by BM_Enumerate().
# * @param[out] bitrate      The detected bitrate.
# * @param[out] tres         The detected terminal resistor.
# * @param[in]  callback     A callback function which will be called on each step of AUTOSET.
# * @param[in]  userarg      Arbitrary user argument of the callback function, this argument will be passed to the callback as is.
# */
#BMAPI BM_StatusTypeDef BM_Autoset(
#    BM_ChannelInfoTypeDef* channelinfo,
#    BM_BitrateTypeDef* bitrate,
#    BM_TerminalResistorTypeDef* tres,
#    BM_AutosetCallbackHandle callback,
#    uintptr_t userarg
#);
BM_Autoset = bmapi_dll.BM_Autoset
BM_Autoset.argtypes = [
    ctypes.POINTER(BM_ChannelInfoTypeDef), 
    ctypes.POINTER(BM_BitrateTypeDef), 
    ctypes.POINTER(BM_TerminalResistorTypeDef), 
    BM_AutosetCallbackHandle, 
    ctypes.c_void_p
]
BM_Autoset.restype = BM_StatusTypeDef
BM_Autoset.errcheck = check_status

#/**
# * @brief Open the specified CAN device port.
# * @param[in] port  Index of the port, starting from zero, note this is the index of all enumerated ports.
# * @return Handle to the opened CAN device channel, return NULL if failed to open the specified port.
# */
#BMAPI BM_ChannelHandle BM_OpenCan(uint16_t port);
BM_OpenCan = bmapi_dll.BM_OpenCan
BM_OpenCan.argtypes = [ctypes.c_uint16]
BM_OpenCan.restype = BM_ChannelHandle

#/**
# * @brief Open the specified device port using given configuration.
# * @param[out] handle      Handle to the opened device channel.
# * @param[in]  channelinfo Info of the device channel to open, usually the info is filled by BM_Enumerate().
# * @param[in]  mode        CAN operation mode option of the opened channel, see BM_CanModeTypeDef for details.
# * @param[in]  tres        Terminal resistor option of the opened channel, see BM_TerminalResistorTypeDef for details.
# * @param[in]  bitrate     Bitrate option of the opened channel, see BM_BitrateTypeDef for details.
# * @param[in]  rxfilter    CAN acceptance filters option of the opened channel, see BM_RxFilterTypeDef for details.
# * @param[in]  nrxfilters  Number of acceptance filters, usually there could be up to 2 filters.
# * @return Operation status, see BM_StatusTypeDef for details.
# */
#BMAPI BM_StatusTypeDef BM_OpenEx(
#        BM_ChannelHandle* handle,
#        BM_ChannelInfoTypeDef* channelinfo,
#        BM_CanModeTypeDef mode,
#        BM_TerminalResistorTypeDef tres,
#        const BM_BitrateTypeDef* bitrate,
#        const BM_RxFilterTypeDef* rxfilter,
#        int nrxfilters
#        );
BM_OpenEx = bmapi_dll.BM_OpenEx
BM_OpenEx.argtypes = [
    ctypes.POINTER(BM_ChannelHandle), 
    ctypes.POINTER(BM_ChannelInfoTypeDef), 
    BM_CanModeTypeDef, 
    BM_TerminalResistorTypeDef, 
    ctypes.POINTER(BM_BitrateTypeDef), 
    ctypes.POINTER(BM_RxFilterListTypeDef),
    ctypes.c_int
]
BM_OpenEx.restype = BM_StatusTypeDef
BM_OpenEx.errcheck = check_status

#/**
# * @brief     Close an opened channel.
# * @param[in] handle  Handle to the channel to be closed.
# * @return    Operation status, see BM_StatusTypeDef for details.
# */
#BMAPI BM_StatusTypeDef BM_Close(BM_ChannelHandle handle);
BM_Close = bmapi_dll.BM_Close
BM_Close.argtypes = [BM_ChannelHandle]
BM_Close.restype = BM_StatusTypeDef
BM_Close.errcheck = check_status

#/**
# * @brief     Reset an opened channel.
# * @param[in] handle  Handle to the channel to be reset.
# * @return    Operation status, see BM_StatusTypeDef for details.
# * @note      The configuration options will not lost when the channel is reset, so BM_Reset() is basically identical to BM_Close() and then BM_OpenEx().
# */
#BMAPI BM_StatusTypeDef BM_Reset(BM_ChannelHandle handle);
BM_Reset = bmapi_dll.BM_Reset
BM_Reset.argtypes = [BM_ChannelHandle]
BM_Reset.restype = BM_StatusTypeDef
BM_Reset.errcheck = check_status

#/**
# * @brief     Reset the device hardware which an opened channel belongs to, in case the device hardware is in unknown error state and is unrecoverable using BM_OpenEx.
# * @param[in] handle  Handle to a opened channel (which belongs to the physical device to reset).
# * @return    Operation status, see BM_StatusTypeDef for details.
# * @note      !!!CAUTION!!! Note this API will break all active USB/Ethernet connection, just like you have manually unplugged it and then plugged it back.
# *            All opened channel handles that belongs to the physical device under reset will be invalid after reset.
# *            You MUST re-open and re-configure all channels using BM_OpenEx or other configuration APIs after reset.
# */
#BMAPI BM_StatusTypeDef BM_ResetDevice(BM_ChannelHandle handle);
BM_ResetDevice = bmapi_dll.BM_ResetDevice
BM_ResetDevice.argtypes = [BM_ChannelHandle]
BM_ResetDevice.restype = BM_StatusTypeDef
BM_ResetDevice.errcheck = check_status

#/**
# * @brief     Activate an opened channel, and thus goes on bus for the selected port and channels. 
#              At this point, the user can transmit and receive messages on the bus.
# * @param[in] handle  Handle to the channel to be activated.
# * @return    Operation status, see BM_StatusTypeDef for details.
# * @note      Channel is default to be activated after BM_OpenEx() is called.
# */
#BMAPI BM_StatusTypeDef BM_Activate(BM_ChannelHandle handle);
BM_Activate = bmapi_dll.BM_Activate
BM_Activate.argtypes = [BM_ChannelHandle]
BM_Activate.restype = BM_StatusTypeDef
BM_Activate.errcheck = check_status

#/**
# * @brief     Deactivate an opened channel, and thus the selected channels goes off the bus and stay in BUSOFF state until re-activation.
# * @param[in] handle  Handle to the channel to be deactivated.
# * @return    Operation status, see BM_StatusTypeDef for details.
# * @note      Any call to BM_Write() or BM_Read() will return BM_ERROR_BUSOFF immediately if the channel is deactivated.
# */
#BMAPI BM_StatusTypeDef BM_Deactivate(BM_ChannelHandle handle);
BM_Deactivate = bmapi_dll.BM_Deactivate
BM_Deactivate.argtypes = [BM_ChannelHandle]
BM_Deactivate.restype = BM_StatusTypeDef
BM_Deactivate.errcheck = check_status

#/**
# * @brief     Clear TX&RX message buffer of an opened channel.
# * @param[in] handle  Handle to the channel to be cleared.
# * @return    Operation status, see BM_StatusTypeDef for details.
# * @note      This function is available since BMAPI1.3, hardware status will not be changed when clearing buffer.
# */
#BMAPI BM_StatusTypeDef BM_ClearBuffer(BM_ChannelHandle handle);
BM_ClearBuffer = bmapi_dll.BM_ClearBuffer
BM_ClearBuffer.argtypes = [BM_ChannelHandle]
BM_ClearBuffer.restype = BM_StatusTypeDef
BM_ClearBuffer.errcheck = check_status

#/**
# * @brief      Read any message/event out of the given channel.
# * @param[in]  handle  Handle to the channel to read from.
# * @param[out] data    A caller-allocated buffer to hold the message/event output, see BM_DataTypeDef for details.
# * @return     Operation status, see BM_StatusTypeDef for details.
# * @note       This function is non-blocked, and thus will return BM_ERROR_QRCVEMPTY if no message is received.
# *             Please use notifications to wait for Rx events and then read message/event out of BMAPI internal RX buffer, otherwise you could also poll the device periodically.
# */
#BMAPI BM_StatusTypeDef BM_Read(BM_ChannelHandle handle, BM_DataTypeDef* data);
BM_Read = bmapi_dll.BM_Read
BM_Read.argtypes = [BM_ChannelHandle, ctypes.POINTER(BM_DataTypeDef)]
BM_Read.restype = BM_StatusTypeDef
BM_Read.errcheck = check_status

#/**
# * @brief        Read multiple messages/events out of the given channel.
# * @param[in]    handle  Handle to the channel to read from.
# * @param[out]   data       A caller-allocated buffer to hold the messages/events array output, see BM_DataTypeDef for details.
# * @param[inout] nmessages  Number of read messages, user shall initialize this param with the size (in messages) of the data buffer.
# * @param[in]    timeout    Timeout (in milliseconds) before the message is received successfully from the bus.
# *                          Set any negative number (i.e. -1) to wait infinitely.
# *                          Set 0 if you would like to receive asynchronously: read from BMAPI internal buffer and return immediately, use BM_WaitForNotifications() before reading.
# * @return       Operation status, see BM_StatusTypeDef for details.
# * @note         This function is non-blocked, and thus will return BM_ERROR_QRCVEMPTY if not all messages are received.
# *               Please use notifications to wait for Rx events and then read message/event out of BMAPI internal RX buffer, otherwise you could also poll the device periodically.
# */
#BMAPI BM_StatusTypeDef BM_ReadMultiple(BM_ChannelHandle handle, BM_DataTypeDef data[], uint32_t* nmessages, int timeout);
BM_ReadMultiple = bmapi_dll.BM_ReadMultiple
BM_ReadMultiple.argtypes = [BM_ChannelHandle, ctypes.POINTER(BM_DataTypeDef), ctypes.POINTER(ctypes.c_uint32), ctypes.c_int]
BM_ReadMultiple.restype = BM_StatusTypeDef
BM_ReadMultiple.errcheck = check_status

#/**
# * @brief        Read data block using ISOTP protocol.
# *               This API enables rapid transmission using ISOTP without app intervention, a simple example would be reading VIN using UDS:
# *               uint8_t request[] = { 0x22, 0xF1, 0x80 };
# *               uint8_t response[4096];
# *               uint32_t nbytes = sizeof(response);
# *               BM_WriteIsotp(channel, request, sizeof(request), config);
# *               BM_ReadIsotp(channel, response, nbytes, config);
# *               assert(response[0] == 0x62 && response[1] == 0xF1 && response[2] == 0x80);
# * @param[in]    handle    Handle to the channel to read from.
# * @param[in]    data      A caller-allocated buffer to hold the data block output.
# * @param[inout] nbytes    Length of the received data block, in bytes. Caller must initialize this arg with the size of the caller-allocated buffer.
# * @param[in]    timeout   Timeout (in milliseconds) before the message is received successfully from the bus.
# *                         Set any negative number (i.e. -1) to wait infinitely.
# *                         Set 0 if you would like to receive asynchronously: read from BMAPI internal buffer and return immediately, use BM_WaitForNotifications() before reading.
# * @param[in]    config    ISOTP configuration used by current transfer.
# * @return     Operation status, see BM_StatusTypeDef for details.
# * @note       This function is allowed to be called from multiple threads since BMAPI1.5.
# */
#BMAPI BM_StatusTypeDef BM_ReadIsotp(BM_ChannelHandle handle, const void* data, uint32_t* nbytes, int timeout, BM_IsotpConfigTypeDef* config);
BM_ReadIsotp = bmapi_dll.BM_ReadIsotp
BM_ReadIsotp.argtypes = [BM_ChannelHandle, ctypes.c_char_p, ctypes.POINTER(ctypes.c_uint32), ctypes.c_int, ctypes.POINTER(BM_IsotpConfigTypeDef)]
BM_ReadIsotp.restype = BM_StatusTypeDef
BM_ReadIsotp.errcheck = check_status

#/**
# * @brief      Read CAN message out of the given channel.
# * @param[in]  handle     Handle to the channel to read from.
# * @param[out] msg        A caller-allocated buffer to hold the CAN message output, see BM_CanMessageTypeDef for details.
# * @param[out] channel    The source channel ID from which the message is received, starting from zero, could be NULL if not required.
# * @param[out] timestamp  The device local high precision timestamp in microseconds, when the message is physically received on the CAN bus, could be NULL if not required.
# * @return     Operation status, see BM_StatusTypeDef for details. 
# * @note       Note this function is a simple wrapper of BM_Read(), see BM_Read() for details.
# */
#BMAPI BM_StatusTypeDef BM_ReadCanMessage(BM_ChannelHandle handle, BM_CanMessageTypeDef* msg, uint32_t* channel, uint32_t* timestamp);
BM_ReadCanMessage = bmapi_dll.BM_ReadCanMessage
BM_ReadCanMessage.argtypes = [BM_ChannelHandle, ctypes.POINTER(BM_CanMessageTypeDef), ctypes.POINTER(ctypes.c_uint32), ctypes.POINTER(ctypes.c_uint32)]
BM_ReadCanMessage.restype = BM_StatusTypeDef
BM_ReadCanMessage.errcheck = check_status

#/**
# * @brief        Read multiple CAN messages out of the given channel.
# * @param[in]    handle  Handle to the channel to read from.
# * @param[out]   data       A caller-allocated buffer to hold the CAN message array output, see BM_CanMessageTypeDef for details.
# * @param[inout] nmessages  Number of read messages, user shall initialize this param with the size (in messages) of the data buffer.
# * @param[in]    timeout    Timeout (in milliseconds) before the message is received successfully from the bus.
# *                          Set any negative number (i.e. -1) to wait infinitely.
# *                          Set 0 if you would like to receive asynchronously: read from BMAPI internal buffer and return immediately, use BM_WaitForNotifications() before reading.
# * @param[out]   channel    The source channel ID from which the message is received, starting from zero, could be NULL if not required.
# * @param[out]   timestamp  The device local high precision timestamp array in microseconds, when the message is physically transmitted on the CAN bus, could be NULL if not required.
# * @return       Operation status, see BM_StatusTypeDef for details.
# * @note         This function is non-blocked, and thus will return BM_ERROR_QRCVEMPTY if not all messages are received.
# *               Please use notifications to wait for Rx events and then read message/event out of BMAPI internal RX buffer, otherwise you could also poll the device periodically.
# */
#BMAPI BM_StatusTypeDef BM_ReadMultipleCanMessage(BM_ChannelHandle handle, BM_CanMessageTypeDef msg[], uint32_t* nmessages, int timeout, uint32_t channel[], uint32_t timestamp[]);
BM_ReadMultipleCanMessage = bmapi_dll.BM_ReadMultipleCanMessage
BM_ReadMultipleCanMessage.argtypes = [BM_ChannelHandle, ctypes.POINTER(BM_CanMessageTypeDef), ctypes.POINTER(ctypes.c_uint32), ctypes.c_int, ctypes.POINTER(ctypes.c_uint32), ctypes.POINTER(ctypes.c_uint32)]
BM_ReadMultipleCanMessage.restype = BM_StatusTypeDef
BM_ReadMultipleCanMessage.errcheck = check_status

#/**
# * @brief      Write any message/event to the given channel.
# * @param[in]  handle  Handle to the channel to write to.
# * @param[in]  data      A caller-allocated buffer to hold the message/event input, see BM_DataTypeDef for details.
# * @param[in]  timeout   Timeout (in milliseconds) before the message is transmitted successfully to the bus.
# *                       Set any negative number (i.e. -1) to wait infinitely.
# *                       Set 0 if you would like to transmit asynchronously: put to BMAPI internal buffer and return immediately, then receive TXCMPLT event over BM_Read() later.
# * @param[in]  timestamp The device local high precision timestamp in microseconds, when the message is physically transmitted on the CAN bus, could be NULL if not required.
# * @return     Operation status, see BM_StatusTypeDef for details.
# */
#BMAPI BM_StatusTypeDef BM_Write(BM_ChannelHandle handle, const BM_DataTypeDef* data, int timeout, uint32_t* timestamp);
BM_Write = bmapi_dll.BM_Write
BM_Write.argtypes = [BM_ChannelHandle, ctypes.POINTER(BM_DataTypeDef), ctypes.c_int, ctypes.POINTER(ctypes.c_uint32)]
BM_Write.restype = BM_StatusTypeDef
BM_Write.errcheck = check_status

#/**
# * @brief        Write multiple messages/events to the given channel.
# * @param[in]    handle    Handle to the channel to write to.
# * @param[in]    data      A caller-allocated buffer to hold the messages/events array input, see BM_DataTypeDef for details.
# * @param[in]    timeout   Timeout (in milliseconds) before the message is transmitted successfully to the bus.
# *                         Set any negative number (i.e. -1) to wait infinitely.
# *                         Set 0 if you would like to transmit asynchronously: put to BMAPI internal buffer and return immediately, then receive TXCMPLT event over BM_Read() later.
# * @param[out]   timestamp The device local high precision timestamp array in microseconds, when the message is physically transmitted on the CAN bus, could be NULL if not required.
# * @param[inout] nmessages Number of written messages, user shall initialize this param with the size (in messages) of the data buffer.
# * @return     Operation status, see BM_StatusTypeDef for details.
# * @note       This function is allowed to be called from multiple threads since BMAPI1.3.
# */
#BMAPI BM_StatusTypeDef BM_WriteMultiple(BM_ChannelHandle handle, const BM_DataTypeDef data[], uint32_t* nmessages, int timeout, uint32_t timestamp[]);
BM_WriteMultiple = bmapi_dll.BM_WriteMultiple
BM_WriteMultiple.argtypes = [BM_ChannelHandle, ctypes.POINTER(BM_DataTypeDef), ctypes.POINTER(ctypes.c_uint32), ctypes.c_int, ctypes.POINTER(ctypes.c_uint32)]
BM_WriteMultiple.restype = BM_StatusTypeDef
BM_WriteMultiple.errcheck = check_status

#/**
# * @brief        Write data block using ISOTP protocol.
# *               This API enables rapid transmission using ISOTP without app intervention, a simple example would be writing VIN using UDS:
# *               uint8_t request[] = { 0x2E, 0xF1, 0x80, ... ... };
# *               BM_WriteIsotp(channel, request, sizeof(request), config);
# * @param[in]    handle    Handle to the channel to write to.
# * @param[in]    data      A caller-allocated buffer to hold the data block input.
# * @param[in]    nbytes    Length of the data block, in bytes.
# * @param[in]    timeout   Timeout (in milliseconds) before any message segment is transmitted successfully to the bus.
# *                         Note this is only for bus level timeout waiting for CAN ACK, for setting ISOTP protocol timeouts, see BM_IsotpConfigTypeDef for details.
# * @param[in]    config    ISOTP configuration used by current transfer.
# * @return     Operation status, see BM_StatusTypeDef for details.
# * @note       This function is allowed to be called from multiple threads since BMAPI1.5.
# */
#BMAPI BM_StatusTypeDef BM_WriteIsotp(BM_ChannelHandle handle, const void* data, uint32_t nbytes, int timeout, BM_IsotpConfigTypeDef* config);
BM_WriteIsotp = bmapi_dll.BM_WriteIsotp
BM_WriteIsotp.argtypes = [BM_ChannelHandle, ctypes.c_char_p, ctypes.c_uint32, ctypes.c_int, ctypes.POINTER(BM_IsotpConfigTypeDef)]
BM_WriteIsotp.restype = BM_StatusTypeDef
BM_WriteIsotp.errcheck = check_status

#/**
# * @brief      Write CAN message to the given channel.
# * @param[in]  handle     Handle to the channel to write to.
# * @param[in]  msg        A caller-allocated buffer to hold the CAN message output, see BM_CanMessageTypeDef for details.
# * @param[in]  _channel   The target channel ID to which the message is transmitted, starting from zero. This parameter is reserved for future, always 0 now.
# * @param[in]  timeout   Timeout (in milliseconds) before the message is transmitted successfully to the bus.
# *                       Set any negative number (i.e. -1) to wait infinitely.
# *                       Set 0 if you would like to transmit asynchronously: put to BMAPI internal buffer and return immediately, then receive TXCMPLT event over BM_Read() later.
# * @param[in]  timestamp The device local high precision timestamp in microseconds, when the message is physically transmitted on the CAN bus, could be NULL if not required.
# * @note       Note this function is a simple wrapper of BM_Write(), see BM_Write() for details.
# */
#BMAPI BM_StatusTypeDef BM_WriteCanMessage(BM_ChannelHandle handle, BM_CanMessageTypeDef* msg, uint32_t _channel, int timeout, uint32_t* timestamp);
BM_WriteCanMessage = bmapi_dll.BM_WriteCanMessage
BM_WriteCanMessage.argtypes = [BM_ChannelHandle, ctypes.POINTER(BM_CanMessageTypeDef), ctypes.c_uint32, ctypes.c_int, ctypes.POINTER(ctypes.c_uint32)]
BM_WriteCanMessage.restype = BM_StatusTypeDef
BM_WriteCanMessage.errcheck = check_status

#/**
# * @brief        Write multiple CAN messages to the given channel.
# * @param[in]    handle    Handle to the channel to write to.
# * @param[in]    msg       A caller-allocated buffer to hold the CAN message array input, see BM_CanMessageTypeDef for details.
# * @param[inout] nmessages Number of written messages, user shall initialize this param with the size (in messages) of the data buffer.
# * @param[in]    _channel  The target channel ID to which the message is transmitted, starting from zero. This parameter is reserved for future, always 0 now, or simply pass NULL into the API.
# * @param[in]    timeout   Timeout (in milliseconds) before the message is transmitted successfully to the bus.
# *                         Set any negative number (i.e. -1) to wait infinitely.
# *                         Set 0 if you would like to transmit asynchronously: put to BMAPI internal buffer and return immediately, then receive TXCMPLT event over BM_Read() later.
# * @param[out]   timestamp The device local high precision timestamp array in microseconds, when the message is physically transmitted on the CAN bus, could be NULL if not required.
# * @return     Operation status, see BM_StatusTypeDef for details.
# * @note       This function is allowed to be called from multiple threads since BMAPI1.3.
# */
#BMAPI BM_StatusTypeDef BM_WriteMultipleCanMessage(BM_ChannelHandle handle, const BM_CanMessageTypeDef msg[], uint32_t* nmessages, uint32_t _channel[], int timeout, uint32_t timestamp[]);
BM_WriteMultipleCanMessage = bmapi_dll.BM_WriteMultipleCanMessage
BM_WriteMultipleCanMessage.argtypes = [BM_ChannelHandle, ctypes.POINTER(BM_CanMessageTypeDef), ctypes.POINTER(ctypes.c_uint32), ctypes.POINTER(ctypes.c_uint32), ctypes.c_int, ctypes.POINTER(ctypes.c_uint32)]
BM_WriteMultipleCanMessage.restype = BM_StatusTypeDef
BM_WriteMultipleCanMessage.errcheck = check_status

#/**
# * @brief Control the given channel, this is an advanced interface and is typically used internally by BMAPI.
# * @param[in]    handle   Handle to the channel to control.
# * @param[in]    command  Control command.
# * @param[in]    value    Control value.
# * @param[in]    index    Control index.
# * @param[inout] data     Control data, could be NULL.
# * @param[inout] nbytes   Length in bytes of the control data, could be zero.
# */
#BMAPI BM_StatusTypeDef BM_Control(BM_ChannelHandle handle, uint8_t command, uint16_t value, uint16_t index, void* data, int nbytes);
BM_Control = bmapi_dll.BM_Control
BM_Control.argtypes = [BM_ChannelHandle, ctypes.c_uint8, ctypes.c_uint16, ctypes.c_uint16, ctypes.c_void_p, ctypes.c_int]
BM_Control.restype = BM_StatusTypeDef
BM_Control.errcheck = check_status

#/**
# * @brief      Get current CAN status of the given channel.
# * @param[in]  handle      Handle to the channel to operate on.
# * @param[out] statusinfo  Detailed information of current CAN status, see BM_CanStatusInfoTypedef for details.
# * @return     Current status code, see BM_StatusTypeDef for details.
# */
#BMAPI BM_StatusTypeDef BM_GetStatus(BM_ChannelHandle handle, BM_CanStatusInfoTypedef* statusinfo);
BM_GetStatus = bmapi_dll.BM_GetStatus
BM_GetStatus.argtypes = [BM_ChannelHandle, ctypes.POINTER(BM_CanStatusInfoTypedef)]
BM_GetStatus.restype = BM_StatusTypeDef

#/**
# * @brief      Get current local high precision device timestamp, in microseconds.
# * @param[in]  handle     Handle to the channel to operate on.
# * @param[out] timestamp  Timestamp value.
# * @return     Operation status, see BM_StatusTypeDef for details.
# */
#BMAPI BM_StatusTypeDef BM_GetTimestamp(BM_ChannelHandle handle, uint32_t* timestamp);
BM_GetTimestamp = bmapi_dll.BM_GetTimestamp
BM_GetTimestamp.argtypes = [BM_ChannelHandle, ctypes.POINTER(ctypes.c_uint32)]
BM_GetTimestamp.restype = BM_StatusTypeDef
BM_GetTimestamp.errcheck = check_status

#/**
# * @brief      Set CAN mode option of the given channel.
# * @param[in]  handle  Handle to the channel to operate on.
# * @param[in]  mode    Expected CAN mode, see BM_CanModeTypeDef for details.
# * @return     Operation status, see BM_StatusTypeDef for details.
# */
#BMAPI BM_StatusTypeDef BM_SetCanMode(BM_ChannelHandle handle, BM_CanModeTypeDef mode);
BM_SetCanMode = bmapi_dll.BM_SetCanMode
BM_SetCanMode.argtypes = [BM_ChannelHandle, BM_CanModeTypeDef]
BM_SetCanMode.restype = BM_StatusTypeDef
BM_SetCanMode.errcheck = check_status

#/**
# * @brief      Set terminal resistor option of the given channel.
# * @param[in]  handle  Handle to the channel to operate on.
# * @param[in]  tres    Expected terminal resistor, see BM_TerminalResistorTypeDef for details.
# * @return     Operation status, see BM_StatusTypeDef for details.
# */
#BMAPI BM_StatusTypeDef BM_SetTerminalRegister(BM_ChannelHandle handle, BM_TerminalResistorTypeDef  tres);
BM_SetTerminalRegister = bmapi_dll.BM_SetTerminalRegister
BM_SetTerminalRegister.argtypes = [BM_ChannelHandle, BM_TerminalResistorTypeDef]
BM_SetTerminalRegister.restype = BM_StatusTypeDef
BM_SetTerminalRegister.errcheck = check_status

#/**
# * @brief      Set bitrate option of the given channel.
# * @param[in]  handle  Handle to the channel to operate on.
# * @param[in]  bitrate Expected bitrate, see BM_BitrateTypeDef for details.
# * @return     Operation status, see BM_StatusTypeDef for details.
# */
#BMAPI BM_StatusTypeDef BM_SetBitrate(BM_ChannelHandle handle, const BM_BitrateTypeDef* bitrate);
BM_SetBitrate = bmapi_dll.BM_SetBitrate
BM_SetBitrate.argtypes = [BM_ChannelHandle, ctypes.POINTER(BM_BitrateTypeDef)]
BM_SetBitrate.restype = BM_StatusTypeDef
BM_SetBitrate.errcheck = check_status

#/**
# * @brief      Set TX tasks option of the given channel.
# * @param[in]  handle    Handle to the channel to operate on.
# * @param[in]  txtasks   An array of TX task information, see BM_TxTaskTypeDef for details.
# * @param[in]  ntxtasks  Number of valid TX tasks in the txtasks array.
# * @return     Operation status, see BM_StatusTypeDef for details.
# */
#BMAPI BM_StatusTypeDef BM_SetTxTasks(BM_ChannelHandle handle, BM_TxTaskTypeDef* txtasks, int ntxtasks);
BM_SetTxTasks = bmapi_dll.BM_SetTxTasks
BM_SetTxTasks.argtypes = [BM_ChannelHandle, ctypes.POINTER(BM_TxTaskTypeDef), ctypes.c_int]
BM_SetTxTasks.restype = BM_StatusTypeDef
BM_SetTxTasks.errcheck = check_status

#/**
# * @brief      Set RX filters option of the given channel.
# * @param[in]  handle      Handle to the channel to operate on.
# * @param[in]  rxfilters   An array of RX filter information, see BM_RxFilterTypeDef for details.
# * @param[in]  nrxfilters  Number of valid RX filters in the txtasks rxfilters.
# * @return     Operation status, see BM_StatusTypeDef for details.
# */
#BMAPI BM_StatusTypeDef BM_SetRxFilters(BM_ChannelHandle handle, BM_RxFilterTypeDef* rxfilters, int nrxfilters);
BM_SetRxFilters = bmapi_dll.BM_SetRxFilters
BM_SetRxFilters.argtypes = [BM_ChannelHandle, ctypes.POINTER(BM_RxFilterListTypeDef), ctypes.c_int]
BM_SetRxFilters.restype = BM_StatusTypeDef
BM_SetRxFilters.errcheck = check_status

#/**
# * @brief Get the platform/OS independent notification handle for the given channel, so that the application could wait for notifications later.
# * @param[in]  handle        Handle to the channel that owns the notification handle.
# * @param[out] notification  The platform/OS independent notification handle.
# * @return     Operation status, see BM_StatusTypeDef for details.
# * @note       By using notification handles in a background thread, it is easy to implement an asynchronous message receiver as below:
# * @code
# *             channel = BM_OpenCan(...);
# *             BM_GetNotification(channel, notification);
# *             while (!exit) {
# *               BM_WaitForNotifications(&notification, 1, -1); // Wait infinitely for new message notification.
# *               BM_ReadCanMessage(...);
# *             }
# * @endcode
# */
#BMAPI BM_StatusTypeDef BM_GetNotification(BM_ChannelHandle handle, BM_NotificationHandle* notification);
BM_GetNotification = bmapi_dll.BM_GetNotification
BM_GetNotification.argtypes = [BM_ChannelHandle, ctypes.POINTER(BM_NotificationHandle)]
BM_GetNotification.restype = BM_StatusTypeDef
BM_GetNotification.errcheck = check_status

#/**
# * @brief     A platform/OS independent implementation to wait for single/multiple notification handles.
# * @param[in] handles     An array of channel notification handles.
# * @param[in] nhandles    Number of valid notification handles.
# * @param[in] ntimeoutms  This function will block current thread for ntimeoutms milliseconds if no notification is received.
# *                        Note this function will return immediately once a new notification is received, the ntimeoutms param is ignored in this normal case.
# * @return    This function returns the index in handles array of the channel from which a new notification is posted.
# */
#BMAPI int BM_WaitForNotifications(BM_NotificationHandle handles[], int nhandles, int ntimeoutms);
BM_WaitForNotifications = bmapi_dll.BM_WaitForNotifications
BM_WaitForNotifications.argtypes = [ctypes.POINTER(BM_NotificationHandle), ctypes.c_int, ctypes.c_int]
BM_WaitForNotifications.restype = ctypes.c_int

#/**
# * @brief      Translate error code to string, this is a helper function to ease application programming.
# * @param[in]  errorcode  The errorcode to be translated.
# * @param[out] buffer     A caller-allocated string buffer to hold the translated string.
# * @param[in]  nbytes     Number in bytes of the string buffer.
# * @param[in]  language   Reserved, only English is supported currently.
# */
#BMAPI void BM_GetErrorText(BM_StatusTypeDef errorcode, char* buffer, int nbytes, uint16_t language);
BM_GetErrorText = bmapi_dll.BM_GetErrorText
BM_GetErrorText.argtypes = [BM_StatusTypeDef, ctypes.c_char_p, ctypes.c_int, ctypes.c_uint16]
BM_GetErrorText.restype = None

#/**
# * @brief      Translate data (i.e. CAN message) to string, this is a helper function to ease application programming.
# * @param[in]  data       The message data to be translated.
# * @param[out] buffer     A caller-allocated string buffer to hold the translated string.
# * @param[in]  nbytes     Number in bytes of the string buffer.
# * @param[in]  language   Reserved, only English is supported currently.
# */
#BMAPI void BM_GetDataText(BM_DataTypeDef* data, char* buffer, int nbytes, uint16_t language);
BM_GetDataText = bmapi_dll.BM_GetDataText
BM_GetDataText.argtypes = [ctypes.POINTER(BM_DataTypeDef), ctypes.c_char_p, ctypes.c_int, ctypes.c_uint16]
BM_GetDataText.restype = None

# /**
#  * @brief      Get library log level.
#  * @return     Current log level, all messages equal to or less than this level are currently printed on debug console.
#  */
# BMAPI BM_LogLevelTypeDef BM_GetLogLevel(void);
BM_GetLogLevel = bmapi_dll.BM_GetLogLevel
BM_GetLogLevel.argtypes = []
BM_GetLogLevel.restype = ctypes.c_int

# /**
#  * @brief      Set library log level.
#  * @param[in]  level       Target log level, all messages equal to or less than this level will be printed on debug console.
#  */
# BMAPI void BM_SetLogLevel(BM_LogLevelTypeDef level);
BM_SetLogLevel = bmapi_dll.BM_SetLogLevel
BM_SetLogLevel.argtypes = [ctypes.c_int]
BM_SetLogLevel.restype = None

# /**
#  * @brief      Get library (*.dll|*.so) BMAPI version.
#  * @return     32-bit version code:
#  *             bit31-28 = major
#  *             bit27-24 = minor
#  *             bit23-16 = revision
#  *             bit15-00 = build
#  * @note       This API is used to get the library version, 
#  *             in case that *.h is mismatch with *.dll|*.so, use the macro BM_API_VERSION defined in bmapi.h to check consistency.
#  */
# BMAPI uint32_t BM_GetVersion(void);
BM_GetVersion = bmapi_dll.BM_GetVersion
BM_GetVersion.argtypes = []
BM_GetVersion.restype = ctypes.c_uint32

# END OF FILE
