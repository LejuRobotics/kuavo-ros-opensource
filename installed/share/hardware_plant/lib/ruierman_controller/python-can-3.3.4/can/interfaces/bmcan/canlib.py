# coding: utf-8

"""
Ctypes wrapper module for BUSMUST CAN Interface on win32/win64 systems.

Authors: busmust <busmust@126.com>, BUSMUST Co.,Ltd.
"""

# Import Standard Python Modules
# ==============================
import ctypes
import logging
import sys
import time

try:
    # Try builtin Python 3 Windows API
    from _winapi import WaitForSingleObject, INFINITE
    HAS_EVENTS = True
except ImportError:
    try:
        # Try pywin32 package
        from win32event import WaitForSingleObject, INFINITE
        HAS_EVENTS = True
    except ImportError:
        # Use polling instead
        HAS_EVENTS = False

# Import Modules
# ==============
from can import BusABC, Message, CanError
from can.bus import BusState
from can.util import len2dlc, dlc2len
from .exceptions import BmError

# Define Module Logger
# ====================
LOG = logging.getLogger(__name__)
import os
os.environ['LD_LIBRARY_PATH'] = '/home/lemon/workspace/kuavo/lib/ruierman_controller/python-can-3.3.4/can/interfaces/bmcan'

# Import safely Vector API module for Travis tests
bmapi = None
try:
    from . import bmapi
except Exception as exc:
    LOG.error('Could not import bmapi, please check your BMAPI path: %s', exc)
    raise exc


class BmCanBus(BusABC):
    """The CAN Bus implemented for the BUSMUST USB-CAN interface."""

    __initialized = False

    @classmethod
    def __init_class__(cls):
        if bmapi is None:
            raise ImportError("The BMAPI has not been loaded")
        if not BmCanBus.__initialized:
            bmapi.BM_Init()
            bmapi.BM_SetLogLevel(bmapi.BM_LOG_ERR)
            BmCanBus.__initialized = True

    def __init__(self, channel, 
                 fd=True, receive_own_messages=False, listen_only=False,
                 bitrate=500000, data_bitrate=500000, 
                 samplepos=75, data_samplepos = 75,
                 tres=False,
                 can_filters=None,
                 **kwargs):
        """
        :param int channel:
            The channel index to create this bus with, which is the index to all available ports when enumerating Busmust devices.
            Can also be a string of the channel's full name. i.e. "BM-CANFD-X1-PRO(1234) CH1"
        :param bool fd:
            If CAN-FD frames should be supported.
        :param bool receive_own_messages:
            If Loopback mode should be supported.
        :param bool listen_only:
            If Listen only mode should be supported, this is the same as setting 'state' property to INACTIVE.
        :param int bitrate:
            Bitrate in bits/s.
        :param int data_bitrate:
            Which bitrate to use for data phase in CAN FD.
            Defaults to arbitration bitrate.
        :param int samplepos:
            Sample position (%).
        :param int data_samplepos:
            Data phase sample pos (%) in CAN FD.
        :param bool tres:
            If 120Ohm CAN terminal register should be enabled.
        """
        BmCanBus.__init_class__()
        self._bmapi = bmapi # Enable external access

        infolist = bmapi.BM_ChannelInfoListTypeDef()
        numOfInfo = ctypes.c_int(len(infolist.entries))
        bmapi.BM_Enumerate(ctypes.byref(infolist), ctypes.byref(numOfInfo))
        if isinstance(channel, int):
            if channel < numOfInfo.value:
                self._channelinfo = infolist.entries[channel]
            else:
                raise BmError(bmapi.BM_ERROR_NODRIVER, "Channel %d is not connected or is in use by another app." % channel, "BmCanBus.__init__")
        elif isinstance(channel, str):
            for info in infolist.entries:
                if info.name.decode() == channel:
                    self._channelinfo = info
                    break
            else:
                raise BmError(bmapi.BM_ERROR_NODRIVER, "Channel %s is not connected or is in use by another app." % channel, "BmCanBus.__init__")

        self._mode = bmapi.BM_CAN_NORMAL_MODE
        if not fd:
            self._mode = bmapi.BM_CAN_CLASSIC_MODE
        elif receive_own_messages:
            self._mode = bmapi.BM_CAN_EXTERNAL_LOOPBACK_MODE
        elif listen_only:
            self._mode = bmapi.BM_CAN_LISTEN_ONLY_MODE

        self._tres = bmapi.BM_TRESISTOR_DISABLED
        if tres:
            self._tres = bmapi.BM_TRESISTOR_120

        self._bitrate = bmapi.BM_BitrateTypeDef()
        self._bitrate.nbitrate = int(bitrate / 1000)
        self._bitrate.dbitrate = int(data_bitrate / 1000)
        self._bitrate.nsamplepos = samplepos
        self._bitrate.dsamplepos = data_samplepos

        self._handle = bmapi.BM_ChannelHandle()
        bmapi.BM_OpenEx(
            ctypes.byref(self._handle), 
            ctypes.byref(self._channelinfo), 
            self._mode, 
            self._tres, 
            ctypes.byref(self._bitrate), 
            ctypes.cast(ctypes.c_void_p(), ctypes.POINTER(bmapi.BM_RxFilterListTypeDef)), 0
        )
        self.channel_info = self._channelinfo.name.decode()

        startTimestamp = ctypes.c_uint32()
        bmapi.BM_GetTimestamp(self._handle, ctypes.byref(startTimestamp))
        self._time_offset = time.time() - startTimestamp.value * 1e-9

        self._notification = bmapi.BM_NotificationHandle()
        bmapi.BM_GetNotification(self._handle, ctypes.byref(self._notification))

        super(BmCanBus, self).__init__(channel=channel, can_filters=can_filters, **kwargs)
        time.sleep(0.05)

        self._state = BusState.ACTIVE

        self._isotp_config = bmapi.BM_IsotpConfigTypeDef()

    def _apply_filters(self, filters):
        if filters:
            # Only up to one filter per ID type allowed
            if len(filters) == 1 or (len(filters) == 2 and filters[0].get("extended") != filters[1].get("extended")):
                bmfilters = bmapi.BM_RxFilterListTypeDef()
                try:
                    for i in range(len(filters)):
                        can_filter = filters[i]
                        bmfilters.entries[i].type = bmapi.BM_RXFILTER_BASIC
                        bmfilters.entries[i].flags_mask = bmapi.BM_MESSAGE_FLAGS_IDE
                        if can_filter.get("extended"):
                            bmfilters.entries[i].flags_value = bmapi.BM_MESSAGE_FLAGS_IDE
                            bmfilters.entries[i].id_mask = (can_filter["can_mask"] >> 18) | ((can_filter["can_mask"] & 0x3FFFF) << 11)
                            bmfilters.entries[i].id_value = (can_filter["can_id"] >> 18) | ((can_filter["can_id"] & 0x3FFFF) << 11)
                        else:
                            bmfilters.entries[i].flags_value = 0
                            bmfilters.entries[i].id_mask = can_filter["can_mask"]
                            bmfilters.entries[i].id_value = can_filter["can_id"]
                    bmapi.BM_SetRxFilters(self._handle, ctypes.byref(bmfilters), len(bmfilters.entries))
                    time.sleep(0.05)
                except BmError as exc:
                    LOG.warning("Could not set filters: %s", exc)
                    # go to fallback
                else:
                    self._is_filtered = True
                    return
            else:
                LOG.warning("Only up to one filter per extended or standard ID allowed")
                # go to fallback

        # fallback: reset filters
        self._is_filtered = False
        try:
            bmfilters = bmapi.BM_RxFilterListTypeDef() # Default as invalid => allow all messages to pass
            bmapi.BM_SetRxFilters(self._handle, ctypes.byref(bmfilters), 2)
            time.sleep(0.05)
        except BmError as exc:
            LOG.warning("Could not reset filters: %s", exc)

    def _recv_internal(self, timeout):
        end_time = time.time() + timeout if timeout is not None else None

        bmmsg = bmapi.BM_CanMessageTypeDef()
        channel = ctypes.c_uint32()
        timestamp = ctypes.c_uint32()

        while True:
            try:
                bmapi.BM_ReadCanMessage(self._handle, ctypes.byref(bmmsg), ctypes.byref(channel), ctypes.byref(timestamp))
            except BmError as exc:
                if exc.error_code != bmapi.BM_ERROR_QRCVEMPTY:
                    raise
            else:
                msg_id = bmmsg.mid.getExtendedId() if bmmsg.ctrl.rx.IDE else bmmsg.mid.getStandardId()
                dlc = dlc2len(bmmsg.ctrl.rx.DLC)
                msg = Message(
                    timestamp=timestamp.value * 1e-6 + self._time_offset,
                    arbitration_id=msg_id & 0x1FFFFFFF,
                    is_extended_id=bool(bmmsg.ctrl.rx.IDE),
                    is_remote_frame=bool(bmmsg.ctrl.rx.RTR),
                    is_error_frame=bool(False),
                    is_fd=bool(bmmsg.ctrl.rx.FDF),
                    error_state_indicator=bool(bmmsg.ctrl.rx.ESI),
                    bitrate_switch=bool(bmmsg.ctrl.rx.BRS),
                    dlc=dlc,
                    data=bmmsg.payload[:dlc],
                    channel=channel.value)
                return msg, self._is_filtered


            if end_time is not None and time.time() > end_time:
                return None, self._is_filtered

            # Wait for receive event to occur
            if timeout is None:
                time_left_ms = INFINITE
            else:
                time_left = end_time - time.time()
                time_left_ms = max(0, int(time_left * 1000))
            bmapi.BM_WaitForNotifications(ctypes.byref(self._notification), 1, time_left_ms)

    def send(self, msg, timeout=None):
        timeoutms = int(timeout*1000) if timeout is not None else -1
        timestamp = ctypes.c_uint32()
        if type(msg) is bmapi.BM_CanMessageTypeDef:
            bmmsg = msg
        else:
            bmmsg = bmapi.BM_CanMessageTypeDef()
            if msg.is_extended_id:
                bmmsg.mid.SID = msg.arbitration_id >> 18
                bmmsg.mid.EID = (msg.arbitration_id & 0x3FFFF)
            else:
                bmmsg.mid.SID = msg.arbitration_id
                bmmsg.mid.EID = 0
            bmmsg.ctrl.tx.IDE = 1 if msg.is_extended_id else 0
            bmmsg.ctrl.tx.FDF = 1 if msg.is_fd else 0
            bmmsg.ctrl.tx.BRS = 1 if msg.bitrate_switch else 0
            bmmsg.ctrl.tx.RTR = 1 if msg.is_remote_frame else 0
            bmmsg.ctrl.tx.ESI = 1 if msg.error_state_indicator else 0
            bmmsg.ctrl.tx.DLC = len2dlc(msg.dlc)
            bmmsg.payload[:len(msg.data)] = msg.data
        bmapi.BM_WriteCanMessage(self._handle, ctypes.byref(bmmsg), 0, timeoutms, ctypes.byref(timestamp))

    def send_multiple(self, msgs, timeout=None):
        assert(len(msgs) >= 0)
        ntotalmsgs = len(msgs)
        nsentmsgs = 0
        nchunkmsgs = 10000
        bmmsgs = (bmapi.BM_CanMessageTypeDef * nchunkmsgs)()
        nullptr = ctypes.cast(0, ctypes.POINTER(ctypes.c_uint32))
        dataptr = ctypes.cast(bmmsgs, ctypes.POINTER(bmapi.BM_CanMessageTypeDef))
        timeoutms = int(timeout*1000) if timeout is not None else -1
        while nsentmsgs < ntotalmsgs:
            nmsgs = min(nchunkmsgs, ntotalmsgs - nsentmsgs)
            if type(msgs[0]) is bmapi.BM_CanMessageTypeDef:
                bmmsgs[0:nmsgs] = msgs[nsentmsgs:nsentmsgs+nmsgs]
            else:
                for i in range(nmsgs):
                    msg = msgs[i + nsentmsgs]
                    bmmsg = bmmsgs[i]
                    if msg.is_extended_id:
                        bmmsg.mid.SID = msg.arbitration_id >> 18
                        bmmsg.mid.EID = (msg.arbitration_id & 0x3FFFF)
                    else:
                        bmmsg.mid.SID = msg.arbitration_id
                        bmmsg.mid.EID = 0
                    bmmsg.ctrl.tx.IDE = 1 if msg.is_extended_id else 0
                    bmmsg.ctrl.tx.FDF = 1 if msg.is_fd else 0
                    bmmsg.ctrl.tx.BRS = 1 if msg.bitrate_switch else 0
                    bmmsg.ctrl.tx.RTR = 1 if msg.is_remote_frame else 0
                    bmmsg.ctrl.tx.ESI = 1 if msg.error_state_indicator else 0
                    bmmsg.ctrl.tx.DLC = len2dlc(msg.dlc)
                    bmmsg.payload[:len(msg.data)] = msg.data
            nbmmsgs = ctypes.c_uint32(nmsgs)
            bmapi.BM_WriteMultipleCanMessage(self._handle, dataptr, ctypes.byref(nbmmsgs), nullptr, timeoutms, nullptr)
            nsentmsgs += nmsgs

    def shutdown(self):
        bmapi.BM_Close(self._handle)
        self._handle = bmapi.BM_ChannelHandle()
        
    def reset(self):
        bmapi.BM_Reset(self._handle)

    @property
    def state(self):
        return self._state

    @state.setter
    def state(self, new_state):
        mode_changed = False
        self._state = new_state
        if new_state is BusState.ACTIVE:
            if self._mode == bmapi.BM_CAN_OFF_MODE or self._mode == bmapi.BM_CAN_LISTEN_ONLY_MODE:
                self._mode = bmapi.BM_CAN_NORMAL_MODE
                mode_changed = True
            else:
                pass # Do not change (i.e. loopback)
        elif new_state is BusState.PASSIVE:
            # When this mode is set, the CAN controller does not take part on active events (eg. transmit CAN messages)
            # but stays in a passive mode (CAN monitor), in which it can analyse the traffic on the CAN bus used by a BMCAN channel.
            if self._mode != bmapi.BM_CAN_LISTEN_ONLY_MODE:
                self._mode = bmapi.BM_CAN_LISTEN_ONLY_MODE
                mode_changed = True
        if mode_changed:
            bmapi.BM_SetCanMode(self._handle, self._mode)

    @classmethod
    def enumerate(cls):
        BmCanBus.__init_class__()
        infolist = bmapi.BM_ChannelInfoListTypeDef()
        numOfInfo = ctypes.c_int(len(infolist.entries))
        bmapi.BM_Enumerate(ctypes.byref(infolist), ctypes.byref(numOfInfo))
        channellist = []
        for i in range(numOfInfo.value):
            channellist.append({
                'index': i,
                'name': infolist.entries[i].name.decode(),
                # Add other exports here
            })
        return channellist

    def send_isotp(self, payload, timeout=-1):
        timeout_ms = int(timeout * 1000.0) if timeout >= 0 else -1
        bmapi.BM_WriteIsotp(self._handle, ctypes.c_char_p(payload), len(payload), timeout_ms, ctypes.byref(self._isotp_config))

    def receive_isotp(self, timeout=-1, max_len=4095):
        timeout_ms = int(timeout * 1000.0) if timeout >= 0 else -1
        buf = ctypes.create_string_buffer(max_len)
        received_len = ctypes.c_uint32(len(buf))
        bmapi.BM_ReadIsotp(self._handle, buf, ctypes.byref(received_len), timeout_ms, ctypes.byref(self._isotp_config))
        return bytes(buf[:received_len.value])

    def config_isotp(self, tester_msg_id, ecu_msg_id, mode=bmapi.BM_ISOTP_NORMAL_TESTER, **kwargs):
        self._isotp_config.mode = mode
        enable_fdf = kwargs.get('fd', False) or kwargs.get('fdf', False)
        enable_brs = kwargs.get('brs', enable_fdf)
        enable_ide = kwargs.get('ide', False) or tester_msg_id > 0x7FF or ecu_msg_id > 0x7FF
        dlc = kwargs.get('dlc', 0xF if enable_fdf else 0x8)
        testerMsg = self._isotp_config.testerDataTemplate.getCanMessage()
        testerMsg.ctrl.tx.FDF = enable_fdf
        testerMsg.ctrl.tx.BRS = enable_brs
        testerMsg.ctrl.tx.IDE = enable_ide
        testerMsg.ctrl.tx.DLC = dlc
        testerMsg.setMessageId(tester_msg_id)
        self._isotp_config.testerDataTemplate.setCanMessage(testerMsg)
        ecuMsg = self._isotp_config.ecuDataTemplate.getCanMessage()
        ecuMsg.ctrl.tx.FDF = enable_fdf
        ecuMsg.ctrl.tx.BRS = enable_brs
        ecuMsg.ctrl.tx.IDE = enable_ide
        ecuMsg.ctrl.tx.DLC = dlc
        ecuMsg.setMessageId(ecu_msg_id)
        self._isotp_config.ecuDataTemplate.setCanMessage(ecuMsg)
        padding = kwargs.get('padding', None)
        if padding is not None:
            self._isotp_config.paddingEnabled = 1
            self._isotp_config.paddingValue = ctypes.c_uint8(padding)
        else:
            self._isotp_config.paddingEnabled = 0
