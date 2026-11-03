#!/usr/bin/env python

from __future__ import print_function

import can
import time
import datetime
import ctypes
import sys
from can.bus import BusState
from can.interfaces.bmcan.canlib import bmapi

t0 = 0
totalrxcount = 0

def handle_incoming_message(channelid, canid, payload, len, timestamp, raw_timestamp):
    print('[%d] canid=%08x, payload=[%02x, %02x, ...], len=%d, ts=%.6f (%d).' % (channelid, canid, payload[0], payload[1], len, timestamp, raw_timestamp))
    pass

def sync_timestamp(bus_handle):
    global t0
    can_ts = ctypes.c_uint32()
    all_ts = [()] * 5
    for i in range(len(all_ts)):
        bmapi.BM_GetTimestamp(bus_handle, ctypes.byref(can_ts))
        utc_ts = time.time()
        all_ts[i] = (can_ts, utc_ts, float(can_ts.value) * 1e-6 - (utc_ts - t0))
    #print([ts[2] for ts in all_ts])
    sorted_all_ts = sorted(all_ts, key=lambda x: x[2])
    median_ts = sorted_all_ts[len(all_ts) // 2] # Take the median to suppress glitch
    #print(median_ts[2])
    return median_ts[0], median_ts[1] 

def echo_all(channels):
    global t0
    global totalrxcount
    ''' Receive and echo CAN messages from multiple BUSMUST CAN channels '''
    print('Open BUSMUST CAN channels using 500kbps baudrate ...')
    channels = [ channels ] if not isinstance(channels, list) else channels
    buses = []
    t1 = t0 = time.time()
    rxchannels = set()
    rxcount = 0
    
    channelid = ctypes.c_uint32()
    timestamp = ctypes.c_uint32()
    bus_notifications = (ctypes.c_void_p * len(channels))()
    bus_messages = (bmapi.BM_CanMessageTypeDef * len(channels))()
    utc_timestamps = [0.0] * len(channels)
    can_timestamps = (ctypes.c_uint32 * len(channels))()
    for i, channel in enumerate(channels):
        bus = can.interface.Bus(bustype='bmcan', channel=int(channel), bitrate=500000, data_bitrate=2000000, tres=True)
        #bus = can.interface.Bus(bustype='pcan', channel='PCAN_USBBUS1', bitrate=250000)
        #bus = can.interface.Bus(bustype='ixxat', channel=0, bitrate=250000)
        #bus = can.interface.Bus(bustype='vector', app_name='CANalyzer', channel=0, bitrate=250000)
        bus.state = BusState.ACTIVE  # or BusState.PASSIVE
        buses.append(bus)
        bus_notifications[i] = bus._notification
        print('Opened ' + bus.channel_info + ': version=%d.%d.%d.%d' % (bus._channelinfo.version[0], bus._channelinfo.version[1], bus._channelinfo.version[2], bus._channelinfo.version[3]))

    for i, bus in enumerate(buses):
        can_timestamps[i], utc_timestamps[i] = sync_timestamp(bus._handle)

    print('Waiting for RX CAN messages ...')
    try:
        while True:
            # Use BMAPI async notification to wait until arrival of new message, and return the incoming message's channel index directly!
            incoming_channel = bmapi.BM_WaitForNotifications(bus_notifications, len(bus_notifications), 100)
            # Read all pending incoming messages from source channel index
            while incoming_channel >= 0:
                try:
                    bmapi.BM_ReadCanMessage(buses[incoming_channel]._handle, ctypes.byref(bus_messages[incoming_channel]), ctypes.byref(channelid), ctypes.byref(timestamp))
                    rxchannels.add(incoming_channel)
                    rxcount += 1
                    totalrxcount += 1
                    elapsed_us = int(timestamp.value) - int(can_timestamps[incoming_channel])
                    if elapsed_us < -2147483648:
                        print('WRAPAROUND: %d %d' % (int(timestamp.value), int(can_timestamps[incoming_channel])))
                        elapsed_us += 4294967296
                    handle_incoming_message(
                        incoming_channel,
                        bus_messages[incoming_channel].getMessageId(), 
                        bus_messages[incoming_channel].payload, 
                        bus_messages[incoming_channel].ctrl.rx.DLC, 
                        elapsed_us * 1e-6 + utc_timestamps[incoming_channel],
                        timestamp.value
                    )
                except bmapi.BmError as exc:
                    incoming_channel = -1
                    if exc.error_code != bmapi.BM_ERROR_QRCVEMPTY:
                        raise
            t2 = time.time()
            if t2 >= t1 + 0.1:
                #print('Received %d messages from channel %s @ %s %d' % (rxcount, str(list(rxchannels)), datetime.datetime.now(), can_timestamps[i]))
                t1 = t2
                rxchannels.clear()
                rxcount = 0
                # Sync CAN hardware timestamps with UTC time.
                for i, bus in enumerate(buses):
                    can_timestamps[i], utc_timestamps[i] = sync_timestamp(bus._handle)
    except KeyboardInterrupt:
        for bus in buses:
            bus.shutdown()
        print('The demo has been running for %d seconds, rx=%d, fps=%.3f.' % (t2-t0, totalrxcount, 1.0 * totalrxcount / (t2 - t0)))


if __name__ == "__main__":
    channels = sys.argv[1:] if len(sys.argv) > 1 else [0, 1, 2, 3]
    echo_all(channels)

