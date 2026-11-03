#!/usr/bin/env python

from __future__ import print_function

import can
from can.interfaces.bmcan import BmCanBus
from can.interfaces.bmcan import bmapi
import ctypes

if __name__ == "__main__":
    #bus = can.interface.Bus(bustype='bmcan', channel=0, bitrate=500000, data_bitrate=2000000, tres=True)
    # Custruct a BmCanBus object directly using its constructor, please add the following line before calling the constructor:
    # from can.interfaces.bmcan import BmCanBus
    bus = can.interface.Bus(bustype='bmcan', channel=0, bitrate=500000, data_bitrate=2000000, tres=True)
    bus.config_isotp(tester_msg_id=0x7df, ecu_msg_id=0x798, fd=True, dlc=8)

    request = bytes([0x2E, 0xF1, 0x80, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x30])
    request = bytes([0x10, 0x03])
    print('Write %s to channel %s' % (str(request), str(bus.channel_info)))
    bus.send_isotp(request, timeout=5.0)
    # print('Wait for response from ECU.')
    # response = bus.receive_isotp(timeout=5.0)
    # if response[0] != 0x6E or response[1] != 0xF1 or response[2] != 0x80:
    #     print('Failed to write DID (which is normal because we are not in extended session).')
    # else:
    #     print('Positive resonse received.')
