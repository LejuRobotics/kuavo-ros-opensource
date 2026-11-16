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
    bus = BmCanBus(channel=0, bitrate=500000, data_bitrate=2000000, tres=True)

    isotp = bus._bmapi.BM_IsotpConfigTypeDef()
    isotp.version = 1
    isotp.mode = bmapi.BM_ISOTP_NORMAL_TESTER
    isotp.paddingEnabled = 1
    isotp.paddingValue = 0xCC
    isotp.testerTimeout.b = 2000
    #isotp.flowcontrol.stmin = 0xF1;
    #isotp.flowcontrol.blockSize = 255;
    #isotp.callbackFunc = ?
    isotp.testerDataTemplate.setCanMessage(bmapi.BM_CanMessageTypeDef(mid=0x7DF, dlc=8, fdf=1, brs=1))
    isotp.ecuDataTemplate.setCanMessage(bmapi.BM_CanMessageTypeDef(mid=0x798, dlc=8, fdf=1, brs=1))

    request = bytes([0x2E, 0xF1, 0x80, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x30])
    print('Write %s to channel %s' % (str(request), str(bus.channel_info)))
    bus._bmapi.BM_WriteIsotp(bus._handle, request, len(request), 10000, ctypes.byref(isotp))
    print('Done.')
