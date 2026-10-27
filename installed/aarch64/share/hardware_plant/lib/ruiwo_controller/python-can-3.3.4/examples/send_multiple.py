#!/usr/bin/env python
# coding: utf-8

"""
This example shows how sending multiple messages works, only 'bmcan' is supported.
"""

from __future__ import print_function
from time import time

import can

def send_multile():
    bus = can.interface.Bus(bustype='bmcan', channel=1, bitrate=1000000, data_bitrate=8000000, tres=True)
    #msgs = []
    msgs = (bus._bmapi.BM_CanMessageTypeDef * 20000)()
    for i in range(len(msgs)):
        msgs[i].mid.setExtendedId(0xc0ffee)
        msgs[i].ctrl.tx.IDE = 1
        msgs[i].ctrl.tx.FDF = 1
        msgs[i].ctrl.tx.BRS = 1
        msgs[i].ctrl.tx.DLC = 8
        msgs[i].payload[0:8] = [0, 25, 0, 1, 3, 1, 4, 1]
        # msg = can.Message(arbitration_id=0xc0ffee,
        #                 data=[0, 25, 0, 1, 3, 1, 4, 1],
        #                 is_extended_id=True)
        # msgs.append(msg)

    try:
        print('Starting ' + bus.channel_info)
        t0 = time()
        bus.send_multiple(msgs, timeout=None)
        t1 = time()
        t = t1 - t0
        fps = len(msgs) / t
        print("Sent {} messages sent on {} in {:.3f} seconds, fps={:.3f}".format(len(msgs), bus.channel_info, t, fps))
    except can.CanError:
        print("Messages NOT sent")

    bus.shutdown()

if __name__ == '__main__':
    send_multile()
