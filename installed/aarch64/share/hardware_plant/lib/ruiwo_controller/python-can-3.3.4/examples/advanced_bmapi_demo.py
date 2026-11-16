#!/usr/bin/env python

from __future__ import print_function

import can
from can.bus import BusState
from can.interfaces.bmcan import BmCanBus

def send_one():
    msg = can.Message(arbitration_id=0xc0ffee,
                      data=[0, 25, 0, 1, 3, 1, 4, 1],
                      is_extended_id=True)

    try:
        bus.send(msg, timeout=None)
        print("Message sent on {}".format(bus.channel_info))
    except can.CanError:
        print("Message NOT sent")

def receive_all():
    print('Waiting for RX messages, press Ctrl+C to break.')
    bus.state = BusState.ACTIVE  # or BusState.PASSIVE

    try:
        while True:
            msg = bus.recv(1)
            if msg is not None:
                print(msg)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    #bus = can.interface.Bus(bustype='bmcan', channel=0, bitrate=500000, data_bitrate=2000000, tres=True)
    # Custruct a BmCanBus object directly using its constructor, please add the following line before calling the constructor:
    # from can.interfaces.bmcan import BmCanBus
    bus = BmCanBus(channel=0, bitrate=500000, data_bitrate=2000000, tres=True)
    # Call BMAPI advanced functions directly if necessary
    bus._bmapi.BM_SetTerminalRegister(bus._handle, bus._bmapi.BM_TRESISTOR_120)
    send_one()
    receive_all()
