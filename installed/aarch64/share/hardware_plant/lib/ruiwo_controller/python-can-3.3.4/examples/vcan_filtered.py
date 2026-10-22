#!/usr/bin/env python
# coding: utf-8

"""
This shows how message filtering works.
"""

import time

import can

if __name__ == '__main__':
    bus = can.interface.Bus(bustype='bmcan',
                            channel=0,
                            receive_own_messages=True)

    can_filters = [
        {"can_id": 1, "can_mask": 0xf, "extended": False},
        {"can_id": 0x18FFAAA0, "can_mask": 0x1FFFFFFF, "extended": True},
    ]
    bus.set_filters(can_filters)
    notifier = can.Notifier(bus, [can.Printer()])
    bus.send(can.Message(arbitration_id=1, is_extended_id=True, dlc=1, data=[1]))
    bus.send(can.Message(arbitration_id=2, is_extended_id=True, dlc=1, data=[2]))
    bus.send(can.Message(arbitration_id=1, is_extended_id=False, dlc=1, data=[3]))
    bus.send(can.Message(arbitration_id=0x18FFAAA0, is_extended_id=True, dlc=1, data=[4]))

    time.sleep(1)
