#!/usr/bin/env python3
import math
from pathlib import Path
import socket
import struct
import sys
from types import SimpleNamespace as NS
import unittest
from unittest.mock import patch
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from tools import runtime_timing_probe as probe
from tools.mapper_probe_process import ProbeProcess

class InputGapTest(unittest.TestCase):
    def test_source_interval_and_thread_gap_are_separate(self):
        previous=(1000000000,20000000,2,1800000000000)
        msg=NS(timestamp_ms=1800008)
        with patch.object(probe.time,'monotonic_ns',return_value=1040000000),patch.object(probe.time,'thread_time_ns',return_value=21000000):
            row=probe.input_entry(msg,previous)
        self.assertEqual(row[3],40);self.assertEqual(row[6],8);self.assertEqual(row[8],1)
        self.assertEqual(row[4],2)
    def test_header_and_first_frame(self):
        msg=NS(header=NS(stamp=NS(to_nsec=lambda:12345)),timestamp_ms=999)
        row=probe.input_entry(msg,None)
        self.assertEqual(row[4:6],[1,12345]);self.assertTrue(math.isnan(row[3]));self.assertTrue(math.isnan(row[6]))
    def test_ipc_keeps_public_v1_boundary(self):
        obj=ProbeProcess.__new__(ProbeProcess);obj.dropped=obj.errors=0
        obj.socket,peer=socket.socketpair(socket.AF_UNIX,socket.SOCK_SEQPACKET)
        try:
            frame=dict(tid=3,serial=4,session=1,generation=2,sequence=5,source_ms=6,overflow=0,
                       points=[(0,10,5,0,0,0,0),(99,20,9,0,0,0,0)],entry=list(range(9)))
            obj.submit(frame);payload=peer.recv(8192);data=struct.unpack('<%dd'%(len(payload)//8),payload)
            boundary=12+7*int(data[11]);self.assertEqual(data[0],2)
            self.assertEqual(len(data)-boundary,14);self.assertEqual(data[boundary],1)
            self.assertEqual(data[12:boundary],tuple(v for r in frame['points'] for v in r))
        finally:obj.socket.close();peer.close()
    def test_callback_end_includes_handoff_and_disabled_is_transparent(self):
        frames=[]
        recorder=NS(enabled=True,serial=0,dropped=0,errors=0,external=None,queue=NS(put_nowait=frames.append))
        obj=NS(_runtime_probe=recorder)
        @probe.callback
        def callback(self,msg):return msg.timestamp_ms
        self.assertEqual(callback(obj,NS(timestamp_ms=1000)),1000)
        callback(obj,NS(timestamp_ms=1008))
        self.assertTrue(math.isnan(frames[0]['entry'][3]))
        self.assertGreaterEqual(frames[1]['entry'][3],0)
        self.assertEqual(frames[1]['entry'][6],8)
        recorder.enabled=False;callback(obj,NS(timestamp_ms=1016));self.assertEqual(len(frames),2)

if __name__=='__main__':unittest.main()
