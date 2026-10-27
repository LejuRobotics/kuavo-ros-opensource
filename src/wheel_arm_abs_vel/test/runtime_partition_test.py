#!/usr/bin/env python3
"""Regression guards: physical-core collision and a stalled diagnostic reader."""
import copy, importlib.util, json, socket, struct, tempfile, unittest
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
def load(name,path):
    spec=importlib.util.spec_from_file_location(name,str(ROOT/path));m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m);return m
partition=load('partition','scripts/runtime_partition.py')
probe=load('probe','scripts/tools/mapper_probe_process.py')
PROFILE=json.loads((ROOT/'config/runtime_partition.json').read_text())
class Regression(unittest.TestCase):
    def test_smt_and_unavailable_cpus_rejected(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp);(root/'online').write_text('0-15');(root/'isolated').write_text('2,3,7,14,15')
            for cpu in range(16):
                p=root/('cpu%d/topology'%cpu);p.mkdir(parents=True);(p/'thread_siblings_list').write_text(str(cpu))
            partition.validate(PROFILE,root)
            (root/'cpu4/topology/thread_siblings_list').write_text('3,4')
            with self.assertRaisesRegex(ValueError,'physical core shared'):partition.validate(PROFILE,root)
            (root/'cpu4/topology/thread_siblings_list').write_text('4')
            invalid=copy.deepcopy(PROFILE);invalid['mapper_cpus']=[16]
            with self.assertRaisesRegex(ValueError,'unavailable'):partition.validate(invalid,root)
    def test_stalled_probe_is_bounded_and_frames_stay_atomic(self):
        parent,child=socket.socketpair(socket.AF_UNIX,socket.SOCK_SEQPACKET)
        try:
            parent.setblocking(False);parent.setsockopt(socket.SOL_SOCKET,socket.SO_SNDBUF,8192)
            sender=probe.ProbeProcess.__new__(probe.ProbeProcess);sender.socket=parent;sender.dropped=sender.errors=0
            for sequence in range(1000):
                sender.submit(dict(tid=7,serial=sequence,session=9,generation=3,sequence=sequence,source_ms=sequence+100,overflow=0,points=[(0,sequence,0,0,0,0,0)]))
            self.assertGreater(sender.dropped,0);self.assertEqual(sender.errors,0)
            child.setblocking(False);received=0
            while True:
                try:payload=child.recv(8193)
                except BlockingIOError:break
                values=struct.unpack('<%dd'%(len(payload)//8),payload)
                self.assertEqual(len(values),19);self.assertEqual(values[3],values[6]);self.assertEqual(values[6],values[13]);self.assertEqual(values[4:6],(9,3));received+=1
            self.assertGreater(received,0);self.assertEqual(received+sender.dropped,1000)
        finally:parent.close();child.close()
if __name__=='__main__':unittest.main()
