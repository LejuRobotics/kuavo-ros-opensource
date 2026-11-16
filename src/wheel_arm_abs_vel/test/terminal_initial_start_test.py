#!/usr/bin/env python3
import os
import pty
from pathlib import Path
import select
import subprocess
import sys
import termios
import unittest
from unittest.mock import patch
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from terminal_initial_start import terminal_input, KeyGate, request_initial_start, SERVICE


class TerminalTest(unittest.TestCase):
    def test_character_without_enter_and_restore_after_exception(self):
        master, slave = pty.openpty()
        before = termios.tcgetattr(slave)
        try:
            with self.assertRaises(RuntimeError):
                with terminal_input(os.ttyname(slave)) as fd:
                    self.assertTrue(termios.tcgetattr(fd)[3] & termios.ISIG)
                    os.write(master, b'o')
                    self.assertTrue(select.select([fd], [], [], .5)[0])
                    self.assertEqual(os.read(fd, 16), b'o')
                    raise RuntimeError('test shutdown')
            self.assertEqual(termios.tcgetattr(slave), before)
        finally:
            os.close(master)
            os.close(slave)

    def test_repeat_and_other_keys_do_not_queue_initialization(self):
        gate = KeyGate()
        self.assertFalse(gate.accept(b'x\na', 0.))
        self.assertTrue(gate.accept(b'o', 1.))
        for at in (1.5, 1.6, 1.7, 2., 2.5):
            self.assertFalse(gate.accept(b'oooo', at))
        self.assertTrue(gate.accept(b'O', 3.6))

    @patch('terminal_initial_start.subprocess.run')
    def test_service_result_is_checked_without_fallback_or_retry(self, run):
        run.return_value = subprocess.CompletedProcess([], 0, 'success: true\nmessage: ready\n', '')
        self.assertEqual(request_initial_start(), (True, 'ready'))
        self.assertEqual(run.call_args[0][0], ['rosservice', 'call', SERVICE, '{}'])
        run.return_value = subprocess.CompletedProcess([], 0, 'success: false\nmessage: rejected\n', '')
        self.assertFalse(request_initial_start()[0])
        run.side_effect = subprocess.TimeoutExpired('rosservice', 3.)
        self.assertFalse(request_initial_start()[0])
        self.assertEqual(run.call_count, 3)


if __name__ == '__main__':
    unittest.main()
