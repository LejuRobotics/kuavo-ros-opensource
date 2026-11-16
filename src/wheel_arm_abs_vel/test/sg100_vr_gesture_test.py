#!/usr/bin/env python3
import importlib.util
import unittest
from pathlib import Path
from types import SimpleNamespace

source = Path(__file__).resolve().parents[1] / 'scripts/tools/sg100_vr_gesture.py'
spec = importlib.util.spec_from_file_location('sg100_vr_gesture', source)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)
GestureTouchGate = module.GestureTouchGate


def packet(**changes):
    values = dict(left_first_button_touched=True, left_first_button_pressed=False,
                  right_first_button_touched=True, right_first_button_pressed=False,
                  right_second_button_touched=False, right_second_button_pressed=False)
    values.update(changes)
    return SimpleNamespace(**values)


class GestureTouchTest(unittest.TestCase):
    def confirm(self, gate, msg, start=0):
        return [gate.update(msg, True, start+t) for t in (0, .1, .2, .31)]

    def test_next_and_previous(self):
        self.assertEqual(self.confirm(GestureTouchGate(), packet()), [0, 0, 0, 1])
        self.assertEqual(self.confirm(GestureTouchGate(), packet(
            right_first_button_touched=False, right_second_button_touched=True)), [0, 0, 0, -1])

    def test_held_touch_only_once_and_release_rearms(self):
        gate = GestureTouchGate()
        self.confirm(gate, packet())
        self.assertEqual(gate.update(packet(), True, .4), 0)
        gate.update(packet(left_first_button_touched=False), True, .5)
        self.assertEqual(self.confirm(gate, packet(), .6)[-1], 1)

    def test_pressed_unlock_combo_does_not_switch(self):
        gate = GestureTouchGate()
        self.assertEqual(self.confirm(gate, packet(
            left_first_button_pressed=True, right_first_button_pressed=True)), [0]*4)
        self.assertEqual(self.confirm(gate, packet(), .4)[-1], 1)

    def test_invalid_or_stale_input_cancels_confirmation(self):
        gate = GestureTouchGate()
        gate.update(packet(), True, 0)
        self.assertEqual(gate.update(packet(), True, .4), 0)
        gate.update(packet(), False, .5)
        self.assertEqual(self.confirm(gate, packet(), .6)[-1], 1)

    def test_direction_change_restarts_confirmation(self):
        gate = GestureTouchGate()
        self.confirm(gate, packet())
        gate.update(packet(left_first_button_touched=False), True, .4)
        gate.update(packet(), True, .5)
        previous = packet(right_first_button_touched=False, right_second_button_touched=True)
        self.assertEqual(self.confirm(gate, previous, .6), [0, 0, 0, -1])


if __name__ == '__main__':
    unittest.main()
