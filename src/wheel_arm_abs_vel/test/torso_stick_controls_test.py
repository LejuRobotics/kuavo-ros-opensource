#!/usr/bin/env python3
"""Quest torso selections, permission ordering and Mapper integration."""
import unittest
from types import SimpleNamespace as NS
from torso_controls_test import TorsoControls, joy
import fixed_follow_operation_test as fixed


class TorsoStickTest(unittest.TestCase):
    def setUp(self):
        self.now = 10.
        self.p = TorsoControls('quest')
        self.sample()
        self.sample()

    def sample(self, ready=True, **kw):
        self.now += .01
        raw = joy(**kw)
        return self.p.update(raw, raw, self.now, ready)

    def hold(self, y, master=False, count=101, **kw):
        for _ in range(count):
            self.sample(right_y=y, left_first_button_pressed=master, **kw)

    def test_position_off_preserves_yaw_and_up_restores(self):
        self.hold(-1.)
        self.assertTrue(self.p.enabled)
        self.assertFalse(self.p.position_enabled)
        seq = self.p.operation_seq
        self.hold(-1.)
        self.assertEqual(self.p.operation_seq, seq)
        self.sample()
        self.hold(1.)
        self.assertTrue(self.p.enabled and self.p.position_enabled)
        self.assertGreater(self.p.operation_seq, seq)

    def test_master_unlock_clears_previous_position_lock(self):
        self.hold(-1.)
        self.sample()
        self.hold(-1., master=True)
        self.assertFalse(self.p.enabled)
        self.assertFalse(self.p.selected_enabled)
        self.sample()
        self.hold(1., master=True)
        self.assertTrue(self.p.enabled)
        self.assertTrue(self.p.position_enabled)

    def test_short_hold_threshold_and_neutral_rearm(self):
        self.hold(-1., count=100)
        self.assertTrue(self.p.position_enabled)
        self.sample(right_y=-1.)
        self.assertFalse(self.p.position_enabled)
        self.hold(1.)  # no neutral: do not rearm
        self.assertFalse(self.p.position_enabled)
        self.sample()
        self.hold(.8)
        self.assertFalse(self.p.position_enabled)
        self.hold(1.)
        self.assertTrue(self.p.position_enabled)

    def test_standalone_down_selects_yaw_and_up_unlocks_from_all_locked(self):
        self.hold(-1., master=True)
        self.sample()
        self.hold(-1.)
        self.assertFalse(self.p.position_enabled)
        self.assertTrue(self.p.enabled)
        self.sample()
        self.hold(-1., master=True)
        self.sample()
        self.hold(1.)
        self.assertTrue(self.p.position_enabled)
        self.assertTrue(self.p.enabled)

    def test_all_four_commands_are_explicit_from_every_stored_mode(self):
        for enabled, position in ((False, False), (False, True), (True, False), (True, True)):
            for master, y, expected in ((True, -1., (False, False)),
                                        (True, 1., (True, True)),
                                        (False, -1., (True, False)),
                                        (False, 1., (True, True))):
                with self.subTest(initial=(enabled,position), master=master, y=y):
                    self.setUp()
                    self.p.enabled = self.p.selected_enabled = enabled
                    self.p.position_enabled = position
                    self.sample()
                    seq = self.p.operation_seq
                    self.hold(y, master=master)
                    self.assertEqual((self.p.enabled,self.p.position_enabled),expected)
                    self.assertEqual(self.p.selected_enabled,expected[0])
                    self.assertEqual(self.p.operation_seq,seq + ((enabled,position) != expected))
                    self.hold(y, master=master, count=250)
                    self.assertEqual((self.p.enabled,self.p.position_enabled),expected)

    def test_hand_buttons_cannot_form_extra_torso_shortcuts(self):
        for button in ('left_second_button_pressed','right_first_button_pressed','right_second_button_pressed'):
            for master in (False, True):
                self.setUp()
                for _ in range(150):
                    self.sample(right_y=-1.,left_first_button_pressed=master,**{button:True})
                if master and button != 'right_second_button_pressed':
                    self.assertFalse(self.p.enabled)  # Existing stop / X+A inhibition.
                else:
                    self.assertTrue(self.p.enabled)
                self.assertTrue(self.p.position_enabled)

    def test_stale_or_fault_cannot_finish_a_partial_enable(self):
        for event in ('stale', 'not_ready', 'nan', 'stop', 'prepare', 'arm_hold'):
            with self.subTest(event=event):
                self.setUp()
                self.hold(-1., master=True)
                self.sample()
                self.hold(1., master=True, count=90)
                if event == 'stale':
                    self.now += 1.
                    self.p.authorized(self.now, True)
                elif event == 'not_ready': self.sample(ready=False, right_y=1.)
                elif event == 'nan': self.sample(right_y=float('nan'))
                else:
                    button = {'stop':'left_second_button_pressed',
                              'prepare':'right_first_button_pressed',
                              'arm_hold':'right_second_button_pressed'}[event]
                    self.sample(left_first_button_pressed=True, right_y=1., **{button:True})
                self.hold(1., master=True, count=150)
                self.assertFalse(self.p.enabled)
                self.sample()
                self.hold(1., master=True)
                self.assertTrue(self.p.enabled)

    def test_startup_held_stick_cannot_enable_and_selected_mode_survives_prepare(self):
        self.p = TorsoControls('quest', enable_on_first_follow=False)
        self.hold(1., master=True, count=150)
        self.assertFalse(self.p.enabled)
        self.sample()
        self.hold(1., master=True)
        self.sample()
        self.hold(-1.)
        self.p.reset()
        self.sample(ready=False)
        self.p.request_follow()
        self.sample()
        self.assertTrue(self.p.enabled)
        self.assertFalse(self.p.position_enabled)

    def test_mapper_exports_ordered_permission_metadata_without_arm_pause(self):
        fixture = fixed.FixedFollowTest()
        fixture.setUp()
        n = fixture.n
        n._absolute_state = fixed.State.ACTIVE
        n._torso_controls = self.p
        n._torso_controls.last_time = fixed.clock.now
        fixture.ack()
        msg = NS()
        n._fill_subsystem_permissions_locked(msg, fixed.clock.now)
        initial_seq = msg.torso_operation_seq
        self.hold(-1.)
        n._torso_controls.last_time = fixed.clock.now
        n._fill_subsystem_permissions_locked(msg, fixed.clock.now)
        self.assertTrue(msg.torso_control_enabled and msg.torso_position_hold)
        self.assertFalse(msg.arm_hold)
        self.assertGreater(msg.torso_operation_seq, initial_seq)
        n._fill_subsystem_permissions_locked(msg, fixed.clock.now)
        seq = msg.torso_operation_seq
        n._fill_subsystem_permissions_locked(msg, fixed.clock.now)
        self.assertEqual(msg.torso_operation_seq, seq)

    def test_mapper_exports_all_modes_without_changing_arm_pause(self):
        for arm_hold in (False, True):
            fixture = fixed.FixedFollowTest()
            fixture.setUp()
            n = fixture.n
            n._absolute_state = fixed.State.ACTIVE
            n._fixed_arm_hold = arm_hold
            n._torso_controls = self.p
            fixture.ack()
            for master, y, expected in ((True,-1.,(False,True)),
                                        (False,-1.,(True,True)),
                                        (True,1.,(True,False)),
                                        (True,-1.,(False,True)),
                                        (False,1.,(True,False))):
                self.sample()
                self.hold(y,master=master)
                self.p.last_time = fixed.clock.now
                msg = NS()
                n._fill_subsystem_permissions_locked(msg,fixed.clock.now)
                self.assertEqual((msg.torso_control_enabled,msg.torso_position_hold),expected)
                self.assertEqual(msg.arm_hold,arm_hold)
                self.p.last_time = self.now


if __name__ == '__main__':
    unittest.main()
