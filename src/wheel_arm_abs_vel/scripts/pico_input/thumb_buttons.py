"""Pico long-X/A alternative for Quest's thumb touch, end-effector path only.

0.50 s and idle triggers follow the official Pico joystick handler. Arm mode,
initialization and stop combinations retain priority. Never publish synthetic
physical touch on /quest_joystick_data or change arm/torso control authority.
"""
import copy


class ThumbButtons:
    def __init__(self):
        self.open = [False, False]
        self.started = [None, None]
        self.fired = [False, False]
        self.was_down = [False, False]
        self.last_packet = None

    def update(self, msg, now, enabled=True):
        down = [msg.left_first_button_pressed, msg.right_first_button_pressed]
        stale = self.last_packet is not None and now - self.last_packet > .25
        self.last_packet = now
        idle_triggers = msg.left_trigger < .5 and msg.right_trigger < .5
        for side in (0, 1):
            alone = (not down[1-side] and not msg.left_second_button_pressed
                     and not msg.right_second_button_pressed)
            allowed = enabled and idle_triggers and alone and not stale
            if not down[side]:
                self.started[side] = None
                self.fired[side] = False
            elif not allowed:
                # A chord/startup or stream interruption consumes this press.
                # Releasing the other button must not turn the chord into a thumb toggle.
                self.started[side] = None
                self.fired[side] = True
            elif not self.was_down[side]:
                self.started[side] = now
                self.fired[side] = False
            if (down[side] and allowed and not self.fired[side]
                    and self.started[side] is not None and now-self.started[side] >= .5):
                self.open[side] = not self.open[side]
                self.fired[side] = True
            self.was_down[side] = down[side]
        result = copy.copy(msg)
        result.left_first_button_touched = self.open[0]
        result.right_first_button_touched = self.open[1]
        return result
