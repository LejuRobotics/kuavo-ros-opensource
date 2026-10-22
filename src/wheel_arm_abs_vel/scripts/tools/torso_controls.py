"""Explicit torso permission and end-effector chord arbitration (no ROS I/O)."""
import copy
import math


class TorsoControls:
    MAX_AGE = .25
    RT_CHORD_WINDOW = .15

    def __init__(self, source, enable_on_first_follow=True):
        if source not in ('quest', 'pico'):
            raise ValueError('unsupported torso input source: ' + source)
        self.source = source
        self.enabled = False
        # Startup preference is consumed once, only after the production
        # readiness and freshness gates admit following. Reset must not rearm
        # this preference or override a later explicit operator toggle.
        self.startup_enable_pending = bool(enable_on_first_follow)
        self.follow_ready_seen = False
        self.last_time = None
        self.chord_down = False
        self.require_release = True
        self.last_hand = None
        self.quest_consumed = False
        self.quest_thumb = False
        self.quest_b_down = False
        self.y_down = False
        self.y_pending = False
        self.y_consumed = False
        self.rt_hold = None
        self.rt_released = False
        self.rt_pending_since = None
        self.rt_down = False
        self.rt_before_press = 0.

    def reset(self):
        # A revocation cancels even a startup preference not yet consumed.
        self.startup_enable_pending = False
        self.wait_for_follow()

    def wait_for_follow(self):
        """Inhibit during initial acquisition; never create/rearm permission."""
        self.enabled = False
        self.require_release = True

    def _check_readiness(self, now, ready):
        fresh = self.last_time is not None and 0 <= now-self.last_time <= self.MAX_AGE
        if ((self.last_time is not None and not fresh)
                or (self.follow_ready_seen and not ready)):
            self.reset()
        elif not ready or not fresh:
            # No first packet / initial execution handshake is ordinary startup,
            # unlike a lost established stream or an explicit reset.
            self.wait_for_follow()
        self.follow_ready_seen = self.follow_ready_seen or ready
        return fresh

    def authorized(self, now, ready):
        self._check_readiness(now, ready)
        return self.enabled

    def update(self, raw, hand, now, ready):
        result = copy.copy(hand)
        fresh = self._check_readiness(now, ready)
        valid = all(math.isfinite(v) for v in
                    (raw.left_trigger, raw.right_trigger, raw.left_grip, raw.right_grip))
        stop = raw.left_first_button_pressed and raw.left_second_button_pressed
        arm = raw.left_first_button_pressed and (raw.right_first_button_pressed or raw.right_second_button_pressed)
        prepare = raw.left_first_button_pressed and raw.right_first_button_pressed
        # X+B pauses arms independently when torso control owns a live session.
        # X+A still revokes torso permission for the ready-pose operation.
        if not valid or stop or prepare:
            self.reset()
        self.last_time = now
        if self.source == 'quest':
            modifier = raw.left_first_button_touched and raw.left_second_button_touched
            chord = modifier and raw.right_second_button_pressed
            chord = chord and not (raw.left_first_button_pressed or raw.left_second_button_pressed
                                   or raw.right_first_button_pressed)
            # Match the legacy Quest FSM: XY must be touched when B goes down.
            # Track raw B even outside the modifier and while arm shortcuts
            # are active, so adding XY or releasing X from X+B is not a press.
            toggle_edge = chord and not self.quest_b_down
            self.quest_b_down = raw.right_second_button_pressed
            if modifier and not self.quest_consumed:
                self.quest_consumed = True
                self.quest_thumb = (self.last_hand.left_first_button_touched
                                    if self.last_hand is not None else False)
            if self.quest_consumed:
                # XY touch is a torso modifier, not a thumb/hand-gesture request.
                result.left_first_button_touched = self.quest_thumb
                result.left_second_button_touched = False
                result.right_first_button_pressed = False
                result.right_second_button_pressed = False
                if not (raw.left_first_button_touched or raw.left_second_button_touched
                        or raw.right_second_button_pressed):
                    self.quest_consumed = False
        else:
            # Delay a fresh standalone RT briefly so RT-first torso chords do
            # not send a gripper close before Y arrives. No attempt is made to
            # undo an already emitted long standalone squeeze.
            rt_pressed = valid and raw.right_trigger > .5
            if rt_pressed and not self.rt_down and fresh and self.rt_hold is None:
                self.rt_pending_since = now
                self.rt_before_press = self.last_hand.right_trigger if self.last_hand is not None else 0.
            if not fresh or not valid or not rt_pressed:
                self.rt_pending_since = None
            if self.rt_pending_since is not None:
                if now - self.rt_pending_since < self.RT_CHORD_WINDOW:
                    result.right_trigger = self.rt_before_press
                else:
                    self.rt_pending_since = None
            self.rt_down = rt_pressed
            y = raw.left_second_button_pressed
            rt = valid and raw.right_trigger > .5
            chord = rt and y and not (raw.left_first_button_pressed or
                    raw.right_first_button_pressed or raw.right_second_button_pressed)
            toggle_edge = chord and not self.chord_down
            # Resolve a lone Y on release, so Y-first and RT-first chords both
            # consume it before the end-effector freeze/gesture handler.
            result.left_second_button_pressed = False
            if y and not self.y_down:
                self.y_pending = True
                self.y_consumed = False
            if chord or stop or arm or not fresh or not valid:
                self.y_consumed = True
            if y and self.rt_hold is None:
                self.rt_hold = self.last_hand.right_trigger if self.last_hand is not None else 0.
                self.rt_released = False
            if chord:
                self.rt_released = False
            if not y and self.y_down:
                result.left_second_button_pressed = self.y_pending and not self.y_consumed
                self.y_pending = False
            self.y_down = y
            if self.rt_hold is not None:
                # Keep the last emitted gripper value throughout the chord and
                # its release. A new isolated RT press explicitly resumes grip.
                if self.rt_released and rt and not y:
                    self.rt_hold = None
                else:
                    result.right_trigger = self.rt_hold
                    if not rt:
                        self.rt_released = True
        if (self.startup_enable_pending and ready and fresh and valid
                and not stop and not arm):
            self.enabled = True
            self.startup_enable_pending = False
        if not chord:
            self.require_release = False
        elif (toggle_edge and not self.require_release and ready and fresh
              and valid and not stop and not arm):
            self.enabled = not self.enabled
        self.chord_down = chord
        if valid:
            self.last_hand = copy.copy(result)
        return result
