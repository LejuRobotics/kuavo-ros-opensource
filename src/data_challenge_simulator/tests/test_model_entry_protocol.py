"""Assertions for the automatic-evaluation handshake.

These lock the contract the upstream harness in
``kuavo_data_challenge/kuavo_deploy/src/eval/sim_auto_test.py`` depends on,
and the ordering rules that make it work against that harness's event
handling.  They are static (source-level) because the entry needs a full ROS
graph to run.
"""

import ast
from pathlib import Path
import threading


PACKAGE = Path(__file__).resolve().parents[1]
ENTRY = PACKAGE / "scripts/model_entry.py"
OBSERVER = PACKAGE / "utils/episode_success.py"
SCORER = PACKAGE / "utils/task_scorer.py"


def test_model_entry_provides_the_harness_services_and_topic():
    source = ENTRY.read_text()
    assert '"/simulator/reset", Trigger, self._reset' in source
    assert '"/simulator/start", Trigger, self._start' in source
    assert '"/simulator/success", Bool, queue_size=1' in source
    assert '"/simulator/init", Trigger' in source


def test_reset_answers_before_notifying_init():
    """The harness clears init_evt *after* a reset returns.

    A notification sent from inside the handler would be wiped before the
    harness reaches its wait loop.  The reset must therefore not block, and
    initialization belongs to the replacement roslaunch, never to a thread
    that still holds the old round's SDK objects.
    """
    source = ENTRY.read_text()
    reset = source.index("def _reset(self, _request):")
    start = source.index("def _start(self, _request):")
    body = source[reset:start]
    assert "_notify_initialized" not in body
    assert "_initialize()" not in body
    assert "thread.start()" not in body
    assert "self.reset_handoff.mark_response_sent()" in body
    assert body.index("self.reset_handoff.mark_response_sent()") < body.index(
        "self.relaunch.set()") < body.index("return response")
    # The gate is raised on the response path and waited on before notifying.
    notify = source.index("def _notify_initialized(self):")
    notify_body = source[notify:source.index("def _next_reset_seed(self):")]
    assert "self.reset_handoff.wait_for(" in notify_body
    # Cleared for each round, or the gate would stay open from the last one.
    assert "self.reset_handoff.prepare(seed)" in body


def test_start_waits_for_the_scene_but_not_for_the_reset_call():
    """_start must be able to block on the scene, never on the reset itself.

    A reset answers as soon as it has begun re-initializing, so the harness
    can reach /simulator/start while the new layout is still being placed.
    Waiting for the scene is required; the reset handler must already be gone.
    """
    source = ENTRY.read_text()
    start = source.index("def _start(self, _request):")
    body = source[start:source.index("def _ensure_observer(self):")]
    assert "self.initialized_evt.wait(" in body
    assert "self.relaunch" not in body
    # The reset handler only requests a relaunch.  main() initializes the
    # replacement round after this handler has returned.
    reset = source[source.index("def _reset(self, _request):"):start]
    assert "thread.start()" not in reset
    assert "_initialize()" not in reset


def test_success_is_reported_once_per_episode():
    """The harness ends its rollout on the first True and clears its event.

    Repeating it would risk ending the *next* episode before it starts.
    """
    source = ENTRY.read_text()
    watcher = source.index("def _watch_success(self):")
    body = source[watcher:source.index("def main():")]
    assert "self.round_complete = True" in body
    assert "if not self.episode_started or self.round_complete:" in body
    assert body.count("self.publish_success(True)") == 1
    # Re-armed for the next episode by the initialization that starts it.
    assert "self.round_complete = False" in source


def test_a_reset_restarts_the_node_with_the_new_rounds_spawn():
    """Task 1 layouts pair each object placement with its own base spawn."""
    source = ENTRY.read_text()
    assert "while True:" in source
    assert "round_seed = protocol.reset_seed" in source
    assert "self.reset_seed = self._next_reset_seed()" in source
    assert "random.SystemRandom().randint(1, 10 ** 6)" in source
    # The seed picked at launch is only the first round's.
    assert "round_seed = args.seed if args.seed is not None else (" in source


def test_reset_response_gate_survives_the_protocol_relaunch():
    """The old round answers reset; the replacement round sends init."""
    source = ENTRY.read_text()
    assert "reset_handoff = ResetHandoff()" in source
    assert "reset_handoff=reset_handoff" in source
    protocol = source[source.index("class EvaluationProtocol:"):source.index(
        "# -- initialization and the /simulator/init callback")]
    assert "reset_handoff if reset_handoff is not None" in protocol
    handoff = source[source.index("class ResetHandoff(object):"):source.index(
        "def request_shutdown")]
    assert "self._seed = int(seed)" in handoff
    assert "return self._seed == int(seed)" in handoff


def test_any_round_failure_restarts_with_a_fresh_seed():
    source = ENTRY.read_text()
    handler = source[source.index("except Exception as error:",
                                  source.index("def main():")):
                     source.index("finally:", source.index("def main():"))]
    recovery = source[source.index("if round_error is not None:"):
                      source.index("if not relaunching:")]

    assert "round_error = error" in handler
    assert "raise" not in handler
    assert "random.SystemRandom().randint(1, 10 ** 6)" in recovery
    assert "reset_handoff.retarget(round_seed)" in recovery
    assert "time.sleep(ROUND_RETRY_DELAY)" in recovery
    assert "continue" in recovery


def test_every_required_topic_timeout_uses_round_recovery():
    source = ENTRY.read_text()
    main = source[source.index("def main():"):source.index(
        'if __name__ == "__main__":')]
    assert "for topic in required:" in main
    assert "wait_for_topic(topic, timeout=60.0)" in main
    assert "except TopicTimeout" not in main
    assert "except Exception as error:" in main


def test_init_handshake_failure_is_a_recoverable_round_failure():
    source = ENTRY.read_text()
    notify = source[source.index("def _notify_initialized(self):"):
                    source.index("def _next_reset_seed(self):")]
    assert "/simulator/init did not appear after reset" in notify
    assert "/simulator/init is advertised but the call failed" in notify
    assert "/simulator/init rejected this round" in notify
    assert '"model entry: /simulator/init is absent' in notify
    assert 'return False' in notify


def test_reset_handoff_releases_only_the_replacement_seed():
    source = ENTRY.read_text()
    tree = ast.parse(source)
    handoff_node = next(
        node for node in tree.body
        if isinstance(node, ast.ClassDef) and node.name == "ResetHandoff")
    namespace = {"threading": threading}
    exec(compile(ast.Module(body=[handoff_node], type_ignores=[]),
                 str(ENTRY), "exec"), namespace)
    handoff = namespace["ResetHandoff"]()

    handoff.prepare(42)
    assert not handoff.wait_for(42, timeout=0.0)
    handoff.mark_response_sent()
    assert handoff.wait_for(42, timeout=0.0)
    assert not handoff.wait_for(41, timeout=0.0)
    assert handoff.pending_seed() == 42

    # An internal failed attempt changes the simulator seed without clearing
    # the already-returned reset gate the harness is waiting behind.
    handoff.retarget(43)
    assert handoff.wait_for(43, timeout=0.0)
    assert not handoff.wait_for(42, timeout=0.0)

    handoff.prepare(44)
    assert handoff.pending_seed() is None
    assert not handoff.wait_for(44, timeout=0.0)


def test_round_services_are_unregistered_before_their_names_are_reused():
    source = ENTRY.read_text()
    shutdown = source[source.index("def shutdown(self):"):source.index(
        "def _watch_success(self):")]
    assert 'self.reset_service.shutdown("model simulator round ended")' in shutdown
    assert 'self.start_service.shutdown("model simulator round ended")' in shutdown


def test_observer_reuses_the_accepted_scorer_rules():
    """No second definition of "done" -- task_scorer.py stays untouched."""
    source = OBSERVER.read_text()
    assert "DEFAULT_SCORE_FILE, SCORERS, PoseTap" in source
    assert "self.scorer._evaluate()" in source
    for name in ("CYLINDERS", "BOXES", "RINGS",
                 "BASE_ARRIVALS_NEEDED", "ARRIVALS_NEEDED"):
        assert name in source, name
    # Time penalties must not gate success: a late round still succeeded.
    assert "time_penalty" not in source


def test_model_score_clock_and_finalization_follow_episode_lifecycle():
    observer_source = OBSERVER.read_text()
    entry_source = ENTRY.read_text()

    # Numeric scoring uses the same scorer rules with a model-only output.
    assert "DEFAULT_SCORE_FILE, SCORERS, PoseTap" in observer_source
    assert "self.scorer.start_time = time.time()" in observer_source
    assert "self.scorer.finish(write_only=True)" in observer_source
    assert "self.score_store.allocate(self.round_seed)" in entry_source
    assert "self.score_store.annotate(self.score_file, reason)" in entry_source

    # The observer is only created by /simulator/start, which already waits
    # for fixed initialization.  Every episode exit path settles it once.
    start = entry_source.index("def _start(self, _request):")
    ensure = entry_source.index("def _ensure_observer(self):")
    assert "self._ensure_observer()" in entry_source[start:ensure]
    assert "self.observer.start()" in entry_source
    assert 'self._finalize_score("success")' in entry_source
    assert 'self._finalize_score("reset")' in entry_source
    assert 'self._finalize_score("shutdown")' in entry_source


def test_scorer_module_is_unchanged_by_the_handshake():
    """The accepted score baseline is shared; this feature only reads it."""
    source = SCORER.read_text()
    assert "class Task1Scorer(BaseScorer):" in source
    assert "class Task2Scorer(BaseScorer):" in source
    assert "class Task3Scorer(BaseScorer):" in source
    assert "FINALIZE_SERVICE = \"/task_scorer/finalize\"" in source


def test_ready_is_published_only_after_arm_handover():
    """External control opens after initialization but before ready.

    ``KuavoBaseRosEnv.reset()`` (called by ``run_single_episode`` at the start
    of every episode) switches the arm to external control and resets the head
    to ``head_init``.  The simulator mirrors both, but only after fixed
    initialization and seed docking. The latched completion signal comes last,
    so model commands cannot race simulator-owned preprocessing or arm mode
    handoff.
    """
    source = ENTRY.read_text()
    initialize = source.index("def _initialize(self):")
    body = source[initialize:source.index("    def publish_success(self, value):")]
    assert "self._hand_over_arm_control()" in body
    assert body.index("INITIALIZERS[self.task_id](") < \
        body.index("self._hand_over_arm_control()") < \
        body.index("self.ready_publisher.publish(Bool(data=True))")
    assert 'if not handed_over:' in body
    assert 'external arm-control handoff failed' in body

    handover = source.index("def _hand_over_arm_control(self):")
    handover_body = source[handover:source.index("    def _notify_initialized")]

    # Both halves of the upstream reset that the simulator has to mirror.
    assert "self.robot.set_external_control_arm_mode()" in handover_body
    assert "self.robot.control_head(*UPSTREAM_HEAD_INIT)" in handover_body

    # Upstream's simulation head_init is [0, 0.209], not zero: the model was
    # trained against this head pose, so this value must not drift.
    assert "UPSTREAM_HEAD_INIT = (0.0, 0.209)" in source

    # The helper reports failure through its return value; _initialize turns
    # that into a hard readiness gate after cleanup.
    assert "raise" not in handover_body
