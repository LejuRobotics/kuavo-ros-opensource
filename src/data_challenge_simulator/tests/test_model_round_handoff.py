"""Exercise reset/start races using the actual protocol and fake ROS I/O."""
import ast
from pathlib import Path
import sys
import threading
import time
from types import SimpleNamespace

import pytest

PACKAGE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE))


@pytest.fixture
def protocol_code():
    tree = ast.parse((PACKAGE / "scripts/model_entry.py").read_text())
    classes = [n for n in tree.body if isinstance(n, ast.ClassDef)
               and n.name in ("ResetHandoff", "EvaluationProtocol")]
    services = []
    def service(name, kind, callback):
        obj = SimpleNamespace(name=name, callback=callback, closed=False)
        obj.shutdown = lambda reason: setattr(obj, "closed", True)
        services.append(obj)
        return obj
    ros = SimpleNamespace(
        is_shutdown=lambda: False, loginfo=lambda *a: None,
        logerr=lambda *a: None, on_shutdown=lambda callback: None,
        Publisher=lambda *a, **kw: SimpleNamespace(publish=lambda msg: None),
        Service=service)
    ns = dict(threading=threading, time=time, rospy=ros, Trigger=object,
              TriggerResponse=lambda **kw: SimpleNamespace(**kw),
              Bool=lambda **kw: SimpleNamespace(**kw), ROUND_READY_TIMEOUT=.5,
              SUCCESS_POLL_INTERVAL=.001)
    exec(compile(ast.Module(body=classes, type_ignores=[]), "model_entry", "exec"), ns)
    return SimpleNamespace(**ns, services=services)


def make_protocol(code, handoff, task_id, seed=28):
    output = []
    publisher = SimpleNamespace(publish=lambda msg: output.append(msg.data))
    p = code.EvaluationProtocol(task_id, None, None, seed, publisher, publisher,
                                reset_handoff=handoff)
    p.starts = []
    def observe():
        p.starts.append(time.monotonic())
        p.score_file = "started"
        return True
    p._ensure_observer = observe
    p.output = output
    return p


def run_request(callback):
    result = []
    thread = threading.Thread(target=lambda: result.append(callback(None)), daemon=True)
    thread.start()
    return thread, result


@pytest.mark.parametrize("task_id", [1, 2, 3])
def test_early_start_follows_reset_to_same_seed_replacement(protocol_code, task_id):
    code = protocol_code
    with code.ResetHandoff() as handoff:
        handoff.serve()
        old = make_protocol(code, handoff, task_id)
        assert handoff.reset(None).success
        thread, result = run_request(handoff.start)
        # The retiring initializer must not release the request, even at the same seed.
        old.initialized_evt.set()
        assert not handoff.mark_ready(old)
        thread.join(.02)
        assert thread.is_alive() and not old.starts
        old.shutdown()
        fresh = make_protocol(code, handoff, task_id)
        thread.join(.02)
        assert thread.is_alive() and not fresh.starts
        fresh.initialized_evt.set()
        assert handoff.mark_ready(fresh)
        thread.join(1)
        assert not thread.is_alive() and result[0].success
        assert not old.starts and len(fresh.starts) == 1
        assert fresh.episode_started and fresh.evaluation_started
        assert handoff.start(None).success
        assert len(fresh.starts) == 1  # No second observer or clock reset.


@pytest.mark.parametrize("task_id", [1, 2, 3])
def test_services_survive_three_rounds_and_score_only_after_start(protocol_code, task_id):
    code = protocol_code
    with code.ResetHandoff() as handoff:
        handoff.serve()
        for seed in (28, 29, 30):
            p = make_protocol(code, handoff, task_id, seed)
            assert not p.starts and not p.evaluation_started
            p.initialized_evt.set()
            assert handoff.mark_ready(p)
            assert not p.starts
            threads = [run_request(handoff.start) for _ in range(2)]
            for thread, result in threads:
                thread.join(1)
                assert not thread.is_alive() and result[0].success
            assert len(p.starts) == 1
            assert handoff.reset(None).success
            assert p.reset_seed == seed + 1
            assert p.output[-2:] == [False, False]
            p.shutdown()
        assert len(code.services) == 2
        assert not any(s.closed for s in code.services)
    assert all(s.closed for s in code.services)


@pytest.mark.parametrize("task_id", [1, 2, 3])
def test_timeout_and_shutdown_do_not_start_scoring(protocol_code, task_id):
    code = protocol_code
    handoff = code.ResetHandoff()
    p = make_protocol(code, handoff, task_id)
    assert not handoff.start(None).success
    assert not p.starts and not p.evaluation_started
    thread, result = run_request(handoff.start)
    time.sleep(.02)
    handoff.close()
    thread.join(1)
    assert not thread.is_alive() and not result[0].success
    assert not p.starts


@pytest.mark.parametrize("task_id", [1, 2, 3])
def test_real_observer_failure_is_not_reported_as_success(protocol_code, task_id):
    code = protocol_code
    with code.ResetHandoff() as handoff:
        p = make_protocol(code, handoff, task_id)
        p.initialized_evt.set()
        handoff.mark_ready(p)
        p._ensure_observer = lambda: False
        assert not handoff.start(None).success
        assert not p.episode_started


@pytest.mark.parametrize("task_id", [1, 2, 3])
@pytest.mark.parametrize("reset_during", ["trajectory", "handover"])
def test_retiring_initializer_never_publishes_ready(
        protocol_code, monkeypatch, task_id, reset_during):
    code = protocol_code
    entered, release = threading.Event(), threading.Event()
    def pause():
        entered.set()
        assert release.wait(1)
    monkeypatch.setitem(sys.modules, "utils.trajectory_controller",
                        SimpleNamespace(TrajectoryController=lambda *a, **kw:
                                        SimpleNamespace(stop=lambda: None)))
    monkeypatch.setitem(sys.modules, "utils.arm_state",
                        SimpleNamespace(wait_for_first_arm_state=lambda: [0]*14))
    globals_ = code.EvaluationProtocol._initialize.__globals__
    globals_["INITIALIZERS"] = {task_id: lambda *a: pause() if reset_during == "trajectory" else None}
    with code.ResetHandoff() as handoff:
        old = make_protocol(code, handoff, task_id)
        old._wait_for_controller_ready = lambda: None
        old._close_external_arm_input = lambda: None
        old._hand_over_arm_control = lambda: (pause() or True) if reset_during == "handover" else True
        worker = threading.Thread(target=old._initialize, daemon=True)
        worker.start()
        assert entered.wait(1)
        assert handoff.reset(None).success
        release.set()
        worker.join(1)
        assert not worker.is_alive()
        assert True not in old.output
        assert not old.initialized_evt.is_set()
        assert not old.starts and not handoff.is_current(old, ready=True)


@pytest.mark.parametrize("task_id", [1, 2, 3])
def test_reset_between_start_selection_and_admission(protocol_code, task_id):
    code = protocol_code
    with code.ResetHandoff() as handoff:
        old = make_protocol(code, handoff, task_id)
        handoff.mark_ready(old)
        selected, release = threading.Event(), threading.Event()
        accept = old._start
        def pause(request):
            selected.set()
            assert release.wait(1)
            return accept(request)
        old._start = pause
        thread, result = run_request(handoff.start)
        assert selected.wait(1)
        assert handoff.reset(None).success
        old.shutdown()
        fresh = make_protocol(code, handoff, task_id)
        handoff.mark_ready(fresh)
        release.set()
        thread.join(1)
        assert not thread.is_alive() and result[0].success
        assert not old.starts and len(fresh.starts) == 1


@pytest.mark.parametrize("task_id", [1, 2, 3])
def test_score_clock_excludes_preparation_and_repeated_start(tmp_path, monkeypatch, task_id):
    from utils import episode_success, task_scorer
    class Tap:
        def __init__(self, names):
            self.task2_state = {name: 8 for name in names}
            self._subscribers = []
        def position(self, name):
            return (0.0, 0.0, 0.0)
        def orientation(self, name):
            return (0.0, 0.0, 0.0, 1.0)
    clock = [1000.0]
    monkeypatch.setattr(task_scorer.time, "time", lambda: clock[0])
    monkeypatch.setattr(episode_success, "PoseTap", Tap)
    observer = episode_success.EpisodeSuccessObserver(task_id, str(tmp_path / "score.txt"))
    scorer = observer.scorer
    clock[0] += 500  # Long initialization must consume no scoring time.
    observer.sample()
    assert scorer.start_time is None
    assert scorer.final_score(scorer.state, clock[0])[1]["time_penalty"] == 0
    observer.start()
    assert scorer.start_time == 1500
    clock[0] += scorer.BASE_TIME_SEC + 1
    observer.start()  # Duplicate start must preserve the original clock.
    assert scorer.start_time == 1500
    components = scorer.final_score(scorer.state, clock[0])[1]
    assert components["over_sec"] == 1
    assert components["time_penalty"] == -scorer.OVER_PENALTY_PER_SEC
