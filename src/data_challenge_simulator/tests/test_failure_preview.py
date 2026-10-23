"""Keep the failed simulator alive until inspection finishes."""
import ast
import os
from pathlib import Path
from types import SimpleNamespace
import pytest

PACKAGE = Path(__file__).resolve().parents[1]

@pytest.mark.parametrize("success,keep", [(False, True), (False, False), (True, True)])
def test_failure_preview_precedes_cleanup(success, keep):
    module = ast.parse((PACKAGE / "examples/helperfunc.py").read_text())
    node = next(n for n in module.body if isinstance(n, ast.FunctionDef)
                and n.name == "_run_only_task")
    events = []
    proc = SimpleNamespace(wait=lambda: events.append("wait"))
    guard = SimpleNamespace(add=lambda p, **kw: p,
                            sweep=lambda: events.append("sweep"))
    def prompt(_):
        assert "sweep" not in events
        assert "finalize" in events
        events.append("inspect")
        raise EOFError()
    namespace = dict(
        os=os, print=lambda *a: None, input=prompt,
        time=SimpleNamespace(time=lambda: 10.0, sleep=lambda _: None),
        datetime=SimpleNamespace(now=lambda: "now"),
        subprocess=SimpleNamespace(Popen=lambda *a, **kw: proc, DEVNULL=None),
        rospy=SimpleNamespace(init_node=lambda *a, **kw: None),
        simulator_launch_command=lambda *a: [],
        wait_for_simulator_ready=lambda **kw: None,
        round_seed=lambda *a: 1,
        start_scorer=lambda *a: (proc, "score"),
        stop_scorer=lambda *a: events.append("finalize"),
        read_task_result=lambda _: success,
        invalidate_score=lambda *a: events.append("invalid"),
        current_score_file=lambda _: "score",
    )
    exec(compile(ast.Module(body=[node], type_ignores=[]), "helperfunc.py", "exec"), namespace)
    result = namespace["_run_only_task"](
        1, 1, False, keep, "task1.py", {}, 0, guard)
    assert result == (10.0, success)
    assert events == ["wait", "finalize"] + (
        ["inspect"] if keep and not success else []) + ["sweep"]


def test_wrapper_does_not_enable_failure_preview_implicitly():
    script = (PACKAGE.parents[1] / "run-scene1.sh").read_text()
    assert '[[ -t 0 && -t 1 ]] && helper_args+=(--keep-sim-on-failure)' not in script
    assert 'if [[ "$keep" == "1" ]]' in script
    assert "trap - INT TERM HUP" in script
