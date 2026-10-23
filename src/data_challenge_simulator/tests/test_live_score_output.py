import json
import threading
from pathlib import Path
from types import SimpleNamespace

import pytest
from utils import episode_success, task_scorer
from utils.model_score_store import ModelScoreStore
from utils.score_output import write_score


def read(path):
    return json.loads(Path(path).with_suffix(".json").read_text())


class Tap:
    def __init__(self, names):
        self.task2_state = {"box_1": 8, "box_2": 8}
        self._subscribers = []


def test_model_live_score_reset_finalization_and_average(tmp_path, monkeypatch):
    monkeypatch.setattr(episode_success, "PoseTap", Tap)
    clock = [100.0]
    monkeypatch.setattr(task_scorer.time, "time", lambda: clock[0])
    store = ModelScoreStore(2, "test", root=str(tmp_path))
    assert read(store.score_file)["status"] == "preparing"
    assert read(store.score_file)["total"] == 0
    path = store.allocate(5)
    observer = episode_success.EpisodeSuccessObserver(2, path, store.metadata(path))
    observer.start()
    observer.sample()
    for flags, score in [(1,10), (2,30), (4,45)]:
        observer.tap.task2_state["box_1"] = flags
        observer.sample()
        payload = read(path)
        assert payload["status"] == "running" and payload["total"] == score
        assert "model_run" not in payload  # External collector must not finalize it.
    clock[0] = 146  # 46 seconds elapsed: one second beyond Task2 grace period.
    observer.sample()
    assert read(path)["total"] == 44
    observer.tap.task2_state["box_2"] = 1
    observer.sample()
    observer.tap.task2_state["box_2"] = 2 | 4
    assert observer.sample()[1]
    assert observer.finish()[0] == 99
    store.annotate(path, "success")
    assert read(path)["status"] == "completed"
    assert read(store.average_file)["valid_rounds"] == 1
    store.prepare(3)
    snapshot = read(path)
    assert (snapshot["round"], snapshot["seed"], snapshot["total"]) == (2,3,0)
    clock[0] = 200
    observer.sample()  # Old observer may still be closing; cannot overwrite round 2.
    observer.finish()
    assert read(path) == snapshot
    path2 = store.allocate(3)
    next_observer = episode_success.EpisodeSuccessObserver(2, path2, store.metadata(path2))
    next_observer.start()
    assert next_observer.sample()[0]["lifted"] == []
    assert read(path2)["total"] == 0
    next_observer.invalidate("lost simulator")
    store.invalidate("lost simulator")
    invalid = read(path2)
    next_observer.sample()
    assert read(path2) == invalid
    with pytest.raises(RuntimeError, match="invalid"):
        next_observer.finish()
    with pytest.raises(ValueError, match="unfinished or invalid"):
        store.annotate(path2, "shutdown")
    assert read(store.average_file)["valid_rounds"] == 1


def test_snapshots_are_parseable_during_replacement(tmp_path):
    path = str(tmp_path / "score.txt")
    latest = str(tmp_path / "current.txt")
    write_score(path, {"total":0, "status":"preparing", "components":{}}, latest)
    stop = threading.Event()
    failures = []
    def reader():
        while not stop.is_set():
            try:
                for name in (path, latest):
                    assert isinstance(read(name)["total"], int)
                    int(Path(name).read_text())
            except Exception as error:
                failures.append(error)
    thread = threading.Thread(target=reader)
    thread.start()
    try:
        for score in range(100):
            write_score(path, {"total":score,"status":"running","components":{}}, latest)
    finally:
        stop.set()
        thread.join()
    assert not failures
    assert read(path) == read(latest)
    for name in (path, latest):
        assert Path(name).stat().st_mode & 0o777 == 0o644
        assert Path(name).with_suffix(".json").stat().st_mode & 0o777 == 0o644
