"""Task 1/3 live scores must survive finalization, reset and invalidation."""
import json
from pathlib import Path

import pytest

from utils import episode_success
from utils.model_score_store import ModelScoreStore


class PoseTap:
    def __init__(self, names):
        self.positions = {name: (0.0, 0.0, 0.0) for name in names}
        self._subscribers = []
        self.task2_state = {name:8 for name in names}

    def position(self, name):
        return self.positions.get(name)

    def orientation(self, name):
        return (0.0, 0.0, 0.0, 1.0)


def read(path):
    return json.loads(Path(path).with_suffix(".json").read_text())


@pytest.mark.parametrize("task_id", [1, 3])
def test_task13_live_scores_reset_and_invalid_round(tmp_path, monkeypatch, task_id):
    monkeypatch.setattr(episode_success, "PoseTap", PoseTap)
    store = ModelScoreStore(task_id, "protocol-test", root=str(tmp_path))
    assert read(store.score_file)["status"] == "preparing"
    path = store.allocate(123)
    observer = episode_success.EpisodeSuccessObserver(
        task_id, path, store.metadata(path))
    observer.start()
    observer.sample()
    assert read(path)["total"] == 0
    assert read(path)["status"] == "running"
    scorer = observer.scorer
    if task_id == 1:
        for index, name in enumerate(scorer.CYLINDERS, 1):
            observer.tap.task2_state[name] = 7
            observer.sample()
            assert read(path)["total"] == 25 * index
        observer.tap.task2_state["source_bin"] = 16
    else:
        for index, name in enumerate(scorer.RINGS, 1):
            observer.tap.positions["base_link"] = (0.0, 0.0, 0.0)
            observer.sample()
            observer.tap.task2_state[name] = 7
            observer.tap.positions["base_link"] = (0.0, 1.10, 0.0)
            observer.sample()
            assert read(path)["total"] == (30 * index if index < 3 else 100)
    assert observer.sample()[1]
    assert observer.finish()[0] == 100
    store.annotate(path, "success")
    assert read(path)["status"] == "completed"
    assert read(store.average_file)["valid_rounds"] == 1

    store.prepare(456)
    fresh = read(path)
    assert (fresh["task_id"], fresh["round"], fresh["seed"], fresh["total"]) == (
        task_id, 2, 456, 0)
    observer.sample()
    observer.finish()
    assert read(path) == fresh  # A retiring observer cannot rewrite the new round.
    second = store.allocate(456)
    next_observer = episode_success.EpisodeSuccessObserver(
        task_id, second, store.metadata(second))
    next_observer.start()
    next_observer.sample()
    assert read(second)["total"] == 0  # No sticky score state from the old scene.
    next_observer.invalidate("test: observation infrastructure unavailable")
    invalid = read(second)
    assert invalid["status"] == "invalid"
    next_observer.sample()
    assert read(second) == invalid
    with pytest.raises(ValueError, match="unfinished or invalid"):
        store.annotate(second, "failure")
    assert read(store.average_file)["valid_rounds"] == 1
