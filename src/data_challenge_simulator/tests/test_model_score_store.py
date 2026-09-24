import json
from datetime import datetime
from pathlib import Path
import sys

import pytest


PACKAGE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE))

from utils.model_score_store import (  # noqa: E402
    ModelScoreStore,
    validate_model_name,
)


def test_model_scores_use_fixed_task_paths_and_overwrite_each_round(tmp_path):
    store = ModelScoreStore(
        2, "policy_a", root=str(tmp_path),
        started_at=datetime(2026, 9, 22, 14, 5, 6), process_id=123)

    first = Path(store.allocate(41))
    second = Path(store.allocate(42))

    assert first == tmp_path.joinpath("task2", "score.txt")
    assert second == first
    assert first.parent == tmp_path.joinpath("task2")


def test_each_store_uses_the_same_task_directory(tmp_path):
    started_at = datetime(2026, 9, 22, 14, 5, 6)
    first = ModelScoreStore(
        3, "policy_a", root=str(tmp_path),
        started_at=started_at, process_id=123)
    second = ModelScoreStore(
        3, "policy_a", root=str(tmp_path),
        started_at=started_at, process_id=123)

    assert first.session_id == "20260922T140506_pid123"
    assert second.session_id == "20260922T140506_pid123"
    assert first.session_dir == second.session_dir == str(tmp_path / "task3")


def test_model_score_json_gets_run_identity(tmp_path):
    store = ModelScoreStore(1, "policy-b", root=str(tmp_path), process_id=7)
    score_file = Path(store.allocate(314))
    json_file = score_file.with_suffix(".json")
    json_file.write_text(
        json.dumps({"total": 88, "components": {"grasp": 30}}),
        encoding="utf-8")

    assert Path(store.annotate(str(score_file), "success")) == json_file
    payload = json.loads(json_file.read_text(encoding="utf-8"))
    assert payload["total"] == 88
    assert payload["model_run"] == {
        "model_name": "policy-b",
        "session_id": store.session_id,
        "task_id": 1,
        "round": 1,
        "seed": 314,
        "finish_reason": "success",
    }


@pytest.mark.parametrize("name", ["", "../other", "a/b", "white space"])
def test_model_name_rejects_paths_and_unsafe_labels(name):
    with pytest.raises(ValueError):
        validate_model_name(name)
