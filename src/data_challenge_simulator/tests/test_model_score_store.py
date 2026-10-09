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


def test_model_session_clears_only_known_outputs_for_its_task(tmp_path):
    task_dir = tmp_path / "task2"
    task_dir.mkdir()
    for name in ("score.txt", "score.json", "average.txt", "average.json"):
        task_dir.joinpath(name).write_text("stale", encoding="utf-8")
    task_dir.joinpath("operator_note.txt").write_text("keep", encoding="utf-8")
    other_task = tmp_path / "task1"
    other_task.mkdir()
    other_task.joinpath("score.txt").write_text("keep", encoding="utf-8")

    ModelScoreStore(2, "policy_a", root=str(tmp_path))

    for name in ("score.txt", "score.json", "average.txt", "average.json"):
        assert not task_dir.joinpath(name).exists()
    assert task_dir.joinpath("operator_note.txt").read_text() == "keep"
    assert other_task.joinpath("score.txt").read_text() == "keep"


def test_only_finalized_rounds_contribute_to_session_average(tmp_path):
    store = ModelScoreStore(3, "policy_a", root=str(tmp_path), process_id=8)

    # Allocating means /simulator/start was accepted.  An infrastructure
    # failure before successful finalization still must not enter the average.
    store.allocate(10)
    assert not Path(store.average_file).exists()

    first_score = Path(store.allocate(20))
    first_score.write_text("80\n", encoding="utf-8")
    first_score.with_suffix(".json").write_text(
        json.dumps({"total": 80, "components": {}}), encoding="utf-8")
    store.annotate(str(first_score), "reset")

    second_score = Path(store.allocate(30))
    second_score.write_text("40\n", encoding="utf-8")
    second_score.with_suffix(".json").write_text(
        json.dumps({"total": 40, "components": {}}), encoding="utf-8")
    store.annotate(str(second_score), "shutdown")

    assert Path(store.average_file).read_text(encoding="utf-8") == "60.00\n"
    summary = json.loads(
        Path(store.average_json_file).read_text(encoding="utf-8"))
    assert summary["model_name"] == "policy_a"
    assert summary["task_id"] == 3
    assert summary["valid_rounds"] == 2
    assert summary["score_sum"] == 120
    assert summary["average"] == 60
    assert summary["rounds"] == [
        {
            "model_name": "policy_a",
            "session_id": store.session_id,
            "task_id": 3,
            "round": 2,
            "seed": 20,
            "finish_reason": "reset",
            "score": 80,
        },
        {
            "model_name": "policy_a",
            "session_id": store.session_id,
            "task_id": 3,
            "round": 3,
            "seed": 30,
            "finish_reason": "shutdown",
            "score": 40,
        },
    ]


def test_one_round_cannot_be_added_to_the_average_twice(tmp_path):
    store = ModelScoreStore(1, "policy_a", root=str(tmp_path))
    score_file = Path(store.allocate(42))
    score_file.write_text("25\n", encoding="utf-8")
    score_file.with_suffix(".json").write_text(
        json.dumps({"total": 25, "components": {}}), encoding="utf-8")

    store.annotate(str(score_file), "success")
    with pytest.raises(ValueError, match="already finalized"):
        store.annotate(str(score_file), "reset")

    summary = json.loads(
        Path(store.average_json_file).read_text(encoding="utf-8"))
    assert summary["valid_rounds"] == 1
    assert summary["average"] == 25


@pytest.mark.parametrize("name", ["", "../other", "a/b", "white space"])
def test_model_name_rejects_paths_and_unsafe_labels(name):
    with pytest.raises(ValueError):
        validate_model_name(name)
