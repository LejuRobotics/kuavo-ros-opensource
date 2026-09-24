from pathlib import Path
import sys

import pytest


PACKAGE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE))

from utils.collect_score_store import (  # noqa: E402
    allocate_collect_score_file,
)


def test_collect_scores_are_separated_into_three_task_directories(tmp_path):
    paths = {
        task_id: Path(allocate_collect_score_file(tmp_path, task_id, 7))
        for task_id in (1, 2, 3)
    }

    assert paths[1] == tmp_path / "task1" / "score_task1_round7.txt"
    assert paths[2] == tmp_path / "task2" / "score_task2_round7.txt"
    assert paths[3] == tmp_path / "task3" / "score_task3_round7.txt"
    assert all(path.exists() for path in paths.values())


def test_repeated_collect_round_never_overwrites_existing_pair(tmp_path):
    first = Path(allocate_collect_score_file(tmp_path, 2, 19))
    first.write_text("81\n", encoding="utf-8")
    first.with_suffix(".json").write_text("{}", encoding="utf-8")

    second = Path(allocate_collect_score_file(tmp_path, 2, 19))
    second.write_text("92\n", encoding="utf-8")
    third = Path(allocate_collect_score_file(tmp_path, 2, 19))

    assert second.name == "score_task2_round19_2.txt"
    assert third.name == "score_task2_round19_3.txt"
    assert first.read_text(encoding="utf-8") == "81\n"
    assert second.read_text(encoding="utf-8") == "92\n"


def test_orphan_json_also_blocks_reuse_of_a_score_name(tmp_path):
    task_dir = tmp_path / "task1"
    task_dir.mkdir()
    (task_dir / "score_task1_round4.json").write_text(
        "{}", encoding="utf-8")

    allocated = Path(allocate_collect_score_file(tmp_path, 1, 4))

    assert allocated.name == "score_task1_round4_2.txt"


@pytest.mark.parametrize("task_id", [0, 4])
def test_collect_score_rejects_unknown_task(tmp_path, task_id):
    with pytest.raises(ValueError):
        allocate_collect_score_file(tmp_path, task_id, 1)
