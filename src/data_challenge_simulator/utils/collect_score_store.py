#!/usr/bin/env python3
"""Allocate non-overwriting score files for recorded collect rounds."""

import os


def allocate_collect_score_file(root, task_id, round_id):
    """Reserve and return one score path below this task's directory.

    The TXT path is created exclusively before the scorer starts.  Therefore
    another collect process, a repeated seed, or a previous interrupted run
    can never make the scorer overwrite an existing TXT/JSON result pair.
    """
    task_id = int(task_id)
    round_id = int(round_id)
    if task_id not in (1, 2, 3):
        raise ValueError("unsupported task id: {}".format(task_id))
    if round_id < 0:
        raise ValueError("round id must be non-negative: {}".format(round_id))

    task_dir = os.path.join(root, "task{}".format(task_id))
    os.makedirs(task_dir, exist_ok=True)
    stem = "score_task{}_round{}".format(task_id, round_id)
    suffix = 1
    while True:
        candidate_stem = stem if suffix == 1 else "{}_{}".format(stem, suffix)
        score_file = os.path.join(task_dir, candidate_stem + ".txt")
        json_file = os.path.join(task_dir, candidate_stem + ".json")
        if os.path.exists(json_file):
            suffix += 1
            continue
        try:
            descriptor = os.open(
                score_file, os.O_CREAT | os.O_EXCL | os.O_WRONLY, 0o644)
        except FileExistsError:
            suffix += 1
            continue
        os.close(descriptor)
        return score_file
