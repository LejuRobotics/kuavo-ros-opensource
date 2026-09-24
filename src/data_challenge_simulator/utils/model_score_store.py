#!/usr/bin/env python3
"""Allocate the fixed model score files for one task."""

import json
import os
import re
from datetime import datetime


PACKAGE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DEFAULT_MODEL_SCORES_DIR = os.path.join(
    PACKAGE_DIR, "examples", "model_scores")
MODEL_NAME_PATTERN = re.compile(r"^[A-Za-z0-9][A-Za-z0-9_.-]{0,63}$")


def validate_model_name(value):
    """Return a filesystem-safe model label or raise a user-facing error."""
    if not MODEL_NAME_PATTERN.fullmatch(value or ""):
        raise ValueError(
            "model name must be 1-64 characters using only letters, digits, "
            "dot, underscore or hyphen, and must start with a letter or digit")
    return value


class ModelScoreStore(object):
    """Own the score paths shared by all rounds of one task."""

    def __init__(self, task_id, model_name="anonymous", root=None,
                 started_at=None, process_id=None):
        if task_id not in (1, 2, 3):
            raise ValueError("unsupported task id: {}".format(task_id))
        self.task_id = int(task_id)
        self.model_name = validate_model_name(model_name)
        self.started_at = started_at or datetime.now()
        self.process_id = os.getpid() if process_id is None else int(process_id)
        base_session_id = "{}_pid{}".format(
            self.started_at.strftime("%Y%m%dT%H%M%S"), self.process_id)
        self.session_id = base_session_id
        self.session_dir = os.path.join(
            root or DEFAULT_MODEL_SCORES_DIR, "task{}".format(self.task_id))
        os.makedirs(self.session_dir, exist_ok=True)
        self.score_file = os.path.join(self.session_dir, "score.txt")
        self._round = 0
        self._records = {}

    def allocate(self, seed):
        """Return the fixed score path; each completed round overwrites it."""
        self._round += 1
        score_file = self.score_file
        self._records[score_file] = {
            "model_name": self.model_name,
            "session_id": self.session_id,
            "task_id": self.task_id,
            "round": self._round,
            "seed": int(seed),
        }
        return score_file

    def annotate(self, score_file, finish_reason):
        """Add model/session identity to the scorer's JSON component file."""
        metadata = self._records.get(score_file)
        if metadata is None:
            raise ValueError("score file was not allocated by this session")
        base, _ = os.path.splitext(score_file)
        json_file = base + ".json"
        with open(json_file, "r", encoding="utf-8") as stream:
            payload = json.load(stream)
        model_run = dict(metadata)
        model_run["finish_reason"] = str(finish_reason)
        payload["model_run"] = model_run
        with open(json_file, "w", encoding="utf-8") as stream:
            json.dump(payload, stream, ensure_ascii=False, indent=2)
        return json_file
