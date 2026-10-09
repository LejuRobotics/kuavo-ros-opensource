import json
import sys
from pathlib import Path


PACKAGE = Path(__file__).resolve().parents[1]
TASK = PACKAGE / "examples/task1.py"
if str(PACKAGE) not in sys.path:
    sys.path.insert(0, str(PACKAGE))


def test_recorder_keeps_multiple_events_in_jsonl(tmp_path):
    from utils.task1_grasp_diagnostics import Task1GraspDiagnosticRecorder

    output = tmp_path / "nested" / "diagnostics.jsonl"
    recorder = Task1GraspDiagnosticRecorder(seed=17, path=output)
    recorder.write("pre_close", cylinder="cylinder_1", error=[1, 2, 3])
    recorder.write("post_close", cylinder="cylinder_1", gap=0.04)

    records = [json.loads(line) for line in output.read_text().splitlines()]
    assert [record["event"] for record in records] == [
        "pre_close", "post_close"]
    assert all(record["seed"] == 17 for record in records)
    assert all(record["schema_version"] == 1 for record in records)


def test_task1_records_grasp_data_before_and_after_failure_points():
    source = TASK.read_text()
    loop = source[source.index("for index, name in enumerate(CYLINDERS)"):]

    assert '"pre_close", name, grasp_center_world' in loop
    assert '"post_close", name, grasp_center_world' in loop
    assert '"post_lift", name, grasp_center_world' in loop
    assert '"post_release", name, drop_center_world' in loop
    assert "orientation_error_deg=" in source
    assert loop.index('"pre_close", name, grasp_center_world') < loop.index(
        "gripper.control_right_gripper(GRASP_CLOSURE_CMD)")
    assert loop.index('"post_release", name, drop_center_world') < loop.index(
        "if not _fully_in_target(final_position)")
    assert "diagnostics.write(\n                \"round_end\"" in source
