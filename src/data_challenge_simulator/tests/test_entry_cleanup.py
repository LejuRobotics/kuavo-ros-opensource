"""Host wrapper cleanup, using a fake Docker client only."""
from pathlib import Path
import subprocess
import pytest

REPO = Path(__file__).resolve().parents[3]


@pytest.mark.parametrize("client_status", [0, 7])
def test_client_failure_runs_cleanup_and_preserves_exit_code(client_status):
    source = (REPO / "run-scene1.sh").read_text()
    function = source[source.index("run_container_entry() {"):
                      source.index("# Patterns for every process")]
    script = function + """
docker() { return %d; }
stop_task() { echo cleaned; }
run_container_entry fake
""" % client_status
    result = subprocess.run(["bash", "-c", script], capture_output=True, text=True)
    assert result.returncode == client_status
    assert result.stdout.strip() == ("cleaned" if client_status else "")
