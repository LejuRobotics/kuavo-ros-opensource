"""Repeated stop requests must not interrupt Python's final ROS cleanup."""
import ast
from pathlib import Path
import signal
import subprocess
import sys

ENTRY = Path(__file__).resolve().parents[1] / "scripts/model_entry.py"


def test_repeated_stop_signals_allow_atexit_cleanup_to_finish():
    source = ENTRY.read_text()
    tree = ast.parse(source)
    nodes = [node for node in tree.body
             if isinstance(node, ast.FunctionDef) and node.name == "request_shutdown"]
    nodes.append(tree.body[-1])  # Execute the real command-line exit handling.
    bootstrap = """
import atexit, signal, sys, time
def cleanup():
    print('ATEXIT', flush=True)
    time.sleep(.3)
    print('CLEANUP_COMPLETE', flush=True)
atexit.register(cleanup)
def main():
    signal.signal(signal.SIGTERM, request_shutdown)
    print('READY', flush=True)
    signal.pause()
"""
    code = bootstrap + "\n".join(ast.get_source_segment(source, node) for node in nodes)
    child = subprocess.Popen([sys.executable, "-u", "-c", code],
                             stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    try:
        assert child.stdout.readline().strip() == "READY"
        child.send_signal(signal.SIGTERM)
        assert child.stdout.readline().strip() == "ATEXIT"
        child.send_signal(signal.SIGTERM)
        child.send_signal(signal.SIGINT)
        output, errors = child.communicate(timeout=5)
        assert "CLEANUP_COMPLETE" in output
        assert not errors, errors
        assert child.returncode == 130
    finally:
        if child.poll() is None:
            child.kill()
            child.wait()
