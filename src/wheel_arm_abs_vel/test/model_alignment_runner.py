#!/usr/bin/env python3
"""Use the launch resolver for models whose private overlay is partial."""
import json
from pathlib import Path
import subprocess
import sys
import tempfile

package = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(package / 'scripts'))
from tools.full_model_contract import resolve_model

binary, version = sys.argv[1:]
assets = package.parent / 'kuavo_assets'
wheel = package.parent / 'humanoid-wheel-control/humanoid_wheel_interface'
overlay = json.loads((package / ('config/model_s' + version + '.json')).read_text())
contract = resolve_model(version, assets, wheel, overlay)
with tempfile.TemporaryDirectory() as directory:
    path = Path(directory) / 'contract.json'
    path.write_text(json.dumps(contract))
    subprocess.run([binary, str(assets), str(path), version], check=True)
