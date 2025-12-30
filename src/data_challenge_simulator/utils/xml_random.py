#!/usr/bin/env python3
"""
MJCF / MuJoCo XML randomizer (importable module)

- Can be imported and called from other files: randomize_mjcf(in_path, out_path, config=DEFAULT_CONFIG, seed=None)
- Also supports CLI usage (args are optional).
- Config is a Python dict (DEFAULT_CONFIG shown below). You may still pass an external dict.
- No automatic backups: output simply overwrites the specified out_path.

Example (from another file):

from mjcf_randomizer import randomize_mjcf, DEFAULT_CONFIG
cfg = DEFAULT_CONFIG.copy()
# ...modify cfg['rules'] as needed...
randomize_mjcf('scene.xml', '/tmp/scene_random.xml', config=cfg, seed=123)

"""
from __future__ import annotations
import argparse
import json
import os
import random
import sys
from typing import Any, Dict, List, Sequence, Tuple, Union
import xml.etree.ElementTree as ET

Number = Union[int, float]

# -------------------------- DEFAULT CONFIG --------------------------
DEFAULT_CONFIG: Dict[str, Any] = {
    "rules": [
        {
            "select": "//body[@name='box_grab']",
            "attributes": {
                "pos": {
                    "per_dim": [
                        {"uniform": [0.7, 1.3]},
                        {"uniform": [0.45, 0.62]},
                        {"set": 0.95}
                    ]
                }
            }
        },
        {
            "select": "//geom[@name='box_on_belt']",
            "attributes": {
                "rgba": {"color": {"alpha": [1.0, 1.0]}}
            }
        },
        {
            "select": "//geom[@name='marker1']",
            "attributes": {
                "rgba": {"choice": ["1 0 0 1", "0 1 0 1", "0 0 1 1", "1 1 0 1"]}
            }
        },
        {
            "select": "//geom[@name='marker2']",
            "attributes": {
                "rgba": {"choice": ["1 0 0 1", "0 1 0 1", "0 0 1 1", "1 1 0 1"]}
            }
        },
        {
            "select": "//actuator[@name='belt_speed']",
            "attributes": {
                "ctrlrange": {"set": [-0.15, 0.15]}
            }
        },
        {
            "select": "//visual/headlight",
            "attributes": {
                "ambient": {"set": [-0.3, -0.3, -0.3]},
                "diffuse": {"jitter_pct": 0.25, "clamp": [0.0, 1.0]}
            }
        }
    ]
}

# -------------------------- Helpers --------------------------

def _to_float_list(val: Union[str, Number, Sequence[Number]] | None) -> List[float] | None:
    if val is None:
        return None
    if isinstance(val, (int, float)):
        return [float(val)]
    if isinstance(val, str):
        parts = val.strip().split()
        out: List[float] = []
        for p in parts:
            if not p:
                continue
            try:
                out.append(float(p))
            except ValueError:
                return []
        return out
    return [float(x) for x in val]


def _fmt_value(v: Union[str, Number, Sequence[Number]]) -> str:
    if isinstance(v, (int, float)):
        return f"{float(v):.10g}"
    if isinstance(v, str):
        return v
    return " ".join(f"{float(x):.10g}" for x in v)


def rand_uniform(a: Number, b: Number) -> float:
    return random.uniform(float(a), float(b))


def rand_color(alpha_range: Tuple[float, float] | None = None) -> List[float]:
    r, g, b = random.random(), random.random(), random.random()
    a = 1.0 if alpha_range is None else rand_uniform(alpha_range[0], alpha_range[1])
    return [r, g, b, a]


def apply_jitter(values: List[float], spec: Dict[str, Any]) -> List[float]:
    out = []
    clamp = spec.get("clamp")
    for v in values:
        v2 = v
        if "jitter" in spec:
            v2 = v + float(spec["jitter"]) * (2 * random.random() - 1)
        if "jitter_pct" in spec:
            v2 = v + float(spec["jitter_pct"]) * v * (2 * random.random() - 1)
        if clamp is not None:
            lo, hi = float(clamp[0]), float(clamp[1])
            v2 = max(lo, min(hi, v2))
        out.append(v2)
    return out


def gen_from_spec(existing: List[float] | None, spec: Any) -> Union[str, List[float]]:
    if isinstance(spec, (int, float, str)):
        return [float(spec)] if not isinstance(spec, str) else spec
    if not isinstance(spec, dict):
        raise ValueError(f"Invalid spec: {spec}")

    if "set" in spec:
        val = spec["set"]
        if isinstance(val, (list, tuple)):
            return [float(x) for x in val]
        if isinstance(val, (int, float)):
            return [float(val)]
        if isinstance(val, str):
            return val

    if "choice" in spec:
        choice_list = spec["choice"]
        ch = random.choice(choice_list)
        if isinstance(ch, str):
            return ch
        if isinstance(ch, (int, float)):
            return [float(ch)]
        return [float(x) for x in ch]

    if "color" in spec:
        color_spec = spec["color"]
        alpha = None
        if isinstance(color_spec, dict) and "alpha" in color_spec:
            ar = color_spec["alpha"]
            if isinstance(ar, (list, tuple)) and len(ar) == 2:
                alpha = (float(ar[0]), float(ar[1]))
        return rand_color(alpha)

    if "uniform" in spec and (
        not isinstance(spec["uniform"], list)
        or (isinstance(spec["uniform"], list) and len(spec["uniform"]) == 2 and not any(isinstance(x, (list, tuple)) for x in spec["uniform"]))
    ):
        a, b = spec["uniform"]
        ndim = int(spec.get("ndim", 1))
        return [rand_uniform(a, b) for _ in range(ndim)]

    if "per_dim" in spec:
        out: List[float] = []
        for dspec in spec["per_dim"]:
            val = gen_from_spec(None, dspec)
            if isinstance(val, list):
                out.extend(val)
            elif isinstance(val, str):
                fl = _to_float_list(val)
                if not fl:
                    raise ValueError("per_dim produced non-numeric string")
                out.extend(fl)
        return out

    if ("jitter" in spec or "jitter_pct" in spec) and existing is not None:
        return apply_jitter(existing, spec)

    raise ValueError(f"Unsupported spec: {spec}")


def _safe_findall(root: ET.Element, selector: str):
    """ElementTree's XPath is limited. Ensure relative search and give clear errors.
    - If selector starts with '//' (absolute), convert to './/' for relative search.
    - Note: functions like starts-with() are NOT supported by xml.etree; use Python-side filtering instead.
    """
    sel = selector.strip()
    if sel.startswith("//"):
        sel = "." + sel  # make it relative
    try:
        return root.findall(sel)
    except SyntaxError as e:
        raise SyntaxError(
            f"XPath syntax not supported by xml.etree: '{selector}'. "
            "Try making it relative (prefix with './/') and avoid functions like starts-with()."
        ) from e


def apply_rules(tree: ET.ElementTree, rules: List[Dict[str, Any]]) -> int:
    root = tree.getroot()
    changed = 0

    for rule in rules:
        select = rule.get("select")
        attrs: Dict[str, Any] = rule.get("attributes", {})
        if not select or not attrs:
            continue

        nodes = _safe_findall(root, select)
        if not nodes:
            continue

        for node in nodes:
            for attr, spec in attrs.items():
                old_text = node.get(attr)
                existing = _to_float_list(old_text)
                try:
                    new_val = gen_from_spec(existing, spec)
                except Exception as e:
                    print(f"[WARN] {node.tag} name={node.get('name')} attr={attr}: {e}")
                    continue
                node.set(attr, _fmt_value(new_val))
                changed += 1

    return changed


def randomize_mjcf(in_path: str, out_path: str, config: Dict[str, Any] | None = None, seed: int | None = None) -> int:
    if seed is not None:
        random.seed(seed)
    cfg = DEFAULT_CONFIG if config is None else config
    rules = cfg.get("rules", [])
    if not isinstance(rules, list) or not rules:
        raise ValueError("config['rules'] must be a non-empty list")

    tree = ET.parse(in_path)
    n = apply_rules(tree, rules)
    tree.write(out_path, encoding="utf-8", xml_declaration=True)
    return n


def _parse_args(argv: List[str]) -> argparse.Namespace:
    ap = argparse.ArgumentParser(description="Randomize attributes in an MJCF/MuJoCo XML using DEFAULT_CONFIG or external JSON")
    ap.add_argument("--in", dest="in_path", help="Input MJCF/XML path")
    ap.add_argument("--out", dest="out_path", help="Output XML path")
    ap.add_argument("--seed", dest="seed", type=int, default=None, help="Random seed")
    ap.add_argument("--config_json", dest="config_json", default=None, help="Path to JSON config (overrides DEFAULT_CONFIG)")
    return ap.parse_args(argv)


def _load_json(path: str) -> Dict[str, Any]:
    with open(path, "r", encoding="utf-8") as f:
        return json.load(f)


def main(argv: List[str] | None = None):
    ns = _parse_args(sys.argv[1:] if argv is None else argv)
    if not ns.in_path or not ns.out_path:
        print("Usage: --in INPUT.xml --out OUTPUT.xml [--seed 123] [--config_json rules.json]")
        sys.exit(2)
    cfg = DEFAULT_CONFIG
    if ns.config_json:
        cfg = _load_json(ns.config_json)
    n = randomize_mjcf(ns.in_path, ns.out_path, config=cfg, seed=ns.seed)
    print(f"Randomization complete. {n} attributes changed.\nSaved: {ns.out_path}")


if __name__ == "__main__":
    main()
