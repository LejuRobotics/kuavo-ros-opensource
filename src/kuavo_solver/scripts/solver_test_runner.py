#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
kuavo_solver 全版本回归测试 Runner。

默认从 solver_version_index.yaml 生成 **按 (spec_module, token) 去重** 的 canonical 矩阵，
运行 position_roundtrip / jacobian_check，汇总 JSON/HTML 报告。

用法:
    # 列出 canonical 矩阵（不执行）
    python3 solver_test_runner.py --list-matrix

    # L1 Smoke（去重矩阵，5 samples，仅位置）
    python3 solver_test_runner.py --smoke

    # L2 Full
    python3 solver_test_runner.py --all --samples 50 --output-dir /tmp/solver_test_results

    # 单版本单模块（使用该 version 字符串 resolve，不必是 representative）
    python3 solver_test_runner.py --version s70 --module ankle --samples 50

    # 按 token 筛选
    python3 solver_test_runner.py --token 7gen --all

    # 全 index 别名矩阵（审计映射，不去重）
    python3 solver_test_runner.py --by-version --all
"""

from __future__ import annotations

import argparse
import csv
import json
import os
import sys
import time
from dataclasses import dataclass
from typing import Any, Dict, List, Optional, Tuple

import numpy as np

# 与 kuavo_solver_validator.py 一致：lib / validation 子目录平铺导入
_SCRIPTS_ROOT = os.path.dirname(os.path.abspath(__file__))
for _sub in ("lib", "validation"):
    _p = os.path.join(_SCRIPTS_ROOT, _sub)
    if _p not in sys.path:
        sys.path.insert(0, _p)
if _SCRIPTS_ROOT not in sys.path:
    sys.path.insert(0, _SCRIPTS_ROOT)

from solver_selection import load_default_index
from solver_validation_spec import build_spec
from solver_validation_runner import run_jacobian_check, run_position_roundtrip
from validation_common import StrictThreshold, format_conclusion, pass_threshold, vector_err


def _setup_solver_path() -> None:
    """与 kuavo_solver_validator._setup_solver_path 一致：自动发现 kuavo_solver_py。"""
    try:
        import kuavo_solver_py as _  # noqa: F401
        return
    except Exception:
        pass

    def _add(p: str) -> None:
        if p and os.path.isdir(p) and p not in sys.path:
            sys.path.insert(0, p)

    KUAVO_SOLVER_ROOT = os.path.abspath(os.path.join(_SCRIPTS_ROOT, ".."))
    REPO_ROOT = os.path.abspath(os.path.join(KUAVO_SOLVER_ROOT, "..", ".."))
    _add(os.path.join(REPO_ROOT, "build", "kuavo_solver", "python"))
    for base in ("devel", "install"):
        root = os.path.join(REPO_ROOT, base)
        for rel in (
            "lib/python3/dist-packages",
            "lib/python3.8/dist-packages",
            "lib/python3.10/dist-packages",
            "lib/python3/site-packages",
            "lib/python3.8/site-packages",
            "lib/python3.10/site-packages",
        ):
            _add(os.path.join(root, rel))

    max_depth = 6
    for start in (os.path.join(REPO_ROOT, "build"), REPO_ROOT):
        if not os.path.isdir(start):
            continue
        for cur, dirs, _files in os.walk(start):
            depth = cur[len(start):].count(os.sep)
            if depth > max_depth:
                dirs[:] = []
                continue
            if os.path.basename(cur) not in ("site-packages", "dist-packages"):
                continue
            try:
                entries = os.listdir(cur)
            except Exception:
                continue
            if any(e.startswith("kuavo_solver_py") for e in entries):
                _add(cur)


def _load_solver_module() -> Any:
    _setup_solver_path()
    try:
        import kuavo_solver_py  # type: ignore

        return kuavo_solver_py
    except ImportError:
        print("错误：无法导入 kuavo_solver_py，请先 catkin build kuavo_solver 或 source installed/setup.bash")
        sys.exit(1)

# ---------------------------------------------------------------------------
# 测试矩阵生成
# ---------------------------------------------------------------------------

# index module → 可验证的 spec module 名列表
_MODULE_TO_SPEC_MODULES = {
    "ankle": ["ankle", "knee"],
    "arm": ["arm_elbow", "arm_wrist"],
    "waist": ["waist"],
}

_SKIP_TOKENS = {"none", "disabled"}

# 仅这些 token 有独立的膝 MJCF（5gen/5gen_2/7gen 的 4-bar knee；定轴踝无膝机构）
_KNEE_TOKENS = {"4gen_pro", "5gen", "5gen_2", "7gen"}

@dataclass(frozen=True)
class CanonicalCase:
    """去重后的唯一测试单元：(spec_module, token) + 代表 version。"""

    spec_module: str
    token: str
    representative_version: str
    alias_versions: Tuple[str, ...]

    @property
    def canonical_key(self) -> str:
        return f"{self.spec_module}:{self.token}"

    def version_for_run(self, override: Optional[str] = None) -> str:
        return override if override else self.representative_version


@dataclass
class TestCase:
    """单次执行用例（version 为 build_spec 实际传入的外部版本字符串）。"""

    version: str
    module: str
    token: str
    canonical_key: str
    representative_version: str


def _version_priority(version_key: str, entry: Dict[str, Any]) -> Tuple[int, int, int, str]:
    """
    越小越优先选为 representative_version：
    1) 含 modules 的多模块条目优先
    2) 非纯数字 key 优先（如 s70、7gen 优于 100045）
    3) key 更短优先
    """
    has_modules = 0 if "modules" in entry else 1
    is_digit_key = 1 if version_key.isdigit() else 0
    return (has_modules, is_digit_key, len(version_key), version_key)


def _expand_index_rows(
    idx: Any, *, filter_token: Optional[str] = None
) -> List[Tuple[str, str, str]]:
    """Yield (version_key, spec_module, token) from index."""
    rows: List[Tuple[str, str, str]] = []
    for version_key, entry in idx.entries.items():
        if "modules" not in entry:
            module = str(entry.get("module", "")).strip()
            token = str(entry.get("token", "")).strip()
            if token.lower() in _SKIP_TOKENS:
                continue
            if filter_token and token != filter_token:
                continue
            for sm in _MODULE_TO_SPEC_MODULES.get(module, [module]):
                if sm == "knee" and token not in _KNEE_TOKENS:
                    continue
                rows.append((version_key, sm, token))
            continue

        modules = entry.get("modules") or {}
        for mod_name, mod_cfg in modules.items():
            if not isinstance(mod_cfg, dict):
                continue
            token = str(mod_cfg.get("token", "")).strip()
            if token.lower() in _SKIP_TOKENS:
                continue
            if filter_token and token != filter_token:
                continue
            for sm in _MODULE_TO_SPEC_MODULES.get(mod_name, [mod_name]):
                if sm == "knee" and token not in _KNEE_TOKENS:
                    continue
                rows.append((version_key, sm, token))
    return rows


def build_canonical_matrix(*, filter_token: Optional[str] = None) -> List[CanonicalCase]:
    """
    按 (spec_module, token) 去重；每个 canonical case 选一个 representative_version。
    """
    idx = load_default_index()
    buckets: Dict[Tuple[str, str], List[Tuple[str, Dict[str, Any]]]] = {}

    for version_key, spec_module, token in _expand_index_rows(idx, filter_token=filter_token):
        key = (spec_module, token)
        entry = idx.entries[version_key]
        buckets.setdefault(key, []).append((version_key, entry))

    cases: List[CanonicalCase] = []
    for (spec_module, token), candidates in sorted(buckets.items()):
        best_vk = min(candidates, key=lambda x: _version_priority(x[0], x[1]))[0]
        aliases = tuple(sorted({vk for vk, _ in candidates}))
        cases.append(
            CanonicalCase(
                spec_module=spec_module,
                token=token,
                representative_version=best_vk,
                alias_versions=aliases,
            )
        )
    return cases


def canonical_to_test_cases(
    canonical: List[CanonicalCase],
    *,
    version_override: Optional[str] = None,
) -> List[TestCase]:
    out: List[TestCase] = []
    for c in canonical:
        ver = c.version_for_run(version_override)
        out.append(
            TestCase(
                version=ver,
                module=c.spec_module,
                token=c.token,
                canonical_key=c.canonical_key,
                representative_version=c.representative_version,
            )
        )
    return out


def build_test_matrix(*, filter_token: Optional[str] = None) -> List[TestCase]:
    """全 index 别名矩阵（不去重），供 --by-version 审计。"""
    idx = load_default_index()
    cases: List[TestCase] = []
    for version_key, spec_module, token in _expand_index_rows(idx, filter_token=filter_token):
        ck = f"{spec_module}:{token}"
        cases.append(
            TestCase(
                version=version_key,
                module=spec_module,
                token=token,
                canonical_key=ck,
                representative_version=version_key,
            )
        )
    return cases


def filter_cases(
    cases: List[TestCase],
    *,
    filter_version: Optional[str] = None,
    filter_module: Optional[str] = None,
    canonical: Optional[List[CanonicalCase]] = None,
) -> List[TestCase]:
    if filter_module:
        cases = [c for c in cases if c.module == filter_module]
    if not filter_version:
        return cases

    if canonical is not None:
        by_key = {c.canonical_key: c for c in canonical}
        out: List[TestCase] = []
        for tc in cases:
            cc = by_key.get(tc.canonical_key)
            if cc is None:
                continue
            if filter_version not in cc.alias_versions:
                continue
            out.append(
                TestCase(
                    version=filter_version,
                    module=tc.module,
                    token=tc.token,
                    canonical_key=tc.canonical_key,
                    representative_version=tc.representative_version,
                )
            )
        return out

    return [
        TestCase(
            version=filter_version,
            module=c.module,
            token=c.token,
            canonical_key=c.canonical_key,
            representative_version=c.representative_version,
        )
        for c in cases
        if c.version == filter_version
    ]


def print_matrix_list(cases: List[TestCase], *, by_version: bool) -> None:
    mode = "by-version (全别名)" if by_version else "canonical (去重)"
    print(f"矩阵模式: {mode}  共 {len(cases)} 条执行项")
    print(f"{'canonical_key':<24} {'version':<16} {'module':<12} {'token':<12} {'repr_version':<16}")
    print("-" * 88)
    for c in cases:
        repr_v = c.representative_version if c.representative_version != c.version else c.representative_version
        print(
            f"{c.canonical_key:<24} {c.version:<16} {c.module:<12} {c.token:<12} {repr_v:<16}"
        )


def preflight_check(solver_module: Any) -> None:
    """L0：import + 对 canonical 矩阵全量检查 MJCF 存在性及 solver 构造。"""
    cases = build_canonical_matrix()
    if not cases:
        raise RuntimeError("canonical 矩阵为空")
    missing: List[str] = []
    failed: List[str] = []
    for c in cases:
        try:
            spec = build_spec(module=c.spec_module, version=c.representative_version, solver_module=solver_module)
            if not os.path.isfile(spec.mjcf_path):
                missing.append(f"{c.canonical_key}: {spec.mjcf_path}")
            spec.construct_solver(solver_module)
        except Exception as e:
            failed.append(f"{c.canonical_key}: {e}")
    if missing:
        raise FileNotFoundError(f"preflight MJCF 缺失 ({len(missing)}):\n  " + "\n  ".join(missing))
    if failed:
        raise RuntimeError(f"preflight solver 构造失败 ({len(failed)}):\n  " + "\n  ".join(failed))
    print(f"[PREFLIGHT] {len(cases)} canonical cases OK")


# ---------------------------------------------------------------------------
# Runner
# ---------------------------------------------------------------------------


@dataclass
class TestResult:
    case: TestCase
    test_type: str
    passed: bool
    summary: Dict[str, Any]
    worst_cases: List[Dict[str, Any]]
    duration_sec: float
    error: Optional[str] = None


class SolverTestRunner:
    def __init__(
        self,
        solver_module: Optional[Any] = None,
        threshold: StrictThreshold = StrictThreshold(),
    ):
        self.solver_module = solver_module if solver_module is not None else _load_solver_module()
        self.threshold = threshold

    def run_single(self, case: TestCase, test_type: str, samples: int = 50, seed: int = 42) -> TestResult:
        t0 = time.time()
        try:
            spec = build_spec(module=case.module, version=case.version, solver_module=self.solver_module)
            solver = spec.construct_solver(self.solver_module)
            if test_type == "position_roundtrip":
                result = run_position_roundtrip(
                    spec=spec, solver=solver, samples=samples, seed=seed, threshold=self.threshold
                )
            elif test_type == "jacobian":
                result = run_jacobian_check(
                    spec=spec, solver=solver, samples=samples, seed=seed, threshold=self.threshold
                )
            else:
                raise ValueError(f"未知 test_type: {test_type}")
            summary = result["summary"]
            worst = result.get("worst_cases", [])
            passed = bool(summary.get("overall_pass", False))
            return TestResult(
                case=case,
                test_type=test_type,
                passed=passed,
                summary=summary,
                worst_cases=worst,
                duration_sec=time.time() - t0,
            )
        except Exception as e:
            return TestResult(
                case=case,
                test_type=test_type,
                passed=False,
                summary={},
                worst_cases=[],
                duration_sec=time.time() - t0,
                error=str(e),
            )

    def run_cases(
        self,
        cases: List[TestCase],
        *,
        test_types: Tuple[str, ...] = ("position_roundtrip", "jacobian"),
        samples: int = 50,
        seed: int = 42,
        quiet: bool = False,
    ) -> List[TestResult]:
        results: List[TestResult] = []
        for case in cases:
            for tt in test_types:
                r = self.run_single(case, tt, samples=samples, seed=seed)
                results.append(r)
                if not quiet or not r.passed:
                    status = "PASS" if r.passed else "FAIL"
                    if r.error:
                        status = "ERROR"
                    dur = f"{r.duration_sec:.1f}s"
                    conclusion = ""
                    if not r.error and r.summary:
                        conclusion = format_conclusion(summary=r.summary, worst_cases=r.worst_cases)
                    print(
                        f"  [{status}] {case.canonical_key} ver={case.version} {case.module}/{tt} "
                        f"{dur} | {conclusion}"
                    )
        return results

    def run_all(
        self,
        *,
        by_version: bool = False,
        filter_token: Optional[str] = None,
        filter_version: Optional[str] = None,
        filter_module: Optional[str] = None,
        test_types: Tuple[str, ...] = ("position_roundtrip", "jacobian"),
        samples: int = 50,
        seed: int = 42,
        quiet: bool = False,
    ) -> List[TestResult]:
        if by_version:
            cases = build_test_matrix(filter_token=filter_token)
            canonical = None
        else:
            canonical = build_canonical_matrix(filter_token=filter_token)
            cases = canonical_to_test_cases(canonical)
        cases = filter_cases(
            cases,
            filter_version=filter_version,
            filter_module=filter_module,
            canonical=canonical,
        )
        if not cases:
            print("警告: 筛选后测试矩阵为空，请检查 --version / --module / --token")
            return []
        return self.run_cases(cases, test_types=test_types, samples=samples, seed=seed, quiet=quiet)

    def run_smoke(self, **kwargs: Any) -> List[TestResult]:
        return self.run_all(test_types=("position_roundtrip",), samples=5, **kwargs)

    def run_convert_all(
        self,
        *,
        filter_token: Optional[str] = None,
        filter_module: Optional[str] = None,
        samples: int = 10,
        seed: int = 42,
        quiet: bool = False,
    ) -> List[TestResult]:
        """
        全版本 motor↔joint 位置转换验证。

        对每个 canonical case，采样 q，执行 j2m→m2j 往返，对比 q 与 q_back。
        与 position_roundtrip 不同：不涉及 MuJoCo，纯粹验证 solver 正逆解自洽。
        """
        canonical = build_canonical_matrix(filter_token=filter_token)
        cases = canonical_to_test_cases(canonical)
        if filter_module:
            cases = [c for c in cases if c.module == filter_module]
        if not cases:
            print("警告: convert 矩阵为空")
            return []

        results: List[TestResult] = []
        rng = np.random.default_rng(int(seed))

        for case in cases:
            t0 = time.time()
            try:
                spec = build_spec(module=case.module, version=case.version, solver_module=self.solver_module)
                solver = spec.construct_solver(self.solver_module)

                case_results: List[Dict[str, Any]] = []
                any_fail = False
                for _ in range(int(samples)):
                    q = np.asarray(spec.sample_q(rng), dtype=float).reshape((spec.dim_joint,))
                    p = np.asarray(spec.joint_to_motor_position(solver, q), dtype=float).reshape((spec.dim_motor,))
                    q_back = np.asarray(spec.motor_to_joint_position(solver, p), dtype=float).reshape((spec.dim_joint,))
                    err = vector_err(q, q_back)
                    ok = pass_threshold(err, self.threshold)
                    any_fail = any_fail or (not ok)
                    case_results.append({"q": q.tolist(), "p": p.tolist(), "q_back": q_back.tolist(), "err": err, "pass": ok})

                worst = sorted(case_results, key=lambda c: c["err"]["abs_max"], reverse=True)[:3]
                summary = {
                    "module": f"{case.module}_convert",
                    "token": case.token,
                    "total_cases": len(case_results),
                    "passed_cases": int(sum(1 for c in case_results if c["pass"])),
                    "failed_cases": int(sum(1 for c in case_results if not c["pass"])),
                    "pass_rate": float(sum(1 for c in case_results if c["pass"]) / max(len(case_results), 1)),
                    "overall_pass": not any_fail,
                    "threshold": {"abs_max": self.threshold.abs_max, "rmse": self.threshold.rmse, "rel_l2": self.threshold.rel_l2},
                }
                results.append(TestResult(
                    case=case, test_type="convert",
                    passed=not any_fail, summary=summary,
                    worst_cases=worst, duration_sec=time.time() - t0,
                ))
            except Exception as e:
                results.append(TestResult(
                    case=case, test_type="convert",
                    passed=False, summary={}, worst_cases=[],
                    duration_sec=time.time() - t0, error=str(e),
                ))

            r = results[-1]
            if not quiet or not r.passed:
                status = "PASS" if r.passed else ("ERROR" if r.error else "FAIL")
                dur = f"{r.duration_sec:.1f}s"
                conclusion = r.error or format_conclusion(summary=r.summary, worst_cases=r.worst_cases)
                print(f"  [{status}] {case.canonical_key} convert {dur} | {conclusion}")

        return results


# ---------------------------------------------------------------------------
# 报告生成
# ---------------------------------------------------------------------------


def _result_to_dict(r: TestResult) -> Dict[str, Any]:
    return {
        "canonical_key": r.case.canonical_key,
        "representative_version": r.case.representative_version,
        "version": r.case.version,
        "spec_module": r.case.module,
        "module": r.case.module,
        "token": r.case.token,
        "test_type": r.test_type,
        "passed": r.passed,
        "duration_sec": round(r.duration_sec, 2),
        "error": r.error,
        "summary": r.summary if not r.error else None,
        "worst_cases": r.worst_cases if not r.error else [],
        "worst_cases_conclusion": (
            format_conclusion(summary=r.summary, worst_cases=r.worst_cases)
            if not r.error and r.summary
            else None
        ),
    }


def _worst_metric(r: TestResult) -> Tuple[str, Dict[str, float]]:
    """Return the worst error dictionary from the first worst case."""
    if not r.worst_cases:
        return "", {}
    c0 = r.worst_cases[0] or {}
    best_name = ""
    best_err: Dict[str, float] = {}
    for name, val in c0.items():
        if not isinstance(val, dict):
            continue
        if not all(k in val for k in ("abs_max", "rmse", "rel_l2")):
            continue
        err = {"abs_max": float(val["abs_max"]), "rmse": float(val["rmse"]), "rel_l2": float(val["rel_l2"])}
        if not best_err or err["abs_max"] > best_err["abs_max"]:
            best_name = str(name)
            best_err = err
    return best_name, best_err


def _metric_ratio(value: float, threshold: float) -> float:
    return float(value) / threshold if threshold > 0 else float("inf")


def write_csv_report(results: List[TestResult], output_path: str) -> None:
    """Write a flat CSV summary for spreadsheet review and trend comparison."""
    fieldnames = [
        "canonical_key",
        "representative_version",
        "version",
        "module",
        "token",
        "test_type",
        "status",
        "duration_sec",
        "passed_cases",
        "total_cases",
        "pass_rate",
        "worst_metric",
        "worst_abs_max",
        "worst_rmse",
        "worst_rel_l2",
        "abs_max_ratio",
        "rmse_ratio",
        "rel_l2_ratio",
        "error",
    ]
    with open(output_path, "w", encoding="utf-8", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        for r in results:
            summary = r.summary or {}
            threshold = summary.get("threshold") or {}
            worst_name, worst = _worst_metric(r)
            status = "PASS" if r.passed else ("ERROR" if r.error else "FAIL")
            writer.writerow({
                "canonical_key": r.case.canonical_key,
                "representative_version": r.case.representative_version,
                "version": r.case.version,
                "module": r.case.module,
                "token": r.case.token,
                "test_type": r.test_type,
                "status": status,
                "duration_sec": round(r.duration_sec, 3),
                "passed_cases": summary.get("passed_cases", ""),
                "total_cases": summary.get("total_cases", ""),
                "pass_rate": summary.get("pass_rate", ""),
                "worst_metric": worst_name,
                "worst_abs_max": worst.get("abs_max", ""),
                "worst_rmse": worst.get("rmse", ""),
                "worst_rel_l2": worst.get("rel_l2", ""),
                "abs_max_ratio": _metric_ratio(worst.get("abs_max", 0.0), float(threshold.get("abs_max", 0.0) or 0.0)) if worst else "",
                "rmse_ratio": _metric_ratio(worst.get("rmse", 0.0), float(threshold.get("rmse", 0.0) or 0.0)) if worst else "",
                "rel_l2_ratio": _metric_ratio(worst.get("rel_l2", 0.0), float(threshold.get("rel_l2", 0.0) or 0.0)) if worst else "",
                "error": r.error or "",
            })
    print(f"CSV 报告已写入: {output_path}")


def write_markdown_summary(results: List[TestResult], output_path: str, *, matrix_mode: str = "canonical") -> None:
    """Write a compact human-readable validation summary."""
    total = len(results)
    passed = sum(1 for r in results if r.passed)
    failed = total - passed
    errors = sum(1 for r in results if r.error)
    lines = [
        "# kuavo_solver 验证摘要",
        "",
        f"- 矩阵: `{matrix_mode}`",
        f"- 总计: **{total}**",
        f"- 通过: **{passed}**",
        f"- 失败/错误: **{failed}**（错误 {errors}）",
        f"- 通过率: **{passed / max(total, 1) * 100.0:.1f}%**",
        f"- 时间: `{time.strftime('%Y-%m-%d %H:%M:%S')}`",
        "",
        "## 结果明细",
        "",
        "| 状态 | canonical | version | module | test | 耗时 | 结论 |",
        "|---|---|---|---|---|---:|---|",
    ]
    for r in results:
        status = "PASS" if r.passed else ("ERROR" if r.error else "FAIL")
        conclusion = r.error or (format_conclusion(summary=r.summary, worst_cases=r.worst_cases) if r.summary else "")
        conclusion = str(conclusion).replace("|", "\\|")
        lines.append(
            f"| {status} | `{r.case.canonical_key}` | `{r.case.version}` | `{r.case.module}` | "
            f"`{r.test_type}` | {r.duration_sec:.1f}s | {conclusion} |"
        )
    lines.append("")
    with open(output_path, "w", encoding="utf-8") as f:
        f.write("\n".join(lines))
    print(f"Markdown 摘要已写入: {output_path}")


def write_plots(results: List[TestResult], output_dir: str) -> List[str]:
    """Generate visual summaries when matplotlib is available."""
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except Exception as e:
        print(f"跳过图表生成: matplotlib 不可用 ({e})")
        return []

    plot_dir = os.path.join(output_dir, "plots")
    os.makedirs(plot_dir, exist_ok=True)
    outputs: List[str] = []

    labels: List[str] = []
    ratios: List[float] = []
    colors: List[str] = []
    for r in results:
        if r.error or not r.summary:
            continue
        threshold = r.summary.get("threshold") or {}
        worst_name, worst = _worst_metric(r)
        if not worst:
            continue
        ratio = max(
            _metric_ratio(worst.get("abs_max", 0.0), float(threshold.get("abs_max", 0.0) or 0.0)),
            _metric_ratio(worst.get("rmse", 0.0), float(threshold.get("rmse", 0.0) or 0.0)),
            _metric_ratio(worst.get("rel_l2", 0.0), float(threshold.get("rel_l2", 0.0) or 0.0)),
        )
        labels.append(f"{r.case.canonical_key}\n{r.test_type}\n{worst_name}")
        ratios.append(ratio)
        colors.append("#2e7d32" if r.passed else "#c62828")

    if labels:
        width = max(10.0, min(36.0, 0.65 * len(labels)))
        fig, ax = plt.subplots(figsize=(width, 6.0))
        ax.bar(range(len(labels)), ratios, color=colors)
        ax.axhline(1.0, color="#f9a825", linestyle="--", linewidth=1.5, label="threshold")
        ax.set_ylabel("worst metric / threshold")
        ax.set_title("kuavo_solver validation worst error ratio")
        ax.set_xticks(range(len(labels)))
        ax.set_xticklabels(labels, rotation=75, ha="right", fontsize=8)
        ax.grid(axis="y", alpha=0.25)
        ax.legend()
        fig.tight_layout()
        out = os.path.join(plot_dir, "worst_error_ratio.png")
        fig.savefig(out, dpi=180)
        plt.close(fig)
        outputs.append(out)

    status_counts = {
        "PASS": sum(1 for r in results if r.passed),
        "FAIL": sum(1 for r in results if (not r.passed and not r.error)),
        "ERROR": sum(1 for r in results if r.error),
    }
    if sum(status_counts.values()) > 0:
        fig, ax = plt.subplots(figsize=(5.5, 4.5))
        ax.bar(status_counts.keys(), status_counts.values(), color=["#2e7d32", "#c62828", "#6a1b9a"])
        ax.set_ylabel("count")
        ax.set_title("kuavo_solver validation status")
        ax.grid(axis="y", alpha=0.25)
        fig.tight_layout()
        out = os.path.join(plot_dir, "status_counts.png")
        fig.savefig(out, dpi=180)
        plt.close(fig)
        outputs.append(out)

    for out in outputs:
        print(f"图表已写入: {out}")
    return outputs


def write_json_report(
    results: List[TestResult], output_path: str, *, matrix_mode: str = "canonical"
) -> None:
    data = [_result_to_dict(r) for r in results]
    total = len(results)
    passed = sum(1 for r in results if r.passed)
    report = {
        "meta": {
            "matrix_mode": matrix_mode,
            "total_cases": total,
            "passed": passed,
            "failed": total - passed,
            "pass_rate": round(passed / max(total, 1), 4),
            "timestamp": time.strftime("%Y-%m-%d %H:%M:%S"),
        },
        "results": data,
    }
    with open(output_path, "w", encoding="utf-8") as f:
        json.dump(report, f, indent=2, ensure_ascii=False)
    print(f"JSON 报告已写入: {output_path}")


def write_html_report(
    results: List[TestResult], output_path: str, *, matrix_mode: str = "canonical"
) -> None:
    total = len(results)
    passed = sum(1 for r in results if r.passed)
    failed = total - passed
    errors = sum(1 for r in results if r.error)

    rows_html = ""
    for r in results:
        status_class = "pass" if r.passed else ("error" if r.error else "fail")
        status_text = "PASS" if r.passed else ("ERROR" if r.error else "FAIL")
        conclusion = ""
        if not r.error and r.summary:
            conclusion = format_conclusion(summary=r.summary, worst_cases=r.worst_cases)
        error_text = r.error or ""
        rows_html += f"""
        <tr class="{status_class}">
            <td>{r.case.canonical_key}</td>
            <td>{r.case.representative_version}</td>
            <td>{r.case.version}</td>
            <td>{r.case.module}</td>
            <td>{r.case.token}</td>
            <td>{r.test_type}</td>
            <td>{status_text}</td>
            <td>{r.duration_sec:.1f}s</td>
            <td>{conclusion}</td>
            <td>{error_text}</td>
        </tr>"""

    html = f"""<!DOCTYPE html>
<html lang="zh-CN">
<head><meta charset="utf-8"><title>kuavo_solver 回归测试报告</title>
<style>
body {{ font-family: monospace; margin: 20px; background: #1a1a2e; color: #eee; }}
h1 {{ color: #e94560; }}
table {{ border-collapse: collapse; width: 100%; }}
th, td {{ padding: 6px 10px; border: 1px solid #333; text-align: left; }}
th {{ background: #16213e; }}
tr.pass td {{ background: #1a3a1a; }}
tr.fail td {{ background: #3a1a1a; }}
tr.error td {{ background: #2a1a3a; }}
.meta {{ margin: 10px 0; font-size: 14px; }}
.meta span {{ margin-right: 20px; }}
</style></head>
<body>
<h1>kuavo_solver 回归测试报告</h1>
<div class="meta">
<span>矩阵: {matrix_mode}</span>
<span>总计: {total}</span>
<span>通过: {passed}</span>
<span>失败: {failed}</span>
<span>错误: {errors}</span>
<span>通过率: {passed/max(total,1)*100:.1f}%</span>
</div>
<table>
<tr><th>canonical</th><th>repr_version</th><th>run_version</th><th>module</th><th>Token</th>
<th>测试类型</th><th>状态</th><th>耗时</th><th>结论</th><th>错误信息</th></tr>
{rows_html}
</table>
</body></html>"""

    with open(output_path, "w", encoding="utf-8") as f:
        f.write(html)
    print(f"HTML 报告已写入: {output_path}")


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------


def main() -> None:
    parser = argparse.ArgumentParser(description="kuavo_solver 全版本回归测试 Runner")
    parser.add_argument("--all", action="store_true", help="运行全回归矩阵")
    parser.add_argument("--smoke", action="store_true", help="快速 smoke（5 samples，position only）")
    parser.add_argument("--convert", action="store_true", help="全版本 motor↔joint 位置转换验证（纯 solver 正逆解，不涉及 MuJoCo）")
    parser.add_argument("--list-matrix", action="store_true", help="打印矩阵后退出（不执行测试）")
    parser.add_argument(
        "--by-version",
        action="store_true",
        help="使用全 index 别名矩阵（不去重），用于映射审计",
    )
    parser.add_argument("--preflight", action="store_true", help="L0 预检（import + 抽查 MJCF）后退出")
    parser.add_argument("--version", type=str, help="只测试指定版本（可为任意别名）")
    parser.add_argument("--module", type=str, help="只测试指定模块 (ankle/knee/waist/arm_elbow/arm_wrist)")
    parser.add_argument("--token", type=str, help="只测试指定 solver token")
    parser.add_argument("--samples", type=int, default=50, help="每 case 采样数（默认 50）")
    parser.add_argument("--seed", type=int, default=42, help="随机种子")
    parser.add_argument("--quiet", action="store_true", help="只输出 FAIL/ERROR 行（CI 友好）")
    parser.add_argument(
        "--format",
        choices=["json", "html", "csv", "md", "plots", "both", "all"],
        default="all",
        help="报告格式：all=JSON/HTML/CSV/Markdown/plots",
    )
    parser.add_argument("--output-dir", type=str, default="/tmp/solver_test_results", help="输出目录")
    parser.add_argument("--threshold-abs-max", type=float, default=1e-4, help="阈值 abs_max")
    parser.add_argument("--threshold-rmse", type=float, default=5e-5, help="阈值 rmse")
    parser.add_argument("--threshold-rel-l2", type=float, default=2e-3, help="阈值 rel_l2")

    args = parser.parse_args()

    matrix_mode = "by-version" if args.by_version else "canonical"

    if args.list_matrix:
        if args.by_version:
            cases = build_test_matrix(filter_token=args.token)
        else:
            canonical = build_canonical_matrix(filter_token=args.token)
            cases = canonical_to_test_cases(canonical)
        cases = filter_cases(
            cases,
            filter_version=args.version,
            filter_module=args.module,
            canonical=build_canonical_matrix(filter_token=args.token) if not args.by_version else None,
        )
        print_matrix_list(cases, by_version=args.by_version)
        return

    if args.preflight:
        print("[PREFLIGHT] kuavo_solver_py + canonical MJCF 抽查 …")
        preflight_check(_load_solver_module())
        print("[PREFLIGHT] OK")
        return

    if not args.all and not args.smoke and not args.convert and not args.version:
        parser.error("需要指定 --all / --smoke / --convert / --version / --list-matrix / --preflight 之一")

    threshold = StrictThreshold(
        abs_max=args.threshold_abs_max,
        rmse=args.threshold_rmse,
        rel_l2=args.threshold_rel_l2,
    )
    runner = SolverTestRunner(threshold=threshold)

    print("=" * 60)
    print("kuavo_solver 回归测试")
    print(f"矩阵: {matrix_mode}")
    print("=" * 60)

    run_kw = dict(
        by_version=args.by_version,
        filter_token=args.token,
        filter_version=args.version,
        filter_module=args.module,
        quiet=args.quiet,
    )

    if args.smoke:
        if not args.quiet:
            print("[SMOKE] canonical 去重矩阵 · 5 samples · position_roundtrip only")
        results = runner.run_smoke(**run_kw)
    elif args.convert:
        if not args.quiet:
            print(f"[CONVERT] canonical 去重矩阵 · {args.samples} samples · motor↔joint 转换")
        results = runner.run_convert_all(
            filter_token=args.token, filter_module=args.module,
            samples=args.samples, seed=args.seed, quiet=args.quiet,
        )
    elif args.all:
        if not args.quiet:
            print(f"[FULL] {args.samples} samples · position_roundtrip + jacobian")
        results = runner.run_all(samples=args.samples, seed=args.seed, **run_kw)
    else:
        if not args.quiet:
            print(f"[SINGLE] version={args.version}, module={args.module or 'all'}, {args.samples} samples")
        results = runner.run_all(samples=args.samples, seed=args.seed, **run_kw)

    total = len(results)
    passed = sum(1 for r in results if r.passed)
    print("=" * 60)
    if total == 0:
        print("结果: 未执行任何测试（矩阵为空或筛选无匹配）")
        print("=" * 60)
        sys.exit(1)
    print(f"结果: {passed}/{total} 通过 ({passed/max(total,1)*100:.1f}%)")

    # 汇总失败项
    failures = [r for r in results if not r.passed]
    if failures:
        print("-" * 60)
        for r in failures:
            status = "ERROR" if r.error else "FAIL"
            info = r.error or format_conclusion(summary=r.summary, worst_cases=r.worst_cases)
            print(f"  [{status}] {r.case.canonical_key} {r.case.module}/{r.test_type} | {info}")
    print("=" * 60)

    os.makedirs(args.output_dir, exist_ok=True)

    fmt = args.format
    if fmt == "both":
        formats = {"json", "html"}
    elif fmt == "all":
        formats = {"json", "html", "csv", "md", "plots"}
    else:
        formats = {fmt}

    if "json" in formats:
        json_path = os.path.join(args.output_dir, "solver_test_report.json")
        write_json_report(results, json_path, matrix_mode=matrix_mode)

    if "html" in formats:
        html_path = os.path.join(args.output_dir, "solver_test_report.html")
        write_html_report(results, html_path, matrix_mode=matrix_mode)

    if "csv" in formats:
        csv_path = os.path.join(args.output_dir, "solver_test_report.csv")
        write_csv_report(results, csv_path)

    if "md" in formats:
        md_path = os.path.join(args.output_dir, "solver_test_summary.md")
        write_markdown_summary(results, md_path, matrix_mode=matrix_mode)

    if "plots" in formats:
        write_plots(results, args.output_dir)

    print(f"报告目录: {args.output_dir}")

    if passed != total:
        sys.exit(1)


if __name__ == "__main__":
    main()
