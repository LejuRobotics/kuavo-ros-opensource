from pathlib import Path
import shutil
import uuid
import time

# ============ CONFIG ===========
DIR         = "/home/ruichen/Downloads/kuavo-ros-control/src/data_challenge_simulator/examples/bags/run_2025-10-18_13-46-59"   # 目标目录
PREFIX      = "task4"           # 前缀
START       = 98                # 起始序号）
WIDTH       = None              # 序号宽度；None=自动
EXT         = ".bag"            # 扩展名
SORT_MODE   = "mtime"           # 排序： "mtime" | "ctime" | "name"
RECURSIVE   = False             # 是否递归子目录
DRY_RUN     = False             # 试跑：True=只打印计划，不真正改名
WRITE_CSV   = None              # 写映射CSV路径
# ========================================

def collect_files(root: Path, ext: str, recursive: bool):
    ext = ext.lower() if ext.startswith(".") else "." + ext.lower()
    pattern = "**/*" + ext if recursive else "*" + ext
    return [p for p in root.glob(pattern) if p.is_file() and p.suffix.lower() == ext]

def sort_files(files, mode: str):
    mode = mode.lower()
    if mode == "name":
        return sorted(files, key=lambda p: str(p).lower())
    if mode == "ctime":
        return sorted(files, key=lambda p: p.stat().st_ctime)
    # 默认 mtime
    return sorted(files, key=lambda p: p.stat().st_mtime)

def calc_width(count: int, start: int, user_width):
    if user_width is not None:
        return max(1, int(user_width))
    max_idx = start + count - 1
    return max(4, len(str(max_idx)))

def plan_targets(files, prefix: str, start: int, width: int, ext: str):
    ext = ext if ext.startswith(".") else "." + ext
    return [f.with_name(f"{prefix}_{str(i).zfill(width)}{ext.lower()}") 
            for i, f in enumerate(files, start=start)]

def two_phase_rename(pairs, dry_run=False):
    tmp_pairs = []
    try:
        # 第一阶段：源 -> 临时
        for src, dst in pairs:
            if src == dst:
                continue
            tmp = src.with_name(src.name + f".__tmp_{uuid.uuid4().hex}")
            tmp_pairs.append((src, tmp))
            if dry_run:
                print(f"[DRY] {src.name} -> {dst.name}  (中间: {tmp.name})")
            else:
                src.rename(tmp)

        if dry_run:
            return

        # 第二阶段：临时 -> 目标
        for (src, tmp), (_, dst) in zip(tmp_pairs, pairs):
            if dst.exists():
                bak = dst.with_name(dst.name + f".bak_{int(time.time())}")
                shutil.move(str(dst), str(bak))
                print(f"[INFO] 目标已存在，已备份为: {bak.name}")
            shutil.move(str(tmp), str(dst))
    finally:
        # 异常清理残留临时文件
        for _, tmp in tmp_pairs:
            if tmp.exists():
                try:
                    tmp.unlink()
                except Exception:
                    pass

def main():
    root = Path(DIR).expanduser().resolve()
    if not root.exists() or not root.is_dir():
        print(f"[ERROR] 目录不存在或不可用: {root}")
        return

    files = collect_files(root, EXT, RECURSIVE)
    if not files:
        print(f"[WARN] 未找到 {EXT} 文件（目录：{root}，recursive={RECURSIVE}）")
        return

    files_sorted = sort_files(files, SORT_MODE)
    width = calc_width(len(files_sorted), START, WIDTH)
    targets = plan_targets(files_sorted, PREFIX, START, width, EXT)

    print(f"[INFO] 计划重命名 {len(files_sorted)} 个文件：排序={SORT_MODE}, 起始={START}, 宽度={width}, 前缀={PREFIX}")
    for s, d in zip(files_sorted, targets):
        arrow = "==" if s == d else "->"
        print(f"{s.name}  {arrow}  {d.name}")

    # 写 CSV 映射（可回滚）
    if WRITE_CSV:
        csv_path = Path(WRITE_CSV).expanduser().resolve()
        try:
            import csv
            with csv_path.open("w", newline="", encoding="utf-8") as f:
                w = csv.writer(f)
                w.writerow(["old_path", "new_path"])
                for s, d in zip(files_sorted, targets):
                    w.writerow([str(s), str(d)])
            print(f"[INFO] 映射已写入: {csv_path}")
        except Exception as e:
            print(f"[WARN] 写 CSV 失败：{e}")

    if DRY_RUN:
        print("[DRY] 试跑完成，未实际改名。")
        return

    two_phase_rename(list(zip(files_sorted, targets)), dry_run=False)
    print("[DONE] 重命名完成。")

if __name__ == "__main__":
    main()
