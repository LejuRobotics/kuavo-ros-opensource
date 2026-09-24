import os
from typing import Optional, List, Tuple

# ======= 在这里配置 =======
ROOT_DIR  = "/home/ruichen/Downloads/kuavo-ros-control/src/data_challenge_simulator/examples/bags/run_2025-11-11_13-58-08"  # 要遍历的目录
EXT       = ".bag"                   # 只检查这个后缀（不区分大小写）
RECURSIVE = True                     # 是否递归子目录

# 设定大小区间（字节）。不想限制就设为 None
# 例如 1.2GB = 1.2 * 1024**3
MIN_SIZE: Optional[int] = int(750 * 1024**2)   # 含下界
MAX_SIZE: Optional[int] = int(1000 * 1024**2)   # 含上界
# =========================
DELETE_BAD_FILES = False 

UNITS = {"B":1, "KB":1024, "MB":1024**2, "GB":1024**3}

def human(n: int) -> str:
    if n < 0: return "N/A"
    for u in ("GB", "MB", "KB"):
        f = UNITS[u]
        if n >= f: return f"{n / f:.2f}{u}"
    return f"{n}B"

def list_files(root: str, recursive: bool) -> List[str]:
    root = os.path.abspath(root)
    if not recursive:
        return [os.path.join(root, f) for f in os.listdir(root)]
    paths: List[str] = []
    for d, _, fs in os.walk(root):
        for f in fs:
            paths.append(os.path.join(d, f))
    return paths

def scan_bags(root: str,
              min_bytes: Optional[int],
              max_bytes: Optional[int],
              ext: str = ".bag",
              recursive: bool = True) -> List[Tuple[str, int]]:
    bad: List[Tuple[str, int]] = []
    root_abs = os.path.abspath(root)
    for full in list_files(root_abs, recursive):
        name = os.path.basename(full)
        if ext and not name.lower().endswith(ext.lower()):
            continue
        try:
            size = os.path.getsize(full)
        except OSError:
            rel = os.path.relpath(full, root_abs)
            bad.append((rel, -1))
            continue

        out_of_range = (
            (min_bytes is not None and size < min_bytes) or
            (max_bytes is not None and size > max_bytes)
        )
        if out_of_range:
            rel = os.path.relpath(full, root_abs)
            bad.append((rel, size))
    return bad

def main():
    print(f"[SCAN] dir={ROOT_DIR} recursive={RECURSIVE} ext={EXT}")
    if MIN_SIZE is not None: print(f"[RULE] min >= {human(MIN_SIZE)}")
    if MAX_SIZE is not None: print(f"[RULE] max <= {human(MAX_SIZE)}")

    bad = scan_bags(ROOT_DIR, MIN_SIZE, MAX_SIZE, ext=EXT, recursive=RECURSIVE)
    bad.sort(key=lambda x: x[1])

    if not bad:
        print("\n[OK] 未发现异常 bag。")
        return

    print("\n[BAD] 以下文件大小不在区间内：")
    for rel, size in bad:
        print(f"{rel}\t{human(size)}")

    print(f"\n共发现异常文件: {len(bad)}")

    if DELETE_BAD_FILES:
        print(f"\n[DELETE] 开始删除异常文件...")
        root_abs = os.path.abspath(ROOT_DIR)
        deleted_count = 0
        failed_count = 0
        for rel, size in bad:
            full_path = os.path.join(root_abs, rel)
            try:
                os.remove(full_path)
                print(f"[DELETED] {rel}")
                deleted_count += 1
            except OSError as e:
                print(f"[FAILED] {rel} - {e}")
                failed_count += 1
        print(f"\n[RESULT] 成功删除: {deleted_count}, 失败: {failed_count}")
if __name__ == "__main__":
    main()
