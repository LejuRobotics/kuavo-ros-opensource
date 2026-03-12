#!/usr/bin/env python3
"""
修复 tibvh 库与 PyTorch 的 uint32 兼容性问题。

问题：tibvh 使用了 PyTorch 不支持的 uint32 类型，导致 Taichi GPU 后端无法使用。
解决：将 uint32 替换为 int32/int64。

使用方法（在 Docker 内执行）：
    python3 /path/to/patch_tibvh.py

或通过 rosrun：
    rosrun craic_simulator patch_tibvh.py
"""
import os
import sys


def find_tibvh_lbvh():
    """查找 tibvh 的 lbvh.py 文件路径"""
    try:
        import tibvh
        pkg_dir = os.path.dirname(tibvh.__file__)
        lbvh_path = os.path.join(pkg_dir, "lbvh", "lbvh.py")
        if os.path.isfile(lbvh_path):
            return lbvh_path
    except ImportError:
        pass

    # 常见安装路径
    common_paths = [
        "/usr/local/lib/python3.8/dist-packages/tibvh/lbvh/lbvh.py",
        "/usr/local/lib/python3.10/dist-packages/tibvh/lbvh/lbvh.py",
        "/usr/lib/python3/dist-packages/tibvh/lbvh/lbvh.py",
    ]
    for p in common_paths:
        if os.path.isfile(p):
            return p
    return None


def patch_tibvh(filepath):
    """修复 tibvh 的 uint32 兼容性问题"""
    with open(filepath, "r") as f:
        content = f.read()

    original = content

    # 1. torch.uint32 -> torch.int32
    content = content.replace("torch.uint32", "torch.int32")

    # 2. 修复 to_torch u32 问题（如果存在）
    # 旧: self.morton_codes.to_torch(device = 'cuda')[:,0].to(torch.int64).sort()
    # 新: torch.from_numpy(self.morton_codes.to_numpy()[:,0].astype('int64')).cuda().sort()
    old_pattern = "self.morton_codes.to_torch(device = 'cuda')[:,0].to(torch.int64).sort()"
    new_pattern = "torch.from_numpy(self.morton_codes.to_numpy()[:,0].astype('int64')).cuda().sort()"
    content = content.replace(old_pattern, new_pattern)

    # 3. 检查是否有修改
    if content == original:
        print(f"[patch_tibvh] {filepath} 已经是最新版本或不需要修改")
        return False

    # 4. 写回文件
    with open(filepath, "w") as f:
        f.write(content)

    print(f"[patch_tibvh] 已修复 {filepath}")
    return True


def main():
    filepath = find_tibvh_lbvh()
    if not filepath:
        print("[patch_tibvh] 错误: 未找到 tibvh 库，请先安装: pip install 'mujoco-lidar[taichi]'")
        sys.exit(1)

    print(f"[patch_tibvh] 找到 tibvh: {filepath}")

    try:
        patched = patch_tibvh(filepath)
        if patched:
            print("[patch_tibvh] 修复完成！现在可以使用 Taichi GPU 后端了。")
        sys.exit(0)
    except PermissionError:
        print(f"[patch_tibvh] 错误: 权限不足，请使用 sudo 或 root 用户运行")
        sys.exit(1)
    except Exception as e:
        print(f"[patch_tibvh] 错误: {e}")
        sys.exit(1)


if __name__ == "__main__":
    main()
