#!/usr/bin/env python3
"""从 OBJ 文件顶点计算包围盒尺寸（用于 MuJoCo 等配置 scaling）。"""
import sys
from pathlib import Path

def main():
    obj_path = Path(__file__).parent / "uploads_files_3223633_traffic+cone.obj"
    if len(sys.argv) > 1:
        obj_path = Path(sys.argv[1])
    if not obj_path.exists():
        print(f"文件不存在: {obj_path}")
        sys.exit(1)

    xs, ys, zs = [], [], []
    for line in obj_path.open():
        if line.startswith("v "):
            parts = line.split()
            xs.append(float(parts[1]))
            ys.append(float(parts[2]))
            zs.append(float(parts[3]))

    minx, maxx = min(xs), max(xs)
    miny, maxy = min(ys), max(ys)
    minz, maxz = min(zs), max(zs)
    wx, wy, wz = maxx - minx, maxy - miny, maxz - minz

    print(f"OBJ: {obj_path.name}")
    print(f"  X: {minx:.4f} ~ {maxx:.4f}  宽度 = {wx:.4f}")
    print(f"  Y: {miny:.4f} ~ {maxy:.4f}  深度 = {wy:.4f}")
    print(f"  Z: {minz:.4f} ~ {maxz:.4f}  高度/长度 = {wz:.4f}")
    print(f"  半尺寸 (MuJoCo size 常用): {wx/2:.4f} {wy/2:.4f} {wz/2:.4f}")

if __name__ == "__main__":
    main()
