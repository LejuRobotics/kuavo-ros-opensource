# 按当前解释器加载对应字节码。x86 是 Python 3.8，AGX 是 Python 3.12，两份 pyc 一起发布。
def _load_torso_ik():
    import importlib.util
    import marshal
    import sys
    from pathlib import Path
    tag = sys.implementation.cache_tag
    path = Path(__file__).with_name("torso_ik.%s.pyc" % tag)
    if not path.is_file():
        raise ImportError("torso_ik 缺少 %s 字节码: %s" % (tag, path))
    data = path.read_bytes()
    if data[:4] != importlib.util.MAGIC_NUMBER:
        raise ImportError("torso_ik 字节码与当前解释器 %s 不匹配: %s" % (tag, path))
    exec(marshal.loads(data[16:]), globals())

_load_torso_ik()
del _load_torso_ik
