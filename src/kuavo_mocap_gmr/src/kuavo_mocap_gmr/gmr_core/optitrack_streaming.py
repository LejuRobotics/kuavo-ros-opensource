"""
optitrack_streaming.py

Public API wrapper for OptiTrack streaming

当 `_optitrack_streaming_impl` 不存在或无法导入时，本模块的函数会抛出
RuntimeError，表示该功能在当前发行版本中是闭源的/不可用的。
"""

import sys
import pathlib

# Add the gmr_core directory to path for .so import
_here = pathlib.Path(__file__).parent
if str(_here) not in sys.path:
    sys.path.insert(0, str(_here))

try:
    # 本地/闭源环境：直接从内部实现模块导出所有公共 API
    from _optitrack_streaming_impl import (  # type: ignore
        start_streaming,
        get_frame,
        get_data_descriptions,
        get_rigid_body_descriptions,
        get_latest_rigid_body_descriptions,
    )
except ImportError:
    # 开源/精简环境：提供同名函数，但在调用时显式报错
    def _closed_source_stub(*args, **kwargs):
        raise RuntimeError(
            "OptiTrack streaming is not available in this build "
            "(closed-source implementation is not included)."
        )

    start_streaming = _closed_source_stub
    get_frame = _closed_source_stub
    get_data_descriptions = _closed_source_stub
    get_rigid_body_descriptions = _closed_source_stub
    get_latest_rigid_body_descriptions = _closed_source_stub

    
