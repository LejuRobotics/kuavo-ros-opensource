import importlib

from kuavo_humanoid_sdk.common.logger import SDKLogger, disable_sdk_logging

_LAZY_MODULES = (
    ".msg",
    ".interfaces",
    ".kuavo",
    ".kuavo_strategy",
    ".kuavo_strategy.grasp_box",
)


def __getattr__(name):
    for module_name in _LAZY_MODULES:
        module = importlib.import_module(module_name, __name__)
        if hasattr(module, name):
            value = getattr(module, name)
            globals()[name] = value
            return value
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
