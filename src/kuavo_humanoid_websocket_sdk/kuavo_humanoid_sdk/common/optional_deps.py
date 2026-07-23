import importlib


def require_optional(modules, extra, feature):
    missing = []
    for module in modules:
        try:
            importlib.import_module(module)
        except ImportError:
            missing.append(module)

    if missing:
        missing_str = ", ".join(missing)
        raise ImportError(
            f"{feature} features require optional dependencies: {missing_str}. "
            f"Install with `pip install kuavo-humanoid-sdk[{extra}]` or "
            f"`pip install kuavo-humanoid-sdk[full]`."
        )
