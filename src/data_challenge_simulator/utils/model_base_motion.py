#!/usr/bin/env python3
"""Release chassis endpoints owned by model-only fixed initialization."""


def close_chassis(chassis):
    """Stop and unregister a model-initialization ``ChassisMotion``."""
    if chassis is None:
        return
    try:
        chassis.stop()
    finally:
        for endpoint_name in ("_publisher", "_subscriber"):
            endpoint = getattr(chassis, endpoint_name, None)
            if endpoint is not None:
                endpoint.unregister()
