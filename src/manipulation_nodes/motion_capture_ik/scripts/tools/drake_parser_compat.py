"""Drake Parser compatibility for pydrake 1.19 (x86) and 1.51 (aarch64)."""

from __future__ import annotations


def add_urdf_model(parser, model_path):
    """Load a URDF/SDF model; works with AddModels (1.51+) and AddModelFromFile (1.19)."""
    if hasattr(parser, "AddModels"):
        instances = parser.AddModels(model_path)
        if not instances:
            raise RuntimeError(f"AddModels returned no instances for: {model_path}")
        return instances[0]
    if hasattr(parser, "AddModelFromFile"):
        return parser.AddModelFromFile(model_path)
    raise RuntimeError("Parser has neither AddModels nor AddModelFromFile")
