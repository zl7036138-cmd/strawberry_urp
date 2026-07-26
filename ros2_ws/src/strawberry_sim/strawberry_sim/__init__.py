"""Simulation-side assets and dependency-light verification logic."""

from .core import (
    AttachmentDecision,
    AttachmentGate,
    BinBounds,
    BinStabilityTracker,
    FruitSpec,
    Pose3D,
    SceneConfig,
    load_scene_config,
)

__all__ = [
    "AttachmentDecision",
    "AttachmentGate",
    "BinBounds",
    "BinStabilityTracker",
    "FruitSpec",
    "Pose3D",
    "SceneConfig",
    "load_scene_config",
]
