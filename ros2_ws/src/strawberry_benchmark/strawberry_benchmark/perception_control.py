"""Frozen engineering-waiver contract for perception-controlled simulation."""

from __future__ import annotations

from dataclasses import dataclass
import hashlib
import json
import math
from pathlib import Path
from typing import Any


@dataclass(frozen=True)
class ArtifactBinding:
    relative_path: str
    sha256: str


@dataclass(frozen=True)
class SceneModel:
    model_name: str
    target_id: int
    position_m: tuple[float, float, float]


@dataclass(frozen=True)
class PerceptionControlWaiver:
    waiver_id: str
    status: str
    scope: str
    model_relative_path: str
    model_size_bytes: int
    model_sha256: str
    confidence_threshold: float
    image_size: int
    validation_summary: ArtifactBinding
    outcome_handoff: ArtifactBinding
    held_out_receipt_relative_path: str
    macro_f1: float
    ripe_f1: float
    unripe_f1: float
    required_macro_f1: float
    engineering_status: str
    detections_topic: str
    target_pose_topic: str
    control_target_topic: str
    ground_truth_association_enabled: bool
    scenario_id: str
    target: SceneModel
    parked_models: tuple[SceneModel, ...]
    seed: int


def sha256_file(path: Path) -> str:
    """Return the lowercase SHA-256 digest for one file."""

    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _mapping(value: Any, name: str) -> dict[str, Any]:
    if not isinstance(value, dict):
        raise ValueError(f"{name} must be an object")
    return value


def _relative_path(value: Any, name: str) -> str:
    path = Path(str(value))
    if path.is_absolute() or ".." in path.parts or not path.parts:
        raise ValueError(f"{name} must be a repository-relative path")
    return path.as_posix()


def _sha256(value: Any, name: str) -> str:
    rendered = str(value).lower()
    if len(rendered) != 64 or any(character not in "0123456789abcdef" for character in rendered):
        raise ValueError(f"{name} must be a lowercase SHA-256 digest")
    return rendered


def _finite_float(value: Any, name: str) -> float:
    try:
        rendered = float(value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{name} must be numeric") from exc
    if not math.isfinite(rendered):
        raise ValueError(f"{name} must be finite")
    return rendered


def _scene_model(value: Any, name: str) -> SceneModel:
    raw = _mapping(value, name)
    model_name = str(raw.get("model_name", "")).strip()
    target_id = raw.get("target_id")
    position = raw.get("position_m")
    if not model_name:
        raise ValueError(f"{name}.model_name must be non-empty")
    if not isinstance(target_id, int) or isinstance(target_id, bool) or target_id <= 0:
        raise ValueError(f"{name}.target_id must be a positive integer")
    if not isinstance(position, list) or len(position) != 3:
        raise ValueError(f"{name}.position_m must contain three coordinates")
    xyz = tuple(
        _finite_float(coordinate, f"{name}.position_m") for coordinate in position
    )
    return SceneModel(model_name=model_name, target_id=target_id, position_m=xyz)


def _binding(value: Any, name: str) -> ArtifactBinding:
    raw = _mapping(value, name)
    return ArtifactBinding(
        relative_path=_relative_path(raw.get("relative_path"), f"{name}.relative_path"),
        sha256=_sha256(raw.get("sha256"), f"{name}.sha256"),
    )


def load_perception_control_waiver(path: Path) -> PerceptionControlWaiver:
    """Load and strictly validate the bounded perception-control waiver."""

    try:
        raw = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise ValueError(f"cannot read waiver manifest: {exc}") from exc
    root = _mapping(raw, "waiver manifest")
    if root.get("schema_version") != 1:
        raise ValueError("waiver schema_version must be 1")

    decision = _mapping(root.get("decision"), "decision")
    authorization = _mapping(root.get("authorization"), "authorization")
    model = _mapping(root.get("model"), "model")
    evidence = _mapping(root.get("evidence"), "evidence")
    validation = _mapping(root.get("validation_result"), "validation_result")
    routing = _mapping(root.get("runtime_routing"), "runtime_routing")
    smoke = _mapping(root.get("smoke_scenario"), "smoke_scenario")
    conditions = _mapping(smoke.get("conditions"), "smoke_scenario.conditions")

    if decision.get("status") != "ENGINEERING_ACCEPTED_WITH_USER_WAIVER":
        raise ValueError("decision must be engineering-accepted with user waiver")
    if decision.get("adr") != (
        "docs/decisions/0026-accept-below-gate-model-for-simulation-control.md"
    ):
        raise ValueError("waiver must reference ADR 0026")
    if authorization.get("scope") != "BOUNDED_SIMULATION_INTEGRATION":
        raise ValueError("waiver scope must remain bounded simulation integration")
    required_authorizations = {
        "perception_control_authorized": True,
        "formal_135_plus_30_matrix_authorized": False,
        "held_out_real_test_authorized": False,
        "new_training_authorized": False,
        "numeric_metric_overridden": False,
        "default_launch_safety_changed": False,
    }
    for key, expected in required_authorizations.items():
        if authorization.get(key) is not expected:
            raise ValueError(f"authorization.{key} must be {expected}")

    macro_f1 = _finite_float(validation.get("macro_f1"), "macro_f1")
    ripe_f1 = _finite_float(validation.get("ripe_f1"), "ripe_f1")
    unripe_f1 = _finite_float(validation.get("unripe_f1"), "unripe_f1")
    required_macro_f1 = _finite_float(
        validation.get("required_macro_f1"), "required_macro_f1"
    )
    if validation.get("numeric_gate_passed") is not False:
        raise ValueError("the numeric gate must remain recorded as failed")
    if validation.get("engineering_status") != "ACCEPTED_WITH_WAIVER":
        raise ValueError("engineering status must be ACCEPTED_WITH_WAIVER")
    if not 0.0 <= macro_f1 < required_macro_f1 <= 1.0:
        raise ValueError("waiver must preserve the below-gate macro-F1 result")
    if not all(0.0 <= score <= 1.0 for score in (ripe_f1, unripe_f1)):
        raise ValueError("class F1 scores must be in [0, 1]")

    if routing.get("target_source") != "perception":
        raise ValueError("waiver routing must use perception control")
    if routing.get("start_oracle_provider") is not False:
        raise ValueError("Oracle provider must remain stopped")
    if routing.get("shadow_enabled") is not False:
        raise ValueError("the authorized perception stream must not be Shadow")
    if routing.get("ground_truth_may_select_target") is not False:
        raise ValueError("ground truth may not select the target")
    if routing.get("ground_truth_may_replace_target_pose") is not False:
        raise ValueError("ground truth may not replace the target pose")
    if routing.get("ground_truth_use") != "TARGET_ID_ASSOCIATION_AND_TEST_VERIFICATION_ONLY":
        raise ValueError("ground truth use must remain identity/test-only")
    if routing.get("ground_truth_association_enabled") is not True:
        raise ValueError("deterministic smoke requires ID-only truth association")

    detections_topic = str(routing.get("detections_topic", ""))
    target_pose_topic = str(routing.get("target_pose_topic", ""))
    control_target_topic = str(routing.get("control_target_topic", ""))
    if detections_topic != "/strawberry/detections":
        raise ValueError("detections must use the frozen control topic")
    if target_pose_topic != "/strawberry/target_pose":
        raise ValueError("localized targets must use the frozen control topic")
    if control_target_topic != target_pose_topic:
        raise ValueError("orchestrator control must equal the perception target topic")

    if smoke.get("formal_acceptance") is not False or smoke.get("trial_count") != 1:
        raise ValueError("waiver smoke must be one non-formal trial")
    if not str(smoke.get("scenario_id", "")).strip():
        raise ValueError("waiver smoke scenario_id must be non-empty")
    if conditions.get("lighting") != "nominal" or conditions.get("occlusion") != "none":
        raise ValueError("waiver smoke must remain nominal and unobstructed")
    seed = conditions.get("seed")
    if not isinstance(seed, int) or isinstance(seed, bool):
        raise ValueError("smoke seed must be an integer")

    target = _scene_model(smoke.get("target"), "smoke_scenario.target")
    parked_raw = smoke.get("parked_models")
    if not isinstance(parked_raw, list) or len(parked_raw) != 2:
        raise ValueError("waiver smoke must park exactly two non-target models")
    parked = tuple(
        _scene_model(item, f"smoke_scenario.parked_models[{index}]")
        for index, item in enumerate(parked_raw)
    )
    all_ids = {target.target_id, *(item.target_id for item in parked)}
    all_names = {target.model_name, *(item.model_name for item in parked)}
    if len(all_ids) != 3 or len(all_names) != 3:
        raise ValueError("target and parked model identities must be unique")

    confidence_threshold = _finite_float(
        model.get("confidence_threshold"), "model.confidence_threshold"
    )
    if not 0.0 <= confidence_threshold <= 1.0:
        raise ValueError("model confidence threshold must be in [0, 1]")
    model_size = model.get("size_bytes")
    image_size = model.get("image_size")
    if not isinstance(model_size, int) or isinstance(model_size, bool) or model_size <= 0:
        raise ValueError("model.size_bytes must be a positive integer")
    if not isinstance(image_size, int) or isinstance(image_size, bool) or image_size <= 0:
        raise ValueError("model.image_size must be a positive integer")
    if model.get("variant") != "yolo11s_640_train_audit_v1":
        raise ValueError("waiver model variant differs from the selected run")
    if model.get("checkpoint") != "best.pt" or image_size != 640:
        raise ValueError("waiver must retain best.pt at 640-pixel input")

    return PerceptionControlWaiver(
        waiver_id=str(root.get("waiver_id", "")),
        status=str(decision["status"]),
        scope=str(authorization["scope"]),
        model_relative_path=_relative_path(model.get("relative_path"), "model.relative_path"),
        model_size_bytes=model_size,
        model_sha256=_sha256(model.get("sha256"), "model.sha256"),
        confidence_threshold=confidence_threshold,
        image_size=image_size,
        validation_summary=_binding(evidence.get("validation_summary"), "validation_summary"),
        outcome_handoff=_binding(evidence.get("outcome_handoff"), "outcome_handoff"),
        held_out_receipt_relative_path=_relative_path(
            evidence.get("held_out_test_receipt_must_remain_absent"),
            "held_out_test_receipt_must_remain_absent",
        ),
        macro_f1=macro_f1,
        ripe_f1=ripe_f1,
        unripe_f1=unripe_f1,
        required_macro_f1=required_macro_f1,
        engineering_status=str(validation["engineering_status"]),
        detections_topic=detections_topic,
        target_pose_topic=target_pose_topic,
        control_target_topic=control_target_topic,
        ground_truth_association_enabled=(
            routing.get("ground_truth_association_enabled") is True
        ),
        scenario_id=str(smoke.get("scenario_id", "")),
        target=target,
        parked_models=parked,
        seed=seed,
    )


def _resolve_repository_path(repository_root: Path, relative_path: str) -> Path:
    root = repository_root.resolve()
    resolved = (root / relative_path).resolve()
    if resolved != root and root not in resolved.parents:
        raise ValueError("artifact path escapes repository root")
    return resolved


def _verify_binding(repository_root: Path, binding: ArtifactBinding) -> Path:
    path = _resolve_repository_path(repository_root, binding.relative_path)
    if not path.is_file():
        raise ValueError(f"bound artifact is missing: {binding.relative_path}")
    if sha256_file(path) != binding.sha256:
        raise ValueError(f"bound artifact digest differs: {binding.relative_path}")
    return path


def verify_perception_control_waiver(
    waiver: PerceptionControlWaiver,
    repository_root: Path,
    model_path: Path | None = None,
) -> dict[str, Path]:
    """Verify frozen model/evidence bytes and cross-check their metric facts."""

    expected_model = _resolve_repository_path(
        repository_root, waiver.model_relative_path
    )
    actual_model = model_path.resolve() if model_path is not None else expected_model
    if not actual_model.is_file():
        raise ValueError(f"waived model is missing: {actual_model}")
    if actual_model.stat().st_size != waiver.model_size_bytes:
        raise ValueError("waived model size differs from the frozen contract")
    if sha256_file(actual_model) != waiver.model_sha256:
        raise ValueError("waived model digest differs from the frozen contract")

    summary_path = _verify_binding(repository_root, waiver.validation_summary)
    handoff_path = _verify_binding(repository_root, waiver.outcome_handoff)
    receipt_path = _resolve_repository_path(
        repository_root, waiver.held_out_receipt_relative_path
    )
    if receipt_path.exists():
        raise ValueError("held-out real-image test receipt must remain absent")

    summary = json.loads(summary_path.read_text(encoding="utf-8"))
    selected = _mapping(summary.get("selected_checkpoint"), "selected_checkpoint")
    metrics = _mapping(selected.get("validation_metrics"), "validation_metrics")
    per_class = _mapping(metrics.get("per_class"), "validation_metrics.per_class")
    ripe = _mapping(per_class.get("0"), "ripe metrics")
    unripe = _mapping(per_class.get("1"), "unripe metrics")
    facts = (
        (float(selected.get("selected_threshold")), waiver.confidence_threshold, "threshold"),
        (float(metrics.get("macro_f1")), waiver.macro_f1, "macro-F1"),
        (float(ripe.get("f1")), waiver.ripe_f1, "ripe F1"),
        (float(unripe.get("f1")), waiver.unripe_f1, "unripe F1"),
    )
    for observed, frozen, name in facts:
        if not math.isclose(observed, frozen, rel_tol=0.0, abs_tol=1e-12):
            raise ValueError(f"validation {name} differs from waiver contract")
    if selected.get("weight_sha256") != waiver.model_sha256:
        raise ValueError("validation summary selects a different model digest")
    if summary.get("validation_promoted") is not False:
        raise ValueError("historical validation must remain recorded as unpromoted")

    handoff = json.loads(handoff_path.read_text(encoding="utf-8"))
    handoff_validation = _mapping(handoff.get("validation"), "handoff.validation")
    if handoff.get("status") != "REJECTED_VALIDATION_BELOW_GATE":
        raise ValueError("historical outcome status was altered")
    if handoff_validation.get("promoted") is not False:
        raise ValueError("historical promotion result was altered")
    if handoff.get("safety", {}).get("held_out_test_accessed") is not False:
        raise ValueError("historical handoff no longer proves the test seal")

    return {
        "model": actual_model,
        "validation_summary": summary_path,
        "outcome_handoff": handoff_path,
        "held_out_receipt": receipt_path,
    }
