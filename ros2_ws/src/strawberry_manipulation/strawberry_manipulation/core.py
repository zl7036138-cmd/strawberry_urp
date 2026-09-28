"""Pure-Python manipulation state machine with a pluggable motion backend."""

from __future__ import annotations

from dataclasses import dataclass, replace
from enum import IntEnum
import math
from typing import Callable, Protocol

from .collision_policy import (
    CollisionPhase,
    selected_fruit_contact_is_authorized,
)
from .payload_lifecycle import PayloadState


LEFT_SINGLE_FRUIT = "LEFT_SINGLE_FRUIT"
RIGHT_SINGLE_FRUIT = "RIGHT_SINGLE_FRUIT"
BILATERAL_SAME_FRUIT = "BILATERAL_SAME_FRUIT"
NO_FRUIT_CONTACT = "NO_FRUIT_CONTACT"
AMBIGUOUS_FRUIT_CONTACT = "AMBIGUOUS_FRUIT_CONTACT"
CONTACT_CLASS_UNAVAILABLE = "CONTACT_CLASS_UNAVAILABLE"


class FailureCode(IntEnum):
    NONE = 0
    NO_TARGET = 1
    LOW_CONFIDENCE = 2
    DEPTH_INVALID = 3
    TF_TIMEOUT = 4
    UNREACHABLE = 5
    PLANNING_FAILED = 6
    COLLISION = 7
    GRASP_FAILED = 8
    PLACE_FAILED = 9
    STALE_DATA = 10


class RecoveryDisposition(IntEnum):
    """Typed permission for motion after an action result.

    Zero is intentionally the fail-closed value used when the executor cannot
    prove that an independent recovery-home command is safe.
    """

    MOTION_WITHHELD = 0
    AT_HOME = 1
    HOME_REQUIRED = 2


@dataclass(frozen=True)
class Pose:
    x: float
    y: float
    z: float
    qx: float = 0.0
    qy: float = 0.0
    qz: float = 0.0
    qw: float = 1.0

    def normalized(self) -> "Pose":
        norm = math.sqrt(self.qx**2 + self.qy**2 + self.qz**2 + self.qw**2)
        if norm <= 1e-9:
            raise ValueError("pose quaternion must be non-zero")
        return replace(
            self,
            qx=self.qx / norm,
            qy=self.qy / norm,
            qz=self.qz / norm,
            qw=self.qw / norm,
        )


@dataclass(frozen=True)
class ExecutionResult:
    success: bool
    failure_code: FailureCode
    message: str
    planning_time_sec: float
    execution_time_sec: float
    stages: tuple[str, ...]
    recovery_disposition: RecoveryDisposition = RecoveryDisposition.MOTION_WITHHELD
    payload_state: PayloadState = PayloadState.EMPTY


@dataclass(frozen=True)
class MotionOutcome:
    success: bool
    planning_time_sec: float = 0.0
    execution_time_sec: float = 0.0
    collision: bool = False


class MotionBackend(Protocol):
    def prepare_pick(self, target_id: int, target_pose: Pose) -> bool: ...

    def allow_target_contact(self, target_id: int) -> bool: ...

    def restore_target_collision(self, target_id: int) -> bool: ...

    def move_to(self, pose: Pose, stage: str) -> MotionOutcome: ...

    def close_gripper(self) -> bool: ...

    def gripper_centering_offset_m(self) -> float | None: ...

    def gripper_fruit_contact_class(self) -> str | None: ...

    def open_gripper(self) -> bool: ...

    def attach(self, target_id: int) -> bool: ...

    def detach(self, target_id: int) -> bool: ...

    def return_via_recorded_place_route(self) -> MotionOutcome: ...

    def move_home(self) -> bool: ...

    def fruit_in_bin(self, target_id: int, stable_for_sec: float) -> bool: ...


def offset_pose(
    pose: Pose, *, dx: float = 0.0, dy: float = 0.0, dz: float = 0.0
) -> Pose:
    """Apply a base-frame translation while preserving orientation."""

    return replace(pose.normalized(), x=pose.x + dx, y=pose.y + dy, z=pose.z + dz)


def offset_along_local_z(pose: Pose, distance_m: float) -> Pose:
    """Translate along the pose's local +Z tool axis."""

    pose = pose.normalized()
    if not math.isfinite(distance_m):
        raise ValueError("tool-axis offset must be finite")
    axis_x = 2.0 * (pose.qx * pose.qz + pose.qw * pose.qy)
    axis_y = 2.0 * (pose.qy * pose.qz - pose.qw * pose.qx)
    axis_z = 1.0 - 2.0 * (pose.qx**2 + pose.qy**2)
    return replace(
        pose,
        x=pose.x + distance_m * axis_x,
        y=pose.y + distance_m * axis_y,
        z=pose.z + distance_m * axis_z,
    )


def offset_along_local_y(pose: Pose, distance_m: float) -> Pose:
    """Translate along the pose's local +Y finger-closing axis."""

    pose = pose.normalized()
    if not math.isfinite(distance_m):
        raise ValueError("finger-axis offset must be finite")
    axis_x = 2.0 * (pose.qx * pose.qy - pose.qw * pose.qz)
    axis_y = 1.0 - 2.0 * (pose.qx**2 + pose.qz**2)
    axis_z = 2.0 * (pose.qy * pose.qz + pose.qw * pose.qx)
    return replace(
        pose,
        x=pose.x + distance_m * axis_x,
        y=pose.y + distance_m * axis_y,
        z=pose.z + distance_m * axis_z,
    )


def rotate_about_base_z(pose: Pose, angle_rad: float) -> Pose:
    """Rotate a pose orientation about base-frame Z."""

    pose = pose.normalized()
    if not math.isfinite(angle_rad):
        raise ValueError("base-Z rotation must be finite")
    half = angle_rad / 2.0
    rz = math.sin(half)
    rw = math.cos(half)
    # q_rotated = q_z * q_original
    return replace(
        pose,
        qx=rw * pose.qx - rz * pose.qy,
        qy=rw * pose.qy + rz * pose.qx,
        qz=rw * pose.qz + rz * pose.qw,
        qw=rw * pose.qw - rz * pose.qz,
    ).normalized()


def alternate_approach(pose: Pose) -> Pose:
    """Rotate the tool 90 degrees about base Z for the approach retry."""

    return rotate_about_base_z(pose, math.pi / 2.0)


DEFAULT_GRASP_QUATERNION = (1.0, 0.0, 0.0, 0.0)
DEFAULT_TOOL_CENTER_OFFSET_M = 0.1054
DEFAULT_PREGRASP_OFFSET_M = 0.15


def hand_pose_for_fruit_center(
    center: Pose,
    *,
    quaternion: tuple[float, float, float, float] = DEFAULT_GRASP_QUATERNION,
    tool_center_offset_m: float = DEFAULT_TOOL_CENTER_OFFSET_M,
) -> Pose:
    """Convert a fruit-centre pose into the matching Panda hand pose."""

    if len(quaternion) != 4:
        raise ValueError("grasp quaternion must contain four values")
    if not math.isfinite(tool_center_offset_m) or tool_center_offset_m <= 0.0:
        raise ValueError("tool center offset must be positive and finite")
    qx, qy, qz, qw = (float(value) for value in quaternion)
    oriented_center = replace(center, qx=qx, qy=qy, qz=qz, qw=qw).normalized()
    return offset_along_local_z(oriented_center, -tool_center_offset_m)


def pregrasp_pose_for_fruit_center(
    center: Pose,
    *,
    quaternion: tuple[float, float, float, float] = DEFAULT_GRASP_QUATERNION,
    tool_center_offset_m: float = DEFAULT_TOOL_CENTER_OFFSET_M,
    pregrasp_offset_m: float = DEFAULT_PREGRASP_OFFSET_M,
) -> Pose:
    """Return the exact pre-grasp goal used by the execution state machine."""

    if not math.isfinite(pregrasp_offset_m) or pregrasp_offset_m <= 0.0:
        raise ValueError("pregrasp offset must be positive and finite")
    hand_pose = hand_pose_for_fruit_center(
        center,
        quaternion=quaternion,
        tool_center_offset_m=tool_center_offset_m,
    )
    return offset_along_local_z(hand_pose, -pregrasp_offset_m)


def bounded_pregrasp_candidates_for_fruit_center(
    center: Pose,
    *,
    quaternion: tuple[float, float, float, float] = DEFAULT_GRASP_QUATERNION,
    tool_center_offset_m: float = DEFAULT_TOOL_CENTER_OFFSET_M,
    pregrasp_offset_m: float = DEFAULT_PREGRASP_OFFSET_M,
) -> tuple[Pose, Pose, Pose, Pose]:
    """Return the four bounded finger orientations used by execution.

    A quarter turn about base Z leaves the vertical tool axis unchanged while
    giving the Panda wrist four deterministic IK branches.  Every candidate is
    still planned and collision checked; this is pose coverage, not a relaxed
    reachability or safety threshold.
    """

    primary = pregrasp_pose_for_fruit_center(
        center,
        quaternion=quaternion,
        tool_center_offset_m=tool_center_offset_m,
        pregrasp_offset_m=pregrasp_offset_m,
    )
    return tuple(
        rotate_about_base_z(primary, quarter_turn * math.pi / 2.0)
        for quarter_turn in range(4)
    )


class PickAndPlaceExecutor:
    def __init__(
        self,
        backend: MotionBackend,
        *,
        pregrasp_offset_m: float = DEFAULT_PREGRASP_OFFSET_M,
        retreat_distance_m: float = 0.08,
        bin_stability_sec: float = 1.0,
        tool_center_offset_m: float = DEFAULT_TOOL_CENTER_OFFSET_M,
        grasp_quaternion: tuple[float, float, float, float] = (
            DEFAULT_GRASP_QUATERNION
        ),
        place_quaternion: tuple[float, float, float, float] = (
            1.0,
            0.0,
            0.0,
            0.0,
        ),
        target_pose_refiner: Callable[[int, Pose], Pose] | None = None,
        maximum_grasp_centering_correction_m: float = 0.010,
        minimum_grasp_centering_correction_m: float = 0.001,
    ) -> None:
        if pregrasp_offset_m <= 0.0 or retreat_distance_m <= 0.0:
            raise ValueError("motion offsets must be positive")
        if tool_center_offset_m <= 0.0:
            raise ValueError("tool center offset must be positive")
        if len(grasp_quaternion) != 4 or len(place_quaternion) != 4:
            raise ValueError("grasp and place quaternions must contain four values")
        if (
            not math.isfinite(maximum_grasp_centering_correction_m)
            or not math.isfinite(minimum_grasp_centering_correction_m)
            or minimum_grasp_centering_correction_m <= 0.0
            or maximum_grasp_centering_correction_m
            <= minimum_grasp_centering_correction_m
        ):
            raise ValueError("grasp centering correction bounds are invalid")
        self.backend = backend
        self.pregrasp_offset_m = pregrasp_offset_m
        self.retreat_distance_m = retreat_distance_m
        self.bin_stability_sec = bin_stability_sec
        self.tool_center_offset_m = tool_center_offset_m
        self.grasp_quaternion = tuple(float(value) for value in grasp_quaternion)
        self.place_quaternion = tuple(float(value) for value in place_quaternion)
        self.target_pose_refiner = target_pose_refiner
        self.maximum_grasp_centering_correction_m = float(
            maximum_grasp_centering_correction_m
        )
        self.minimum_grasp_centering_correction_m = float(
            minimum_grasp_centering_correction_m
        )
        # A retained fruit makes the planning-scene lifecycle intentionally
        # non-reentrant.  There is no generic autonomous recovery that can
        # prove an arbitrary carried payload route is safe, so a later goal is
        # refused until an explicit operator/supervisor intervention restarts
        # the manipulation node in a known empty state.
        self._payload_state = PayloadState.EMPTY
        self._retained_payload_target_id: int | None = None

    @property
    def payload_state(self) -> PayloadState:
        """Current lifecycle state, exposed for diagnostics and pure tests."""

        return self._payload_state

    def _set_payload_state(self, state: PayloadState) -> None:
        self._payload_state = PayloadState(state)

    def _hand_pose_for_fruit_center(
        self,
        center: Pose,
        quaternion: tuple[float, float, float, float],
    ) -> Pose:
        return hand_pose_for_fruit_center(
            center,
            quaternion=quaternion,
            tool_center_offset_m=self.tool_center_offset_m,
        )

    def execute(
        self,
        target_id: int,
        target_pose: Pose,
        place_pose: Pose,
        feedback: Callable[[str, float], None] | None = None,
    ) -> ExecutionResult:
        if self._retained_payload_target_id is not None:
            return ExecutionResult(
                False,
                FailureCode.PLANNING_FAILED,
                "payload interlock active for target "
                f"{self._retained_payload_target_id}; retained fruit remains "
                f"in {self._payload_state.name}, so all new motion is withheld",
                0.0,
                0.0,
                ("PAYLOAD_INTERLOCK",),
                RecoveryDisposition.MOTION_WITHHELD,
                self._payload_state,
            )

        self._set_payload_state(PayloadState.EMPTY)
        if target_id <= 0:
            return ExecutionResult(
                False,
                FailureCode.NO_TARGET,
                "target_id must be positive",
                0.0,
                0.0,
                (),
                RecoveryDisposition.HOME_REQUIRED,
            )

        if not self.backend.prepare_pick(target_id, target_pose):
            return ExecutionResult(
                False,
                FailureCode.PLANNING_FAILED,
                "failed to configure target collision obstacle",
                0.0,
                0.0,
                (),
                RecoveryDisposition.HOME_REQUIRED,
            )

        restore_succeeded = False
        result: ExecutionResult | None = None
        try:
            result = self._execute_prepared(
                target_id,
                target_pose,
                place_pose,
                feedback,
            )
        finally:
            # A held fruit must stay attached to both the physical simulator and
            # MoveIt's carried-body model. Restoring its old world obstacle would
            # erase that lifecycle and can duplicate the fruit in the planning
            # scene. All ordinary post-grasp failures therefore stop in place.
            if self._payload_state.retains_fruit:
                self._retained_payload_target_id = target_id
                restore_succeeded = True
            else:
                # _execute_prepared performs all physical failure recovery (and
                # the normal move home) before returning. Keeping scene
                # restoration here closes every empty/released contact corridor.
                try:
                    restore_succeeded = bool(
                        self.backend.restore_target_collision(target_id)
                    )
                except Exception:
                    restore_succeeded = False

        # Let an unexpected backend exception propagate after the finally block.
        assert result is not None

        if restore_succeeded and result.success:
            if feedback is not None:
                feedback("DONE", 1.00)
            return replace(result, stages=result.stages + ("DONE",))
        if restore_succeeded:
            return result

        message = f"{result.message}; failed to restore target collision obstacle"
        return replace(
            result,
            success=False,
            failure_code=(
                FailureCode.PLANNING_FAILED if result.success else result.failure_code
            ),
            message=message,
            recovery_disposition=RecoveryDisposition.MOTION_WITHHELD,
        )

    def _execute_prepared(
        self,
        target_id: int,
        target_pose: Pose,
        place_pose: Pose,
        feedback: Callable[[str, float], None] | None = None,
    ) -> ExecutionResult:
        stages: list[str] = []
        planning_time = 0.0
        execution_time = 0.0
        attached = False
        unattached_recovery_retreat: Pose | None = None
        place_route_recovery_available = False

        def mark(stage: str, progress: float) -> None:
            stages.append(stage)
            if feedback is not None:
                feedback(stage, progress)

        def fail(
            code: FailureCode,
            message: str,
            *,
            recover_home: bool = True,
        ) -> ExecutionResult:
            nonlocal attached, place_route_recovery_available
            nonlocal planning_time, execution_time
            recovery_disposition = RecoveryDisposition.MOTION_WITHHELD
            # Once attachment is confirmed, a generic detach/open/home recovery
            # turns a transport-planning failure into an uncontrolled fruit
            # release. Retain the physical constraint, carried-body collision
            # geometry and closed gripper instead. Only the authorized AT_BIN
            # release transition below may detach a held fruit.
            if attached:
                mark("PAYLOAD_HELD_MOTION_WITHHELD", 0.75)
                return ExecutionResult(
                    False,
                    code,
                    f"{message}; payload remains {self._payload_state.name} with "
                    "gripper closed, so detach/open/recovery motion are withheld",
                    planning_time,
                    execution_time,
                    tuple(stages),
                    RecoveryDisposition.MOTION_WITHHELD,
                    self._payload_state,
                )

            opened = self.backend.open_gripper()
            if self._payload_state is PayloadState.CONTACT and opened:
                self._set_payload_state(PayloadState.EMPTY)
            if not opened:
                message = f"{message}; failed to open empty or released gripper"
            elif recover_home:
                retreat_succeeded = True
                if place_route_recovery_available:
                    # Once the hand has entered the collection-bin corridor,
                    # an independent home plan can sweep through the bin or
                    # neighbouring fruit.  The successful forward PLACE path
                    # is the only reviewed way back to the safe retreat
                    # checkpoint, including release/verification failures.
                    return_motion = self.backend.return_via_recorded_place_route()
                    place_route_recovery_available = False
                    planning_time += return_motion.planning_time_sec
                    execution_time += return_motion.execution_time_sec
                    stages.append("RECOVERY_RETURN_ROUTE")
                    retreat_succeeded = return_motion.success
                    if not retreat_succeeded:
                        message = (
                            f"{message}; recorded place-route recovery failed, "
                            "home motion withheld"
                        )
                elif unattached_recovery_retreat is not None:
                    retreat = self.backend.move_to(
                        unattached_recovery_retreat,
                        "RECOVERY_RETREAT",
                    )
                    planning_time += retreat.planning_time_sec
                    execution_time += retreat.execution_time_sec
                    stages.append("RECOVERY_RETREAT")
                    retreat_succeeded = retreat.success
                    if not retreat_succeeded:
                        message = (
                            f"{message}; recovery retreat failed, home motion withheld"
                        )
                if retreat_succeeded:
                    if self.backend.move_home():
                        recovery_disposition = RecoveryDisposition.AT_HOME
                    else:
                        message = f"{message}; recovery home motion failed"
            return ExecutionResult(
                False,
                code,
                message,
                planning_time,
                execution_time,
                tuple(stages),
                recovery_disposition,
                self._payload_state,
            )

        if not self.backend.open_gripper():
            return ExecutionResult(
                False,
                FailureCode.GRASP_FAILED,
                "failed to open gripper before approach",
                planning_time,
                execution_time,
                tuple(stages),
                RecoveryDisposition.HOME_REQUIRED,
            )

        mark("PLAN", 0.10)
        # The goal pose denotes the fruit centre. Convert it to the Panda hand
        # origin, then approach along the hand's local tool axis.
        primary_grasp_pose = self._hand_pose_for_fruit_center(
            target_pose, self.grasp_quaternion
        )
        pregrasp_candidates = bounded_pregrasp_candidates_for_fruit_center(
            target_pose,
            quaternion=self.grasp_quaternion,
            tool_center_offset_m=self.tool_center_offset_m,
            pregrasp_offset_m=self.pregrasp_offset_m,
        )
        pregrasp = None
        grasp_pose = None
        last_approach = None
        selected_orientation_index = None
        for orientation_index, candidate in enumerate(pregrasp_candidates):
            stage = "APPROACH" if orientation_index == 0 else (
                "APPROACH_RETRY"
                if orientation_index == 1
                else f"APPROACH_RETRY_{orientation_index}"
            )
            approach = self.backend.move_to(candidate, stage)
            planning_time += approach.planning_time_sec
            execution_time += approach.execution_time_sec
            last_approach = approach
            if approach.success:
                pregrasp = candidate
                grasp_pose = rotate_about_base_z(
                    primary_grasp_pose,
                    orientation_index * math.pi / 2.0,
                )
                selected_orientation_index = orientation_index
                break
            # Once a controller moved, it is unsafe to search another branch
            # from an unreviewed physical state.
            if approach.execution_time_sec > 1.0e-6:
                break
        if pregrasp is None or grasp_pose is None:
            code = (
                FailureCode.COLLISION
                if last_approach is not None and last_approach.collision
                else FailureCode.PLANNING_FAILED
            )
            return fail(
                code,
                "approach planning failed after four bounded orientations",
                recover_home=False,
            )

        mark("APPROACH", 0.25)
        if self.target_pose_refiner is not None:
            try:
                target_pose = self.target_pose_refiner(
                    target_id, target_pose
                ).normalized()
            except Exception as exc:
                return fail(
                    FailureCode.STALE_DATA,
                    f"target pose refinement failed: {exc}",
                )
            primary_grasp_pose = self._hand_pose_for_fruit_center(
                target_pose, self.grasp_quaternion
            )
            grasp_pose = rotate_about_base_z(
                primary_grasp_pose,
                int(selected_orientation_index) * math.pi / 2.0,
            )
        if not selected_fruit_contact_is_authorized(CollisionPhase.GRASP_CONTACT):
            return fail(
                FailureCode.PLANNING_FAILED,
                "selected-fruit contact is not authorized in this execution phase",
            )
        if not self.backend.allow_target_contact(target_id):
            return fail(
                FailureCode.PLANNING_FAILED,
                "failed to open the target contact corridor",
            )
        # Until bilateral contact creates an attachment, every failure at the
        # fruit first reverses the already collision-checked approach.  A
        # direct long home sweep from physical single-sided contact is both
        # less predictable and harder for the controller to track.
        unattached_recovery_retreat = pregrasp
        grasp_motion = self.backend.move_to(grasp_pose, "GRASP_POSE")
        planning_time += grasp_motion.planning_time_sec
        execution_time += grasp_motion.execution_time_sec
        if not grasp_motion.success and grasp_motion.collision:
            # A moving fruit can shift the final Cartesian descent onto a
            # low-elbow IK branch even though the reviewed pre-grasp remains
            # valid. Keep the same vertical tool axis and inspect the three
            # remaining quarter-turn finger orientations. Every reorientation
            # and descent remains collision checked, and the search is bounded.
            alternate_orientation_indices = tuple(
                index
                for index in range(4)
                if index != selected_orientation_index
            )
            for retry_number, orientation_index in enumerate(
                alternate_orientation_indices, start=1
            ):
                alternate_grasp_pose = rotate_about_base_z(
                    primary_grasp_pose,
                    orientation_index * math.pi / 2.0,
                )
                alternate_pregrasp_pose = offset_along_local_z(
                    alternate_grasp_pose,
                    -self.pregrasp_offset_m,
                )
                suffix = "" if retry_number == 1 else f"_{retry_number}"
                retry_preparation = self.backend.move_to(
                    alternate_pregrasp_pose,
                    f"GRASP_RETRY_PREP{suffix}",
                )
                planning_time += retry_preparation.planning_time_sec
                execution_time += retry_preparation.execution_time_sec
                grasp_motion = retry_preparation
                if not retry_preparation.success:
                    continue
                unattached_recovery_retreat = alternate_pregrasp_pose
                grasp_motion = self.backend.move_to(
                    alternate_grasp_pose,
                    f"GRASP_POSE_RETRY{suffix}",
                )
                planning_time += grasp_motion.planning_time_sec
                execution_time += grasp_motion.execution_time_sec
                if grasp_motion.success:
                    grasp_pose = alternate_grasp_pose
                    unattached_recovery_retreat = alternate_pregrasp_pose
                    break
        if not grasp_motion.success:
            code = (
                FailureCode.COLLISION
                if grasp_motion.collision
                else FailureCode.PLANNING_FAILED
            )
            return fail(
                code,
                "failed to reach grasp pose after three alternate orientations",
            )

        mark("GRASP", 0.40)
        gripper_closed = self.backend.close_gripper()
        if gripper_closed:
            self._set_payload_state(PayloadState.CONTACT)
            mark("CONTACT", 0.42)
        attachment_confirmed = (
            self.backend.attach(target_id) if gripper_closed else False
        )
        if not attachment_confirmed:
            # A single-finger stall or strict dual-contact rejection can be
            # evidence of an off-centre grasp, but joint asymmetry alone also
            # occurs when foliage or the mechanism blocks a finger.  The
            # generalized runtime therefore requires an identity-free raw
            # fruit-contact class consistent with the measured direction.
            # Legacy fixed-scene backends retain the bounded orthogonal retry.
            contact_class_reader = getattr(
                self.backend, "gripper_fruit_contact_class", None
            )
            fruit_contact_class = (
                contact_class_reader() if callable(contact_class_reader) else None
            )
            centering_offset_m = self.backend.gripper_centering_offset_m()
            if not self.backend.open_gripper():
                return fail(
                    FailureCode.GRASP_FAILED,
                    "failed to reopen gripper after asymmetric contact",
                )
            self._set_payload_state(PayloadState.EMPTY)
            if (
                centering_offset_m is not None
                and abs(centering_offset_m)
                > self.maximum_grasp_centering_correction_m
            ):
                return fail(
                    FailureCode.GRASP_FAILED,
                    "measured finger asymmetry requires an out-of-bounds "
                    "centering correction",
                )
            if fruit_contact_class is not None:
                if centering_offset_m is None:
                    return fail(
                        FailureCode.GRASP_FAILED,
                        "fruit contact was observed but finger asymmetry could "
                        "not be measured safely",
                    )
                if (
                    abs(centering_offset_m)
                    < self.minimum_grasp_centering_correction_m
                ):
                    return fail(
                        FailureCode.GRASP_FAILED,
                        "single-sided fruit contact has no reliable measurable "
                        "centering correction",
                    )
                if fruit_contact_class == LEFT_SINGLE_FRUIT:
                    centering_offset_m = abs(centering_offset_m)
                elif fruit_contact_class == RIGHT_SINGLE_FRUIT:
                    centering_offset_m = -abs(centering_offset_m)
                else:
                    return fail(
                        FailureCode.GRASP_FAILED,
                        "centering retry rejected because anonymous fruit contact "
                        f"class {fruit_contact_class} is not a unique single-sided "
                        "fruit contact",
                    )
            use_centering = (
                centering_offset_m is not None
                and abs(centering_offset_m)
                >= self.minimum_grasp_centering_correction_m
            )
            alternate_grasp_pose = (
                offset_along_local_y(grasp_pose, centering_offset_m)
                if use_centering
                else rotate_about_base_z(grasp_pose, math.pi / 2.0)
            )
            alternate_pregrasp_pose = rotate_about_base_z(
                offset_along_local_z(grasp_pose, -self.pregrasp_offset_m),
                math.pi / 2.0,
            )
            if use_centering:
                alternate_pregrasp_pose = offset_along_local_y(
                    offset_along_local_z(grasp_pose, -self.pregrasp_offset_m),
                    centering_offset_m,
                )
            retry_stage_prefix = (
                "CONTACT_CENTERING" if use_centering else "CONTACT_RETRY"
            )
            retry_preparation = self.backend.move_to(
                alternate_pregrasp_pose,
                f"{retry_stage_prefix}_PREP",
            )
            planning_time += retry_preparation.planning_time_sec
            execution_time += retry_preparation.execution_time_sec
            if retry_preparation.success:
                unattached_recovery_retreat = alternate_pregrasp_pose
                retry_grasp = self.backend.move_to(
                    alternate_grasp_pose,
                    f"{retry_stage_prefix}_GRASP",
                )
                planning_time += retry_grasp.planning_time_sec
                execution_time += retry_grasp.execution_time_sec
                if retry_grasp.success:
                    grasp_pose = alternate_grasp_pose
                    unattached_recovery_retreat = alternate_pregrasp_pose
                    gripper_closed = self.backend.close_gripper()
                    if gripper_closed:
                        self._set_payload_state(PayloadState.CONTACT)
                        mark("CONTACT", 0.42)
                    attachment_confirmed = (
                        self.backend.attach(target_id)
                        if gripper_closed
                        else False
                    )
        if not attachment_confirmed:
            return fail(
                FailureCode.GRASP_FAILED,
                "dual-finger contact or simulated attachment failed after one "
                "bounded contact retry",
            )
        attached = True
        self._set_payload_state(PayloadState.HOLDING)
        unattached_recovery_retreat = None
        mark("HOLDING", 0.45)

        mark("RETREAT", 0.55)
        # Escape is the reverse of the active tool-axis approach, not an
        # implicitly base-vertical displacement.  This remains compatible with
        # the current top-down grasp, while preserving the correct direction
        # if a later bounded candidate changes the hand orientation.
        retreat = offset_along_local_z(grasp_pose, -self.retreat_distance_m)
        retreat_motion = self.backend.move_to(retreat, "RETREAT")
        planning_time += retreat_motion.planning_time_sec
        execution_time += retreat_motion.execution_time_sec
        if not retreat_motion.success:
            return fail(FailureCode.PLANNING_FAILED, "retreat planning failed")
        self._set_payload_state(PayloadState.ESCAPED)
        mark("ESCAPED", 0.60)

        mark("PLACE", 0.75)
        # Enter the open bin from above.  Local +Z points down, so the hand
        # origin remains above the requested fruit-centre release point while
        # the fingers descend between the collision walls.
        place_hand_pose = self._hand_pose_for_fruit_center(
            place_pose, self.place_quaternion
        )
        place_motion = self.backend.move_to(place_hand_pose, "PLACE")
        planning_time += place_motion.planning_time_sec
        execution_time += place_motion.execution_time_sec
        if not place_motion.success:
            if place_motion.execution_time_sec > 1.0e-6:
                return fail(
                    FailureCode.PLACE_FAILED,
                    "failed to reach collection bin after controller motion "
                    "began; no complete recorded return route is available, "
                    "home motion withheld",
                    recover_home=False,
                )
            return fail(FailureCode.PLACE_FAILED, "failed to reach collection bin")
        place_route_recovery_available = True
        self._set_payload_state(PayloadState.AT_BIN)
        mark("AT_BIN", 0.82)
        # Remove the rigid simulation constraint before opening the physical
        # fingers.  Opening while the fruit is still welded to the hand can
        # preload it against one finger; the later detach then releases that
        # asymmetric impulse and makes an otherwise central bin drop
        # nondeterministic.  Closed fingers continue to support the fruit for
        # the short interval between detach and the symmetric open command.
        if not self.backend.detach(target_id):
            return fail(FailureCode.PLACE_FAILED, "failed to detach fruit for release")
        attached = False
        self._set_payload_state(PayloadState.RELEASED)
        if not self.backend.open_gripper():
            return fail(FailureCode.PLACE_FAILED, "failed to open gripper for release")
        mark("RELEASED", 0.86)

        mark("VERIFY", 0.90)
        if not self.backend.fruit_in_bin(target_id, self.bin_stability_sec):
            return fail(FailureCode.PLACE_FAILED, "fruit did not remain in bin")

        # Leave the bin along the same connected transport corridor that was
        # actually executed on the way in.  The backend retimes that route
        # from fresh joint feedback and revalidates it against the current
        # open-gripper collision scene.  A failed check is a hard stop: an
        # independently replanned long sweep from inside the bin is exactly
        # the recovery path that contacted an unharvested fruit and pinned
        # joint 5 in the diagnosed DART failures.
        mark("RETURN_ROUTE", 0.95)
        return_motion = self.backend.return_via_recorded_place_route()
        place_route_recovery_available = False
        planning_time += return_motion.planning_time_sec
        execution_time += return_motion.execution_time_sec
        if not return_motion.success:
            return ExecutionResult(
                False,
                (
                    FailureCode.COLLISION
                    if return_motion.collision
                    else FailureCode.PLANNING_FAILED
                ),
                "pick-and-place completed but recorded place-route return failed; "
                "home motion withheld",
                planning_time,
                execution_time,
                tuple(stages),
                RecoveryDisposition.MOTION_WITHHELD,
                self._payload_state,
            )

        if not self.backend.move_home():
            return ExecutionResult(
                False,
                FailureCode.PLANNING_FAILED,
                "pick-and-place completed but final home motion failed",
                planning_time,
                execution_time,
                tuple(stages),
                RecoveryDisposition.MOTION_WITHHELD,
                self._payload_state,
            )
        return ExecutionResult(
            True,
            FailureCode.NONE,
            "pick-and-place completed",
            planning_time,
            execution_time,
            tuple(stages),
            RecoveryDisposition.AT_HOME,
            self._payload_state,
        )
