"""Single-support transfer using bounded transport/confirmation callbacks."""


def state_confirmed(*, known, value, generation, desired, after_generation=None):
    return (known and value is desired and
            (after_generation is None or generation > after_generation))


def transfer_stem_to_gripper(*, release_stem, stem_released, attach_gripper,
                            gripper_attached, detach_gripper, gripper_detached):
    release_stem()
    if not stem_released():
        return False, "stem release unconfirmed; no gripper attachment requested"
    attach_gripper()
    if gripper_attached():
        return True, "Gazebo attachment confirmed after stem release"
    # Cancel even if the delayed attach request has not yet created a joint.
    detach_gripper()
    cancelled = gripper_detached()
    return False, ("stem released but gripper attachment unconfirmed; "
                   + ("gripper detach confirmed" if cancelled else "gripper detach unconfirmed"))
