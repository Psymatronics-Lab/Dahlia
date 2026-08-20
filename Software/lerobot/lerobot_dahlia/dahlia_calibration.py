"""Calibration for the Dahlia arm.

Unlike a hobby servo arm, Dahlia does not need an interactive calibration pass. Its
travel is already fixed in firmware: `sketch/src/low_level/servo_interface.h` holds a
`joint_configs` table that clamps every commanded tick, and the MCU enforces it no
matter what the host asks for. Those stops are the calibration, so it is defined here
rather than measured by hand-moving the arm.

`joint_configs` rows are `{servo_id, min_pos, max_pos, min_angle, max_angle}`, where
`min_pos` is the tick the joint sits at when it is at `min_angle`:

    {1, -1250,  1250, -1.917,  1.917}   base
    {2,  2200,  -200, -3.375,  0.307}   shoulder
    {3,  2250,     0,  0.000,  3.451}   elbow
    {4,  1024, -1024, -1.534,  1.534}   wrist_pitch
    {5,  2700,   800, -1.457,  1.457}   wrist_roll

Two things fall out of that table:

  * `range_min` / `range_max` are the ticks sorted numerically, since MotorCalibration
    wants an interval rather than a direction.
  * `drive_mode` is 1 wherever ticks *decrease* as the angle increases, which is every
    joint except the base. Setting it that way is what makes -100 mean `min_angle` and
    +100 mean `max_angle` on all five joints, instead of the normalised direction
    flipping about depending on how each servo happens to be mounted.

That second point is worth holding onto: because normalisation is monotonic in angle
for every joint, converting an angle to a normalised action is the same expression
everywhere, which is what `dahlia_teleop` relies on.

`homing_offset` is 0 throughout. The ranges above are already in the servos' own raw
tick space, and the bus applies offsets host-side, so there is nothing to shift.
"""

from lerobot.motors import MotorCalibration

# name: (servo id, tick at min_angle, tick at max_angle, min_angle, max_angle)
# Mirrors joint_configs in servo_interface.h. Radians.
JOINT_CONFIGS = {
    "base": (1, -1250, 1250, -1.917, 1.917),
    "shoulder": (2, 2200, -200, -3.375, 0.307),
    "elbow": (3, 2250, 0, 0.0, 3.451),
    "wrist_pitch": (4, 1024, -1024, -1.534, 1.534),
    "wrist_roll": (5, 2700, 800, -1.457, 1.457),
}

# The clamp is a PWM hobby servo with no encoder, so its "calibration" is just the
# byte range the firmware accepts. 0 is fully open, 255 fully closed.
GRIPPER_ID = 6
GRIPPER_MIN = 0
GRIPPER_MAX = 255

JOINT_ANGLE_LIMITS = {
    name: (min_angle, max_angle)
    for name, (_, _, _, min_angle, max_angle) in JOINT_CONFIGS.items()
}


def _joint_calibration(name: str) -> MotorCalibration:
    servo_id, tick_at_min, tick_at_max, _, _ = JOINT_CONFIGS[name]

    return MotorCalibration(
        id=servo_id,
        # 1 where the servo counts down as the joint angle goes up.
        drive_mode=int(tick_at_max < tick_at_min),
        homing_offset=0,
        range_min=min(tick_at_min, tick_at_max),
        range_max=max(tick_at_min, tick_at_max),
    )


def dahlia_calibration() -> dict[str, MotorCalibration]:
    """The arm's calibration, as fixed by the flashed joint limits."""
    calibration = {name: _joint_calibration(name) for name in JOINT_CONFIGS}

    calibration["gripper"] = MotorCalibration(
        id=GRIPPER_ID,
        drive_mode=0,
        homing_offset=0,
        range_min=GRIPPER_MIN,
        range_max=GRIPPER_MAX,
    )

    return calibration


def angle_to_normalized(name: str, angle: float) -> float:
    """Radians to the arm's normalised action space, [-100, 100].

    One expression for every joint, because `drive_mode` above is chosen so that
    normalised position always increases with joint angle.
    """
    low, high = JOINT_ANGLE_LIMITS[name]
    bounded = min(high, max(low, angle))

    return ((bounded - low) / (high - low)) * 200.0 - 100.0


def gripper_to_normalized(closure: int) -> float:
    """Clamp closure byte to [0, 100], 0 fully open."""
    bounded = min(GRIPPER_MAX, max(GRIPPER_MIN, closure))

    return (bounded - GRIPPER_MIN) / (GRIPPER_MAX - GRIPPER_MIN) * 100.0
