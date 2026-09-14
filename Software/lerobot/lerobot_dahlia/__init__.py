"""LeRobot plugin for the Dahlia arm.

Importing this package is what registers `dahlia` and `dahlia_sec` with LeRobot.
Both are looked up by name out of a registry that the `@register_subclass`
decorators populate at import time, so `--robot.type=dahlia` only resolves once
something has imported this module.

    lerobot-teleoperate \\
        --robot.type=dahlia --robot.port=http://dahlia.local:9000 \\
        --teleop.type=dahlia_sec --teleop.port=http://dahlia.local:9000 \\
        --robot.discover_packages_path=lerobot_dahlia

See README.md for how the discovery flag works and what to do if your LeRobot
version does not have it.
"""

from .dahlia_bus import GRIPPER_NAME, JOINT_NAMES, HXServoMotorsBus, dahlia_motors
from .dahlia_calibration import (
    JOINT_ANGLE_LIMITS,
    JOINT_CONFIGS,
    angle_to_normalized,
    dahlia_calibration,
    gripper_to_normalized,
)
from .dahlia_robot import (
    OVERHEAD_CAMERA,
    WRIST_CAMERA,
    DahliaRobot,
    DahliaRobotConfig,
    dahlia_cameras,
)
from .dahlia_teleop import DahliaSECTeleop, DahliaSECTeleopConfig

__all__ = [
    "DahliaRobot",
    "DahliaRobotConfig",
    "OVERHEAD_CAMERA",
    "WRIST_CAMERA",
    "DahliaSECTeleop",
    "DahliaSECTeleopConfig",
    "HXServoMotorsBus",
    "JOINT_ANGLE_LIMITS",
    "JOINT_CONFIGS",
    "JOINT_NAMES",
    "GRIPPER_NAME",
    "angle_to_normalized",
    "dahlia_calibration",
    "dahlia_cameras",
    "dahlia_motors",
    "gripper_to_normalized",
]
