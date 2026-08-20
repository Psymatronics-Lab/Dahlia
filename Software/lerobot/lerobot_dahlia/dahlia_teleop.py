"""LeRobot Teleoperator for the SEC controller.

The SEC is not a leader arm, but LeRobot never asks whether it is. A Teleoperator has
one job, return an action each tick, and has no opinion on where that action came
from. `SO101Leader.get_action()` reads joint positions off a serial bus; this reads
them off an HTTP endpoint. Structurally the same thing.

What makes that work here is that the SEC control loop already runs on the UNO Q:

    SEC --BLE--> controller_service --> python/main.py (joystick -> IK -> targets)
                                            |
                                        spi_service  --> SPI --> MCU --> servos
                                            |
                                        GET /command --> this teleoperator --> LeRobot

So the mapping from controller input to joint targets happens in exactly one place,
on the board, where it also needs to be for latency. This class only reads the result.

That single-place property is the whole point, and it is worth spelling out why the
obvious alternative fails. The mapping in `python/control/control.py` is stateful and
rate dependent: the pose is integrated from a thresholded joystick, the gyro
reference is latched on the sample where the button is first seen held, and the clamp
toggles on a *release edge*. Running a second copy here, fed the same BLE stream at
LeRobot's polling rate, would drift from the copy actually driving the arm. The clamp
is the sharp edge: a click shorter than one poll interval is invisible to the slower
sampler, and because it is a toggle, one missed click inverts the gripper for the rest
of the episode. The arm grasps, the dataset says it did not, and every frame after
that is mislabelled.

Reading the command rather than the measured pose also matters. The measurement lags
the command and settles onto it, so recording it as the action would teach a policy to
output the state it is already in.
"""

from __future__ import annotations

import logging
from dataclasses import dataclass

import requests
from lerobot.teleoperators.config import TeleoperatorConfig
from lerobot.teleoperators.teleoperator import Teleoperator

from .dahlia_bus import GRIPPER_NAME, JOINT_NAMES
from .dahlia_calibration import (
    JOINT_ANGLE_LIMITS,
    angle_to_normalized,
    gripper_to_normalized,
)

logger = logging.getLogger(__name__)

# Mirrors ControlMode in sketch/src/spi/spi_service.h. The on-board SEC loop commands
# in radians, so only MODE_ANGLE means "the SEC is driving".
MODE_ANGLE = 1


@TeleoperatorConfig.register_subclass("dahlia_sec")
@dataclass
class DahliaSECTeleopConfig(TeleoperatorConfig):
    # The SPI service URL, same value as --robot.port. Named port for symmetry with
    # the robot and with LeRobot's other teleoperators.
    port: str = "http://dahlia.local:9000"

    # Short enough that a hiccup does not stall the record loop; a dropped read just
    # repeats the previous action.
    timeout_s: float = 0.2


class DahliaSECTeleop(Teleoperator):
    """Reads whatever the on-board SEC loop is currently commanding."""

    config_class = DahliaSECTeleopConfig
    name = "dahlia_sec"

    def __init__(self, config: DahliaSECTeleopConfig):
        super().__init__(config)
        self.config = config
        self.session = requests.Session()

        self._connected = False
        self._last_action: dict[str, float] | None = None
        self._warned_idle = False

    @property
    def _url(self) -> str:
        return self.config.port.rstrip("/")

    # ----- features -----

    @property
    def action_features(self) -> dict:
        # Must match DahliaRobot.action_features, or recording rejects the pairing.
        return {f"{name}.pos": float for name in (*JOINT_NAMES, GRIPPER_NAME)}

    @property
    def feedback_features(self) -> dict:
        # The SEC has an LED matrix that could surface arm state, but nothing consumes
        # feedback yet.
        return {}

    # ----- connection -----

    @property
    def is_connected(self) -> bool:
        return self._connected

    @property
    def is_calibrated(self) -> bool:
        # The angle limits come from the firmware, so there is nothing to calibrate.
        return True

    def connect(self, calibrate: bool = True) -> None:
        config = self._get("/config")

        if config is None:
            raise ConnectionError(
                f"Could not reach the Dahlia SPI service at {self._url}. "
                "Check the board is powered and the spi_service brick is running."
            )

        self._check_angle_limits(config)
        self._connected = True

        # Seed from the arm so the first action is never a jump, even if the SEC loop
        # has not commanded anything yet.
        self._last_action = self._hold_action()

    def disconnect(self) -> None:
        self.session.close()
        self._connected = False

    def calibrate(self) -> None:
        pass

    def configure(self) -> None:
        pass

    def _check_angle_limits(self, config: dict) -> None:
        """Warn if the board's joint limits have drifted from the baked-in table.

        Both sides derive from the same `joint_configs` in the firmware, but by
        duplication rather than construction. If they ever disagree, recorded actions
        and policy outputs end up in different normalised spaces and nothing else
        would say so.
        """
        reported = config.get("angle_limits")
        joints = config.get("joints")

        if not reported or not joints:
            return

        for name, limits in zip(joints, reported):
            expected = JOINT_ANGLE_LIMITS.get(name)

            if expected is None:
                continue

            if max(abs(a - b) for a, b in zip(expected, limits)) > 1e-6:
                logger.warning(
                    f"Joint '{name}' angle limits differ: the board reports {tuple(limits)}, "
                    f"dahlia_calibration has {expected}. Normalised actions will not line up "
                    "with the robot until they agree."
                )

    # ----- actions -----

    def get_action(self) -> dict[str, float]:
        if not self.is_connected:
            raise ConnectionError(f"{self} is not connected.")

        command = self._get("/command")

        if command is None or not command.get("ok"):
            return self._repeat("the SPI service did not answer")

        if command.get("mode") != MODE_ANGLE:
            # Either nothing is driving the arm, or something else is commanding in
            # raw ticks. Either way the SEC is not the source, so hold.
            return self._repeat("the on-board SEC loop is not commanding")

        self._warned_idle = False

        action = self._action(command["positions"], command["gripper"])
        self._last_action = action
        return action

    def send_feedback(self, feedback: dict) -> None:
        pass

    def _action(self, angles, gripper) -> dict[str, float]:
        action = {
            f"{name}.pos": angle_to_normalized(name, angle)
            for name, angle in zip(JOINT_NAMES, angles)
        }
        action[f"{GRIPPER_NAME}.pos"] = gripper_to_normalized(int(gripper))

        return action

    def _hold_action(self) -> dict[str, float]:
        """Where the arm actually is, as a normalised action."""
        state = self._get("/motors")

        if state is None or not state.get("ok"):
            raise ConnectionError(
                f"The SPI service at {self._url} has no valid frame from the MCU, so "
                "the teleoperator cannot establish a starting pose."
            )

        return self._action(state["positions"], state["gripper"])

    def _repeat(self, reason: str) -> dict[str, float]:
        """Hold the previous action rather than commanding a jump."""
        if not self._warned_idle:
            logger.warning(f"Holding the last SEC action because {reason}.")
            self._warned_idle = True

        if self._last_action is None:
            self._last_action = self._hold_action()

        return self._last_action

    def _get(self, path: str) -> dict | None:
        try:
            response = self.session.get(f"{self._url}{path}", timeout=self.config.timeout_s)
            response.raise_for_status()
            return response.json()

        except (requests.RequestException, ValueError) as e:
            logger.debug(f"GET {path} failed: {e!r}")
            return None
