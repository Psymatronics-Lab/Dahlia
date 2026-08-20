"""LeRobot Robot for the Dahlia arm.

Six actuated degrees of freedom: five HX bus servo joints plus the PWM clamp, all
reached over HTTP through the UNO Q's SPI service rather than a serial port. See
`dahlia_bus` for why the transport looks like that.

Calibration is fixed by the firmware's joint limits rather than measured, so this
robot is usable the moment it connects; see `dahlia_calibration`.
"""

import logging
from dataclasses import dataclass, field
from typing import Any

from lerobot.cameras import CameraConfig, ColorMode, Cv2Rotation, make_cameras_from_configs
from lerobot.cameras.opencv import OpenCVCameraConfig
from lerobot.robots import Robot, RobotConfig

from .dahlia_bus import GRIPPER_NAME, JOINT_NAMES, HXServoMotorsBus, dahlia_motors
from .dahlia_calibration import dahlia_calibration

logger = logging.getLogger(__name__)

# Every actuated joint, in the order the firmware packs them, clamp last.
MOTOR_NAMES = (*JOINT_NAMES, GRIPPER_NAME)


@RobotConfig.register_subclass("dahlia")
@dataclass
class DahliaRobotConfig(RobotConfig):
    # The SPI service URL rather than a serial device, which keeps this usable as a
    # plain --robot.port on the command line.
    port: str

    cameras: dict[str, CameraConfig] = field(
        default_factory=lambda: {
            "cam_1": OpenCVCameraConfig(
                index_or_path=1,
                fps=15,
                width=1280,
                height=720,
                color_mode=ColorMode.RGB,
                rotation=Cv2Rotation.NO_ROTATION,
            ),
        }
    )

    # Torque is left enabled on disconnect by default: the arm holds its pose instead
    # of dropping under its own weight the moment a script ends.
    disable_torque_on_disconnect: bool = False

    # Whether LeRobot may command the arm.
    #
    # python/main.py runs continuously as the App Lab application and holds the
    # service's single command space whenever the SEC is connected, so during
    # teleoperation and recording this robot has to observe rather than drive: the
    # two POST routes overwrite each other's mode field and the arm jerks between the
    # two command spaces. Replaying a dataset or evaluating a policy is the case that
    # needs to drive, and is the case that has to say so.
    #
    # Defaulted to observing because that is both the common case and the safe one.
    # Forgetting it on a replay leaves the arm still and logs why; forgetting it the
    # other way corrupts a recording while fighting a live teleoperator.
    read_only: bool = True


class DahliaRobot(Robot):
    config_class = DahliaRobotConfig
    name = "dahlia"

    def __init__(self, config: DahliaRobotConfig):
        super().__init__(config)
        self.config = config

        # Robot.__init__ loads any saved calibration file. There is nothing to measure
        # on this arm, so fall back to the firmware's limits and be usable immediately.
        if not self.calibration:
            self.calibration = dahlia_calibration()

        self.bus = HXServoMotorsBus(
            port=config.port,
            motors=dahlia_motors(),
            calibration=self.calibration,
        )
        self.cameras = make_cameras_from_configs(config.cameras)
        self._warned_observing = False
        self._warned_conflict = False

    # ----- observation and action features -----

    @property
    def _motors_ft(self) -> dict[str, type]:
        return {f"{name}.pos": float for name in MOTOR_NAMES}

    @property
    def _cameras_ft(self) -> dict[str, tuple]:
        return {
            cam: (self.cameras[cam].height, self.cameras[cam].width, 3) for cam in self.cameras
        }

    @property
    def observation_features(self) -> dict:
        return {**self._motors_ft, **self._cameras_ft}

    @property
    def action_features(self) -> dict:
        return self._motors_ft

    # ----- connections -----

    @property
    def is_connected(self) -> bool:
        return self.bus.is_connected and all(cam.is_connected for cam in self.cameras.values())

    def connect(self, calibrate: bool = True) -> None:
        self.bus.connect()

        if not self.is_calibrated and calibrate:
            self.calibrate()

        for cam in self.cameras.values():
            cam.connect()

        self.configure()

    def disconnect(self) -> None:
        self.bus.disconnect(disable_torque=self.config.disable_torque_on_disconnect)

        for cam in self.cameras.values():
            cam.disconnect()

    # ----- calibration and configuration -----

    @property
    def is_calibrated(self) -> bool:
        return self.bus.is_calibrated

    def calibrate(self) -> None:
        """Apply the calibration the firmware's joint limits already imply.

        Deliberately not the usual interactive routine. The MCU clamps every joint to
        a flashed range that a host cannot exceed, so those stops *are* the
        calibration; hand-moving the arm to rediscover them would only reproduce the
        same numbers, less accurately.
        """
        self.calibration = dahlia_calibration()
        self.bus.write_calibration(self.calibration)
        self._save_calibration()

    def configure(self) -> None:
        self.bus.configure_motors()
        self.bus.enable_torque()

    # ----- observation and action -----

    def get_observation(self) -> dict[str, Any]:
        if not self.is_connected:
            raise ConnectionError(f"{self} is not connected.")

        obs_dict = {
            f"{motor}.pos": value
            for motor, value in self.bus.sync_read("Present_Position").items()
        }

        for cam_key, cam in self.cameras.items():
            obs_dict[cam_key] = cam.async_read()

        return obs_dict

    def send_action(self, action: dict[str, Any]) -> dict[str, Any]:
        """Command a normalised goal pose, unless the on-board SEC loop owns the arm.

        Returns what was actually commanded rather than what was asked for, since the
        service and the MCU both clamp to the joint limits. Recording the requested
        value would put actions in the dataset that the arm never carried out.

        LeRobot calls this every tick whether or not anything else is driving, and the
        on-board SEC loop runs continuously, so which of them owns the arm cannot be
        inferred and is set by `read_only` on the config instead. While observing, the
        action is still returned so the dataset records what the operator did.
        """
        goal_pos = {
            key.removesuffix(".pos"): value
            for key, value in action.items()
            if key.endswith(".pos")
        }

        if self.config.read_only:
            if not self._warned_observing:
                logger.info(
                    "read_only is set, so actions are reported for the dataset but not "
                    "sent. Pass --robot.read_only=false to drive the arm from here."
                )
                self._warned_observing = True

            return {f"{motor}.pos": value for motor, value in goal_pos.items()}

        # Driving is allowed, but the SEC path may still hold the command space, in
        # which case both writers overwrite each other's mode and the arm alternates
        # between radians and ticks every servo cycle. Say so rather than fight
        # quietly. Disconnecting the SEC is enough: main.py only posts /targets while
        # the controller is connected, which leaves this the only writer.
        if self.bus.commanding_in_radians() and not self._warned_conflict:
            logger.warning(
                "The command space is still set to radians, so python/main.py is likely "
                "commanding the arm too. Disconnect the SEC controller, or stop the App "
                "Lab application, or the arm will jerk between the two commands."
            )
            self._warned_conflict = True

        self.bus.sync_write("Goal_Position", goal_pos)

        return {f"{motor}.pos": value for motor, value in goal_pos.items()}
