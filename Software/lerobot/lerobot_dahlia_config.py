from dataclasses import dataclass, field
from typing import Any

from lerobot.cameras import CameraConfig, ColorMode, Cv2Rotation, make_cameras_from_configs
from lerobot.cameras.opencv import OpenCVCameraConfig
from lerobot.robots import RobotConfig, Robot
from lerobot.motors import Motor, MotorNormMode

from .lerobot_hxservo import HXServoMotorsBus


@RobotConfig.register_subclass("dahlia")
@dataclass
class DahliaRobotConfig(RobotConfig):
    port: str
    cameras: dict[str, CameraConfig] = field(
        default_factory={
            "cam_1": OpenCVCameraConfig(
                index_or_path=1,
                fps=15,
                width=1280,
                height=720,
                color_mode=ColorMode.RGB,
                rotation=Cv2Rotation.NO_ROTATION
            ),
        }
    )

class DahliaRobot(Robot):
    config_class = DahliaRobotConfig
    name = "dahlia"

    def __init__(self, config: DahliaRobotConfig):
        super().__init__(config)
        self.bus =HXServoMotorsBus(
            port=self.config.port,
            motors={
                "joint_1": Motor(1, "hx30hm", MotorNormMode.RANGE_M100_100),
                "joint_2": Motor(2, "hx65hm", MotorNormMode.RANGE_M100_100),
                "joint_3": Motor(3, "hx30hm", MotorNormMode.RANGE_M100_100),
                "joint_4": Motor(4, "hx30hm", MotorNormMode.RANGE_M100_100),
                "joint_5": Motor(5, "hx30hm", MotorNormMode.RANGE_M100_100),
            },
            calibration=self.calibration,
        )
        self.cameras = make_cameras_from_configs(config.cameras)

# ----- observation and action features -----

    @property
    def _motors_ft(self) -> dict[str, type]:
        return {
            "joint_1.pos": float,
            "joint_2.pos": float,
            "joint_3.pos": float,
            "joint_4.pos": float,
            "joint_5.pos": float,
        }

    @property
    def _cameras_ft(self) -> dict[str, tuple]:
        return {
            cam: (self.cameras[cam].height, self.cameras[cam].width, 3) for cam in self.cameras
        }

    @property
    def observation_features(self) -> dict:
        return {**self._motors_ft, **self._cameras_ft}

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
        self.bus.disconnect()
        for cam in self.cameras.values():
            cam.disconnect()

# ----- calibration and configuration -----

    @property
    def is_calibrated(self) -> bool:
        return self.bus.is_calibrated

    def calibrate(self) -> None:
        pass

    def configure(self) -> None:
        pass

# ----- observattion -----
    def get_observation(self) -> dict[str, Any]:
        if not self.is_connected:
            raise ConnectionError(f"{self} is not connected.")

        # Read arm position
        obs_dict = self.bus.sync_read("Present_Position")
        obs_dict = {f"{motor}.pos": val for motor, val in obs_dict.items()}

        # Capture images from cameras
        for cam_key, cam in self.cameras.items():
            obs_dict[cam_key] = cam.async_read()

        return obs_dict

    def send_action(self, action: dict[str, Any]) -> dict[str, Any]:
        goal_pos = {key.removesuffix(".pos"): val for key, val in action.items()}

        # Send goal position to the arm
        self.bus.sync_write("Goal_Position", goal_pos)
        return action