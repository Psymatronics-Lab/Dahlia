"""lerobot MotorsBus for the Dahlia arm's Hiwonder HX bus servos.

The arm is not wired the way a lerobot follower normally is. There is no serial port
on this host that reaches the servos:

    lerobot (this PC)
       |  HTTP, raw servo ticks
    spi_service brick        (UNO Q, Linux side)
       |  SPI, 100 Hz
    sketch.ino               (UNO Q, Zephyr MCU -- owns the servo loop at 200 Hz)
       |  UART, Hiwonder 0xFF protocol      |  PWM
    HX-HM bus servos (5 joints)             clamp servo (gripper)

So this is a proxy bus rather than a serial one. Two things force that shape:

  * The gripper is a plain PWM hobby servo on an MCU pin, not a node on the servo
    bus. A direct host-to-bus connection physically cannot reach it, so the sixth
    degree of freedom only exists through the firmware.
  * The MCU runs the closed servo loop, the mechanical joint limits and the command
    watchdog. Going around it would give those up.

Everything on the wire is in raw servo ticks, so lerobot's own calibration,
normalisation and teleoperation flows work exactly as they do on a serial bus.

Notes on the hardware, which differs from Feetech in two ways that matter:

  * Position is signed and centred on 0 (roughly -30719..30719), not unsigned
    0..4095. "Half turn homing" therefore targets 0, the servo's own centre.
  * Calibration is kept host-side. The servos' non-volatile position-offset register
    is deliberately never written -- offsets are applied here, in the read and write
    paths, which keeps calibration reproducible and avoids wearing the servo NVS.

The register map is otherwise identical to Feetech STS, because these servos speak
the same protocol.

Bring-up:

    python lerobot_hxservo.py http://dahlia.local:9000
"""

from __future__ import annotations

import http.client
import json
import logging
import socket
import time
from collections.abc import Sequence
from contextlib import contextmanager
from pprint import pformat
from urllib.parse import urlparse

from lerobot.motors.motors_bus import (
    Motor,
    MotorCalibration,
    MotorNormMode,
    MotorsBusBase,
    NameOrID,
    Value,
)
from lerobot.utils.decorators import check_if_already_connected, check_if_not_connected
from lerobot.utils.utils import enter_pressed, move_cursor_up

logger = logging.getLogger(__name__)


# --- the arm ----------------------------------------------------------------

# Order is the packet order the firmware uses, so index i here is joint i there.
JOINT_NAMES = ("base", "shoulder", "elbow", "wrist_pitch", "wrist_roll")
GRIPPER_NAME = "gripper"

# The clamp has no encoder, so its "present position" is the echo of what was last
# commanded. Modelled as a motor anyway, so a policy sees one uniform action vector.
GRIPPER_MODEL = "dahlia-clamp"

# 12-bit magnetic encoder. Only used for degree-mode normalisation and as the default
# calibration span; the real travel of each joint comes from the firmware.
HX_RESOLUTION = 4096
GRIPPER_RESOLUTION = 256

# Servo units, matching the clamps in HXServo and in the firmware.
MAX_VELOCITY = 3400
MAX_ACCELERATION = 254

MODEL_RESOLUTION_TABLE = {
    "hx-10hm": HX_RESOLUTION,
    "hx-30hm": HX_RESOLUTION,
    "hx-65hm": HX_RESOLUTION,
    GRIPPER_MODEL: GRIPPER_RESOLUTION,
}

# Kept for reference and for anyone who later drives these servos directly: the HX-HM
# control table is the Feetech STS table, so lerobot's FeetechMotorsBus addresses
# apply unchanged. The proxy addresses registers by name, not by address.
HX_CONTROL_TABLE = {
    "ID": (5, 1),
    "Baud_Rate": (6, 1),
    "Homing_Offset": (31, 2),
    "Operating_Mode": (33, 1),
    "Torque_Enable": (40, 1),
    "Acceleration": (41, 1),
    "Goal_Position": (42, 2),
    "Goal_Velocity": (46, 2),
    "Maximum_Torque": (48, 2),
    "Present_Position": (56, 2),
    "Present_Velocity": (58, 2),
    "Present_Load": (60, 2),
    "Present_Voltage": (62, 1),
    "Present_Temperature": (63, 1),
    "Moving": (66, 1),
    "Present_Current": (69, 2),
}

# Register name -> key in the service's motor-state response.
_READABLE = {
    "Present_Position": "raw_positions",
    "Present_Velocity": "raw_velocities",
    "Present_Load": "raw_loads",
    "Present_Voltage": "voltages",
    "Present_Temperature": "temperatures",
    "Torque_Enable": "torque",
}

# Registers only the bus servos can answer. The clamp is PWM with no sensing at all,
# and unlike velocity or load there is no honest stand-in value -- reporting 0 V or
# 0 C would be a lie a safety check could act on. So a read over "every motor" means
# every motor that can actually answer.
_JOINT_ONLY = {"Present_Voltage", "Present_Temperature"}

# Register name -> key in the service's command body.
_WRITABLE = {
    "Goal_Position": "positions",
    "Goal_Velocity": "velocities",
    "Acceleration": "acceleration",
    "Torque_Enable": "torque",
}


def dahlia_motors(
    joint_model: str = "hx-65hm",
    norm_mode: MotorNormMode = MotorNormMode.RANGE_M100_100,
) -> dict[str, Motor]:
    """The standard six-motor layout, ids matching joint_configs in the firmware."""
    motors = {
        name: Motor(id=index + 1, model=joint_model, norm_mode=norm_mode)
        for index, name in enumerate(JOINT_NAMES)
    }
    # The clamp is driven by PWM, so its id is nominal; it is never addressed on the bus.
    motors[GRIPPER_NAME] = Motor(id=6, model=GRIPPER_MODEL, norm_mode=MotorNormMode.RANGE_0_100)
    return motors


# --- transport --------------------------------------------------------------


class _Session:
    """Keep-alive JSON client for the brick service.

    A control loop makes two requests per tick from another machine, so the
    connection is held open across them. urllib would open a fresh TCP connection
    every time, which at 30 Hz is most of the latency budget spent on handshakes.
    """

    def __init__(self, base_url: str, timeout: float):
        parsed = urlparse(base_url if "//" in base_url else f"http://{base_url}")

        if parsed.scheme not in ("http", ""):
            raise ValueError(f"Only http:// is supported, got '{base_url}'")

        self.host = parsed.hostname or "localhost"
        self.port = parsed.port or 9000
        self.timeout = timeout
        self._conn: http.client.HTTPConnection | None = None

    def __repr__(self) -> str:
        return f"http://{self.host}:{self.port}"

    def connect(self) -> None:
        self.close()
        self._conn = http.client.HTTPConnection(self.host, self.port, timeout=self.timeout)
        self._conn.connect()

    def close(self) -> None:
        if self._conn is not None:
            self._conn.close()
            self._conn = None

    @property
    def is_connected(self) -> bool:
        return self._conn is not None

    def request(self, method: str, path: str, body: dict | None = None, num_retry: int = 0) -> dict:
        """One JSON round trip, reopening the connection if it has gone away.

        A keep-alive connection can be closed by the far end at any time, so the
        first attempt after an idle gap is expected to fail occasionally. That is a
        reconnect, not an error worth surfacing, hence the retry floor of one.
        """
        payload = json.dumps(body).encode("utf-8") if body is not None else None
        headers = {"Accept": "application/json"}

        if payload is not None:
            headers["Content-Type"] = "application/json"
            headers["Content-Length"] = str(len(payload))

        last_error: Exception | None = None

        for attempt in range(2 + num_retry):
            try:
                if self._conn is None:
                    self.connect()

                self._conn.request(method, path, body=payload, headers=headers)
                response = self._conn.getresponse()
                data = response.read()

                if response.status != 200:
                    raise ConnectionError(
                        f"{method} {path} returned {response.status} {response.reason}: "
                        f"{data.decode('utf-8', 'replace')[:200]}"
                    )

                return json.loads(data)

            except (http.client.HTTPException, OSError, socket.timeout, ValueError) as e:
                last_error = e
                self.close()
                logger.debug(f"{method} {path} failed ({attempt=}): {e!r}")

        raise ConnectionError(
            f"Could not reach the Dahlia SPI service at {self}. "
            f"Check the board is up and the brick is running. Last error: {last_error!r}"
        ) from last_error


# --- the bus ----------------------------------------------------------------


class HXServoMotorsBus(MotorsBusBase):
    """A lerobot MotorsBus backed by the Dahlia firmware's raw motor API.

    `port` is the service URL rather than a serial device, e.g.
    `"http://dahlia.local:9000"`, which keeps it usable as a plain `port` field in a
    lerobot RobotConfig.

    Example:
        ```python
        bus = HXServoMotorsBus("http://dahlia.local:9000", dahlia_motors())
        bus.connect()
        bus.enable_torque()
        print(bus.sync_read("Present_Position", normalize=False))
        bus.disconnect()
        ```
    """

    apply_drive_mode = True
    normalized_data = ["Goal_Position", "Present_Position"]
    model_resolution_table = MODEL_RESOLUTION_TABLE
    model_ctrl_table = {model: HX_CONTROL_TABLE for model in MODEL_RESOLUTION_TABLE}

    # The clamp's echo is not a measurement, so it is left out of anything that needs
    # a joint to actually move.
    default_velocity = MAX_VELOCITY
    default_acceleration = MAX_ACCELERATION

    def __init__(
        self,
        port: str,
        motors: dict[str, Motor] | None = None,
        calibration: dict[str, MotorCalibration] | None = None,
        timeout: float = 1.0,
    ):
        super().__init__(port, motors if motors is not None else dahlia_motors(), calibration)

        self._session = _Session(port, timeout)
        self._config: dict = {}

        # The service takes whole arrays, so a write that touches one motor still has
        # to state the others. These hold what was last commanded.
        self._goal_position: dict[str, int] = {}
        self._goal_velocity: dict[str, int] = dict.fromkeys(self.motors, self.default_velocity)
        self._goal_acceleration: dict[str, int] = dict.fromkeys(self.motors, self.default_acceleration)
        self._torque: dict[str, bool] = dict.fromkeys(self.motors, False)

        self._validate_motors()

    def __len__(self) -> int:
        return len(self.motors)

    def __repr__(self) -> str:
        return (
            f"{self.__class__.__name__}(\n"
            f"    Service: '{self._session}',\n"
            f"    Motors: \n{pformat(self.motors, indent=8, sort_dicts=False)},\n"
            ")',\n"
        )

    # --- motor bookkeeping -------------------------------------------------

    def _validate_motors(self) -> None:
        missing = [name for name in JOINT_NAMES if name not in self.motors]
        if missing:
            raise ValueError(
                f"The firmware addresses joints by packet position, so all of {JOINT_NAMES} "
                f"must be present. Missing: {missing}"
            )

        ids = [m.id for m in self.motors.values()]
        if len(ids) != len(set(ids)):
            raise ValueError(f"Some motors have the same id!\n{self}")

        for name, motor in self.motors.items():
            if motor.model not in self.model_resolution_table:
                raise KeyError(f"Unknown model '{motor.model}' for motor '{name}'.")

    def _get_motors_list(self, motors: NameOrID | Sequence[NameOrID] | None) -> list[str]:
        if motors is None:
            return list(self.motors)
        if isinstance(motors, str):
            return [motors]
        if isinstance(motors, int):
            return [self._id_to_name(motors)]
        if isinstance(motors, Sequence):
            return [m if isinstance(m, str) else self._id_to_name(m) for m in motors]
        raise TypeError(motors)

    def _id_to_name(self, motor_id: int) -> str:
        for name, motor in self.motors.items():
            if motor.id == motor_id:
                return name
        raise KeyError(f"No motor with id {motor_id}.")

    def _joint_index(self, motor: str) -> int | None:
        """Packet index of a motor, or None for the gripper, which is not a joint."""
        return JOINT_NAMES.index(motor) if motor in JOINT_NAMES else None

    def _has_feedback(self, motor: str) -> bool:
        """Whether a motor actually measures its position."""
        return motor in JOINT_NAMES

    # --- connection --------------------------------------------------------

    @property
    def is_connected(self) -> bool:
        return self._session.is_connected

    @check_if_already_connected
    def connect(self, handshake: bool = True) -> None:
        """Open the session to the service and adopt the arm's current pose.

        Args:
            handshake: Verify the service is talking to a live MCU before returning.

        Raises:
            ConnectionError: The service is unreachable, or is up but not exchanging
                frames with the MCU.
        """
        self._session.connect()

        try:
            self._config = self._session.request("GET", "/config")

            if handshake:
                self._handshake()

            # Seed the goals from where the arm actually is, so the first sync_write
            # cannot fling a joint across its range for the motors it does not name.
            state = self._state()
            for motor in self.motors:
                self._goal_position[motor] = self._raw_from_state(state, motor, "Present_Position")
                self._torque[motor] = bool(self._raw_from_state(state, motor, "Torque_Enable"))

        except Exception:
            self._session.close()
            raise

        logger.debug(f"{self.__class__.__name__} connected to {self._session}.")

    def _handshake(self) -> None:
        service_joints = self._config.get("joints")

        if service_joints is not None and list(service_joints) != list(JOINT_NAMES):
            raise ConnectionError(
                f"The service reports joints {service_joints}, this bus expects {list(JOINT_NAMES)}. "
                "One side is out of date."
            )

        state = self._state()

        if state.get("comm_errors") and any(state["comm_errors"]):
            offline = [n for n, bad in zip(JOINT_NAMES, state["comm_errors"]) if bad]
            raise ConnectionError(
                f"The MCU is not getting telemetry from: {offline}. Check servo power, "
                "the bus wiring and that each servo has its expected id."
            )

    @check_if_not_connected
    def disconnect(self, disable_torque: bool = True) -> None:
        """Close the session, optionally letting the arm go limp first.

        Args:
            disable_torque: If True, torque is disabled before disconnecting. Note
                that this drops the arm under its own weight -- pass False to leave
                it holding its pose.
        """
        if disable_torque:
            try:
                self.disable_torque(num_retry=2)
            except ConnectionError as e:
                logger.warning(f"Could not disable torque on disconnect: {e}")

        self._session.close()
        logger.debug(f"{self.__class__.__name__} disconnected.")

    # --- service access ----------------------------------------------------

    def _state(self, num_retry: int = 0) -> dict:
        """Newest motor state, or raise if the service cannot supply one."""
        payload = self._session.request("GET", "/motors", num_retry=num_retry)
        return self._check_state(payload)

    def _command(self, body: dict, num_retry: int = 0) -> dict:
        """Push a command and take the state that comes back in the same response."""
        payload = self._session.request("POST", "/motors", body=body, num_retry=num_retry)
        return self._check_state(payload)

    def _check_state(self, payload: dict) -> dict:
        if not payload.get("ok"):
            raise ConnectionError(
                f"The SPI service has no valid frame from the MCU: {payload.get('error')}"
            )

        if payload.get("stale"):
            # The MCU has fallen back to holding its pose because commands stopped
            # arriving in time. Reads are still valid, so warn rather than raise.
            logger.warning(
                "The MCU reports a stale command: the host is not keeping up with the "
                "watchdog. The arm is holding its pose."
            )

        if any(payload.get("comm_errors", [])):
            offline = [n for n, bad in zip(JOINT_NAMES, payload["comm_errors"]) if bad]
            logger.warning(f"The MCU missed telemetry from: {offline}")

        return payload

    def _raw_from_state(self, state: dict, motor: str, data_name: str) -> int:
        """Pull one motor's raw value for a register out of a state response."""
        index = self._joint_index(motor)

        if index is None:
            # The clamp only exists as the gripper byte, which is the commanded
            # closure echoed back rather than anything measured.
            if data_name in ("Present_Position", "Goal_Position"):
                return int(state["gripper"])
            if data_name == "Torque_Enable":
                return 1   # PWM is always driven; there is no torque to switch
            if data_name in ("Present_Velocity", "Present_Load"):
                return 0
            raise KeyError(f"'{data_name}' is not available for '{motor}'.")

        key = _READABLE.get(data_name)

        if key is None:
            raise KeyError(
                f"'{data_name}' is not readable through the firmware proxy. "
                f"Available: {sorted(_READABLE)}"
            )

        value = state[key][index]
        return int(value) if not isinstance(value, bool) else int(value)

    # --- calibration -------------------------------------------------------

    @property
    def is_calibrated(self) -> bool:
        return set(self.calibration) == set(self.motors)

    def read_calibration(self) -> dict[str, MotorCalibration]:
        """The calibration in effect.

        Unlike a serial bus this does not query the servos: offsets and ranges are
        applied host-side and never written to servo NVS, so this bus is the only
        place they live.
        """
        return dict(self.calibration)

    def write_calibration(
        self, calibration_dict: dict[str, MotorCalibration], cache: bool = True
    ) -> None:
        unknown = set(calibration_dict) - set(self.motors)
        if unknown:
            raise ValueError(f"Calibration for motors that are not on this bus: {sorted(unknown)}")

        if cache:
            self.calibration = calibration_dict

    def reset_calibration(self, motors: NameOrID | Sequence[NameOrID] | None = None) -> None:
        """Restore the full mechanical range and a zero offset for the given motors."""
        for motor in self._get_motors_list(motors):
            self.calibration[motor] = MotorCalibration(
                id=self.motors[motor].id,
                drive_mode=0,
                homing_offset=0,
                range_min=self._default_range(motor)[0],
                range_max=self._default_range(motor)[1],
            )

    def _default_range(self, motor: str) -> tuple[int, int]:
        """The joint's travel as the firmware enforces it, in ticks."""
        index = self._joint_index(motor)

        if index is None:
            return 0, GRIPPER_RESOLUTION - 1

        limits = self._config.get("tick_limits")

        if limits is not None and index < len(limits):
            return int(limits[index][0]), int(limits[index][1])

        # No config yet (not connected): fall back to a full symmetric turn.
        half = self.model_resolution_table[self.motors[motor].model] // 2
        return -half, half

    def set_half_turn_homings(
        self, motors: NameOrID | Sequence[NameOrID] | None = None
    ) -> dict[str, Value]:
        """Make the present pose read as the centre of each joint's range.

        These servos are signed and centred on 0 rather than unsigned around 2048, so
        "half turn" here means the servo's own centre. The offset is stored in the
        calibration; the servo's own offset register is not touched.
        """
        names = self._get_motors_list(motors)

        # Reset every requested motor, including the clamp, so it still ends up with a
        # default entry. Only the joints get an offset computed below, but a motor with
        # no calibration at all would make normalisation raise later.
        self.reset_calibration(names)

        measured = [motor for motor in names if self._has_feedback(motor)]
        positions = self.sync_read("Present_Position", measured, normalize=False)

        offsets: dict[str, Value] = {}
        for motor, position in positions.items():
            offsets[motor] = int(position)
            self.calibration[motor].homing_offset = int(position)

        return offsets

    def record_ranges_of_motion(
        self, motors: NameOrID | Sequence[NameOrID] | None = None, display_values: bool = True
    ) -> tuple[dict[str, Value], dict[str, Value]]:
        """Record each joint's travel while it is moved by hand.

        Torque has to be off for this. The clamp is skipped: it has no encoder, so
        there is nothing to record, and it keeps its full commanded range.
        """
        names = self._get_motors_list(motors)
        recorded = [m for m in names if self._has_feedback(m)]

        if any(self._torque[m] for m in recorded):
            logger.warning("Recording ranges with torque enabled; call disable_torque() first.")

        start = self.sync_read("Present_Position", recorded, normalize=False, num_retry=5)
        mins = dict(start)
        maxes = dict(start)

        user_pressed_enter = False
        while not user_pressed_enter:
            positions = self.sync_read("Present_Position", recorded, normalize=False, num_retry=5)
            mins = {motor: min(positions[motor], value) for motor, value in mins.items()}
            maxes = {motor: max(positions[motor], value) for motor, value in maxes.items()}

            if display_values:
                print("\n-------------------------------------------")
                print(f"{'NAME':<15} | {'MIN':>6} | {'POS':>6} | {'MAX':>6}")
                for motor in recorded:
                    print(f"{motor:<15} | {mins[motor]:>6} | {positions[motor]:>6} | {maxes[motor]:>6}")

            if enter_pressed():
                user_pressed_enter = True

            if not user_pressed_enter:
                if display_values:
                    move_cursor_up(len(recorded) + 3)
                time.sleep(0.02)

        same_min_max = [motor for motor in recorded if mins[motor] == maxes[motor]]
        if same_min_max:
            raise ValueError(f"Some motors have the same min and max values:\n{pformat(same_min_max)}")

        for motor in names:
            if not self._has_feedback(motor):
                low, high = self._default_range(motor)
                mins[motor], maxes[motor] = low, high

        return mins, maxes

    # --- normalisation -----------------------------------------------------

    def _calibrated(self, motor: str, raw: int) -> int:
        """Raw servo ticks -> the calibrated space ranges are recorded in."""
        if motor not in self.calibration:
            return raw
        return raw - self.calibration[motor].homing_offset

    def _uncalibrated(self, motor: str, value: int) -> int:
        """The inverse of _calibrated."""
        if motor not in self.calibration:
            return value
        return value + self.calibration[motor].homing_offset

    def _normalize(self, values: dict[str, int]) -> dict[str, float]:
        if not self.calibration:
            raise RuntimeError(f"{self} has no calibration registered.")

        normalized = {}
        for motor, value in values.items():
            calibration = self.calibration[motor]
            min_, max_ = calibration.range_min, calibration.range_max
            drive_mode = self.apply_drive_mode and calibration.drive_mode

            if max_ == min_:
                raise ValueError(f"Invalid calibration for motor '{motor}': min and max are equal.")

            bounded = min(max_, max(min_, value))
            norm_mode = self.motors[motor].norm_mode

            if norm_mode is MotorNormMode.RANGE_M100_100:
                norm = (((bounded - min_) / (max_ - min_)) * 200) - 100
                normalized[motor] = -norm if drive_mode else norm
            elif norm_mode is MotorNormMode.RANGE_0_100:
                norm = ((bounded - min_) / (max_ - min_)) * 100
                normalized[motor] = 100 - norm if drive_mode else norm
            elif norm_mode is MotorNormMode.DEGREES:
                mid = (min_ + max_) / 2
                max_res = self.model_resolution_table[self.motors[motor].model] - 1
                normalized[motor] = (value - mid) * 360 / max_res
            else:
                raise NotImplementedError(norm_mode)

        return normalized

    def _unnormalize(self, values: dict[str, float]) -> dict[str, int]:
        if not self.calibration:
            raise RuntimeError(f"{self} has no calibration registered.")

        unnormalized = {}
        for motor, value in values.items():
            calibration = self.calibration[motor]
            min_, max_ = calibration.range_min, calibration.range_max
            drive_mode = self.apply_drive_mode and calibration.drive_mode

            if max_ == min_:
                raise ValueError(f"Invalid calibration for motor '{motor}': min and max are equal.")

            norm_mode = self.motors[motor].norm_mode

            if norm_mode is MotorNormMode.RANGE_M100_100:
                value = -value if drive_mode else value
                bounded = min(100.0, max(-100.0, value))
                unnormalized[motor] = int(((bounded + 100) / 200) * (max_ - min_) + min_)
            elif norm_mode is MotorNormMode.RANGE_0_100:
                value = 100 - value if drive_mode else value
                bounded = min(100.0, max(0.0, value))
                unnormalized[motor] = int((bounded / 100) * (max_ - min_) + min_)
            elif norm_mode is MotorNormMode.DEGREES:
                mid = (min_ + max_) / 2
                max_res = self.model_resolution_table[self.motors[motor].model] - 1
                unnormalized[motor] = int((value * max_res / 360) + mid)
            else:
                raise NotImplementedError(norm_mode)

        return unnormalized

    # --- reads -------------------------------------------------------------

    @check_if_not_connected
    def read(self, data_name: str, motor: str, *, normalize: bool = True, num_retry: int = 0) -> Value:
        """Read one register from one motor.

        Args:
            data_name: Register name, e.g. `"Present_Position"`.
            motor: Motor name.
            normalize: Scale to the motor's norm mode using the calibration.
            num_retry: Extra attempts before giving up.
        """
        return self.sync_read(data_name, [motor], normalize=normalize, num_retry=num_retry)[motor]

    @check_if_not_connected
    def sync_read(
        self,
        data_name: str,
        motors: NameOrID | Sequence[NameOrID] | None = None,
        *,
        normalize: bool = True,
        num_retry: int = 0,
    ) -> dict[str, Value]:
        """Read one register from several motors in a single round trip.

        Args:
            data_name: Register name.
            motors: Motors to query, or None for every motor.
            normalize: Scale to the motor's norm mode using the calibration.
            num_retry: Extra attempts before giving up.

        Returns:
            Mapping of motor name to value.
        """
        names = self._get_motors_list(motors)

        # Naming the clamp explicitly for one of these still raises, which is the
        # useful answer: the caller asked for something that does not exist.
        if data_name in _JOINT_ONLY and motors is None:
            names = [motor for motor in names if self._has_feedback(motor)]

        state = self._state(num_retry=num_retry)

        # Volts are already a physical unit, and the tenth of a volt matters, so this
        # neither goes through the int raw path nor gets normalised against ticks.
        if data_name == "Present_Voltage":
            return {motor: state["voltages"][self._joint_index(motor)] for motor in names}

        raw = {motor: self._raw_from_state(state, motor, data_name) for motor in names}

        if data_name == "Present_Position":
            raw = {motor: self._calibrated(motor, value) for motor, value in raw.items()}

        if normalize and data_name in self.normalized_data:
            return self._normalize(raw)

        return raw

    # --- writes ------------------------------------------------------------

    @check_if_not_connected
    def write(
        self, data_name: str, motor: str, value: Value, *, normalize: bool = True, num_retry: int = 0
    ) -> None:
        """Write one register on one motor."""
        self.sync_write(data_name, {motor: value}, normalize=normalize, num_retry=num_retry)

    @check_if_not_connected
    def sync_write(
        self,
        data_name: str,
        values: Value | dict[str, Value],
        *,
        normalize: bool = True,
        num_retry: int = 0,
    ) -> None:
        """Write one register on several motors in a single round trip.

        Args:
            data_name: Register name.
            values: A single value applied to every motor, or a mapping of motor name
                to value.
            normalize: Convert from the motor's norm mode back to raw ticks.
            num_retry: Extra attempts before giving up.
        """
        if data_name == "Homing_Offset":
            # Host-side by design: see the module docstring.
            for motor, value in self._as_values_dict(values).items():
                self.calibration[motor].homing_offset = int(value)
            return

        if data_name not in _WRITABLE:
            raise KeyError(
                f"'{data_name}' is not writable through the firmware proxy. "
                f"Available: {sorted(_WRITABLE)}"
            )

        requested = self._as_values_dict(values)

        if data_name == "Goal_Position":
            if normalize:
                requested = self._unnormalize(requested)
            for motor, value in requested.items():
                self._goal_position[motor] = self._uncalibrated(motor, int(value))

        elif data_name == "Goal_Velocity":
            for motor, value in requested.items():
                self._goal_velocity[motor] = int(min(MAX_VELOCITY, max(0, abs(value))))

        elif data_name == "Acceleration":
            for motor, value in requested.items():
                self._goal_acceleration[motor] = int(min(MAX_ACCELERATION, max(0, value)))

        elif data_name == "Torque_Enable":
            for motor, value in requested.items():
                self._torque[motor] = bool(value)

        self._push(data_name, num_retry=num_retry)

    def _as_values_dict(self, values: Value | dict[str, Value]) -> dict[str, Value]:
        if isinstance(values, (int, float)):
            return dict.fromkeys(self.motors, values)
        if isinstance(values, dict):
            unknown = set(values) - set(self.motors)
            if unknown:
                raise KeyError(f"Not motors on this bus: {sorted(unknown)}")
            return values
        raise TypeError(f"'values' should be a single value or a dict. Got {values}")

    def _push(self, data_name: str, num_retry: int = 0) -> None:
        """Send the command state the service needs for this register.

        The service takes whole arrays, so every write restates the motors it did not
        name from what was last commanded.
        """
        body: dict = {}

        if data_name == "Goal_Position":
            body["positions"] = [self._goal_position[name] for name in JOINT_NAMES]
            body["gripper"] = self._clamp_gripper(self._goal_position[GRIPPER_NAME])
        elif data_name == "Goal_Velocity":
            body["velocities"] = [self._goal_velocity[name] for name in JOINT_NAMES]
        elif data_name == "Acceleration":
            body["acceleration"] = [self._goal_acceleration[name] for name in JOINT_NAMES]
        elif data_name == "Torque_Enable":
            body["torque"] = [self._torque[name] for name in JOINT_NAMES]

        self._command(body, num_retry=num_retry)

    def _clamp_gripper(self, value: int) -> int:
        return int(min(GRIPPER_RESOLUTION - 1, max(0, value)))

    # --- torque ------------------------------------------------------------

    @check_if_not_connected
    def enable_torque(self, motors: str | list[str] | None = None, num_retry: int = 0) -> None:
        """Enable torque on the selected motors, so they hold and drive to targets."""
        self._set_torque(motors, True, num_retry)

    @check_if_not_connected
    def disable_torque(self, motors: str | list[str] | None = None, num_retry: int = 0) -> None:
        """Disable torque, letting the joints be backdriven by hand.

        Needed for calibration. Note the arm will sag under its own weight.
        """
        self._set_torque(motors, False, num_retry)

    def _set_torque(self, motors: str | list[str] | None, enabled: bool, num_retry: int) -> None:
        for motor in self._get_motors_list(motors):
            if self._has_feedback(motor):
                self._torque[motor] = enabled

        self._push("Torque_Enable", num_retry=num_retry)
        self._await_torque()

    def _await_torque(self, timeout: float = 1.0) -> None:
        """Block until the MCU confirms the torque state that was just requested.

        Commands reach the MCU through a 100 Hz SPI stream, so they are not in effect
        when the POST returns. Streaming a goal position slightly early is harmless,
        but torque is not: calibration disables it and immediately starts sampling a
        joint the operator is about to move by hand, which has to be limp before the
        first sample. So this one write is synchronous.

        Raises:
            RuntimeError: A joint never reported the requested state, which usually
                means it is offline rather than merely slow.
        """
        wanted = [self._torque[name] for name in JOINT_NAMES]
        deadline = time.monotonic() + timeout
        state = None

        while time.monotonic() < deadline:
            state = self._state()

            if [bool(value) for value in state["torque"]] == wanted:
                return

            time.sleep(0.01)

        reported = [bool(value) for value in state["torque"]] if state else []
        disagreed = [
            name for index, name in enumerate(JOINT_NAMES)
            if index >= len(reported) or reported[index] != wanted[index]
        ]
        raise RuntimeError(
            f"The MCU did not confirm the requested torque state within {timeout}s for: "
            f"{disagreed}. Check those servos are powered and answering on the bus."
        )

    @contextmanager
    def torque_disabled(self, motors: str | list[str] | None = None):
        """Context manager that guarantees torque is re-enabled."""
        self.disable_torque(motors)
        try:
            yield
        finally:
            self.enable_torque(motors)

    # --- setup -------------------------------------------------------------

    @check_if_not_connected
    def configure_motors(self, velocity: int | None = None, acceleration: int | None = None) -> None:
        """Push the default slew rate and acceleration ramp to every joint."""
        velocity = self.default_velocity if velocity is None else velocity
        acceleration = self.default_acceleration if acceleration is None else acceleration

        self._goal_velocity = dict.fromkeys(self.motors, int(velocity))
        self._goal_acceleration = dict.fromkeys(self.motors, int(acceleration))

        self._push("Goal_Velocity")
        self._push("Acceleration")

    @check_if_not_connected
    def ping(self, motor: NameOrID, num_retry: int = 0) -> bool:
        """Whether the MCU is currently getting telemetry from a motor."""
        motor = motor if isinstance(motor, str) else self._id_to_name(motor)

        if not self._has_feedback(motor):
            return True   # The clamp is PWM: it cannot be pinged, and never drops out

        state = self._state(num_retry=num_retry)
        return not state["comm_errors"][self._joint_index(motor)]

    @check_if_not_connected
    def diagnostics(self) -> dict[str, dict]:
        """Per-joint load, voltage and temperature, for watching a servo under load."""
        state = self._state()

        return {
            name: {
                "position": state["raw_positions"][index],
                "velocity": state["raw_velocities"][index],
                "load": state["raw_loads"][index],
                "voltage": state["voltages"][index],
                "temperature": state["temperatures"][index],
                "torque": state["torque"][index],
                "comm_error": state["comm_errors"][index],
            }
            for index, name in enumerate(JOINT_NAMES)
        }


if __name__ == "__main__":
    import sys

    logging.basicConfig(level=logging.INFO, format="%(levelname)s %(message)s")

    url = sys.argv[1] if len(sys.argv) > 1 else "http://localhost:9000"

    bus = HXServoMotorsBus(url)
    bus.connect()

    try:
        print(f"Connected to {url}")
        print(f"Service config: {pformat(bus._config)}\n")

        print("Per-joint state:")
        for name, values in bus.diagnostics().items():
            print(f"  {name:<12} {values}")

        print("\nRaw positions:", bus.sync_read("Present_Position", normalize=False))

        bus.reset_calibration()
        print("Normalised with the default calibration:",
              {k: round(v, 1) for k, v in bus.sync_read("Present_Position").items()})

    finally:
        # Left holding rather than limp: dropping the arm is not a good default for a
        # script whose whole job is to print numbers.
        bus.disconnect(disable_torque=False)
