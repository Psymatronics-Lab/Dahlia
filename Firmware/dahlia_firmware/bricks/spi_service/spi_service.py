"""SPI state exchange service between the Linux MPU and Arduino MCU.

Every SPI transaction exchanges the newest command (TX) for the newest feedback (RX).
The HTTP server and SPI loop are independent, thus allowing any controller to POST
to targets to send commands. SPI thread syncs with the MCU at nominal 100 Hz.

Commands are states, not events. Each POST updates only fields it carries and
all other data fields are left alone by the SPI loop and resent.

Two command spaces share the frame:

    /targets   radians, joint limits enforced by the MCU based on the URDF. Used by the
               BLE teleoperation path in python/main.py.
    /motors    raw servo ticks. Used by anything that owns its own calibration, such
               as the lerobot MotorsBus in Software/lerobot/lerobot_hxservo.py.

Only one of them is in effect at a time, selected by the mode field.
"""

import json
import os
import struct
import threading
import time
from dataclasses import dataclass, field
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import spidev

HOST = os.environ.get("SPI_SERVICE_HOST", "0.0.0.0")
PORT = int(os.environ.get("SPI_SERVICE_PORT", "9000"))

NUM_JOINTS = 5
COMMAND_MAGIC = 0xA5
FEEDBACK_MAGIC = 0x5A

# Mirrors ControlMode in `spi_service.h`
MODE_HOLD, MODE_ANGLE, MODE_RAW = 0, 1, 2

# Mirrors FeedbackPacket in `spi_service.h`
TORQUE_APPLY = 0x80

STATUS_TORQUE = 0x01
STATUS_MOVING = 0x02
STATUS_STALE = 0x04

_N = NUM_JOINTS
COMMAND_FORMAT = "<BBBB" + f"{_N}f{_N}f{_N}h{_N}h{_N}B" + "B"
FEEDBACK_FORMAT = "<BBBB" + f"{_N}f{_N}f{_N}h{_N}h{_N}h{_N}B{_N}B" + "BBBH"

COMMAND_SIZE = struct.calcsize(COMMAND_FORMAT)
FEEDBACK_SIZE = struct.calcsize(FEEDBACK_FORMAT)

# MCU transceives data in fixed buffer of the larger size of the data structs.
FRAME_SIZE = max(COMMAND_SIZE, FEEDBACK_SIZE)

SPI_PERIOD = 0.01   # 100 Hz
SPI_SPEED_HZ = 1000000

# Joint limits from `servo_interface.h`, ordered numerically rather than by travel direction.
JOINT_NAMES = ("base", "shoulder", "elbow", "wrist_pitch", "wrist_roll")
JOINT_TICK_LIMITS = ((-1250, 1250), (-200, 2200), (0, 2250), (-1024, 1024), (800, 2700))
JOINT_ANGLE_LIMITS = ((-1.917, 1.917), (-3.375, 0.307), (0.0, 3.451), (-1.534, 1.534), (-1.457, 1.457))

TICKS_PER_TURN = 4096

GRIPPER_MIN = 0
GRIPPER_MAX = 255

MAX_BODY_BYTES = 64 * 1024


@dataclass
class RobotCommand:
    """The command state resent on every SPI cycle."""

    mode: int = MODE_HOLD
    torque: int = 0   # TORQUE_APPLY | per-joint bits, or 0 to leave torque alone
    positions: list = field(default_factory=lambda: [0.0] * NUM_JOINTS)
    velocities: list = field(default_factory=lambda: [0.0] * NUM_JOINTS)
    raw_positions: list = field(default_factory=lambda: [0] * NUM_JOINTS)
    raw_velocities: list = field(default_factory=lambda: [3400] * NUM_JOINTS)
    raw_accelerations: list = field(default_factory=lambda: [254] * NUM_JOINTS)
    gripper: int = 0


lock = threading.Lock()
command = RobotCommand()
latest_feedback = {"ok": False, "error": "No frame yet"}

# Set once the SPI thread has stopped completely
spi_failed = threading.Event()

spi = spidev.SpiDev()

def open_spi():
    """Open the SPI device, reporting failure instead of taking the service down."""
    global latest_feedback

    try:
        spi.open(0, 0)
        spi.max_speed_hz = SPI_SPEED_HZ
        spi.mode = 0
        return True
    except (OSError, IOError) as e:
        with lock:
            latest_feedback = {"ok": False, "error": "SPI device unavailable: %s" % e}
        spi_failed.set()
        return False


def pack_command(current, sequence):
    """Build the outgoing frame, padded to the size the MCU always clocks."""
    frame = struct.pack(
        COMMAND_FORMAT,
        COMMAND_MAGIC,
        sequence,
        current.mode,
        current.torque,
        *current.positions,
        *current.velocities,
        *current.raw_positions,
        *current.raw_velocities,
        *current.raw_accelerations,
        current.gripper
    )
    return frame.ljust(FRAME_SIZE, b"\x00")


def unpack_feedback(frame):
    """Decode a feedback frame into the dict both APIs are built from."""
    fields = struct.unpack_from(FEEDBACK_FORMAT, frame)

    n = NUM_JOINTS
    blocks = fields[4:4 + 7 * n]   # the seven per-joint arrays, in packet order
    torque_mask, comm_mask, gripper, loop_count = fields[4 + 7 * n:]

    def block(index):
        return list(blocks[index * n:(index + 1) * n])

    status = fields[3]

    return {
        "ok": True,
        "ts": time.monotonic(),
        "sequence": fields[1],
        "mode": fields[2],
        "status": status,
        "stale": bool(status & STATUS_STALE),
        "moving": bool(status & STATUS_MOVING),
        "positions": block(0),
        "velocities": block(1),
        "raw_positions": block(2),
        "raw_velocities": block(3),
        "raw_loads": block(4),
        "voltages": [v / 10.0 for v in block(5)],   # 0.1 V per count on the wire, reported as volts
        "temperatures": block(6),
        "torque": [bool(torque_mask & (1 << i)) for i in range(n)],
        "comm_errors": [bool(comm_mask & (1 << i)) for i in range(n)],
        "gripper": gripper,
        "loop_count": loop_count,
    }


def spi_loop():
    """Send the newest command and store the newest feedback, forever."""
    global latest_feedback

    sequence = 0
    consecutive_errors = 0

    while True:
        with lock:   # Lock to prevent race conditions when packing the command
            tx = pack_command(command, sequence)

        try:
            rx = bytes(spi.xfer2(list(tx)))
            transport_error = None

        except (OSError, IOError) as e:   # Handle exceptions gracefully
            rx = b""
            transport_error = str(e)

        if transport_error is not None:
            consecutive_errors += 1
            feedback = {
                "ok": False,
                "ts": time.monotonic(),
                "error": "SPI transfer failed: %s" % transport_error,
                "consecutive_errors": consecutive_errors,
            }

        elif len(rx) == FRAME_SIZE and rx[0] == FEEDBACK_MAGIC:
            consecutive_errors = 0
            feedback = unpack_feedback(rx)

        else:
            consecutive_errors += 1
            feedback = {
                "ok": False,
                "ts": time.monotonic(),
                "error": "Invalid frame",
                "consecutive_errors": consecutive_errors,
            }

        with lock:
            latest_feedback = feedback

        sequence = (sequence + 1) & 0xFF
        time.sleep(SPI_PERIOD)


def clamp(value, low, high):
    return max(low, min(high, value))

def as_list(data, key, count, cast):
    """Read a fixed length list out of a request body."""
    values = data[key]
    if not isinstance(values, list) or len(values) != count:
        raise ValueError("'%s' must be a list of %d values" % (key, count))
    return [cast(v) for v in values]


class Handler(BaseHTTPRequestHandler):
    protocol_version = "HTTP/1.1"

    def do_POST(self):
        routes = {
            "/targets": self.post_targets,
            "/motors": self.post_motors,
        }

        handler = routes.get(self.path)

        if handler is None:
            self.send_error(404)
            return

        try:
            data = self.read_body()
            payload = handler(data)
        except ValueError as e:
            self.send_error(400, str(e))
            return

        self.reply(payload)

    def do_GET(self):
        routes = {
            "/feedback": self.get_feedback,
            "/motors": self.get_motors,
            "/config": self.get_config,
            "/health": self.get_health,
        }

        handler = routes.get(self.path)

        if handler is None:
            self.send_error(404)
            return

        self.reply(handler())

    # ----- request bodies -----

    def read_body(self):
        length = self.headers.get("Content-Length")

        if length is None:
            raise ValueError("Content-Length required")

        try:
            length = int(length)
        except (TypeError, ValueError):
            raise ValueError("Malformed Content-Length")

        if length < 0 or length > MAX_BODY_BYTES:
            raise ValueError("Body too large")

        try:
            return json.loads(self.rfile.read(length))
        except (ValueError, UnicodeDecodeError) as e:
            raise ValueError("Malformed JSON: %s" % e)

    # ----- radian API, used by SEC controller BLE and similar -----

    def post_targets(self, data):
        positions = as_list(data, "positions", NUM_JOINTS, float)
        velocities = as_list(data, "velocities", NUM_JOINTS, float)
        gripper = clamp(int(data["gripper"]), GRIPPER_MIN, GRIPPER_MAX)

        with lock:   # Lock to prevent spi_loop from reading the command while changing
            command.mode = MODE_ANGLE
            command.positions = positions
            command.velocities = velocities
            command.gripper = gripper

        return {"ok": True}

    def get_feedback(self):
        with lock:
            payload = dict(latest_feedback)

        if not payload.get("ok"):
            return payload

        return {
            "ok": True,
            "sequence": payload["sequence"],
            "positions": payload["positions"],
            "velocities": payload["velocities"],
            "status": payload["status"],
        }

    # ----- raw tick API, used by LeRobot MotorsBus and similar -----

    def post_motors(self, data):
        """Update the raw command and answer with the state it produced."""
        updates = {}

        if "positions" in data:
            updates["raw_positions"] = [
                clamp(int(round(v)), low, high)
                for v, (low, high) in zip(
                    as_list(data, "positions", NUM_JOINTS, float), JOINT_TICK_LIMITS)
            ]

        if "velocities" in data:
            updates["raw_velocities"] = [
                clamp(int(round(v)), 0, 3400)
                for v in as_list(data, "velocities", NUM_JOINTS, float)
            ]

        if "acceleration" in data:
            updates["raw_accelerations"] = [
                clamp(int(round(v)), 0, 254)
                for v in as_list(data, "acceleration", NUM_JOINTS, float)
            ]

        if "gripper" in data:
            updates["gripper"] = clamp(int(data["gripper"]), GRIPPER_MIN, GRIPPER_MAX)

        if data.get("torque") is not None:
            mask = TORQUE_APPLY
            for i, on in enumerate(as_list(data, "torque", NUM_JOINTS, bool)):
                mask |= (1 << i) if on else 0
            updates["torque"] = mask

        if "raw_positions" in updates or "gripper" in updates:
            updates["mode"] = MODE_RAW

        if data.get("hold"):
            updates["mode"] = MODE_HOLD

        with lock:
            for name, value in updates.items():
                setattr(command, name, value)

            payload = dict(latest_feedback)

        return self.motor_state(payload)

    def get_motors(self):
        with lock:
            payload = dict(latest_feedback)

        return self.motor_state(payload)

    def motor_state(self, payload):
        if not payload.get("ok"):
            return payload

        payload = dict(payload)
        payload["age"] = time.monotonic() - payload["ts"]
        return payload

    # ----- service telemetry -----

    def get_config(self):
        return {
            "ok": True,
            "joints": list(JOINT_NAMES),
            "tick_limits": [list(limit) for limit in JOINT_TICK_LIMITS],
            "angle_limits": [list(limit) for limit in JOINT_ANGLE_LIMITS],
            "ticks_per_turn": TICKS_PER_TURN,
            "gripper_range": [GRIPPER_MIN, GRIPPER_MAX],
            "frame_size": FRAME_SIZE,
        }

    def get_health(self):
        with lock:
            payload = dict(latest_feedback)

        return {
            "ok": bool(payload.get("ok")),
            "spi_thread_alive": not spi_failed.is_set(),
            "age": time.monotonic() - payload["ts"] if "ts" in payload else None,
            "consecutive_errors": payload.get("consecutive_errors", 0),
            "error": payload.get("error"),
        }

    # ----- plumbing -----

    def reply(self, payload):
        body = json.dumps(payload).encode("utf-8")

        self.send_response(200)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()

        self.wfile.write(body)

    def log_message(self, format, *args):
        pass


def supervised_spi_loop():
    """Keep the SPI loop running, and report stopping."""
    global latest_feedback

    try:
        spi_loop()
    except BaseException as e:
        with lock:
            latest_feedback = {
                "ok": False,
                "ts": time.monotonic(),
                "error": "SPI thread stopped: %r" % e,
            }
        spi_failed.set()
        raise

def main():
    if open_spi():
        threading.Thread(target=supervised_spi_loop, daemon=True).start()
    else:
        print("SPI device could not be opened, serving health only", flush=True)

    server = ThreadingHTTPServer((HOST, PORT), Handler)
    server.daemon_threads = True

    print("SPI state exchange service started on %s:%d (frame %d bytes)"
          % (HOST, PORT, FRAME_SIZE), flush=True)

    try:
        server.serve_forever()
    finally:
        spi.close()

if __name__ == "__main__":
    main()
