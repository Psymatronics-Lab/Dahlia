"""SPI state exchange service.

Every SPI transaction swaps the newest command for the newest feedback. The HTTP
server and the SPI loop are independent: any source (BLE teleop, a planner, a VLA)
sets a command, and the SPI thread keeps the MCU synchronized at 100 Hz.

Call start() to run the service in the background, stop() to shut it down. In
process sources use set_command() and get_feedback(); everyone else POSTs to
/targets and GETs /feedback.
"""

import json
import struct
import threading
import time
from dataclasses import dataclass, field
from http.server import BaseHTTPRequestHandler, HTTPServer

import spidev

HOST = "0.0.0.0"
PORT = 9000

NUM_JOINTS = 5
COMMAND_MAGIC = 0xA5
FEEDBACK_MAGIC = 0x5A

# magic, sequence, 5 positions, 5 velocities, gripper (or status on the way back)
PACKET_FORMAT = "<BB" + "f" * (2 * NUM_JOINTS) + "B"
PACKET_SIZE = struct.calcsize(PACKET_FORMAT)

SPI_PERIOD = 0.01   # 100 Hz


@dataclass
class RobotCommand:
    positions: list = field(default_factory=lambda: [0.0] * NUM_JOINTS)
    velocities: list = field(default_factory=lambda: [0.0] * NUM_JOINTS)
    gripper: int = 0


lock = threading.Lock()
latest_command = RobotCommand()
latest_feedback = {"ok": False, "error": "No frame yet"}

running = threading.Event()   # Cleared by stop() to end the exchange thread
spi = None
spi_thread = None
server = None


def set_command(positions, velocities, gripper):
    """Replace the newest command, from any source."""
    global latest_command

    if len(positions) != NUM_JOINTS or len(velocities) != NUM_JOINTS:
        raise ValueError("Expected %d joints" % NUM_JOINTS)

    command = RobotCommand(
        [float(x) for x in positions],
        [float(x) for x in velocities],
        int(gripper) & 0xFF
    )

    with lock:   # Lock to prevent spi_loop from reading the command while changing
        latest_command = command


def get_feedback():
    """Newest feedback frame received from the MCU."""
    with lock:
        return dict(latest_feedback)


def spi_loop():
    """Send the newest command and store the newest feedback until stopped."""
    global latest_feedback

    sequence = 0

    while running.is_set():
        with lock:   # Lock to prevent race conditions when formulating the command
            command = RobotCommand(
                list(latest_command.positions),
                list(latest_command.velocities),
                latest_command.gripper
            )

        tx = struct.pack(
            PACKET_FORMAT,
            COMMAND_MAGIC,
            sequence,
            *command.positions,
            *command.velocities,
            command.gripper
        )

        rx = bytes(spi.xfer2(list(tx)))   # Send tx, read rx

        if len(rx) == PACKET_SIZE and rx[0] == FEEDBACK_MAGIC:

            fields = struct.unpack(PACKET_FORMAT, rx)

            feedback = {
                "ok": True,
                "sequence": fields[1],
                "positions": list(fields[2:2 + NUM_JOINTS]),
                "velocities": list(fields[2 + NUM_JOINTS:2 + 2 * NUM_JOINTS]),
                "status": fields[-1]
            }

        else:
            feedback = {
                "ok": False,
                "error": "Invalid frame"
            }

        with lock:
            latest_feedback = feedback

        sequence = (sequence + 1) & 0xFF
        time.sleep(SPI_PERIOD)


class Handler(BaseHTTPRequestHandler):

    def do_POST(self):   # Handle an incoming POST request

        if self.path != "/targets":
            self.send_error(404)
            return

        try:
            length = int(self.headers["Content-Length"])

            data = json.loads(self.rfile.read(length))

            set_command(data["positions"], data["velocities"], data["gripper"])

        except Exception as e:
            self.send_error(400, str(e))
            return

        self.reply({"ok": True})

    def do_GET(self):   # Handle an outgoing GET request for data

        if self.path != "/feedback":
            self.send_error(404)
            return

        self.reply(get_feedback())

    def reply(self, payload):   # Build the HTML reply

        body = json.dumps(payload).encode("utf-8")

        self.send_response(200)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()

        self.wfile.write(body)

    def log_message(self, format, *args):
        pass


def start():
    """Open the bus and run the exchange thread and HTTP server in the background."""
    global spi, spi_thread, server

    if running.is_set():
        return

    spi = spidev.SpiDev()
    spi.open(0, 0)
    spi.max_speed_hz = 1000000
    spi.mode = 0

    running.set()

    spi_thread = threading.Thread(target=spi_loop, daemon=True)
    spi_thread.start()

    server = HTTPServer((HOST, PORT), Handler)
    threading.Thread(target=server.serve_forever, daemon=True).start()

    print("SPI state exchange service started on port %d" % PORT, flush=True)


def stop():
    """Stop serving, end the exchange thread and close the bus. Safe to call twice."""
    global spi, spi_thread, server

    if not running.is_set():
        return

    running.clear()   # Cleared first so the exchange thread stops before the bus closes

    if server is not None:
        server.shutdown()
        server.server_close()
        server = None

    if spi_thread is not None:
        spi_thread.join(timeout=1.0)
        spi_thread = None

    if spi is not None:
        spi.close()
        spi = None

    print("SPI state exchange service stopped", flush=True)