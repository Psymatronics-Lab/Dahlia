"""Live comparison of what the Dahlia arm is told to do against what it actually does.

Runs on a workstation and reads only. Three traces are kept for every joint:

    measured       /feedback on the SPI service, the angles the servos report
    commanded      /command on the SPI service, the angles python/main.py is asking for
    independent    this process running control.ArmController on the same SEC input

The pairs answer different questions. Commanded against measured is servo tracking,
which is where gravity droop and per joint lag show up. Independent against commanded
is the teleoperation and IK, which diverging means the arm is not being sent what the
control code says it should be. The approach pitch panel sums shoulder, elbow, and
wrist pitch, so it carries all three joints' error at once, and is the quantity the
end effector is visibly failing to hold.

Every request is a GET against an endpoint in READ_ONLY. Nothing here can move the
arm, change a target, or alter a mode. The only cost to the running system is serving
the polls, three small JSON reads per tick.

Usage:
    python dahlia_visualizer.py --board 192.168.1.50
    python dahlia_visualizer.py --board dahlia.local --log climb.csv

Press r in the plot window to reseed the independent controller from the measured arm.
"""

import argparse
import csv
import sys
import threading
import time
from collections import deque
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import requests
from matplotlib.animation import FuncAnimation

FIRMWARE_PYTHON = Path(__file__).resolve().parents[2] / "Firmware" / "dahlia_firmware" / "python"
sys.path.insert(0, str(FIRMWARE_PYTHON))

from control.control import ArmController, JOINT_NAMES, PERIOD   # noqa: E402
from control.kinematics import B_APPROACH, NUM_JOINTS   # noqa: E402

CONTROLLER_PORT = 8000   # ble_service.py, uvicorn on 0.0.0.0
SPI_PORT = 9000   # spi_service.py, published on the board by brick_compose.yaml

# The only requests this tool is allowed to make. Both services also expose POST
# routes that command the arm, and none of them appear here.
READ_ONLY = {
    (CONTROLLER_PORT, "/controller"),
    (SPI_PORT, "/command"),
    (SPI_PORT, "/feedback"),
}

MODE_HOLD, MODE_ANGLE, MODE_RAW = 0, 1, 2   # Mirrors ControlMode in spi_service.h
MODE_NAMES = {MODE_HOLD: "hold", MODE_ANGLE: "angle", MODE_RAW: "raw"}
STATUS_STALE = 0x04

SHOULDER, ELBOW, WRIST_PITCH = 1, 2, 3
PLANAR = (SHOULDER, ELBOW, WRIST_PITCH)   # The joints the approach pitch is built from

COLORS = ("#4c72b0", "#dd8452", "#55a868", "#c44e52", "#8172b3")


def approach_pitch(joints):
    """Approach pitch of a joint vector, in radians.

    Matches kinematics.fk_task: the three parallel joints sum, so any one of them
    being off lands on the end effector angle undiluted.
    """
    return sum(joints[j] for j in PLANAR) + B_APPROACH


class Poller(threading.Thread):
    """Polls both services and mirrors the control loop locally, read only."""

    daemon = True

    def __init__(self, board, hz, window, log=None):
        super().__init__()
        self.board = board
        self.period = 1.0 / hz
        self.timeout = min(0.25, self.period)
        self.session = requests.Session()

        self.arm = ArmController()   # Purely local, its output never leaves this process
        self.stop_event = threading.Event()
        self.resync_event = threading.Event()
        self.lock = threading.Lock()

        depth = max(2, int(window * hz))
        self.t = deque(maxlen=depth)
        self.measured = deque(maxlen=depth)
        self.commanded = deque(maxlen=depth)
        self.independent = deque(maxlen=depth)

        self.status = "connecting"
        self.errors = 0

        self.log_file = open(log, "w", newline="") if log else None
        self.writer = csv.writer(self.log_file) if self.log_file else None

        if self.writer:
            names = [n.replace(" ", "_") for n in JOINT_NAMES]
            self.writer.writerow(
                ["t"]
                + [f"measured_{n}" for n in names]
                + [f"commanded_{n}" for n in names]
                + [f"independent_{n}" for n in names]
            )

    # ----- transport -----

    def get(self, port, path):
        """GET one read-only endpoint. Refuses anything not in READ_ONLY."""
        if (port, path) not in READ_ONLY:
            raise ValueError(f"request outside the read-only set: {port}{path}")

        response = self.session.get(f"http://{self.board}:{port}{path}", timeout=self.timeout)
        response.raise_for_status()

        return response.json()

    def run(self):
        start = time.monotonic()
        next_tick = time.monotonic()

        while not self.stop_event.is_set():
            next_tick += self.period

            try:
                controller = self.get(CONTROLLER_PORT, "/controller")
                command = self.get(SPI_PORT, "/command")
                feedback = self.get(SPI_PORT, "/feedback")

                self.record(time.monotonic() - start, controller, command, feedback)
                self.errors = 0

            except Exception as e:   # A dropped poll is a gap in the plot, never a crash
                self.errors += 1
                self.status = f"{type(e).__name__}: {e}"

            time.sleep(max(0.0, next_tick - time.monotonic()))

    # ----- sampling -----

    def record(self, t, controller, command, feedback):
        measured = np.array(feedback["positions"], float) if feedback.get("ok") else None

        # Radians are only meaningful in MODE_ANGLE; in raw or hold the field is stale
        commanded = (np.array(command["positions"], float)
                     if command.get("ok") and command.get("mode") == MODE_ANGLE else None)

        independent = self.advance(controller, feedback, commanded)

        with self.lock:
            self.t.append(t)
            self.measured.append(measured)
            self.commanded.append(commanded)
            self.independent.append(independent)

        stale = bool(feedback.get("status", 0) & STATUS_STALE)
        self.status = (
            f"controller {'up' if controller.get('connected') else 'down'}"
            f" | mcu {MODE_NAMES.get(command.get('mode'), '?')}"
            f"{' | STALE' if stale else ''}"
        )

        if self.writer:
            blank = [""] * NUM_JOINTS
            row = [f"{t:.4f}"]
            for block in (measured, commanded, independent):
                row += blank if block is None else [f"{v:.6f}" for v in block]
            self.writer.writerow(row)

    def advance(self, controller, feedback, commanded):
        """Run the real control code locally against the same input the arm just got.

        Seeded from the commanded vector rather than the measured one where possible,
        so that this trace starts level with the commanded trace and any divergence
        between them is the control path alone. Seeding from the measured arm instead
        would bake in whatever droop happened to exist at that instant as a constant
        offset, which is already the subject of the commanded against measured panel.
        """
        if self.resync_event.is_set():
            self.arm.ready = False
            self.arm.start = None   # seed() only takes a start pose while this is None
            self.resync_event.clear()

        if not self.arm.ready:
            origin = ({"ok": True, "positions": list(commanded)}
                      if commanded is not None else feedback)

            if not self.arm.seed(origin):
                return None

            return np.array(self.arm.joints, float)

        # main.py skips update() entirely while the controller is down, so mirror that
        if not controller.get("connected"):
            return np.array(self.arm.joints, float)

        return np.array(self.arm.update(controller["state"], feedback), float)

    def snapshot(self):
        with self.lock:
            return (np.array(self.t), list(self.measured),
                    list(self.commanded), list(self.independent))

    def close(self):
        self.stop_event.set()

        # Joined before the log is closed, otherwise a poll already inside record()
        # writes into a closed file and the last sample is lost
        if self.is_alive():
            self.join(timeout=2.0)

        if self.log_file:
            self.log_file.close()


def series(samples, pick):
    """One trace out of a list of joint vectors, with gaps as nan so the line breaks."""
    return np.array([np.nan if s is None else pick(s) for s in samples], float)


def build_plot(poller, window):
    fig, axes = plt.subplots(3, 1, sharex=True, figsize=(11, 9))
    fig.canvas.manager.set_window_title("Dahlia arm monitor")
    ax_pitch, ax_joint, ax_error = axes

    lines = {}

    ax_pitch.set_ylabel("approach pitch (deg)")
    ax_pitch.set_title("shoulder + elbow + wrist pitch, the angle the end effector holds",
                       fontsize=9, loc="left")
    for key, label, style in (("p_meas", "measured", "-"),
                              ("p_cmd", "commanded", "--"),
                              ("p_ind", "independent IK", ":")):
        lines[key], = ax_pitch.plot([], [], style, linewidth=1.6, label=label)

    ax_joint.set_ylabel("joint angle (deg)")
    ax_joint.set_title("solid measured, dashed commanded", fontsize=9, loc="left")
    for n, j in enumerate(PLANAR):
        color = COLORS[j]
        lines[f"j_meas{j}"], = ax_joint.plot([], [], "-", color=color, linewidth=1.5,
                                             label=JOINT_NAMES[j])
        lines[f"j_cmd{j}"], = ax_joint.plot([], [], "--", color=color, linewidth=1.0)

    ax_error.set_ylabel("commanded - measured (deg)")
    ax_error.set_xlabel("seconds")
    ax_error.set_title("positive means the joint is behind its target", fontsize=9, loc="left")
    for j in range(NUM_JOINTS):
        lines[f"e{j}"], = ax_error.plot([], [], "-", color=COLORS[j], linewidth=1.2,
                                        label=JOINT_NAMES[j])

    for ax in axes:
        ax.grid(alpha=0.25)
        ax.margins(y=0.2)
        # Parked outside the axes, as the traces wander over the whole plot area
        ax.legend(loc="center left", bbox_to_anchor=(1.01, 0.5), fontsize=8, framealpha=0.9)
    ax_error.axhline(0.0, color="0.4", linewidth=0.8)

    banner = fig.suptitle("", fontsize=10)
    fig.tight_layout(rect=(0, 0, 1, 0.96))   # Leaves the banner its own strip

    def update(_):
        t, measured, commanded, independent = poller.snapshot()

        if t.size == 0:
            return list(lines.values())

        deg = np.degrees

        lines["p_meas"].set_data(t, deg(series(measured, approach_pitch)))
        lines["p_cmd"].set_data(t, deg(series(commanded, approach_pitch)))
        lines["p_ind"].set_data(t, deg(series(independent, approach_pitch)))

        for j in PLANAR:
            lines[f"j_meas{j}"].set_data(t, deg(series(measured, lambda s, j=j: s[j])))
            lines[f"j_cmd{j}"].set_data(t, deg(series(commanded, lambda s, j=j: s[j])))

        for j in range(NUM_JOINTS):
            error = series(commanded, lambda s, j=j: s[j]) - series(measured, lambda s, j=j: s[j])
            lines[f"e{j}"].set_data(t, deg(error))

        for ax in axes:
            ax.set_xlim(max(0.0, t[-1] - window), max(window, t[-1]))
            ax.relim(visible_only=True)
            ax.autoscale_view(scalex=False)

        banner.set_text(f"{poller.status}    dropped polls: {poller.errors}")
        return list(lines.values())

    def on_key(event):
        if event.key == "r":
            poller.resync_event.set()
            print("Reseeding the independent controller from the measured arm")

    fig.canvas.mpl_connect("key_press_event", on_key)

    return fig, update


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--board", default="dahlia.local",
                        help="hostname or IP of the board (default: dahlia.local)")
    parser.add_argument("--hz", type=float, default=1.0 / PERIOD,
                        help=f"poll rate (default: {1.0 / PERIOD:.0f}, matching control.PERIOD)")
    parser.add_argument("--window", type=float, default=20.0, help="seconds shown (default: 20)")
    parser.add_argument("--log", help="also append every sample to this CSV")
    args = parser.parse_args()

    if abs(args.hz - 1.0 / PERIOD) > 1e-6:
        print(f"Warning: control.PERIOD assumes {1.0 / PERIOD:.0f} Hz. At {args.hz:g} Hz the "
              f"independent trace still solves correctly, but its reach and height rates "
              f"advance per poll, so it travels at the wrong speed in wall clock time.")

    poller = Poller(args.board, args.hz, args.window, args.log)
    print(f"Polling {args.board} on {CONTROLLER_PORT} and {SPI_PORT}, read only. Ctrl-C to stop.")
    poller.start()

    fig, update = build_plot(poller, args.window)
    animation = FuncAnimation(fig, update, interval=66, blit=False,
                              cache_frame_data=False)   # ~15 fps, independent of poll rate

    try:
        plt.show()
    except KeyboardInterrupt:
        pass
    finally:
        poller.close()
        if args.log:
            print(f"Wrote {args.log}")

    del animation


if __name__ == "__main__":
    main()
