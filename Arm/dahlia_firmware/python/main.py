import atexit
import signal
import sys
import time

import requests

from arduino.app_utils import App
from spi import spi_service

CONTROLLER_URL = "http://172.17.0.1:8000/controller"

NUM_JOINTS = 5
PERIOD = 0.05

JOY_RANGE = 1000.0   # Full scale joystick counts
JOY_DEADZONE = 0.05   # Normalized joystick deadzone
MAX_JOINT_VEL = 1.5   # Radians per second at full deflection
ENC_COUNTS_CLOSED = 30   # Encoder counts from fully open to fully closed

# Joint limits in radians, matching servo_interface.h
JOINT_LIMITS = [
    (-1.917, 1.917),   # BASE
    (-0.307, 3.375),   # SHOULDER
    (0.0, 3.467),   # ELBOW
    (-1.534, 1.534),   # WRIST_PITCH
    (-1.457, 1.457)    # WRIST_ROLL
]

targets = [0.0] * NUM_JOINTS
seeded = False


def clamp(value, low, high):
    return max(low, min(high, value))


def axis(counts):
    """Normalize a joystick axis and apply a deadzone."""
    value = clamp(counts / JOY_RANGE, -1.0, 1.0)

    if abs(value) < JOY_DEADZONE:
        return 0.0

    return value


def seed_targets():
    """Start from the arm's measured pose so the first command does not jump."""
    global targets

    feedback = spi_service.get_feedback()

    if not feedback["ok"]:
        return False

    targets = list(feedback["positions"])
    return True


def loop():

    global seeded

    if not seeded:
        seeded = seed_targets()
        time.sleep(PERIOD)
        return

    try:
        data = requests.get(
            CONTROLLER_URL,
            timeout=0.1
        ).json()

    except Exception as e:
        print("Controller API error:", e)
        time.sleep(PERIOD)
        return

    if not data["connected"]:
        print("Controller disconnected")
        time.sleep(PERIOD)
        return

    state = data["state"]

    # Joystick jogs the base and shoulder, encoder sets the gripper
    velocities = [0.0] * NUM_JOINTS

    for joint, counts in enumerate([state["joy_x"], state["joy_y"]]):

        rate = axis(counts) * MAX_JOINT_VEL

        low, high = JOINT_LIMITS[joint]

        targets[joint] = clamp(targets[joint] + rate * PERIOD, low, high)
        velocities[joint] = abs(rate)

    gripper = int(clamp(state["enc_pos"] / ENC_COUNTS_CLOSED, 0.0, 1.0) * 255)

    spi_service.set_command(targets, velocities, gripper)

    time.sleep(PERIOD)


def shutdown(signum, frame):   # Turn a kill signal into a normal exit so atexit runs
    sys.exit(0)


spi_service.start()

atexit.register(spi_service.stop)
signal.signal(signal.SIGTERM, shutdown)
signal.signal(signal.SIGINT, shutdown)

App.run(user_loop=loop)
