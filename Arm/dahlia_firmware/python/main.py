import time

import requests

from arduino.app_utils import App
from spi_service import SPIService

CONTROLLER_URL = "http://172.17.0.1:8000/controller"

NUM_JOINTS = 5
PERIOD = 0.05

JOY_RANGE = 1000.0   # Full scale joystick counts
JOY_DEADZONE = 0.05   # Normalized joystick deadzone
JOINT_VEL = 5.0   # Constant velocity limit sent for every joint, near the servo maximum

WRIST_PITCH = 3
WRIST_ROLL = 4

# Slew rate in radians per second at full deflection, independent of the velocity limit
JOG_RATES = [
    1.5,   # BASE
    1.5,   # SHOULDER
    1.5,   # ELBOW
    1.5,   # WRIST_PITCH
    1.5    # WRIST_ROLL
]

# Joint limits in radians, matching servo_interface.h
JOINT_LIMITS = [
    (-1.917, 1.917),   # BASE
    (-0.307, 3.375),   # SHOULDER
    (0.0, 3.467),      # ELBOW
    (-1.534, 1.534),   # WRIST_PITCH
    (-1.457, 1.457)    # WRIST_ROLL
]

spi = SPIService()
targets = [0.0] * NUM_JOINTS
seeded = False

gripper_closed = False
enc_was_pressed = False


def clamp(value, low, high):
    return max(low, min(high, value))


def axis(counts):
    """Normalize a joystick axis and apply a deadzone."""
    value = clamp(counts / JOY_RANGE, -1.0, 1.0)

    if abs(value) < JOY_DEADZONE:
        return 0.0

    return value


def update_gripper(pressed):
    """Toggle the gripper on each encoder button press.
    @return Clamp closure byte, 0 = open, 255 = closed
    """
    global gripper_closed, enc_was_pressed

    if pressed and not enc_was_pressed:
        gripper_closed = not gripper_closed

    enc_was_pressed = pressed

    return 255 if gripper_closed else 0


def seed_targets():
    """Start from the arm's measured pose so the first command does not jump."""
    global targets

    feedback = spi.read_feedback()

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

    # Joystick slews the wrist targets, every other joint holds its seeded pose
    velocities = [JOINT_VEL] * NUM_JOINTS

    for joint, counts in [(WRIST_PITCH, state["joy_y"]), (WRIST_ROLL, state["joy_x"])]:

        low, high = JOINT_LIMITS[joint]

        targets[joint] = clamp(targets[joint] + axis(counts) * JOG_RATES[joint] * PERIOD, low, high)

    gripper = update_gripper(state["enc_pressed"])

    if not spi.write_targets(targets, velocities, gripper):
        print("SPI service unavailable")
        seeded = False   # Re-seed from the measured pose once it comes back

    time.sleep(PERIOD)


if not spi.begin():
    print("SPI service did not come up, retrying in the loop")

App.run(user_loop=loop)
