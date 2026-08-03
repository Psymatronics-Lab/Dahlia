"""Task space teleoperation for the Dahlia arm.

Controller input drives an end effector pose in cylindrical task space, the analytic
IK turns that pose into joint angles, and every solution is rate limited and checked
against the measured arm before it becomes an SPI command.
"""

import time

import numpy as np

from control.kinematics import L3, fk, ik

NUM_JOINTS = 5
PERIOD = 0.05   # Control period in seconds

JOY_RANGE = 1000.0   # Full scale joystick counts
JOY_DEADZONE = 0.05   # Normalized joystick deadzone

REACH_RATE = 140.0   # Millimetres per second at full joystick deflection
HEIGHT_RATE = 140.0   # Currently unbound, no input drives height

# Task space guards. Reach must stay positive: a negative radial coordinate flips the
# base branch and the pose parameterization stops being one to one.
REACH_RANGE = (60.0, 480.0)
HEIGHT_RANGE = (-250.0, 550.0)

# Applied to (yaw, pitch, roll) before they drive (yaw, phi, psi). Flip a sign here
# if the arm mirrors the controller on that axis.
GYRO_SIGNS = (1.0, 1.0, 1.0)

ENC_COUNTS_FULL = 30.0   # Encoder counts from fully open to fully closed

MAX_JOINT_STEP = 0.25   # Radians per tick, caps how fast a solution is approached
MAX_JOINT_ERROR = 1.20   # Radians from the measured arm, beyond this a joint is held

JOINT_NAMES = ("base", "shoulder", "elbow", "wrist pitch", "wrist roll")
REPORT_PERIOD = 2.0   # Seconds between repeats of the same stuck joint warning

# Joint limits in radians, matching servo_interface.h
JOINT_LIMITS = [
    (-1.917, 1.917),   # BASE
    (-3.375, 0.307),   # SHOULDER
    (0.0, 3.451),      # ELBOW
    (-1.534, 1.534),   # WRIST_PITCH
    (-1.457, 1.457)    # WRIST_ROLL
]


def clamp(value, low, high):
    return max(low, min(high, value))


def axis(counts):
    """Normalize a joystick axis and apply a deadzone."""
    value = clamp(counts / JOY_RANGE, -1.0, 1.0)

    if abs(value) < JOY_DEADZONE:
        return 0.0

    return value


def wrap(angle):
    """Fold an angle into [-pi, pi]."""
    return (angle + np.pi) % (2 * np.pi) - np.pi


class ArmController:
    """Holds the commanded end effector pose and turns controller input into joint targets."""

    def __init__(self):
        self.ready = False
        self.enabled = True   # Encoder button gates arm motion, the clamp ignores it
        self.closure = 0.0   # Normalized clamp closure, 0 = open
        self.targets = [0.0] * NUM_JOINTS   # Joint angles in radians
        self.pose = None   # [reach, height, yaw, phi, psi] in mm and radians
        self.joints = None   # Last commanded joint vector
        self.enc_last = None
        self.enc_held = False
        self.gyro_ref = None
        self.reported = 0.0

    def seed(self, feedback):
        """Adopt the arm's measured pose so the first command does not jump."""
        if not feedback["ok"]:
            return False

        q = np.array(feedback["positions"], dtype=float)
        position = fk(q)[:3, 3]
        yaw = float(q[0])

        self.pose = [
            float(position[0] * np.cos(yaw) + position[1] * np.sin(yaw)),
            float(position[2]),
            yaw,
            float(q[1] + q[2] + q[3]),   # Approach pitch is the sum of the planar joints
            float(q[4])
        ]
        self.joints = q
        self.targets = [float(a) for a in q]
        self.ready = True
        return True

    def update(self, state, feedback):
        """Fold one controller sample into the pose. Returns the joint targets to send."""
        self._update_closure(state)
        self._update_enable(state)

        if not self.enabled:
            self.gyro_ref = None   # Re-reference the gyro when control resumes
            return self.targets

        pose = list(self.pose)
        self._apply_joystick(pose, state)
        self._apply_gyro(pose, state)
        self._solve(pose, feedback)

        return self.targets

    def gripper(self):
        """Clamp closure as the byte the firmware expects."""
        return int(round(self.closure * 255))

    def _update_closure(self, state):
        """Integrate encoder motion, so reversing at an extreme moves the clamp at once."""
        counts = state["enc_pos"]

        if self.enc_last is None:
            self.enc_last = counts

        self.closure = clamp(self.closure + (counts - self.enc_last) / ENC_COUNTS_FULL, 0.0, 1.0)
        self.enc_last = counts

    def _update_enable(self, state):
        """The encoder button toggles arm control. Torque stays on either way."""
        pressed = bool(state["enc_pressed"])

        if pressed and not self.enc_held:
            self.enabled = not self.enabled

        self.enc_held = pressed

    def _apply_joystick(self, pose, state):
        """Y drives reach along the current yaw. X and height are unbound."""
        pose[0] = clamp(pose[0] + axis(state["joy_y"]) * REACH_RATE * PERIOD, *REACH_RANGE)

    def _apply_gyro(self, pose, state):
        """Joystick button held: mirror controller rotation onto the end effector."""
        if not state["joy_pressed"]:
            self.gyro_ref = None
            return

        gyro = np.radians([state["imu_yaw"], state["imu_pitch"], state["imu_roll"]]) * GYRO_SIGNS

        if self.gyro_ref is None:
            # Reference on engage, so the arm never jumps and controller drift is shed
            self.gyro_ref = (gyro, np.array(pose[2:5]))

        reference, held = self.gyro_ref
        previous_phi = pose[3]
        pose[2:5] = [float(a + wrap(b - c)) for a, b, c in zip(held, gyro, reference)]

        # Pitch about the wrist joint rather than the tool tip. Holding the wrist centre
        # still leaves the shoulder and elbow untouched, so wrist pitch takes the change.
        pose[0] = clamp(pose[0] + L3 * (np.cos(pose[3]) - np.cos(previous_phi)), *REACH_RANGE)
        pose[1] = clamp(pose[1] + L3 * (np.sin(pose[3]) - np.sin(previous_phi)), *HEIGHT_RANGE)

    def _solve(self, pose, feedback):
        """Accept the proposed pose, falling back to whatever part of it is reachable."""
        target = self._nearest(pose)

        if target is None:
            pose[3] = self.pose[3]   # Pitch is the axis most often out of reach, so hold it
            target = self._nearest(pose)

        if target is None:
            return   # Nothing about this proposal is reachable

        step = np.clip(target - self.joints, -MAX_JOINT_STEP, MAX_JOINT_STEP)

        if feedback["ok"]:
            # Hold a joint the arm is not tracking, rather than vetoing every other joint
            stuck = np.abs(target - np.array(feedback["positions"], dtype=float)) > MAX_JOINT_ERROR
            step[stuck] = 0.0
            self._report(stuck)

        self.joints = self.joints + step
        self.pose = pose
        self.targets = [clamp(float(a), lo, hi) for a, (lo, hi) in zip(self.joints, JOINT_LIMITS)]

    def _nearest(self, pose):
        """Closest solution to the current command, or None if the pose is unreachable."""
        reach, height, yaw, phi, psi = pose

        solutions = ik(reach * np.cos(yaw), reach * np.sin(yaw), height, phi, psi)

        if not solutions:
            return None

        return np.array(min(solutions, key=lambda s: np.max(np.abs(np.array(s) - self.joints))))

    def _report(self, stuck):
        """Name any joint that is being held back, so a dead servo is visible."""
        now = time.monotonic()

        if not stuck.any() or now - self.reported < REPORT_PERIOD:
            return

        print("Not tracking:", ", ".join(n for n, s in zip(JOINT_NAMES, stuck) if s))
        self.reported = now
