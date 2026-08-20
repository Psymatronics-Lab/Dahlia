"""Teleoperation for the Dahlia arm using the SEC.

The commanded state is an end effector in the arm's cylindrical task space.
    joystick X          height, straight up and down in the root frame
    joystick Y          reach, along whatever direction the base plate points
    joystick button     gyroscopic following while held: controller roll drives the
                        arm's yaw, controller pitch drives the approach pitch
    encoder turn        roll about the approach axis
    encoder button      snaps the clamp fully open or fully closed
    both buttons        freeze and return to the pose the arm started in, ignoring
                        every input until it settles there

Each coordinate is taken only if the arm can actually reach the result, so a pose
that cannot be solved is never commanded and the operator simply stops at the edge
of the workspace on that axis alone.
"""

import time

import numpy as np

from control.kinematics import JOINT_LIMITS, NUM_JOINTS, fk_task, ik

PERIOD = 0.05   # Control period in seconds

REACH, HEIGHT, YAW, PITCH, ROLL = range(5)   # Task pose layout, matching kinematics.fk_task

JOY_THRESHOLD = 800.0   # Joystick reading deadzone, no proportional input

REACH_RATE = 140.0   # Millimetres per second, along the base plate's heading
HEIGHT_RATE = 140.0   # Millimetres per second, vertically in the root frame

ROLL_PER_COUNT = 0.08   # Radians of approach roll per encoder count

# Orientations for (joystick X, joystick Y) and (controller roll, controller pitch)
JOY_SIGNS = (1.0, 1.0)
GYRO_SIGNS = (-1.0, 1.0)

MAX_JOINT_STEP = 0.2608   # Radians per tick, 5.2155 rad/s at 20 Hz, the servo maximum
MAX_JOINT_ERROR = 1.20   # Radians from the measured arm, beyond this the command waits

HOME_TOLERANCE = 0.05   # Radians per joint that still counts as settled at home
HOME_TIMEOUT = 10.0   # Seconds before homing gives up on a joint that will not arrive

JOINT_NAMES = ("base", "shoulder", "elbow", "wrist pitch", "wrist roll")
REPORT_PERIOD = 2.0   # Seconds between repeats of the same stalled joint warning

LOW = np.array([low for low, _ in JOINT_LIMITS])
HIGH = np.array([high for _, high in JOINT_LIMITS])


def clamp(value, low, high):
    return max(low, min(high, value))


def axis(counts):
    """Joystick is an thresholded switch, no proportional output."""
    if abs(counts) < JOY_THRESHOLD:
        return 0.0

    return 1.0 if counts > 0 else -1.0


def wrap(angle):
    """Fold an angle into [-pi, pi], so crossing +/-180 degrees does not jump."""
    return (angle + np.pi) % (2 * np.pi) - np.pi


class ArmController:
    """Turns controller samples into joint targets through the task space pose."""

    def __init__(self):
        self.ready = False
        self.start = None   # The pose the arm was in at startup, homing returns here

        self.pose = None   # [reach, height, yaw, pitch, roll] in mm and radians
        self.target = None   # Joint vector solving self.pose
        self.joints = None   # Joint vector actually commanded, rate limited

        self.closure = 0.0   # Clamp, 0 = fully open, 1 = fully closed
        self.homing = False
        self.deadline = 0.0
        self.stalled = False

        self.encoder_at = None
        self.encoder_was = False
        self.click_armed = False
        self.gyro_ref = None

        self.reported = 0.0

    def seed(self, feedback):
        """Adopt the arm's measured pose so the first command does not jump."""
        if not feedback["ok"]:
            return False

        measured = np.array(feedback["positions"], dtype=float)

        if self.start is None:
            self.start = measured.copy()   # Copy initial pose once

        self._adopt(measured)
        self.ready = True
        return True

    def update(self, state, feedback):
        """Fold one controller sample into the command. Returns the joint targets."""
        self._read_buttons(state)

        turn = self._encoder_delta(state)

        if self.homing or self.stalled:
            self.gyro_ref = None   # Re-reference on the next engage, never jump
        else:
            self._steer(state, turn)

        self._advance(self.start if self.homing else self.target, feedback)

        if self.homing and self._settled(feedback):
            self.homing = False
            self._adopt(self.start)
            print("Homing: settled, resuming control")

        return [float(a) for a in self.joints]

    def gripper(self):
        """Clamp closure as the byte the firmware expects, 0 open to 255 closed."""
        return int(round(self.closure * 255))

    # ----- input -----

    def _read_buttons(self, state):
        """Encoder click works the clamp; both buttons together start homing.

        The clamp toggles on release rather than press so the two button gesture can
        cancel it. Pressing them together in either order must not also flip the clamp,
        and no operator presses two buttons on exactly the same tick.
        """
        encoder = bool(state["enc_pressed"])
        joystick = bool(state["joy_pressed"])

        if encoder and joystick:
            self.click_armed = False
            self._start_homing()

        elif encoder and not self.encoder_was:
            self.click_armed = True

        elif self.encoder_was and not encoder and self.click_armed:
            self.closure = 0.0 if self.closure > 0.5 else 1.0
            self.click_armed = False

        self.encoder_was = encoder

    def _encoder_delta(self, state):
        """Counts turned since the last tick."""
        counts = int(state["enc_pos"])
        delta = 0 if self.encoder_at is None else counts - self.encoder_at
        self.encoder_at = counts

        return delta

    def _steer(self, state, turn):
        """Build this tick's proposed pose from the controller."""
        pose = list(self.pose)

        pose[HEIGHT] += axis(state["joy_x"]) * JOY_SIGNS[0] * HEIGHT_RATE * PERIOD
        pose[REACH] += axis(state["joy_y"]) * JOY_SIGNS[1] * REACH_RATE * PERIOD

        # Integrated as counts rather than mapped absolutely, so the extrema move with
        # the operator: once roll saturates, turning back moves it on the very next
        # count instead of waiting for the encoder to wind back to where it stuck.
        pose[ROLL] = clamp(pose[ROLL] + turn * ROLL_PER_COUNT, *JOINT_LIMITS[4])

        self._apply_gyro(pose, state)
        self._commit(pose)

    def _apply_gyro(self, pose, state):
        """Joystick button held: controller roll swings the arm's yaw, controller
        pitch tilts the approach axis. Roll about the approach axis stays on the
        encoder, so the gyro never touches it.
        """
        if not state["joy_pressed"]:
            self.gyro_ref = None
            return

        reading = np.radians([state["imu_roll"], state["imu_pitch"]]) * GYRO_SIGNS

        if self.gyro_ref is None:
            # Reference controller against already committed pose as following is relative
            self.gyro_ref = (reading, np.array([self.pose[YAW], self.pose[PITCH]]))

        engaged, held = self.gyro_ref
        yaw, pitch = held + wrap(reading - engaged)

        # Absolute against the reference rather than integrated, so a pitch the arm
        # cannot reach is simply not taken and is picked up again on the way back,
        # instead of winding up an offset while it is refused.
        pose[YAW] = clamp(float(yaw), *JOINT_LIMITS[0])
        pose[PITCH] = float(pitch)

    # ----- solving -----

    def _commit(self, pose):
        """Take each coordinate that leaves the pose solvable and drop the rest.

        Per coordinate rather than all or nothing, so an unreachable pitch cannot veto
        the translation asked for in the same tick. Each axis simply stops where the
        workspace ends while the others keep moving.
        """
        for coordinate in (YAW, ROLL, REACH, HEIGHT, PITCH):
            if pose[coordinate] == self.pose[coordinate]:
                continue

            trial = list(self.pose)
            trial[coordinate] = pose[coordinate]
            solution = self._solve(trial)

            if solution is not None:
                self.pose, self.target = trial, solution

    def _solve(self, pose):
        """Whichever solution is closest to what is already commanded, or None.

        Picking the nearest branch keeps the arm from flipping elbow up to elbow down
        in the middle of a motion.
        """
        solutions = ik(*pose)

        if not solutions:
            return None

        return min((np.array(s) for s in solutions),
                   key=lambda s: np.max(np.abs(s - self.joints)))

    def _advance(self, goal, feedback):
        """Step the command toward a joint vector, rate limited and leashed."""
        step = goal - self.joints
        overshoot = np.max(np.abs(step)) / MAX_JOINT_STEP

        if overshoot > 1.0:
            # Scale the whole vector rather than clipping each joint
            step /= overshoot

        joints = self.joints + step
        self.stalled = False

        if feedback["ok"]:
            # Leash the command to the measured arm rather than freezing it
            measured = np.array(feedback["positions"], dtype=float)
            leashed = np.clip(joints, measured - MAX_JOINT_ERROR, measured + MAX_JOINT_ERROR)
            stuck = np.abs(leashed - joints) > 1e-9

            joints = leashed
            self.stalled = bool(stuck.any())
            self._report(stuck)

        self.joints = np.clip(joints, LOW, HIGH)

    # ----- homing -----

    def _start_homing(self):
        if self.homing:
            return

        self.homing = True
        self.deadline = time.monotonic() + HOME_TIMEOUT
        print("Homing: returning to the startup pose, input ignored until it settles")

    def _settled(self, feedback):
        """Whether the arm has arrived back at the pose it started in."""
        if time.monotonic() > self.deadline:
            print("Homing: gave up waiting for the arm to settle")
            return True

        if np.max(np.abs(self.joints - self.start)) > HOME_TOLERANCE:
            return False   # The command itself has not finished walking home

        if not feedback["ok"]:
            return True   # Nothing to measure against, trust the command

        measured = np.array(feedback["positions"], dtype=float)

        return bool(np.max(np.abs(measured - self.start)) <= HOME_TOLERANCE)

    # ----- helpers -----
    def _adopt(self, joints):
        """Take a joint vector as both the command and the task state."""
        self.joints = np.array(joints, dtype=float)
        self.target = self.joints.copy()
        self.pose = list(fk_task(self.joints))
        self.gyro_ref = None
        self.stalled = False

    def _report(self, stuck):
        """Name any joint the command is waiting on, so a dead servo is visible."""
        now = time.monotonic()

        if not stuck.any() or now - self.reported < REPORT_PERIOD:
            return

        print("Not tracking:", ", ".join(n for n, s in zip(JOINT_NAMES, stuck) if s))
        self.reported = now
