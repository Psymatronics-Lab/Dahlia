"""Closed-form FK and IK for the Dahlia 5-DOF arm.

The arm follows a right-handed world coordinate axis using the axis toward the
end-effector as the positive X-axis, the axis rightward as the positive Y-axis,
and the axis upward as the positive Z-axis.

The base rotation turntable yaws about the vertical; the shoulder, elbow, and wrist
pitch are all parallel to each other and perpendicular to the turntable axis, forming
a planar 3R chain. Wrist roll spins the end-effector about its own approach axis without
moving the end-effector position.

The end-effector is considered the tip of the wrist roll joint, which sits on the
roll axis, and thus roll never moves the end-effector position. Because the arm is
5-DOF, an arbitary 6-DOF pose is unreachable. The approach axis must lie in the plane the
base selects, and thus the task sapce is 5D and cylindrical.

    reach   distance from the base axis, in the arm plane   (mm)
    height  above the root origin                           (mm)
    yaw     of the arm plane, = the base joint              (rad)
    pitch   of the approach axis, + is up                   (rad)
    roll    about the approach axis, = the wrist roll joint (rad)

Units are millimetres and radians. Geometry is measured off `dahlia_m1_full.urdf` and
`python control/kinematics.py` verifies correct solving.
"""

import numpy as np

BASE_X = 2.2324   # Offset from root origin along +x
BASE_H = 83.5115   # Shoulder axis height above the root origin
SIDE = 0.4502   # Offset from root origin along +y of the end-effector
B_APPROACH = -0.0038214   # Offset of approach pitch to planar joint sum

# Planar links as (length, bend), where the bend is the link's angle with in the arm plane
# when its joint reads zero.
L1, B1 = 157.0616, 2.9569172   # shoulder -> elbow
L2, B2 = 175.7507, -0.0009798   # elbow -> wrist pitch
L3, B3 = 75.5005, -0.0005102   # wrist pitch -> end effector

# Joint limits in radians, mirroring servo_interface.h, enforced by the MCU.
JOINT_LIMITS = (
    (-1.917, 1.917),   # base yaw
    (-3.375, 0.307),   # shoulder
    (0.0, 3.451),   # elbow
    (-1.534, 1.534),   # wrist pitch
    (-1.457, 1.457)   # wrist roll
)
NUM_JOINTS = len(JOINT_LIMITS)


def fk_task(joints):
    """Task pose (reach, height, yaw, pitch, roll) of a joint vector."""
    yaw, shoulder, elbow, wrist_pitch, roll = joints

    a1 = shoulder + B1
    a2 = shoulder + elbow + B2
    a3 = shoulder + elbow + wrist_pitch + B3

    return (
        L1 * np.cos(a1) + L2 * np.cos(a2) + L3 * np.cos(a3),
        BASE_H + L1 * np.sin(a1) + L2 * np.sin(a2) + L3 * np.sin(a3),
        yaw,
        shoulder + elbow + wrist_pitch + B_APPROACH,
        roll
    )


def ik(reach, height, yaw, pitch, roll, check_limits=True):
    """Joint vectors reaching a task pose: elbow up and elbow down, or [] if unreachable.

    Yaw and roll are directly mapped to joints. Only the planar 3R has to be solved, and the
    wrist can be decoupled as the last link is collinear with the approach axis.
    """
    planar = pitch - B_APPROACH   # shoulder + elbow + wrist pitch

    # Wrist pitch center: step back down the approach axis from the end effector
    wr = reach - L3 * np.cos(planar + B3)
    wh = height - BASE_H - L3 * np.sin(planar + B3)

    # Elbow interior angle, from the law of cosines on the remaining 2R
    cosine = (wr * wr + wh * wh - L1 * L1 - L2 * L2) / (2 * L1 * L2)

    if abs(cosine) > 1.0 + 1e-9:
        return []   # Wrist centre outside the annulus the two links can span

    interior = np.arccos(np.clip(cosine, -1.0, 1.0))
    solutions = []

    # The two signs are elbow up and elbow down; at full extension they coincide.
    for bend in ([interior] if interior == 0.0 else [interior, -interior]):
        shoulder = (np.arctan2(wh, wr)
                    - np.arctan2(L2 * np.sin(bend), L1 + L2 * np.cos(bend)) - B1)
        elbow = bend + B1 - B2
        candidate = [yaw, shoulder, elbow, planar - shoulder - elbow, roll]

        if check_limits:
            candidate = _fit_limits(candidate)
        if candidate is not None:
            solutions.append(candidate)

    return solutions


def _fit_limits(joints):
    """The joint vector shifted into its limits, or None if a joint cannot fit.

    Shoulder and elbow travel past +/-pi, thus folding into [-pi, pi] would throw
    away valid solutions. Every limit spans less than 2*pi, thus one shift is enough.
    """
    fitted = []

    for angle, (low, high) in zip(joints, JOINT_LIMITS):
        angle -= 2 * np.pi * np.round((angle - 0.5 * (low + high)) / (2 * np.pi))

        if not low - 1e-9 <= angle <= high + 1e-9:
            return None

        fitted.append(float(np.clip(angle, low, high)))

    return fitted

def fk(joints):
    """4x4 end effector pose in the root frame.
    
    The frame is +x along the approach axis, +z up in the arm plane at zero roll,
    thus columns are read straight off the task pose.
    """
    reach, height, yaw, pitch, roll = fk_task(joints)

    pose = np.eye(4)
    pose[:3, :3] = _rz(yaw) @ _ry(-pitch) @ _rx(roll)
    pose[:3, 3] = (BASE_X + reach * np.cos(yaw) - SIDE * np.sin(yaw),
                   reach * np.sin(yaw) + SIDE * np.cos(yaw),
                   height)
    return pose

def _rx(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])

def _ry(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])

def _rz(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])

# IK solving validation
if __name__ == "__main__":
    rng = np.random.default_rng(0)
    samples = 20000

    recovered = 0
    worst_task = 0.0
    worst_point = 0.0
    branches = 0

    for _ in range(samples):
        q = [rng.uniform(low, high) for low, high in JOINT_LIMITS]
        pose = fk_task(q)
        solutions = ik(*pose)

        assert solutions, "no solution for a pose that forward kinematics produced"
        branches = max(branches, len(solutions))

        for s in solutions:
            worst_task = max(worst_task, np.max(np.abs(np.array(fk_task(s)) - pose)))
            worst_point = max(worst_point, np.linalg.norm(fk(s)[:3, 3] - fk(q)[:3, 3]))

        if min(np.max(np.abs(np.array(s) - q)) for s in solutions) < 1e-9:
            recovered += 1

    print(f"{samples} random configurations within joint limits:")
    print(f"Original joint vector among the branches: {recovered}/{samples}")
    print(f"Worst task pose error of fk(ik(pose)): {worst_task:.2e}")
    print(f"Worst point error of fk(ik(pose)): {worst_point:.2e} mm")
    print(f"Most simultaneous solutions: {branches}")
