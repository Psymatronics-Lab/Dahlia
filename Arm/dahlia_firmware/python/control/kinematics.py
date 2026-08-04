"""Closed-form kinematics for the Dahlia 5-DOF arm.

The base yaws about the vertical; shoulder, elbow and wrist pitch are all parallel
to each other and perpendicular to it, so they form a planar 3R chain living in the
vertical plane the base selects. Wrist roll then spins the tool about its own
approach axis without moving it. That structure makes both directions analytic.

The end effector is the tip of the wrist roll joint, which sits on the roll axis, so
roll never moves the point and position and orientation decouple completely.

Because the arm is 5-DOF, an arbitrary 6-DOF pose is unreachable: the approach axis
must lie in the plane the base selects. The natural task space is therefore
five-dimensional and cylindrical, which is what this module uses throughout:

    reach   distance from the base axis, in the arm plane   (mm)
    height  above the root origin                           (mm)
    yaw     of the arm plane, = the base joint              (rad)
    pitch   of the approach axis, + is up                   (rad)
    roll    about the approach axis, = the wrist roll joint (rad)

Working in these coordinates rather than Cartesian x/y/z removes the base-yaw
branch, the negative-reach degeneracy and the out-of-plane reachability test
entirely: yaw and roll are joints, and only three numbers ever reach the solver.

Units are millimetres and radians. Geometry is measured off dahlia_m1_full.urdf;
`python control/kinematics.py` re-verifies it and the round trip.
"""

import numpy as np

# Base rotation axis: vertical, offset from the root origin along +x.
BASE_X = 2.2324
# Shoulder axis height above the root origin. It meets the base axis to within 1 um.
BASE_H = 83.5115
# The end effector sits this far to the +y side of the arm plane at zero yaw. The
# planar joints cannot change it, so it is a rigid sideways shift of the whole arm.
SIDE = 0.4502

# Planar links as (length, bend). The bend is the link's angle within the arm plane
# when its joint reads zero, so a link is just a polar offset in its parent's frame.
# B1 is real: the upper arm is a bent casting. B2 and B3 are sub-0.06 deg assembly
# asymmetries, kept only so the model reproduces the URDF exactly.
L1, B1 = 157.0616, 2.9569172    # shoulder -> elbow
L2, B2 = 175.7507, -0.0009798   # elbow -> wrist pitch
L3, B3 = 75.5005, -0.0005102    # wrist pitch -> end effector

# The roll axis is not quite parallel to the link it rides on, so approach pitch
# leads the planar joint sum by this much.
B_APPROACH = -0.0038214

# Radians, mirroring servo_interface.h. These are what the MCU actually enforces.
JOINT_LIMITS = (
    (-1.917, 1.917),    # base yaw
    (-3.375, 0.307),    # shoulder
    (0.0, 3.451),       # elbow
    (-1.534, 1.534),    # wrist pitch
    (-1.457, 1.457)     # wrist roll
)

NUM_JOINTS = len(JOINT_LIMITS)


def fk_task(joints):
    """Task pose (reach, height, yaw, pitch, roll) of a joint vector."""
    yaw, shoulder, elbow, wrist_pitch, roll = joints

    # Each link's absolute angle in the arm plane is the sum of the joints before it.
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

    Yaw and roll are joints outright. Only the planar 3R has to be solved, and the
    wrist decouples from it because the last link is collinear with the approach axis.
    """
    planar = pitch - B_APPROACH   # shoulder + elbow + wrist pitch

    # Wrist pitch centre: step back down the approach axis from the end effector.
    wr = reach - L3 * np.cos(planar + B3)
    wh = height - BASE_H - L3 * np.sin(planar + B3)

    # Elbow interior angle, from the law of cosines on the remaining 2R.
    cosine = (wr * wr + wh * wh - L1 * L1 - L2 * L2) / (2 * L1 * L2)

    if abs(cosine) > 1.0 + 1e-9:
        return []   # Wrist centre outside the annulus the two links can span

    # The tolerance above admits full extension, where rounding drifts a hair past 1;
    # the clip then keeps that same case inside arccos's domain instead of returning NaN.
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

    Shoulder and elbow travel past +/-pi, so folding into [-pi, pi] would throw
    away valid solutions. Every limit spans less than 2*pi, so one shift is enough.
    """
    fitted = []

    for angle, (low, high) in zip(joints, JOINT_LIMITS):
        angle -= 2 * np.pi * np.round((angle - 0.5 * (low + high)) / (2 * np.pi))

        if not low - 1e-9 <= angle <= high + 1e-9:
            return None

        fitted.append(float(np.clip(angle, low, high)))

    return fitted


# --- Cartesian, for display and for checking against the URDF ----------------

def fk(joints):
    """4x4 end effector pose in the root frame.

    The frame is +x along the approach axis, +z up in the arm plane at zero roll,
    so its columns read straight off the task pose.
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
    print(f"  original joint vector among the branches: {recovered}/{samples}")
    print(f"  worst task pose error of fk(ik(pose)):    {worst_task:.2e}")
    print(f"  worst point error of fk(ik(pose)):        {worst_point:.2e} mm")
    print(f"  most simultaneous solutions:              {branches}")
