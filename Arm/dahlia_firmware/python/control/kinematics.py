"""
Analytic IK for the Dahlia 5-DOF arm.  Matches the FK document:

  T = Rz(tB) T_BS Rz(tS) T_SE Rz(tE) T_EWP Rz(tWP) T_WPWR Rz(tWR) T_WREE

with H=83.5, T_SE=(-154.5,29,0), T_EWP=(176,0,0),
T_WPWR=(75,0,0) & R=Ry(90)Rz(90), T_WREE=(0,0,99).   Units: mm, rad.

Task space (5 DOF): (x, y, z, phi, psi)
  x,y,z : EE point in the root frame
  phi   : pitch of the approach axis (gripper z) in the arm's vertical
          plane, from horizontal (+ = up), for base yaw tB = atan2(y,x)
  psi   : roll about the approach axis (= tWR when the base is at atan2(y,x))

Because the arm has only 5 DOF, an arbitrary 6-DOF pose is reachable only if
the approach axis lies in the vertical plane through the base axis and the
target point.  ik_pose() checks this and extracts (phi, psi) for you.
"""
import numpy as np

H     = 83.5
L1    = np.hypot(154.5, 29.0)       # effective upper arm, 157.198
ALPHA = np.arctan2(29.0, -154.5)    # built-in bend of link 1, 169.37 deg
L2    = 176.0
L3    = 75.0 + 99.0                 # wrist-pitch joint -> EE along approach

LIM = [(-1.91986, 1.91986), (-3.36466, 0.317981), (0.0, 3.45575),
       (-1.5708, 1.5708), (-1.44331, 1.45394)]

def _fit(a, lo, hi, tol=1e-9):
    """A +/- 2k*pi representation of angle `a` inside [lo, hi], else None."""
    for k in (0, 1, -1, 2, -2):
        c = a + 2*np.pi*k
        if lo - tol <= c <= hi + tol:
            return c
    return None

def ik(x, y, z, phi, psi, check_limits=True):
    """All joint solutions [tB,tS,tE,tWP,tWR] for the task pose (up to 4)."""
    sols = []
    tB0, r0 = np.arctan2(y, x), np.hypot(x, y)
    branches = [(tB0, r0, phi, psi),                          # arm toward target
                (tB0 + np.pi if tB0 <= 0 else tB0 - np.pi,    # base flipped 180
                 -r0, np.pi - phi, psi - np.pi)]
    for tB, r, ph, ps in branches:
        rw = r - L3 * np.cos(ph)              # planar wrist-pitch center
        hw = (z - H) - L3 * np.sin(ph)
        cc = (rw*rw + hw*hw - L1*L1 - L2*L2) / (2.0 * L1 * L2)
        if abs(cc) > 1.0 + 1e-9:
            continue                          # out of reach on this branch
        cc = np.clip(cc, -1.0, 1.0)
        for c in {np.arccos(cc), -np.arccos(cc)}:             # elbow branches
            tE  = ALPHA + c
            tS  = (np.arctan2(hw, rw)
                   - np.arctan2(L2*np.sin(c), L1 + L2*np.cos(c)) - ALPHA)
            tWP = ph - tS - tE
            raw = [tB, tS, tE, tWP, ps]
            if check_limits:
                q = [_fit(a, lo, hi) for a, (lo, hi) in zip(raw, LIM)]
                if any(v is None for v in q):
                    continue
            else:
                q = [(a + np.pi) % (2*np.pi) - np.pi for a in raw]
            if not any(np.allclose(q, p, atol=1e-9) for p in sols):
                sols.append(q)
    return sols

def ik_pose(M, check_limits=True, plane_tol=1e-6):
    """IK from a full 4x4 target pose.  Returns (solutions, lateral_error).
    lateral_error is how far the approach axis is out of the reachable plane
    (mm-free, it's the sine of the out-of-plane angle); if it exceeds
    plane_tol the pose is not exactly reachable with 5 DOF."""
    x, y, z = M[:3, 3]
    a  = M[:3, :3] @ np.array([0.0, 0.0, 1.0])            # approach axis
    tB = np.arctan2(y, x)
    radial  =  a[0]*np.cos(tB) + a[1]*np.sin(tB)
    lateral = -a[0]*np.sin(tB) + a[1]*np.cos(tB)
    phi = np.arctan2(a[2], radial)
    # roll: strip everything before the last joint off the target rotation
    R_pre = _rz(tB) @ _rx(np.pi/2) @ _rz(phi) @ _ry(np.pi/2)
    W = R_pre.T @ M[:3, :3]                                # = Rz(90 + tWR)
    psi = np.arctan2(W[1, 0], W[0, 0]) - np.pi/2
    if abs(lateral) > plane_tol:
        return [], abs(lateral)
    return ik(x, y, z, phi, psi, check_limits), abs(lateral)

# ------------------------------ FK ------------------------------------------
def _rx(a):
    c,s=np.cos(a),np.sin(a); return np.array([[1,0,0],[0,c,-s],[0,s,c]])
def _ry(a):
    c,s=np.cos(a),np.sin(a); return np.array([[c,0,s],[0,1,0],[-s,0,c]])
def _rz(a):
    c,s=np.cos(a),np.sin(a); return np.array([[c,-s,0],[s,c,0],[0,0,1]])
def _T(p, R=np.eye(3)):
    M=np.eye(4); M[:3,:3]=R; M[:3,3]=p; return M

def fk(q):
    tB,tS,tE,tWP,tWR = q
    return (_T([0,0,0],_rz(tB)) @ _T([0,0,H],_rx(np.pi/2)) @ _T([0,0,0],_rz(tS))
            @ _T([-154.5,29,0]) @ _T([0,0,0],_rz(tE)) @ _T([176,0,0])
            @ _T([0,0,0],_rz(tWP)) @ _T([75,0,0],_ry(np.pi/2)@_rz(np.pi/2))
            @ _T([0,0,0],_rz(tWR)) @ _T([0,0,99]))

# ------------------------- round-trip verification ---------------------------
if __name__ == "__main__":
    rng = np.random.default_rng(0)
    N = 20000; n_rec = 0; worst_pos = 0.0; worst_rot = 0.0; max_sols = 0
    for _ in range(N):
        q = [rng.uniform(lo, hi) for lo, hi in LIM]
        M = fk(q)
        sols, lat = ik_pose(M)
        assert sols, "no solution for an FK-generated pose"
        max_sols = max(max_sols, len(sols))
        worst_pos = max(worst_pos, max(np.linalg.norm(fk(s)[:3,3]-M[:3,3]) for s in sols))
        worst_rot = max(worst_rot, max(np.linalg.norm(fk(s)[:3,:3]-M[:3,:3]) for s in sols))
        if min(max(abs(a-b) for a,b in zip(s,q)) for s in sols) < 1e-8:
            n_rec += 1
    print(f"{N} random configs within joint limits:")
    print(f"  original joint vector recovered among branches: {n_rec}/{N}")
    print(f"  worst FK(IK) position error over ALL branches:  {worst_pos:.2e} mm")
    print(f"  worst FK(IK) rotation error over ALL branches:  {worst_rot:.2e}")
    print(f"  max simultaneous valid solutions: {max_sols}")