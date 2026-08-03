# Dahlia Arm — Kinematics and Control

Host-side control for the 5-DOF Dahlia arm. Everything here runs on the Linux side; the
MCU only ever receives joint angles over SPI and never solves kinematics itself.

| File | Role |
| --- | --- |
| [`kinematics/ik.py`](kinematics/ik.py) | Analytic forward and inverse kinematics |
| [`control.py`](control.py) | Controller input → end effector pose → joint targets |
| [`main.py`](main.py) | Poll the BLE controller, run the loop, push commands over SPI |

Units are millimetres and radians throughout.

---

## Part 1 — Inverse Kinematics

### Why this arm admits a closed form

The three middle joints — shoulder, elbow, wrist pitch — are parallel, so joints 2–4 form a
planar 3R mechanism living in a vertical plane selected by the base yaw. The lateral offsets
cancel by design, so that plane passes exactly through the base axis, which is what makes the
decoupling clean. No numerical solver, no iteration, no seed sensitivity.

### Task space

Five degrees of freedom cannot reach an arbitrary 6-DOF pose. The gripper's approach axis can
only ever lie in the vertical plane containing the base axis and the target point, so the
natural task space is five-dimensional:

```
(x, y, z, φ, ψ)
  x, y, z : end effector point in the root frame
  φ       : pitch of the approach axis within the arm's vertical plane (+ = up)
  ψ       : roll about the approach axis
```

The end effector frame follows the convention **+Z forward (approach), +X up, +Y right**, where
"right" is as seen from behind the arm looking forward. This falls out of the FK natively at
ψ = 0 — verified directly:

```
home, ψ=0     X=[0.00 0.00 1.00]  Y=[0.00 -1.00 0.00]  Z=[1.00 0.00 0.00]
              (root frame is +x forward, +y left, +z up)
```

So EE +Z is root forward, EE +X is root up, and EE +Y is root −y, i.e. right. Base yaw carries
the whole frame around with it, so the convention holds at any azimuth.

If you hand `ik_pose()` a full 4×4 pose it first checks that the pose's z-axis lies in the plane
through the base axis and the target point. When it doesn't, the pose is unreachable no matter
what, and the function returns the out-of-plane error rather than a wrong answer.

### Step 1 — base yaw

```
θ_B = atan2(y, x)        r = √(x² + y²)        h = z − 83.5
```

A second branch exists at `θ_B ± π` with `r → −r`, reaching backward over the base, where the
in-plane angles remap as `φ' = π − φ` and `ψ' = ψ − π`.

### Step 2 — wrist decoupling

The last two links (75 mm wrist, 99 mm tool) are collinear with the approach axis, so the
wrist-pitch joint sits at a known offset from the target — the 5-DOF analogue of the spherical
wrist trick. Position and orientation decouple there:

```
r_w = r − 174·cos φ        h_w = h − 174·sin φ
```

### Step 3 — planar 2R with a bent link

The upper arm isn't straight. The `(−154.5, 29)` offset gives an effective link with a built-in
bend that the solution absorbs as a constant:

```
L₁ = √(154.5² + 29²) ≈ 157.198        α = atan2(29, −154.5) ≈ 169.37°
L₂ = 176
cos c = (r_w² + h_w² − L₁² − L₂²) / (2 L₁ L₂)
θ_E = α ± arccos(c)          # elbow-up / elbow-down branches
θ_S = atan2(h_w, r_w) − atan2(L₂ sin c, L₁ + L₂ cos c) − α
```

`|cos c| > 1` means the wrist centre is out of reach on this branch.

### Step 4 — the rest falls out

```
θ_WP = φ − θ_S − θ_E          # wrist pitch just closes the orientation sum
θ_WR = ψ
```

The trailing `Rz(90°)` in `T_WP→WR` only shifts where roll-zero points; `ik_pose()` strips it off
when extracting ψ from a pose.

### Branches and filtering

Two base branches × two elbow branches gives up to four candidates, filtered through joint
limits. In practice at most **two** survive — confirmed over 20 000 random configurations.

### Two implementation traps

Both were caught in testing and both matter if this is ever ported to firmware:

1. **Don't wrap angles into [−π, π].** The shoulder and elbow limits extend past ±π (−3.375 and
   +3.451 rad). Naive wrapping silently discards valid solutions. `_fit()` tests the ±2kπ
   representations against the limits instead.
2. **Clip the arccos argument.** Near the workspace boundary it drifts a hair past 1.0 from
   floating point and `arccos` returns NaN.

### Singularities

- **Base axis** (`r ≈ 0`): yaw is undefined. Pick any value, or hold the previous one for
  continuity. The controller sidesteps this by keeping reach ≥ 60 mm.
- **Full extension** (`cos c = ±1`): the two elbow branches collapse into one.

### Model accuracy

This solves the model, which sits ~2–3 mm off the URDF, mostly from an ignored 2.2 mm base
offset. Fine at grasping scale. To recover it, add 2.2 mm as a fixed radial offset in step 1.

### Verification

`python kinematics/ik.py` runs a round trip over 20 000 random in-limit configurations:

```
original joint vector recovered among branches: 20000/20000
worst FK(IK) position error over ALL branches:  4.36e-13 mm
worst FK(IK) rotation error over ALL branches:  2.75e-13
max simultaneous valid solutions: 2
```

---

## Part 2 — Control System

### Task space state

The controller holds the commanded pose in **cylindrical** coordinates rather than Cartesian,
because that is the shape of the operator's mental model — reach out, go up, swing around:

```
pose = [reach, height, yaw, φ, ψ]
```

`x = reach·cos(yaw)`, `y = reach·sin(yaw)`, `z = height`. Joystick jogging is therefore always
relative to the current yaw with no extra bookkeeping: pushing forward extends along whatever
direction the arm currently faces.

**Reach must stay positive.** A negative radial coordinate flips the base branch and the
parameterization stops being one-to-one — the round trip fails about 30% of the time and errors
reach π rad. With reach clamped positive it is exact: 0 failures over 13 383 samples, worst
joint error 2.02e-12 rad.

### Input mapping

| Input | Effect | Active when |
| --- | --- | --- |
| Joystick Y | Reach — forward/backward along the current yaw | Always (unless arm disabled) |
| Joystick X | *unbound* | — |
| Joystick button (**hold**) | Gyroscopic following | While held |
| Gyro yaw / pitch / roll | Drives yaw / φ / ψ | Only while the joystick button is held |
| Rotary encoder | Clamp closure | Always, even when the arm is disabled |
| Encoder button (**press**) | Toggles all arm control | Always |

Height is currently unbound — no input drives it, so it holds whatever value it was seeded
with. `HEIGHT_RATE` is still defined, so binding an axis to it is a one-line change in
`_apply_joystick()`.

### Arm enable toggle

The encoder button toggles `enabled`. When off, joystick and gyro input are ignored and the last
joint targets keep being sent, so **the arm stays powered and holds position** — it is not
limp. The clamp keeps working. Pressing again resumes, and the gyro reference is dropped on
disable so re-engaging never jumps.

### Gyroscopic following

While the joystick button is held, controller rotation drives the end effector orientation. With
5 DOF the yaw of the approach axis is not independent of position — the approach axis must lie in
the plane through the base axis and the target — so gyro yaw rotates the base, carrying the end
effector around at constant reach and height. Pitch drives φ and roll drives ψ directly.

Following is **relative, not absolute**. On engage, the current gyro reading *and* the current
pose orientation are captured as a reference; only the delta from that reference is applied, and
the reference is dropped the moment the button is released. Three things follow:

- Engaging never jumps the arm, whatever attitude the controller is in.
- You can release, carry the controller to a more comfortable position, and re-engage — the arm
  holds still throughout and resumes from wherever it was.
- Long-term gyro drift is shed on every press instead of accumulating.

Absolute mimicry would snap the arm to the controller's attitude on press, which the safety layer
would reject anyway. Measured: 0.0000 rad jump on engage, 0.0000 rad after releasing, moving the
controller 90°+ on two axes, and re-engaging.

`GYRO_SIGNS` in `control.py` flips any axis that mirrors the wrong way on hardware.

### Clamp with anti-windup

Encoder motion is integrated as **deltas** into a normalized closure that saturates at [0, 1],
rather than mapping the absolute encoder count. This is what makes reversal immediate: once the
clamp bottoms out, turning back moves it on the very next count instead of waiting for the
encoder to return to wherever it first hit the limit. Verified: saturated at 1.00, one reverse
step → 0.90.

### Safety layer

Every proposed pose passes three gates before it becomes a command:

1. **Reachability, with fallback.** If `ik()` returns no solution, the solve is retried holding
   the *previous* φ, since pitch is by far the axis most often out of reach (see below). Only if
   that also fails is the whole pose reverted. Without the fallback, an unreachable pitch would
   veto the reach and yaw requested in the same tick — position control would go dead whenever
   you tilted the controller too far.
2. **Branch continuity.** Among surviving solutions, the one minimizing the largest per-joint
   change from the current command is chosen, so the arm never flips elbow-up to elbow-down
   mid-motion.
3. **Per-joint deviation guard.** Each joint's solution is compared against the *measured* angle
   from SPI feedback. A joint further than `MAX_JOINT_ERROR` (1.20 rad) is **held**, and the
   others still move. This matters when a servo loses torque: an all-or-nothing check would let
   one dead joint freeze the entire arm in bursts, which reads as stuttering on joints that are
   working fine. Held joints are named on stdout at most every 2 s, so a dead servo is visible.

Surviving steps are clamped to `MAX_JOINT_STEP` (0.25 rad/tick ≈ 5 rad/s, matching the velocity
ceiling sent to the MCU), and the result is clamped to the firmware joint limits as a backstop.

### Pitch rotates about the wrist, not the tool tip

Pitch needs a choice that the task space alone does not settle: when φ changes, *what stays
fixed?* The two options behave completely differently.

Holding the **tool tip** fixed forces the wrist centre to sweep a 174 mm arc around it, which the
shoulder and elbow have to produce. Wrist pitch ends up contributing almost nothing until the big
joints hit their limits, and much of the range is simply unreachable — from the home pose, *no*
positive pitch solved at all, because the wrist centre sits only 21.5 mm off the base axis and
pitching up walked it behind the base.

Holding the **wrist centre** fixed is the natural choice. Step 3 of the IK derives θ_S and θ_E
from `(r_w, h_w)` alone, so freezing the wrist centre leaves both untouched and `θ_WP = φ − θ_S −
θ_E` absorbs the entire change. Pitch becomes a pure wrist-pitch motion.

`_apply_gyro()` implements this by compensating the tool-tip position whenever φ moves:

```python
pose[0] += L3 * (cos φ_new − cos φ_old)      # reach
pose[1] += L3 * (sin φ_new − sin φ_old)      # height
```

Measured effect — a 40° controller pitch from the home pose:

```
base +0.00   shoulder +0.00   elbow +0.00   wristP +40.00   wristR +0.00  (degrees)
```

Reachable pitch from home widens from "nothing above 0°" to roughly **−90° … +79°**, which is
essentially the wrist pitch joint's own limit (±1.534 rad). The tool tip now swings on a 174 mm
arc as you pitch, which is what a wrist does.

Joint velocity sent to the MCU is a constant `JOINT_VEL = 5.0` rad/s. It is a slew *ceiling*, not
a speed setting — actual speed comes from how fast the targets move. Sending a velocity at or
below the demanded rate starves the MCU's rate limiter and causes windup.

### Data flow

```
SEC controller (BLE)
  → controller_service  :8000/controller     (JSON controller state)
    → main.py           20 Hz loop
      → control.py      pose update → IK → rate limit → joint targets
        → spi_service   :9000/targets        (HTTP)
          → SPI brick   100 Hz full-duplex exchange
            → MCU       200 Hz servo loop
              ← feedback: measured joints, status, per-joint fault bytes
```

Feedback returns on the same SPI transaction that carries the command. `main.py` seeds the task
pose from measured joints at startup, so the first command never jumps, and re-seeds if the SPI
service drops out.

### Tuning constants

All in [`control.py`](control.py):

| Constant | Default | Meaning |
| --- | --- | --- |
| `REACH_RATE` | 140 mm/s | Jog speed at full deflection |
| `HEIGHT_RATE` | 140 mm/s | Unused until an axis is bound to height |
| `REACH_RANGE` | (60, 480) mm | Radial guard; low end avoids the base singularity |
| `HEIGHT_RANGE` | (−250, 550) mm | Vertical guard |
| `GYRO_SIGNS` | (1, 1, 1) | Per-axis flip for yaw/pitch/roll |
| `ENC_COUNTS_FULL` | 30 | Encoder counts from open to closed |
| `MAX_JOINT_STEP` | 0.25 rad | Per-tick rate limit (≈ 5 rad/s at 20 Hz) |
| `MAX_JOINT_ERROR` | 1.20 rad | Per-joint hold threshold vs measured |
| `JOY_DEADZONE` | 0.05 | Normalized joystick deadzone |

### Control verification

Simulated against a perfect-tracking arm, 4 000 randomized ticks plus targeted cases:

```
reach jog out and back      195.5 -> 335.5 -> 195.5 mm   (height held)
joy_y drives reach          195.5 -> 263.9 mm;  joy_x moves nothing
FK of commanded joints      matches commanded pose to 0.01 mm
clamp anti-windup           1.00 saturated, 0.90 after one reverse step
disabled                    joints frozen, pose frozen, clamp still live
gyro engage                 0.0000 rad jump; +10 deg yaw -> base joint +10.0 deg
gyro release / re-engage    0.0000 rad jump after moving the controller 90 deg+ away
pitch drives the wrist      40 deg pitch -> wristP +40.00 deg, all other joints +0.00
pitch range from home       -90 .. +79 deg (was: no positive pitch reachable)
step limiting               max observed step 0.217 rad -> 4.3 rad/s
unreachable pitch           reach and yaw still applied (195.5 -> 332.2 mm, yaw 25 deg)
limp shoulder simulated     wrist roll still tracks 60 deg, monotonic, no stutter
joint limit violations      0 / 3000 randomized ticks
```

Hardware behaviour is unverified — the gyro axis signs and `ENC_COUNTS_FULL` in particular are
best confirmed on the bench.
