# Dahlia Arm — Kinematics and Control

Host-side control for the 5-DOF Dahlia arm. Everything here runs on the Linux side; the
MCU only ever receives joint angles over SPI and never solves kinematics itself.

| File | Role |
| --- | --- |
| [`control/kinematics.py`](control/kinematics.py) | Closed-form forward and inverse kinematics |
| [`control/control.py`](control/control.py) | Controller input → end effector pose → joint targets |
| [`main.py`](main.py) | Poll the BLE controller, run the loop, push commands over SPI |

Units are millimetres and radians throughout.

---

## Part 1 — Kinematics

### The end effector

The end effector is the **tip of the wrist roll joint**. It sits exactly on the roll axis,
so roll spins the tool without moving the point: position and orientation decouple
completely, and the last joint drops out of the position problem entirely.

### Why this arm admits a closed form

The base yaws about the vertical. Shoulder, elbow and wrist pitch are all parallel to each
other and perpendicular to the base axis, so they form a planar 3R chain living in the
vertical plane the base selects. Wrist roll then spins the tool about its own approach
axis. No numerical solver, no iteration, no seed sensitivity.

### Task space is cylindrical, not Cartesian

Five degrees of freedom cannot reach an arbitrary 6-DOF pose — the approach axis is stuck
in whatever plane the base selects. So rather than accept Cartesian input and then check
whether it happens to be legal, this module uses the mechanism's own coordinates:

```
reach    distance from the base axis, in the arm plane    (mm)
height   above the root origin                            (mm)
yaw      of the arm plane, = the base joint               (rad)
pitch    of the approach axis, + is up                    (rad)
roll     about the approach axis, = the wrist roll joint  (rad)
```

This is the single biggest simplification over a Cartesian formulation, and it deletes
three separate sources of trouble rather than working around them:

- **No base-yaw branch.** `yaw` *is* the base joint, so there is nothing to invert. The
  old Cartesian version needed a second `θ_B ± π` branch with `r → −r`, `φ → π − φ`,
  `ψ → ψ − π`, and that reparameterization stopped being one-to-one for negative reach.
- **No base singularity.** At `reach = 0` a Cartesian `atan2(y, x)` is undefined; here
  yawing at zero reach just spins the base, which is perfectly well defined.
- **No out-of-plane test.** A pose that misses the reachable plane cannot be expressed in
  the first place, so there is no lateral-error check to run and no way to ask for one.

Only three numbers — reach, height, pitch — ever reach the solver. Yaw and roll are joints.

### Geometry

Every constant is measured off `dahlia_m1_full.urdf` by collapsing its fixed joints. Each
link is stored as a plain polar offset in its parent's frame — a length and the bend angle
it sits at when its joint reads zero:

```
base axis      offset  2.2324 mm along +x from the root origin
shoulder axis  height 83.5115 mm, meeting the base axis to within 1 µm
L1, B1  157.0616 mm,  2.9569172 rad    shoulder → elbow
L2, B2  175.7507 mm, −0.0009798 rad    elbow → wrist pitch
L3, B3   75.5005 mm, −0.0005102 rad    wrist pitch → end effector
approach axis, −0.0038214 rad off the wrist pitch frame
sideways offset of the end effector from the arm plane, 0.4502 mm
```

Only `B1` is a design feature — the upper arm is a bent casting. `B2`, `B3` and the
approach tilt are sub-0.06° assembly asymmetries in the CAD export; they are kept because
carrying them costs nothing (every link already has a bend term) and it makes the model
reproduce the URDF exactly instead of approximately.

The sideways offset deserves a note: the individual joint axes sit 15–34 mm off the arm
plane, but those offsets **cancel down the chain** to 0.45 mm at the end effector. That is
what makes the planar decomposition legitimate. Since no planar joint can change it, it is
a rigid sideways shift of the whole arm and only appears when converting to Cartesian.

### Forward kinematics

Each link's absolute angle in the arm plane is just the sum of the joints before it:

```
a₁ = θ_S + B₁
a₂ = θ_S + θ_E + B₂
a₃ = θ_S + θ_E + θ_WP + B₃

reach  =          L₁cos a₁ + L₂cos a₂ + L₃cos a₃
height = 83.5115 + L₁sin a₁ + L₂sin a₂ + L₃sin a₃
pitch  = θ_S + θ_E + θ_WP − 0.0038214
yaw    = θ_B                    roll = θ_WR
```

### Inverse kinematics

```
Σ   = pitch + 0.0038214                       # θ_S + θ_E + θ_WP
w   = (reach, height − 83.5115) − L₃·u(Σ + B₃)   # step back to the wrist pitch centre
cos c = (|w|² − L₁² − L₂²) / (2 L₁ L₂)        # law of cosines on the remaining 2R
θ_E = ±c + B₁ − B₂
θ_S = atan2(w_h, w_r) − atan2(L₂ sin c, L₁ + L₂ cos c) − B₁
θ_WP = Σ − θ_S − θ_E                          # wrist pitch just closes the sum
```

`|cos c| > 1` means the wrist centre is outside the annulus the two links can span. The
two signs of `c` are the elbow-up and elbow-down branches, which is the *only* remaining
multiplicity — at most **two** solutions, down from four.

### Two implementation traps

Both matter if this is ever ported to firmware:

1. **Don't wrap angles into [−π, π].** Shoulder and elbow travel past ±π (−3.375 and
   +3.451 rad), so naive wrapping silently discards valid solutions. `_fit_limits()`
   shifts each angle by the multiple of 2π nearest the middle of its own limit range
   instead. Every limit spans less than 2π, so one shift is always enough.
2. **Guard the arccos argument at both ends.** Near full extension it drifts a hair past
   1.0 from floating point. A strict `> 1.0` rejection would refuse legitimate
   full-extension poses, and no rejection at all returns NaN — so the range check carries
   a 1e-9 tolerance and the argument is then clipped before `arccos` sees it.

### Verification against the URDF

The model is checked against a numeric FK built straight from the URDF, over 5 000 random
in-limit configurations:

```
end effector position error   max 0.0037 mm
approach axis error           max 0.00068 deg
roll joint rotation           max 5.9e-15 rad
```

For comparison, the previous model — which used rounded link lengths, ignored the 2.2 mm
base offset and ignored the 0.45 mm sideways offset — was **2.27 mm mean, 3.19 mm max**
against the same reference. The rewrite is effectively exact.

`python control/kinematics.py` runs the round trip over 20 000 random in-limit configs:

```
original joint vector among the branches: 20000/20000
worst task pose error of fk(ik(pose)):    3.98e-13
worst point error of fk(ik(pose)):        4.02e-13 mm
most simultaneous solutions:              2
```

### Workspace

Straight out, the arm reaches `L₁ + L₂ + L₃ = 408.3 mm`. Within joint limits the envelope
is roughly 408 mm of reach and −231 … +492 mm of height.

Pitch range available with the end effector point held fixed, sampled on a grid:

| reach | height 0 | 100 | 200 | 300 |
| --- | --- | --- | --- | --- |
| 120 mm | 67° | 108° | 129° | 129° |
| 200 mm | 87° | 112° | 129° | 129° |
| 280 mm | 108° | 125° | 129° | 120° |
| 350 mm | 125° | 139° | 111° | unreachable |

This is what putting the end effector at the wrist roll tip buys. When the tool tip was
99 mm further out, holding it fixed forced the wrist centre onto a 174 mm arc that the
shoulder and elbow had to produce, and from the home pose *no* positive pitch solved at
all. At 75.5 mm the arc is small enough that pitch is simply a free axis nearly everywhere,
so the controller can treat it as a pure rotation with no compensation trickery.

---

## Part 2 — Control System

### Input mapping

| Input | Effect |
| --- | --- |
| Joystick X | Height — straight up and down in the root frame |
| Joystick Y | Reach — along the direction the base plate points |
| Joystick button (**hold**) | Gyroscopic following |
| ↳ controller roll | Arm yaw |
| ↳ controller pitch | Approach pitch |
| Rotary encoder (**turn**) | Roll about the approach axis |
| Encoder button (**click**) | Snaps the clamp fully open / fully closed |
| **Both buttons** | Freeze and return to the startup pose |

Because the task space is already cylindrical, both stick axes are one-line increments:

```python
pose[HEIGHT] += axis(joy_x) * HEIGHT_RATE * PERIOD
pose[REACH]  += axis(joy_y) * REACH_RATE  * PERIOD
```

**Joystick X is absolute**, in the root frame — pure vertical motion regardless of where
the tool points. **Joystick Y follows the yaw only** — it moves along the base plate's
heading, and the tool's pitch and roll never steer it. Measured across three start poses
(yaw 0°/+34°/−46°, pitch +40°/−23°/+69°, roll 0°/+52°/−63°) and both directions, every tick
is a clean 7.00 mm step with **off-axis motion below 1e-13 mm and zero pitch drift**.

**The joystick is a switch, not a proportional axis.** Past `JOY_THRESHOLD` (800 counts)
the axis commands full rate; below it, nothing. Short corrections land predictably instead
of depending on how far the stick was pushed.

### Reachability is the guard

There are no hand-tuned reach or height boxes. Each coordinate is applied **one at a time**
and kept only if the resulting pose still solves:

```python
for coordinate in (YAW, ROLL, REACH, HEIGHT, PITCH):
    trial = list(self.pose)
    trial[coordinate] = pose[coordinate]
    solution = self._solve(trial)

    if solution is not None:
        self.pose, self.target = trial, solution
```

Per coordinate rather than all-or-nothing, so an unreachable pitch cannot veto the
translation asked for in the same tick — each axis stops where the workspace ends while the
others keep moving. Measured: driven to the reach limit, reach pins at 284.3 mm while
height still moves freely. Because the commanded pose is never allowed to leave the
reachable set, it also cannot wind up, so no clamping is needed to bound it.

Yaw and roll are clamped directly to their joint limits instead, since each maps to exactly
one joint and the mapping is exact.

### Gyroscopic following

While the joystick button is held, controller **roll** drives the arm's yaw and controller
**pitch** drives the approach pitch. With 5 DOF the approach axis must lie in the plane the
base selects, so "yaw" necessarily means rotating the base, carrying the end effector
around at constant reach and height. Roll about the approach axis stays on the encoder.

Following is **relative**: on engage, the controller attitude *and* the committed pose are
captured as a reference, and only the delta is applied. So engaging never jumps, the
controller can be released and repositioned freely, and gyro drift is shed on every press
instead of accumulating. Measured: **0.0000 rad jump** on engage, on release, and on
re-engage after moving the controller 90°+ on two axes; +30° of controller roll gives
+30.000° of yaw, −25° of controller pitch gives −25.000° of pitch, neither touching the
other.

The delta is applied **absolutely against the reference** rather than integrated. A pitch
the arm cannot reach is simply not taken, and is picked up again on the way back, instead
of winding up an offset while it is being refused.

Pitch is a **pure rotation** — the end effector point does not move. Measured 5.7e-14 mm of
translation across a 75° pitch sweep. The old code had to shove reach and height around to
fake rotation about the wrist centre, and then undo that shove when the pose was rejected;
with the end effector at the roll tip, none of that is needed.

### Roll on the encoder

Encoder counts integrate into roll at `ROLL_PER_COUNT` (0.08 rad ≈ 4.6° per count) and
clamp at the joint limit. Integrating deltas rather than mapping absolutely is what gives
the **moving extrema**: once roll saturates, turning back moves it on the very next count
instead of waiting for the encoder to wind back to wherever it first hit the stop.
Measured: 10 counts gives +45.84°, spinning far past the stop saturates at exactly the
+83.480° limit, and a single count back drops it to +78.896°.

Counts are consumed every tick and banked by none, so a tick that ignores input (homing, or
a stalled joint) discards the motion that happened during it rather than applying it later
in a lump.

### Clamp

The encoder button snaps the clamp fully open or fully closed. It toggles on **release**
rather than press, which is what lets the two-button gesture cancel it: pressing both
buttons in either order starts homing and leaves the clamp untouched. No operator presses
two buttons on exactly the same 50 ms tick, so a press-triggered clamp would flip on the
way into every homing gesture. Verified both orders.

### Return to the startup pose

Pressing **both buttons** freezes everything and walks the arm back to the configuration it
was in at startup, captured once on the first seed so it always means the same place.
While homing:

- every input is ignored — joystick, gyro, encoder and clamp alike;
- the commanded pose is not touched, so nothing changes underneath the operator;
- the gyro reference is dropped and encoder counts are discarded, so nothing that happened
  during homing leaks out afterwards.

It ends when the *measured* joints are all within `HOME_TOLERANCE` (0.05 rad) of the
startup pose, at which point the task pose is re-synced from it and control resumes.
Verified: pose constant for every tick of homing, clamp unchanged, no banked encoder
motion, and against an arm slewing at 2 rad/s it settles in 9 ticks (0.45 s).

`HOME_TIMEOUT` (10 s) is a deliberate escape hatch rather than part of the gesture: a joint
that will never arrive would otherwise leave the arm permanently uncommandable. It prints
when it fires. Homing itself stays available even when a joint is stalled, so it is also
the way out of a stall.

### Safety layer

1. **Rate limit.** Steps are capped at `MAX_JOINT_STEP` (0.25 rad/tick ≈ 5 rad/s at 20 Hz,
   matching the velocity ceiling sent to the MCU). The whole step vector is **scaled**, not
   clipped per joint — clipping one joint but not the others changes their ratio and walks
   the end effector off the commanded path. Measured peak 0.185 rad/tick = 3.71 rad/s.

2. **Leash to the measured arm.** Each commanded joint is held within `MAX_JOINT_ERROR`
   (1.20 rad) of its measured angle. A leash cannot latch the way a freeze does: the
   command tracks alongside the measured angle, so the error closes the moment the joint
   frees up. While any joint is leashed the task pose is frozen, so it cannot wander off
   from the hardware and drag the other four joints toward a pose that will never happen.
   Leashed joints are named on stdout at most every 2 s.

   Measured against a dead base servo: the command holds at exactly 1.20 rad away, the task
   pose drifts 0.00, and both recover on their own once the joint is freed.

3. **Joint limit clamp** as a backstop. 0 violations over 4 000 randomized ticks.

On headroom: a 5 rad/s arm tracks with 0.000 rad of error. Driven adversarially — full
random stick and gyro input every single tick — a 2 rad/s arm peaks at 1.100 rad and a
1 rad/s arm at 1.150 rad, so the leash does engage briefly under abuse on a slow or heavily
loaded arm. That is the mechanism working as intended, not a fault, but if a loaded arm
nuisance-trips it on the bench, lower `MAX_JOINT_STEP` to match what the servos can
actually deliver rather than raising `MAX_JOINT_ERROR`.

Joint velocity sent to the MCU is a constant `JOINT_VEL = 5.0` rad/s. It is a slew
*ceiling*, not a speed setting — actual speed comes from how fast the targets move. Sending
a velocity at or below the demanded rate starves the MCU's rate limiter and causes windup.

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

Feedback returns on the same SPI transaction that carries the command. `main.py` seeds the
task pose from measured joints at startup, so the first command never jumps, and re-seeds
if the SPI service drops out. Re-seeding does **not** move the homing target.

### Tuning constants

All in [`control.py`](control.py):

| Constant | Default | Meaning |
| --- | --- | --- |
| `JOY_THRESHOLD` | 800 counts | Stick travel before an axis engages; no ramp above it |
| `REACH_RATE` | 140 mm/s | Jog speed along the base heading |
| `HEIGHT_RATE` | 140 mm/s | Jog speed, vertical |
| `ROLL_PER_COUNT` | 0.08 rad | Approach roll per encoder count |
| `JOY_SIGNS` | (1, 1) | Per-axis flip for joystick X / Y |
| `GYRO_SIGNS` | (1, 1) | Per-axis flip for controller roll / pitch |
| `MAX_JOINT_STEP` | 0.25 rad | Per-tick rate limit (≈ 5 rad/s at 20 Hz) |
| `MAX_JOINT_ERROR` | 1.20 rad | Leash distance from the measured arm |
| `HOME_TOLERANCE` | 0.05 rad | Per-joint error that counts as settled at home |
| `HOME_TIMEOUT` | 10 s | Escape hatch if a joint never arrives |

### Control verification

47 behavioural checks against a simulated arm, all passing:

```
joystick X, 3 poses x 2 directions   7.00 mm/tick vertical, off-axis < 1e-13 mm, 0 pitch drift
joystick Y, 3 poses x 2 directions   7.00 mm/tick along heading, off-heading < 1e-13 mm
joystick threshold                   799 -> 0.0, 801 -> 1.0, -900 -> -1.0  (no ramp)
encoder click                        clamp byte toggles 0 / 255 on each click
encoder turn                         10 counts -> +45.84 deg
roll extrema                         saturates at +83.480 deg, one count back -> +78.896
gyro engage / release / re-engage    0.0000 rad jump in all three
gyro roll -> yaw                     +30 deg controller -> +30.000 deg arm, pitch untouched
gyro pitch -> pitch                  -25 deg controller -> -25.000 deg arm, yaw untouched
gyro pitch is a pure rotation        point moves 5.7e-14 mm over a 75 deg sweep
both buttons                         homes, re-syncs, clamp untouched, pose constant throughout
press order                          neither order flips the clamp
homing vs a 2 rad/s arm              settles in 9 ticks (0.45 s)
workspace edge                       reach pins at 284.3 mm, height still free
stalled joint                        command holds at exactly 1.20 rad, pose drift 0.00
stall recovery                       recovers unaided, control resumes
rate limiting                        peak 0.185 rad/tick = 3.71 rad/s
fk(target) vs commanded pose         agrees to 1.1e-13
joint limit violations               0 / 4000 randomized ticks
```

Hardware behaviour is unverified — `JOY_SIGNS`, `GYRO_SIGNS` and `ROLL_PER_COUNT` in
particular are best confirmed on the bench.
