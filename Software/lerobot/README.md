# lerobot-dahlia
A LeRobot plugin providing the Dahlia arm as a robot and the SEC controller as a teleoperator. Runs on a PC; the arm is reached over HTTP.

```text
  SEC controller
     |  BLE
  controller_service           (UNO Q host, systemd)
     |  HTTP :8000
  python/main.py               (UNO Q, joystick -> IK -> joint targets)
     |  POST /targets                        |  GET /command
  bricks/spi_service           (UNO Q, :9000)  ---------------+
     |  SPI, 100 Hz                                           |
  sketch/                      (MCU, 200 Hz servo loop)       |  HTTP
     |  UART                        |  PWM                    |
  HX-HM bus servos (5 joints)   clamp (gripper)               |
                                                     LeRobot on this PC
                                                     robot: dahlia
                                                     teleop: dahlia_sec
```

The SEC control loop stays on the board, where it needs to be for latency. LeRobot reads its output rather than re-deriving it.

## Install
```bash
pip install -e Software/lerobot          # add [camera] for the OpenCV backend
```

## Registering with LeRobot's CLI
`--robot.type=dahlia` resolves through a registry that the `@register_subclass` decorators fill at import time, so something has to import `lerobot_dahlia` before the CLI parses arguments. LeRobot's plugin discovery flag does that:

```bash
uv run lerobot-teleoperate --robot.type=dahlia --robot.port=http://dahlia.local:9000 --teleop.type=dahlia_sec --teleop.port=http://dahlia.local:9000 --robot.discover_packages_path=lerobot_dahlia
```

Confirm your version has it with `lerobot-record --help | grep discover`. If it does not, importing the package before the CLI runs has the same effect — the registration is a plain import side effect, nothing LeRobot-version specific.

## Recording
```bash
lerobot-record \
  --robot.type=dahlia --robot.port=http://dahlia.local:9000 \
  --teleop.type=dahlia_sec --teleop.port=http://dahlia.local:9000 \
  --robot.discover_packages_path=lerobot_dahlia \
  --dataset.repo_id=you/dahlia_pick --dataset.num_episodes=10 \
  --dataset.single_task="Pick up the block"
```

## Modules
| Module | Contents |
|---|---|
| `dahlia_bus.py` | `HXServoMotorsBus`, a `MotorsBus` speaking raw servo ticks over HTTP |
| `dahlia_robot.py` | `DahliaRobot` / `DahliaRobotConfig`, six joints plus cameras |
| `dahlia_teleop.py` | `DahliaSECTeleop` / `DahliaSECTeleopConfig` |
| `dahlia_calibration.py` | the fixed calibration, and radians to normalised conversion |

## Calibration
Dahlia needs no interactive calibration pass. Its travel is fixed in firmware: `servo_interface.h` holds a `joint_configs` table the MCU clamps every command against, so those stops **are** the calibration. `dahlia_calibration()` derives `MotorCalibration` straight from that table, and `DahliaRobot` applies it on construction, so the arm is usable the moment it connects.

`drive_mode` is set to 1 on every joint whose ticks count *down* as the angle goes up, which is all of them except the base. That is what makes **-100 mean minimum angle and +100 mean maximum angle on every joint**, instead of the normalised direction flipping around depending on how each servo happens to be mounted. `dahlia_teleop` depends on that: it means one expression converts an angle to a normalised action for every joint.

## Why the teleoperator only reads
The SEC is not a leader arm, but LeRobot never asks whether it is. A `Teleoperator` returns an action each tick and has no opinion on where it came from — `SO101Leader.get_action()` reads joint positions off a serial bus, this reads them off `GET /command`.

Re-deriving the action here instead, from the raw BLE samples, would look simpler and be wrong. The mapping in `python/control/control.py` is stateful and rate dependent: the pose integrates a thresholded joystick, the gyro reference latches on the sample where the button is first seen held, and the clamp toggles on a *release edge*. A second copy polling at LeRobot's rate drifts from the copy actually driving the arm. The clamp is the sharp edge — a click shorter than one poll interval is invisible to the slower sampler, and because it is a toggle, one missed click inverts the gripper for the rest of the episode. The arm grasps, the dataset says it did not, and every frame after that is mislabelled.

Reading the *command* rather than the measured pose matters too: the measurement lags the command and settles onto it, so recording it as the action would teach a policy to output the state it is already in.

When nothing is driving the arm in radians, `get_action()` holds its previous action instead of jumping.

## Who owns the arm

The SPI service keeps **one** command struct with **one** mode field, and its two POST routes overwrite each other's: `/targets` sets `MODE_ANGLE` and commands radians, `/motors` sets `MODE_RAW` and commands ticks. Whichever posted last wins, and the MCU re-reads the mode every servo cycle.

LeRobot's loops call `robot.send_action()` every tick regardless of what else is driving. With `python/main.py` running that makes two writers, the mode flips at the rate the two of them post, and the arm jerks around the commanded pose as the MCU alternates between the two command spaces. The teleoperator's own `MODE_ANGLE` guard then trips on the robot's write from the previous tick and starts holding stale actions, which makes it worse.

Ownership cannot be detected, so it is declared. `python/main.py` is the App Lab application and runs continuously; there is also no liveness signal to read even if it did not, because the service resends the command struct at 100 Hz whether or not anything is updating it. The MCU therefore stays `fresh` as long as the brick runs, and the mode keeps whatever was last posted — a single POST to `/targets` leaves the mode reading `MODE_ANGLE` indefinitely, even if the sender has stopped.

So `DahliaRobotConfig.read_only` decides it, defaulting to observing:

| Task | `read_only` | Behaviour |
|---|---|---|
| `lerobot-teleoperate`, `lerobot-record` | `true` (default) | Returns the action for the dataset, sends nothing. The SEC drives. |
| `lerobot-replay`, policy evaluation | `--robot.read_only=false` | Commands the arm. |

The default is the safe direction: forgetting the flag on a replay leaves the arm still and logs why, while forgetting it the other way corrupts a recording and fights a live teleoperator.

Driving also needs the SEC path to actually stand down, which does not require stopping the App Lab application — `main.py` only posts `/targets` while the controller is connected, so disconnecting the SEC is enough. If you drive while the command space is still set to radians, `send_action()` warns once rather than fighting quietly.

## Cameras

The views recorded into every episode:

| Key | Mounting | Observation feature | |
|---|---|---|---|
| `wrist` | on the end effector | `observation.images.wrist` | active, device index 1 |
| `overhead` | fixed above the workspace | `observation.images.overhead` | commented out in `dahlia_cameras()` |

Only the wrist view is live at the moment. Restoring the overhead one is uncommenting its parameter and its dict entry in `dahlia_cameras()`, but note that adding a camera changes `observation_features`, so episodes recorded with one view and with two are not the same dataset.

`wrist` is LeRobot's name for an end effector camera, so datasets and policy configs that expect that key line up without remapping. It sits alongside the `wrist_pitch` and `wrist_roll` *joints*, which are a different thing — joints always carry a `.pos` suffix, cameras never do.

Device indices are per machine, because OpenCV numbers cameras by enumeration order. `lerobot-find-cameras opencv` lists what is attached; set `WRIST_CAMERA_INDEX` and `OVERHEAD_CAMERA_INDEX` in `dahlia_robot.py`, or override the pair on the command line:

```bash
--robot.cameras='{ wrist: {type: opencv, index_or_path: 1, width: 1280, height: 720, fps: 15}, overhead: {type: opencv, index_or_path: 2, width: 1280, height: 720, fps: 15}}'
```

Both are MJPG at 1280x720/15 through the DirectShow backend. MJPG is not cosmetic: two uncompressed streams of this size do not fit through one USB controller, and the second camera opens and then starves. If a camera refuses the format, set `CAMERA_FOURCC = None` and drop the resolution instead, and put the two cameras on separate USB controllers rather than a shared hub.

There are no intrinsics or extrinsics here. LeRobot treats a camera as an image source — `OpenCVCameraConfig` carries only index, size, fps, colour and rotation — and its policies learn from pixels, so nothing in the record or replay path needs a calibrated camera. Mapping an overhead detection into arm coordinates, or hand-eye calibrating the wrist camera against the URDF, would need both, but that belongs with `Software/perception/` rather than here.

## Bring-up
Check the bus on its own, without LeRobot:
```bash
python -m lerobot_dahlia.dahlia_bus http://dahlia.local:9000
```
Prints the service config, per-joint load/voltage/temperature, and raw plus normalised positions.
