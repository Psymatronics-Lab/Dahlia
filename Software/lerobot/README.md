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
lerobot-teleoperate \
  --robot.type=dahlia --robot.port=http://dahlia.local:9000 \
  --teleop.type=dahlia_sec --teleop.port=http://dahlia.local:9000 \
  --robot.discover_packages_path=lerobot_dahlia
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

## Bring-up
Check the bus on its own, without LeRobot:
```bash
python -m lerobot_dahlia.dahlia_bus http://dahlia.local:9000
```
Prints the service config, per-joint load/voltage/temperature, and raw plus normalised positions.
