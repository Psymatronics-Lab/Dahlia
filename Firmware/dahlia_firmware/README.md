# Dahlia Arm Firmware
Firmware for the Dahlia Arm, running on the Arduino UNO Q MPU and MCU as an App Lab app.

```text
  host (BLE teleop on the MPU, or lerobot on another machine)
     |  HTTP :9000
  bricks/spi_service        MPU, Debian container
     |  SPI, 100 Hz, fixed 89-byte frame
  sketch/                   MCU, Zephyr -- owns the 200 Hz servo loop
     |  UART, Hiwonder 0xFF protocol       |  PWM
  HX-HM bus servos (5 joints)              clamp servo (gripper)
```

The MCU owns the servo loops, mechanical joint limits in raw servo ticks and rotation angles,
and stale command filter. Anything above provides servo commands through the SPI service.


## Command Spaces
A single SPI frame carriers two command spaces, using a mode field to indicate which space
is in effect. A single host should only produce a frame consistent with a single command space
unless dynamic switching is implemented.

| Mode | Units | Endpoint | Used by |
|---|---|---|---|
| `MODE_ANGLE` | radians | `POST /targets` | BLE teleoperation (`python/main.py`) and similar |
| `MODE_RAW` | servo ticks | `POST /motors` | anything owning its own calibration, e.g. `Software/lerobot` |
| `MODE_HOLD` | — | `POST /motors {"hold": true}` | hold the present pose |

Raw commands are still clamped to joint limits, both on software and flashed servo firmware. Hosts
that conduct their own calibration cannot drive joints beyond these stops.


## Safety Behaviors
- **Command Authority.** A command older than 500ms stops being authoritative and the arm holds its measured pose with torque rather than continuing toward a commanded target point.
- **Torque Authority.** Torque setting commands are only authoritative when the `TORQUE_APPLY`  bit is set by a controller.
- **No-Torque Tracking.** When torque is off, commands continuously update with the measured position, so re-enabling torque prevents jerking back to old targets. This enables safe hand-teaching and range-of-motion recording.


## HTTP API
- `GET /config`, returns joint names, tick limits, angle limits, encoder resolution.
- `GET /motors`, returns full motor state, including raw positions and velocities, load, voltage, temperature, per-joint torque and comm-error flags, gripper echo, MCU loop counter.
- `POST /motors`, input partial update of servo targets, with every field as optional and unnamed fields keeping their previous value; returns with the same body as `GET /motors`.

```json
{"positions": [0,0,2250,0,1750], "velocities": [3400,...],
 "acceleration": [254,...], "torque": [true,...], "gripper": 128}
```

- `GET /feedback`, `POST /targets`, radian based API, similar to the `/motors` API.
- `GET /command`, returns what is currently being *commanded* in both spaces, plus the active mode. This is the readback a recorder needs: the measured pose lags the command and settles onto it, so recording the measurement as the action would teach a policy to output the state it is already in.
- `GET /health`, returns whether the SPI thread is alive, how old the last frame is, and the consecutive error count.


## Protocol Format
`CommandPacket` is 70 bytes, `FeedbackPacket` 89, and every transfer clocks 89 either way. The sketch `static_assert`s both against a per-joint formula, so changing a field without mirroring it in `bricks/spi_service/spi_service.py` is a build error.


## Interfacing with LeRobot
`brick_compose.yaml` publishes port 9000 on the board, so an off-board host can reach the raw API. LeRobot runs on a PC through the `lerobot-dahlia` plugin in `Software/lerobot`, whose `--robot.port` is this service's URL.

The SEC control loop stays on the board for latency, and LeRobot reads its output through `GET /command` rather than re-deriving it from the raw controller samples. `Software/lerobot/README.md` explains why re-deriving it would silently corrupt recorded episodes.

Check the arm without LeRobot in the way:
```bash
python -m lerobot_dahlia.dahlia_bus http://dahlia.local:9000
```