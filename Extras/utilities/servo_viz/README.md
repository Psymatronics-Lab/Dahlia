# Arm monitor

A workstation-side plot of what the arm is told to do against what it actually does,
built to find where the approach angle is being lost when the end effector changes
height.

It needs no firmware change and cannot affect the arm. Every request is a `GET`
against one of three endpoints that already exist:

| service | port | endpoint | what it gives |
| --- | --- | --- | --- |
| `ble_service.py` | 8000 | `/controller` | the live SEC sample |
| `spi_service.py` | 9000 | `/command` | the joint angles `main.py` is asking for |
| `spi_service.py` | 9000 | `/feedback` | the joint angles the servos report |

Both services already bind `0.0.0.0`, and the SPI service is published on the board by
`brick_compose.yaml`, so both are reachable from the LAN as they stand. The set is
declared in `READ_ONLY` and `Poller.get` refuses anything outside it, so the `POST`
routes that command the arm are unreachable from this tool by construction.

## Running

```
python Extras/utilities/servo_viz/servo_viz.py --board 192.168.1.50
```

`--board` takes the board's hostname or IP and defaults to `dahlia.local`. Add
`--log climb.csv` to record every sample alongside the plot. Press `r` in the window
to reseed the independent controller.

## The three traces

Each joint carries three lines, and the pairs answer different questions.

**commanded against measured** is servo tracking. Gravity droop and per joint lag live
here. This is the pair to watch while driving the arm up and down, since the approach
pitch is the sum of shoulder, elbow, and wrist pitch, so all three joints' errors land
on the end effector angle undiluted.

**independent against commanded** is the control path. The independent trace is this
process running the real `control.ArmController` against the same SEC sample the arm
just received, so if the two diverge, the arm is not being sent what the control code
says it should be. If they sit on top of each other, teleoperation and IK are fine and
the fault is downstream.

The top panel plots the approach pitch itself, which is the quantity the arm is
visibly failing to hold.

## Reading it honestly

The independent controller is seeded from the commanded vector, not the measured one,
so it starts level with the commanded trace and any gap between them is genuinely the
control path rather than inherited droop. It is still an open loop mirror: it began at
a different time than the arm's own controller and integrates encoder counts and gyro
references from that moment, so slow divergence over minutes is expected rather than a
finding. Press `r` to reseed before a measurement that matters.

`/command` only carries radians in `MODE_ANGLE`. Under the LeRobot path the arm runs in
`MODE_RAW` and the field is stale, so the commanded trace is blanked and the banner
shows the mode.

Polling is three small JSON reads per tick, 60 requests per second at the default rate.
That is modest but not free, and it defaults to 20 Hz because `control.PERIOD` assumes
it: the independent controller advances its reach and height by a fixed amount per
poll, so at another rate it still solves correctly but travels at the wrong speed in
wall clock time. `--hz` warns when you change it.
