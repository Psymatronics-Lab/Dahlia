# HXServo — Arduino driver for Hiwonder magnetic-encoder bus servos

An Arduino library for Hiwonder's **HX "HM" magnetic-encoder intelligent serial bus
servos** — `HX-10HM`, `HX-30HM`, **`HX-65HM`**, and the rest of the family. Every
servo in this family uses the same wire protocol and register map, so one driver
class controls all of them; only the mechanical/torque specs differ between models.

This is a port of Hiwonder's original ESP32-only `HX_30HM` reference. It has been
renamed to reflect the whole family, made portable to standard Arduino cores
(AVR / STM32 / etc.), and had its comments rewritten in English.

> **Protocol note.** These servos speak the `0xFF 0xFF`-header *Hiwonder
> Magnetic-Encoder Bus Servo* protocol (a Dynamixel/Feetech-style register
> protocol). This is **not** the `0x55`-header LewanSoul/LX controller-board
> protocol used by the older `LobotServoController` library — that library
> targets a different product line (the LSC controller + LX servos) and cannot
> talk to an HX servo.

---

## Contents
1. [What talks to what](#what-talks-to-what)
2. [Installation](#installation)
3. [Wiring](#wiring)
4. [Quick start](#quick-start)
5. [Bus-servo background you should know](#bus-servo-background-you-should-know)
6. [API reference](#api-reference)
7. [Reading the `ServoStatus_t` result](#reading-the-servostatus_t-result)
8. [Common recipes](#common-recipes)
9. [Troubleshooting](#troubleshooting)
10. [Changes from the original](#changes-from-the-original)

---

## What talks to what

```
  Arduino UNO Q  ──UART(TX/RX/GND)──►  BusLinker V3.0  ──single-wire bus──►  HX-65HM
   (bus master,                        (transparent TTL      (and any other
    runs HXServo)                       <-> bus adapter)       HX-HM servos, daisy-chained)
```

- Your **Arduino is the bus master.** It builds and sends the raw servo protocol.
- The **BusLinker V3.0 is a transparent adapter.** It does not interpret commands;
  it just bridges your full-duplex UART to the servos' half-duplex single-wire bus
  (and provides servo power from its own DC input). So you write full-duplex TX/RX
  code and the adapter handles bus turnaround.
- The **servos** are addressable nodes on a shared bus, each with a unique ID.

---

## Installation

`HXServo` is a plain source library (`.h`/`.cpp`), so it installs anywhere the
Arduino build can find it. The procedure differs between the classic IDE and
**Arduino App Lab** — the latter is what you use on the UNO Q.

The folder already has the layout both toolchains expect:

```
HXServo/
├── HXServo.h
├── HXServo.cpp
├── HXServo_Registers.h      control-table addresses & protocol constants
├── library.properties
├── keywords.txt
├── README.md
└── examples/
    ├── BasicControl/BasicControl.ino
    ├── SyncMotion/SyncMotion.ino
    └── Telemetry/Telemetry.ino
```

### Arduino IDE (classic)

1. Copy the whole `HXServo/` folder into your Arduino `libraries/` directory:
   - Windows: `Documents\Arduino\libraries\HXServo\`
2. Restart the Arduino IDE.
3. `#include <HXServo.h>`.

### Arduino App Lab (Arduino UNO Q)

App Lab differs from the classic IDE in one fundamental way: **your app — the
sketch, its bricks, and its Arduino libraries — is built and stored on the UNO Q
itself, not on your PC.** Your computer's `Documents/Arduino/libraries` folder is
therefore irrelevant here; the library files must be placed on the board's Linux
filesystem, where the on-device toolchain looks for them. App Lab currently has no
"Add .ZIP Library" button, so this is a manual copy.

> App Lab's custom-library workflow is still evolving and the steps below are the
> community-established method rather than an official one-click flow — check the
> forum thread linked at the end of this section for the current state.

1. **Copy the `HXServo/` folder onto the board.** The UNO Q runs Debian Linux, so
   use whichever transfer method you prefer:
   - **SFTP** (e.g. WinSCP) into the board — the community's recommended method,
   - the board's own file manager or a terminal in the App Lab desktop, or
   - `scp` / `adb push` from your PC.

   App Lab apps themselves live under `/home/arduino/arduino_apps/`.

2. **Put the library in a persistent folder you control**, for example:

   ```
   /home/arduino/Arduino/libraries/HXServo/
   ```

   (create the folders if they don't exist). Keeping your source here means a
   toolchain update can't wipe it.

3. **Make the build see it.** App Lab compiles UNO Q sketches with a
   **Zephyr-based** Arduino core, which does *not* auto-scan local library folders
   the way the classic IDE does. In a terminal on the board, find the installed
   core version and symlink the library into its `libraries` path:

   ```bash
   # 1. find the installed Zephyr core version (e.g. 0.1.0)
   ls ~/.arduino15/packages/arduino/hardware/zephyr/

   # 2. symlink the library into that core's libraries folder
   #    (replace <version> with what step 1 printed; mkdir it if missing)
   ln -s /home/arduino/Arduino/libraries/HXServo \
         ~/.arduino15/packages/arduino/hardware/zephyr/<version>/libraries/HXServo
   ```

   Use a **symlink**, not a plain copy into that path: your real files stay in
   your own folder, so a core update won't delete them. (Copying the files
   directly into the core path also works, but they are lost on the next update.)
   As an alternative, you can reference the library from your app's `sketch.yaml`.

4. **Rebuild and run the app** in App Lab, then `#include <HXServo.h>` in your
   sketch.

> **MCU core note.** Because App Lab uses the Zephyr core rather than the classic
> STM32duino core, some libraries need porting — but `HXServo` only uses the
> standard Arduino serial API (`begin(baud, SERIAL_8N1)`, `available`, `read`,
> `write`, `millis`), which the Zephyr core implements, so no code changes are
> needed. If a serial name like `Serial1` doesn't map to the header pins you
> wired, check App Lab's UNO Q pin/serial documentation and adjust `SERVO_SERIAL`
> in the sketch.
>
> Reference: [How to Manually Install a Custom Library — Arduino App Lab
> forum](https://forum.arduino.cc/t/how-to-manually-install-a-custom-library/1415413).

---

## Wiring

| Arduino (UNO Q) | BusLinker V3.0 | Notes |
|---|---|---|
| `Serial1` **TX** | **RX** | TX→RX (crossed) |
| `Serial1` **RX** | **TX** | RX→TX (crossed) |
| **GND** | **GND** | **Mandatory** — a shared ground reference |
| — | **Servo V+ / DC in** | Power the servos from the BusLinker's DC input, sized for the model (e.g. HX-65HM wants a stiff supply) |

Rules of thumb:

- **Cross TX and RX.** The master's transmit goes to the adapter's receive.
- **Common ground is not optional.** Signals are referenced to GND; without a
  shared ground the UART reads garbage or nothing.
- **Do not power a large servo from the Arduino's 5 V pin.** An HX-65HM can pull
  amps under load. Use the BusLinker's dedicated servo power input.
- **Match the baud everywhere.** Servo, BusLinker, and `HXServo(...)` must agree.
  HX-HM servos ship at **1,000,000 bps**. The STM32 on the UNO Q handles 1 Mbps
  fine; if you see corruption, drop all three to 115200 to isolate.

> Picking the right UART: `SERVO_SERIAL` in the sketch must be the hardware serial
> port physically wired to the BusLinker. Use the USB `Serial` only for the debug
> monitor, not for the servo bus.

---

## Quick start

```cpp
#include <HXServo.h>

HXServo servo(Serial1, 1000000);   // bind to the UART wired to the BusLinker

void setup() {
  Serial.begin(115200);            // USB monitor (debug only)
  servo.begin();                   // open the servo bus

  ServoStatus_t st = servo.ping(1);
  if (!st.error_bits.bit_rx) {
    Serial.print("Servo online, ID = ");
    Serial.println(st.id);
  }
  servo.enable_torque(1);          // required before it will move/hold
}

void loop() {
  servo.write_pos_ex(1, /*acc=*/50, /*speed=*/1000, /*pos=*/ 2048);
  delay(1000);
  servo.write_pos_ex(1, /*acc=*/50, /*speed=*/1000, /*pos=*/-2048);
  delay(1000);
}
```

**First-bring-up tip:** open `HXServo.h` and set `#define HX_DEBUG 1`. Every frame
sent and received is then printed (in hex) to the USB `Serial`, which makes
wiring/baud/ID problems obvious. Set it back to `0` for normal operation.

---

## Bus-servo background you should know

If you have only used hobby PWM servos before, serial bus servos work differently.
The essentials:

- **Bus, not one-wire-per-servo.** Every servo shares one data bus and is
  addressed by a unique **ID** (0–253). You daisy-chain them; the master talks to
  one ID at a time (or to all of them at once with the **broadcast ID `0xFE`**).
- **Set IDs one at a time.** A factory-fresh servo usually has ID 1. If you chain
  two servos that share an ID they will both answer and corrupt the bus. Connect
  **one new servo at a time**, give it a unique ID, then add the next.
- **Half-duplex single wire.** The servo's data line both receives commands and
  sends replies. A read is *command → turn the bus around → reply*. The BusLinker
  handles that turnaround for you; if you ever wire a servo straight to a bare
  UART you need direction-control hardware and precise timing.
- **One master only.** Two devices driving the same bus collide. During a read,
  give the servo time to answer before sending the next command (this library
  blocks for the reply, with a timeout).
- **Torque enable.** With torque **off** the servo is limp (free to backdrive) and
  ignores position targets — useful for hand-teaching poses or reading back where
  something was pushed. With torque **on** it holds/drives to the target. Call
  `enable_torque()` before expecting motion.
- **Position is a signed count, and center is 0.** Targets range roughly
  `-30719 … 30719`. `0` is the *calibrated center*. Use `write_pos_offset()` for a
  persistent zero trim, or `cali_pos()` to declare the current shaft angle as the
  new center.
- **Operating modes** (`select_mode`):
  - `POSITION_MODE` — go to and hold an absolute angle (normal servo behavior).
  - `CLOSED_LOOP_MOTOR_MODE` — continuous rotation at a regulated **speed**
    (signed; sign = direction). Set with `write_speed()`.
  - `OPEN_LOOP_MOTOR_MODE` — continuous rotation at a fixed **PWM duty**; actual
    speed varies with load. Set with `write_pwm_speed()`.
- **`acc` / `speed` limits.** In position moves, `acc` (0–254) shapes the
  acceleration ramp and `speed` (±3400) caps how fast it slews toward the target —
  smoother, quieter motion and less current spike than a bare position write.
- **Telemetry is free.** These servos report live **position, speed, load,
  current, voltage, temperature** and a moving flag. Watch temperature/current on
  a high-torque servo like the HX-65HM to catch overload before it faults.
- **`reg_write` + `action` = synchronized start.** A buffered `reg_write` (or
  `write_reg_pos_ex`) is *staged* but not executed until a single `reg_action()`
  (often broadcast) fires — so several joints begin moving on the same tick.
- **`sync_write` = one packet, many servos.** Update several servos in a single
  frame (`sync_write_pos_ex`) to save bus bandwidth and keep them in step.

---

## API reference

Construction & setup:

| Method | Description |
|---|---|
| `HXServo(HardwareSerial& s, uint32_t baud = 1000000)` | Bind to the UART wired to the bus. Does **not** open the port. |
| `void begin()` / `begin(uint32_t baud)` | Open the port (8N1). Call from `setup()`. |

Discovery & raw register access:

| Method | Description |
|---|---|
| `ping(id)` | Check a servo is online. `id = BROADCAST_ID` discovers a lone servo. |
| `general_write(id, addr, *data, len)` | Write raw bytes to a register (waits for ACK). |
| `general_read(id, addr, *data, len)` | Read raw bytes from a register. |
| `reg_write(id, addr, *data, len)` | Buffered write; applied by `reg_action()`. |
| `reg_action(id)` | Execute a buffered `reg_write` (broadcast to sync many servos). |
| `sync_write(addr, *data, len, param_len)` | Write a register block to many servos in one frame. |
| `sync_read(addr, byte_num, *ids, id_num, *data)` | Read a register block from many servos. |

State & mode:

| Method | Description |
|---|---|
| `enable_torque(id)` / `disable_torque(id)` | Power the servo / let it free-wheel. |
| `cali_pos(id)` | Calibrate the current shaft angle as center. |
| `select_mode(id, mode)` | `POSITION_MODE` / `CLOSED_LOOP_MOTOR_MODE` / `OPEN_LOOP_MOTOR_MODE`. |

Motion (write). Values are clamped to the ranges shown:

| Method | Range | Description |
|---|---|---|
| `write_pos(id, pos)` | pos ∈ [-30719, 30719] | Set target position. |
| `write_pos_ex(id, acc, speed, pos)` | acc [0,254], speed [-3400,3400], pos [-30719,30719] | Position move with accel + speed limit. |
| `write_reg_pos_ex(id, acc, speed, pos)` | same | Buffered `write_pos_ex`; fires on `reg_action`. |
| `sync_write_pos_ex(rows[][4], n)` | rows of `{id, acc, speed, pos}`, n ≤ 30 | Position move for many servos in one frame. |
| `write_speed(id, speed)` | [-3400, 3400] | Target speed (closed-loop motor mode). |
| `write_pwm_speed(id, speed)` | [-1000, 1000] | PWM duty (open-loop motor mode). |
| `write_acc(id, acc)` | [0, 254] | Acceleration. |
| `write_pos_offset(id, offset)` | [-2047, 2047] | Persistent zero trim. |
| `write_max_torque(id, torque)` | [0, 1000] | Torque ceiling. |

Telemetry (read). Each returns a `ServoStatus_t` and writes through the pointer:

| Method | Out |
|---|---|
| `read_pos(id, &pos)` | present position |
| `read_speed(id, &speed)` | present speed |
| `read_pos_speed(id, &pos, &speed)` | both, one transaction |
| `read_load(id, &load)` | present load |
| `read_current(id, &cur)` | present current |
| `read_voltage(id, &vol)` | supply voltage (0.1 V/count) |
| `read_temperature(id, &temp)` | temperature (°C) |
| `read_moving_status(id, &status)` | 0 = stopped, 1 = moving |
| `read_pos_offset(id, &offset)` | stored zero trim |
| `sync_read_cur_pos_ex(ids, n, out[][5])` | rows of `{pos, speed, load, voltage, temp}` |

---

## Reading the `ServoStatus_t` result

Every call returns a `ServoStatus_t`. On full success `error_byte == 0` and, for
replies, `id` is the responding servo. Check specific flags:

```cpp
ServoStatus_t st = servo.read_pos(1, &pos);

if (st.error_bits.bit_tx)       { /* frame could not be sent          */ }
else if (st.error_bits.bit_rx)  { /* no / invalid reply (timeout)     */ }
else if (st.error_byte) {
    if (st.error_bits.bit_overheat) { /* servo over-temperature       */ }
    if (st.error_bits.bit_overload) { /* sustained overload           */ }
    if (st.error_bits.bit_voltage)  { /* supply voltage out of range  */ }
    // also: bit_sensor, bit_current, bit_angle
}
```

| Flag | Meaning |
|---|---|
| `bit_voltage` | Supply voltage out of range |
| `bit_sensor` | Angle / magnetic sensor fault |
| `bit_overheat` | Over-temperature |
| `bit_current` | Over-current |
| `bit_angle` | Commanded angle out of range |
| `bit_overload` | Sustained overload |
| `bit_tx` | *Driver:* frame failed to transmit |
| `bit_rx` | *Driver:* no/invalid reply within the timeout |

> `bit_tx` / `bit_rx` are raised by this library (transport problems); the other
> six are reported by the servo itself.

---

## Common recipes

**Discover an unknown servo's ID** (only one servo on the bus):
```cpp
ServoStatus_t st = servo.ping(BROADCAST_ID);
if (!st.error_bits.bit_rx) Serial.println(st.id);
```

**Hand-teach a pose, then read it back:**
```cpp
servo.disable_torque(1);            // limp — move it by hand
int16_t pos;
servo.read_pos(1, &pos);            // capture where you left it
```

**Move several joints in lock-step:**
```cpp
int16_t frame[3][4] = {   // {id, acc, speed, pos}
  {1, 50, 1000,  1500},
  {2, 50, 1000, -800 },
  {3, 50, 1000,  2048},
};
servo.sync_write_pos_ex(frame, 3);  // one packet, simultaneous motion
```

**Continuous rotation (wheel) at a set speed:**
```cpp
servo.select_mode(1, CLOSED_LOOP_MOTOR_MODE);
servo.enable_torque(1);
servo.write_speed(1, 1500);         // negative reverses direction
```

**Guard a high-torque servo against overload:**
```cpp
uint8_t temp; uint16_t cur;
servo.read_temperature(1, &temp);
servo.read_current(1, &cur);
if (temp > 65 /* °C */) servo.disable_torque(1);
```

---

## Troubleshooting

| Symptom | Likely cause |
|---|---|
| `ping` always times out (`bit_rx`) | Baud mismatch; TX/RX not crossed; no common GND; wrong UART; wrong ID. |
| Servo powered but won't move | Torque not enabled — call `enable_torque()`. |
| Two servos twitch / bus garbled | Duplicate IDs on the bus. Re-ID one at a time. |
| Works at 115200, fails at 1 Mbps | Wiring/length marginal at high baud, or the servo isn't set to 1 Mbps. Match all three. |
| Reads return zeros | The read timed out (`bit_rx` set) — check the status before trusting the value. |
| Big servo resets/browns out under load | Underpowered servo supply. Feed it from the BusLinker DC input, not the Arduino 5 V. |

Turn on `#define HX_DEBUG 1` in `HXServo.h` to see the exact bytes on the wire —
the fastest way to tell a wiring problem from a protocol problem.

---

## Changes from the original

This library is behavior-compatible with Hiwonder's `HX_30HM` reference, with the
following deliberate changes:

1. **Ported off ESP32.** The original constructor called `setRxBufferSize()` and
   the 4-argument `begin(baud, config, rx, tx)` — both ESP32-only and neither
   present on the UNO Q's STM32 core (or AVR). Serial start-up now lives in a
   portable `begin()` method using the standard `begin(baud, SERIAL_8N1)`, so the
   library compiles and runs on ordinary Arduino cores.
2. **`begin()` split out of the constructor.** Constructing a driver no longer
   opens the port. Call `servo.begin()` in `setup()` — the idiomatic, safe Arduino
   pattern (globals are constructed before the core is fully initialized).
3. **Renamed for the whole family.** `HX_30HM` → `HXServo`, `SerialServo` →
   `HXServo`, `HX_30HM_Def.h` → `HXServo_Registers.h`. The protocol is identical
   across the HX-HM family, so this is purely a naming clarification — the same
   class drives the **HX-65HM** and every other `*HM` model.
4. **Fixed the goal-speed field in `write_pos_ex` / `write_reg_pos_ex`.** The
   original wrote the *position* value into the goal-speed register (and
   `write_pos_ex` never even computed the speed), so the speed limit you passed was
   ignored. Both now compute and send the goal speed correctly. *(If you were
   relying on the original quirk, this is the one behavioral difference to be aware
   of.)*
5. **English documentation.** All in-code comments are rewritten in English, plus
   this README.
6. **Minor cleanups.** Removed a dead `#else` branch that referenced a nonexistent
   variable; error paths now clear outputs to `0` instead of assigning `NULL` to
   integer pointers; fixed the `read_temperture` → `read_temperature` typo; the
   debug flag is now `HX_DEBUG` (default **off**) and prints to a configurable
   `HX_DEBUG_STREAM`.

The register map, framing, checksum, and all command semantics are unchanged.
