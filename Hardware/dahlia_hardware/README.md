# Dahlia Arm Hardware
All the `.step` files needed to print a Dahlia arm are included in this section of the repository, alongside `.urdf` files and meshes for simulation and RL training.

The full URDF contains a few unnecessary joints and sections that may hamper simulation performance, and thus a simulation-ready version with merged meshes and proper joints can be generated using the following code. Only `numpy` needs to be installed for this.
```bash
python urdf_merge_subassemblies.py path/to/robot_full/urdf/robot_full.urdf -o path/to/robot_sim
```

## Printed Parts
| **Folder**      | **Files**                                                                                   |
| --------------- | ------------------------------------------------------------------------------------------- |
| `base/`         | `base_body`, `base_servo_mount`, `base_rotation_plate`, `base_rotation_bracket`             |
| `arm/`          | `arm_upper_arm`, `arm_lower_arm`, `arm_wrist`, `arm_wrist_rotation`                         |
| `end_effector/` | `ef_body`, `ef_gear`, `ef_clamp` (print two, one per side), `ef_front_cap`                  |

## Components
| **Component**         | **Units** | **Purpose**                                                         |
| --------------------- | --------: | ------------------------------------------------------------------- |
| Hiwonder HX-65HM      |         1 | Shoulder joint                                                      |
| Hiwonder HX-30HM      |         4 | Base rotation, elbow, wrist pitch, and wrist roll joints            |
| Hiwonder LFD-01M      |         1 | PWM servo driving the end effector gear and clamp                   |
| Hiwonder BusLinker v3 |         1 | Bridges the UNO Q's UART to the servos' single-wire serial bus      |
| Arduino UNO Q         |         1 | Runs the servo loop (MCU) and the control and BLE services (MPU)    |
| 9V power supply       |         1 | Servo power, through the BusLinker's DC input                       |
| Logitech C270 webcam  |         1 | Optional, mounted to the wrist-roll bracket for vision              |

See the BOM in the [main README](../../README.md) for purchase links and fasteners.

## Wiring
```text
  9V supply ──► BusLinker v3 DC input
                     │
  UNO Q Serial1 ◄──► BusLinker v3 ──bus──► HX servos (IDs 1-5, daisy-chained)
  UNO Q D3 (PWM) ─────────────────────────► LFD-01M clamp servo
```

### Bus Servos
| **UNO Q**     | **BusLinker v3** | **Notes**                       |
| ------------- | ---------------- | ------------------------------- |
| `Serial1` TX  | RX               | TX and RX are crossed           |
| `Serial1` RX  | TX               | TX and RX are crossed           |
| GND           | GND              | A shared ground is required     |

The servos are daisy-chained from the BusLinker's servo bus and powered from the BusLinker's DC input, never from the UNO Q's 5V pin. The bus runs at 1,000,000 baud, and each servo must be assigned the ID the firmware expects, from [`servo_interface.h`](../../Firmware/dahlia_firmware/sketch/src/low_level/servo_interface.h):

| **Joint**    | **Servo** | **ID** |
| ------------ | --------- | -----: |
| Base rotation| HX-30HM   |      1 |
| Shoulder     | HX-65HM   |      2 |
| Elbow        | HX-30HM   |      3 |
| Wrist pitch  | HX-30HM   |      4 |
| Wrist roll   | HX-30HM   |      5 |

Set IDs with one servo connected at a time, since servos sharing an ID on the same bus will corrupt it. [`Extras/utilities/id_hotfix.ino`](../../Extras/utilities/id_hotfix.ino) can be used to change a servo's ID.

### Clamp Servo
The LFD-01M signal wire connects to UNO Q pin D3 (`SIGPIN` in `servo_interface.h`), which is driven with a 50 Hz PWM signal. Its ground must be shared with the UNO Q.

### UNO Q and Webcam
The UNO Q is powered over USB-C. The webcam connects over USB to the device used for vision and data collection (not necessarily the UNO Q). Route servo cables through the wiring channels built into the printed parts.

For further wiring and bus troubleshooting details, see the [HXServo README](../../Firmware/dahlia_services/HXServo/README.md).
