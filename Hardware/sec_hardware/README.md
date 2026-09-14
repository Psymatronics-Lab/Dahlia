# Semi-Extensible Controller Hardware
The 3D-printable parts for the SEC (Semi-Extensible Controller) are in this section of the repository. The SEC is a handheld Bluetooth Low Energy controller used to teleoperate the Dahlia arm. Firmware and setup instructions are in [`Firmware/sec_firmware`](../../Firmware/sec_firmware/).

## Printed Parts
| **File**               | **Description**                        |
| ---------------------- | -------------------------------------- |
| `sec_body.step`        | Main controller body                   |
| `sec_encoder_knob.step`| Knob for the rotary encoder shaft      |

## Components
| **Component**              | **Model**        | **Purpose**                                         |
| -------------------------- | ---------------- | --------------------------------------------------- |
| ESP32-S3 development board | ESP32-S3         | Reads the inputs and broadcasts them over BLE       |
| 2-axis joystick            | KY-023           | Reach and height jogging, gyro-follow button        |
| Rotary encoder             | KY-040           | Wrist roll, clamp open/close button                 |
| 6-axis IMU                 | MPU6050          | Controller roll and pitch for gyroscopic following  |
| 8x8 LED matrix             | MAX7219          | Minimal diagnostic and telemetry display            |
| Breadboard                 | —                | Integrated into the controller for extensibility    |

## Wiring
Pin assignments are defined in [`Firmware/sec_firmware/config.h`](../../Firmware/sec_firmware/config.h). All numbers below are ESP32-S3 GPIO numbers, and every component must share a common ground with the ESP32-S3.

### Joystick (KY-023)
| **KY-023 Pin** | **ESP32-S3** |
| -------------- | ------------ |
| VRx            | GPIO 4       |
| VRy            | GPIO 5       |
| SW             | GPIO 6       |
| +5V            | 3.3V         |
| GND            | GND          |

The X and Y outputs are read by the ESP32's ADC, so power the joystick from 3.3V to keep the analog outputs within the ADC's input range. The switch uses the ESP32's internal pull-up, so no external resistor is needed.

### Rotary Encoder (KY-040)
| **KY-040 Pin** | **ESP32-S3** |
| -------------- | ------------ |
| CLK (A)        | GPIO 9       |
| DT (B)         | GPIO 10      |
| SW             | GPIO 11      |
| +              | 3.3V         |
| GND            | GND          |

All three encoder inputs use the ESP32's internal pull-ups.

### IMU (MPU6050, I2C)
| **MPU6050 Pin** | **ESP32-S3** |
| --------------- | ------------ |
| SDA             | GPIO 13      |
| SCL             | GPIO 14      |
| VCC             | 3.3V         |
| GND             | GND          |

The firmware expects the IMU at I2C address `0x68`, so leave `AD0` unconnected or tied low.

### LED Matrix (MAX7219, SPI)
| **MAX7219 Pin** | **ESP32-S3** |
| --------------- | ------------ |
| DIN             | GPIO 40      |
| CS              | GPIO 41      |
| CLK             | GPIO 42      |
| VCC             | 5V           |
| GND             | GND          |

### Power
The ESP32-S3 is powered through its USB port, and supplies the 3.3V and 5V rails to the components. Once powered, the controller calibrates the joystick center and IMU gyroscope, so keep the joystick centered and the controller still for a moment after plugging it in.

> [!TIP]
> If a joystick axis, the encoder direction, or an IMU axis responds in the opposite direction from what is expected after assembly, flip it in firmware using the `FLIP_*` constants in `joystick.h`, `rotencoder.h`, and `imu.h` rather than rewiring.
