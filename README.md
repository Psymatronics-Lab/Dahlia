<p align="center">
  <img src="Extras/dahlia_logo.png" alt="Dahlia Logo" width="70">
</p>

# Dahlia
**Dahlia** is a open-source, 5+1 DOF robotic arm designed for small scale pick and place, educational, and data collection tasks.
<p align="center">
  <img src="Extras/dahlia_outline.png" alt="Dahlia Outline" width="500">
</p>

The primary inspiration for the design was the [LeRobot and Huggingface SO-101](https://huggingface.co/docs/lerobot/en/so101) open-source robotic arm, and the [HiWonder NexArm](https://www.hiwonder.com/products/nexarm). The goal of the project was to design an open-source arm with 3D-printable hardware, at a fairly accessible price, while improving upon the specifications of the SO-101 and similar arms in terms of load capacity, reach, and form factor.

This repository contains the hardware, firmware, and software for the Dahlia arm and its SEC controller, organized as follows:
```text
Dahlia/
├── Hardware/
│   ├── dahlia_hardware/          # Arm printed parts (base, arm, end effector), URDF, and sim scripts
│   └── sec_hardware/             # SEC controller printed parts, components, and wiring
├── Firmware/
│   ├── dahlia_firmware/          # Arduino App Lab app for the UNO Q
│   │   ├── sketch/               # MCU servo loop and SPI service
│   │   ├── bricks/spi_service/   # MPU-side HTTP to SPI bridge
│   │   └── python/               # Kinematics and SEC teleoperation control
│   ├── dahlia_services/
│   │   ├── HXServo/              # Arduino library for Hiwonder HX bus servos
│   │   └── controller_service/   # BLE proxy service for the SEC on the UNO Q
│   └── sec_firmware/             # ESP32-S3 firmware for the SEC controller
├── Software/
│   ├── lerobot/                  # LeRobot plugin for the arm and SEC teleoperator
│   └── perception/               # Prototype MicroTag detection pipeline
└── Extras/
    ├── microtag_block/           # Printable MicroTag fiducial blocks
    └── utilities/                # Servo ID hotfix and servo visualization tools
```

## Design, Stack, and BOM
The full CAD design for the Dahlia arm were created using OnShape and a bit of Autodesk Fusion. All editable CAD files are available on the [OnShape Document](https://cad.onshape.com/documents/8e567048457d8f314f3cf9ea/w/dc4364e539049e2561969525/e/322bc6f20473a79fe384e2b4). 

Dahlia is a 5+1 DOF arm, consisting of base rotation joint, co-planar shoulder, elbow, and wrist-pitch joints, and a wrist-roll joint, as well as a gear-driven linear clamp as the end effector. HiWonder bus servos were chosen to be the main actuators for the design due to their relatively low cost for power trade off and similar firmware interface as Feetech STS servos. A Arduino UNO Q is used to directly interface with the BusLinker servo controller to control the arm. A webcam can be mounted on-top of the arm, attached to the wrist-roll bracket, for vision and data collection. Wiring channels and lightening holes are built into the design to reduce material usage and improve wire management.

A full bill of materials needed to construct the arm is listed below. *Note that some manufacturing and connective equipment, such as a 3D printer (BambuLab X2D was used for all prints in construction), hex keys, soldering iron and USB-C cabling are not included.*
| **Component**               | **Link**                                                                                                   | **Units** |       **Price (at time)** | **Total Price** |
| --------------------------- | ---------------------------------------------------------------------------------------------------------- | --------: | --------------: | --------------: |
| Hiwonder HX-30HM            | [Hiwonder](https://www.hiwonder.com/products/hx-30hm?variant=42135141253207)                               |         4 |          $19.99 |          $79.96 |
| Hiwonder HX-65HM            | [Hiwonder](https://www.hiwonder.com/products/hx-65hm?variant=42066913132631)                               |         1 |          $49.99 |          $49.99 |
| Hiwonder BusLinker v3       | [Hiwonder](https://www.hiwonder.com/products/buslinker)                                                    |         1 |           $9.99 |           $9.99 |
| Hiwonder LFD-01M            | [Hiwonder](https://www.hiwonder.com/products/lfd-01m?variant=39328236896343)                               |         1 |           $6.99 |           $6.99 |
| Arduino UNO Q 4GB           | [Arduino](https://store-usa.arduino.cc/pages/uno-q)                                                        |         1 |          $59.99 |          $59.99 |
| M3 Heat Set Inserts         | [Amazon](https://www.amazon.com/Threaded-Inserts-Plastic-3mm-10mm-Pringting/dp/B0FD88XTV4/)                |         1 |           $9.99 |           $9.99 |
| M3 Screws                   | [Amazon](https://www.amazon.com/Fgruh-750PCS-Assortment-Washers-Assorted/dp/B0FGV5FCBN/)                   |         1 |           $9.99 |           $9.99 |
| M2 Screws                   | [Amazon](https://www.amazon.com/Fgruh-1260pcs-M2-Assortment-Stainless/dp/B0FG2CC91J/)                      |         1 |           $9.99 |           $9.99 |
| Logitech C270 Webcam        | [Amazon](https://www.amazon.com/Logitech-Desktop-Widescreen-Calling-Recording/dp/B004FHO5Y6)               |         1 |          $16.89 |          $16.89 |
| 9V Barrel Jack Power Supply | [Amazon](https://www.amazon.com/Interchangeable-Switching-Compatible-Universal-Electronics/dp/B0CPPPKP91/) |         1 |           $9.99 |           $9.99 |
|                             |                                                                                                            |           | **Total Cost:** |     **$263.77** |
> [!NOTE]
> A [HiWonder bus servo controller](https://www.hiwonder.com/products/serial-bus-servo-controller) instead of a BusLinker and Arduino UNO Q setup, to directly command the arm from a local computer, may be more cost-efficient and effective for some applications.

## Assembly and Setup
To assemble the arm, acquire the components listed in the BOM, and print the 3D-printable components in the hardware `.step` files for each section of the arm, using any 3D printer (around 0.5kg of PLA filament should be sufficient).

Use the following instructions to construct the arm.
1. Attach all included servo horns to the HiWonder servos, and set each servo to a different ID using the BusLinker board.
2. Use a soldering iron to set the 4 heat-set inserts into the holes in the base plate.
3. Use M2 screws to attach a HX-30HM servo (**base rotation**) to the base servo mount, and attach the base rotation bracket to the servo horns using M2 screws.
4. Attach the base servo mount to the base plate using M3 screws, ensuring the servo wire ports are oriented upward. Attach two wires into the servo ports, and route one wire through the wire divot in the base plate.
5. Attach the rotation plate to the base rotation servo and base rotation bracket using M2 screws.
6. Mount the HX-65HM servo (**shoulder**) into the rotation plate, using its included screws to mount it to the plate, and connect the base rotation servo wire not routed through the divot to the shoulder servo.
7. Mount a HX-30HM servo (**elbow**) into the upper arm using M2 screws, then attach the upper arm to the shoulder servo using M2 screws.
8. Mount a HX-30HM servo (**wrist pitch**) into the lower arm using M2 screws, then attach the lower arm to the elbow servo using M2 screws.
9. Mount the wrist roll bracket on the remaining HX-30HM servo (**wrist roll**) using M2 screws, then mount the wrist roll servo into the wrist using M2 screws.
10. Attach the wrist to the wrist pitch servo, using M2 screws.
11. Mount the end effector housing onto the wrist roll servo and wrist roll bracket using M2 screws.
12. Mount the LFD-01 servo (**end effector**) into the end effector housing using M2 screws, and attach the end effector gear directly to the servo using the servo's horn screw.
13. Slot the linear clamps through the end effector cap and into the housing channels symmetrically on either side of the gear, then attach the cap to the housing using M3 screws.
14. Connect the bus servos together with wiring, threaded through the wiring channels in the upper and lower arms, and connect the end effector servo to the Arduino UNO Q.
15. Connect the other bus wire on the base rotation servo to the BusLinker, and connect the TX/RX channels of the BusLinker to the Arduino UNO Q, ensuring that the bridges on the BusLinker are between Servo and TTL, and not Servo and USB.

All firmware and software needed to run and command the arm is provided in the repository. Refer to the `README.md` and related documentation in the `Software/` and `Firmware/` folders to conduct this setup, which involves creating an AppLab app, setting up services on the Arduino UNO Q, installing the app, and using an external Bluetooth controller to send commands to the UNO Q.

The repository additionally includes a full HXServo library, for control of HiWonder servos, adapted from HiWonder firmware code.
- BusLinker Documentation [https://docs.hiwonder.com/projects/BusLinker/en/latest/index.html](https://docs.hiwonder.com/projects/BusLinker/en/latest/index.html)
- Public Firmware for HX-30HM [https://drive.google.com/drive/folders/1ZyeM3iiXFUqOsPx-hs1AgcEjACiQLcII](https://drive.google.com/drive/folders/1ZyeM3iiXFUqOsPx-hs1AgcEjACiQLcII)

> [!IMPORTANT]
> Due to resourcing constraints in constructing a leader-follower arm setup, Dahlia is originally designed to work with a SEC (Semi-Extensible Controller), for which there is provided firmware and hardware designs in this repository. However, because the Bluetooth Low Energy service is implementation-agnostic and instead uses a standardized packet format, it is possible to use another Bluetooth-compatible controller to control the arm.

## Usage
Once the arm is assembled and the firmware is set up, the arm can be driven directly with the SEC, recorded and replayed through LeRobot, or commanded by any program that reaches the UNO Q over HTTP.

```text
  SEC controller ──BLE──► controller_service (:8000) ──► python/main.py ──► spi_service (:9000) ──SPI──► MCU ──► servos
                                                                                 ▲
                                                           LeRobot, servo_viz, or custom programs over HTTP
```

### Controlling the Arm with the SEC
1. Make sure the `sec-controller` service is running on the UNO Q (see [`controller_service`](Firmware/dahlia_services/controller_service/README.md)).
2. Power the arm and start the Dahlia Firmware app in App Lab. On startup, the MCU drives every joint to its zero pose, so keep the workspace clear.
3. Power the SEC, holding it still with the joystick centered while it calibrates. The arm begins responding once the controller connects.

| **Input**                   | **Effect**                                                  |
| --------------------------- | ----------------------------------------------------------- |
| Joystick X                  | Moves the end effector straight up and down                 |
| Joystick Y                  | Moves the end effector toward or away from the base         |
| Joystick button (**hold**)  | Gyroscopic following: controller roll turns the base, controller pitch tilts the end effector |
| Rotary encoder (**turn**)   | Rolls the end effector                                      |
| Encoder button (**click**)  | Toggles the clamp fully open or fully closed                |
| **Both buttons**            | Returns the arm to its startup pose                         |

The LED matrix on the SEC shows the joystick, button, and IMU readings for quick diagnostics. Motion speeds, joystick thresholds, and safety limits can be tuned in [`control.py`](Firmware/dahlia_firmware/python/control/control.py), and are explained in the [control README](Firmware/dahlia_firmware/python/README.md).

### Using LeRobot
The [`lerobot-dahlia`](Software/lerobot/README.md) plugin runs on a separate PC and provides the arm as the `dahlia` robot and the SEC as the `dahlia_sec` teleoperator, both reached at the UNO Q's SPI service URL.

```bash
pip install -e "Software/lerobot[camera]"

# check the connection to the arm without LeRobot
python -m lerobot_dahlia.dahlia_bus http://dahlia.local:9000

# record episodes while driving the arm with the SEC
lerobot-record \
  --robot.type=dahlia --robot.port=http://dahlia.local:9000 \
  --teleop.type=dahlia_sec --teleop.port=http://dahlia.local:9000 \
  --robot.discover_packages_path=lerobot_dahlia \
  --dataset.repo_id=you/dahlia_pick --dataset.num_episodes=10 \
  --dataset.single_task="Pick up the block"
```

- **Teleoperating and recording.** The SEC control loop stays on the UNO Q, and LeRobot only reads what the arm is being commanded. The robot defaults to `read_only=true` for these tasks, so LeRobot never fights the SEC.
- **Replaying and running policies.** Pass `--robot.read_only=false` so LeRobot commands the arm, and disconnect the SEC first so only one program is driving it.
- **Cameras.** The wrist camera is recorded as `observation.images.wrist`. Find its device index with `lerobot-find-cameras opencv` and set it in `dahlia_robot.py` or with `--robot.cameras`.

### Developing with Dahlia
The SPI service on the UNO Q exposes the arm over HTTP on port `9000`, so new software can be written in any language on the board or on another machine on the network.

| **Endpoint**           | **Use**                                                              |
| ---------------------- | -------------------------------------------------------------------- |
| `GET /config`          | Joint names, tick limits, and angle limits                           |
| `GET /feedback`        | Measured joint angles in radians                                     |
| `POST /targets`        | Command joint angles in radians                                      |
| `GET /motors`          | Full raw motor state, including load, voltage, and temperature       |
| `POST /motors`         | Command raw servo ticks, or `{"hold": true}` to hold the current pose |
| `GET /command`         | What the arm is currently being commanded, and in which mode         |
| `GET /health`          | SPI service status                                                   |

> [!WARNING]
> Only one program should command the arm at a time. `POST /targets` and `POST /motors` overwrite each other, so a second writer will cause the arm to jerk between commands. All commands are still clamped to the joint limits set in the MCU firmware.

A few starting points in the repository:
- **New control schemes.** [`kinematics.py`](Firmware/dahlia_firmware/python/control/kinematics.py) provides closed-form forward and inverse kinematics, and [`main.py`](Firmware/dahlia_firmware/python/main.py) is a short example of reading a controller and posting joint targets.
- **Other controllers.** Any BLE device that sends the SEC packet format in [`ble.h`](Firmware/sec_firmware/src/ble/ble.h) can replace the SEC, and the SEC's breadboard and firmware can be extended with new inputs.
- **Monitoring.** [`servo_viz`](Extras/utilities/servo_viz/README.md) plots commanded against measured joint angles live from a workstation, using only read-only endpoints.
- **Simulation.** The URDF in [`Hardware/dahlia_hardware`](Hardware/dahlia_hardware/README.md) can be converted to a simulation-ready model for simulation and RL training.
- **Perception.** [`Software/perception`](Software/perception/) contains an OpenCV detection pipeline for the [MicroTag blocks](Extras/microtag_block/README.md).

## Notes and Acknowledgements
Dahlia was designed in part for the "Invent the Future with Arduino UNO Q and App Lab" challenge, hosted by [Hackster.io](https://www.hackster.io/contests/invent-the-future-with-arduino-uno-q-and-app-lab). Thank you to Hackster.io for providing the Arduino UNO Q hardware that was used to design and construct the project.

> [!NOTE]
> As the first project for the Psymatronics Lab, there is definitely more extension to be done on Dahlia and much was learned from designing and troubleshooting the arm. I hope to continue to build out the software stack of Dahlia, and begin designing **Dahlia II**, which will be an enhanced arm, designed for low-cost research and data-collection applications.