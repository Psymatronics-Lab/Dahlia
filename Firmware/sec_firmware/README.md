# Semi-Extensible Controller
The semi-extensible controller is a custom, programmable controller designed to control the Dahlia arm. It's current configuration includes a 2-axis joystick, rotary encoder, and 6-axis IMU wired to an ESP32-S3 board, with a LED matrix for minimal diagnostic and telemetry display, and an integrated breadboard for extensibility.

## Setup
After assembling the controller, flash the ESP32 using Arduino IDE, with the `NimBLE-Arduino` library installed, to install the `sec_firmware`. Then, when the controller is connected to power, it will automatically begin broadcasting telemetry packets on Bluetooth Low Energy.
- Use LightBlue to check if BLE is actually functioning correctly, and that it is possible to subscribe to notifications.
- Ensure that receiving device (i.e., the Dahlia Arm) has a Bluetooth client server that searches for a UUID matching the SEC broadcast UUID.

