# SEC BLE Service
Arduino App Lab currently sandboxes internal apps from accessing Linux Bluetooth functions. Thus a proxy webserver is setup to accept the BLE packets from the SEC controller and communicate them to the App Lab application over TCP.

## Installation
In the user root folder of the Arduino UNO Q (i.e., `~`) create a directory named `ExternalServices` (the name isn't important, but must be consistent as chosen in the system service file), and copy in the `controller_service` directory containing the `ble_service.py`, `protocol.py` and `server.py` files. Then install dependencies using `uv` already installed on the UNO Q:
```bash
# in the ~\ExternalServices\controller_service\ directory
uv init

uv add bleak fastapi uvicorn
```

Then, create the `systemd` system service by create the service file:
```bash
sudo nano /etc/systemd/system/sec-controller.service

# copy in the contents of the `sec-controller.service` file

sudo systemctl daemon-reload
sudo systemctl enable sec-controller.service
sudo systemctl start sec-controller.service
```

Use the following to confirm that the service is running correctly:
```bash
systemctl status sec-controller.service

journalctl -u sec-controller.service -f

ss -tlnp | grep 8000
```

Confirm that the UNO Q can find the controller while it is broadcasting using:
```bash
bluetoothctl scan on

# check that data is outputting correctly
curl http://172.17.0.1:8000/controller
```