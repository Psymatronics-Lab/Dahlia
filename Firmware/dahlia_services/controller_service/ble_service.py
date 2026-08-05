import asyncio
import time

from bleak import BleakScanner, BleakClient
from fastapi import FastAPI
import uvicorn

from protocol import decode_packet


DEVICE_NAME = "SEC Controller"

CHAR_UUID = ("87654321-4321-6789-4321-cba987654321")

app = FastAPI()

latest_state = None
last_receive_time = None
last_seq = None

def notification_callback(sender, data):
    global latest_state
    global last_receive_time
    global last_seq

    state = decode_packet(data)

    # Detect dropped packets
    if last_seq is not None:
        dropped = state.seq - last_seq - 1

        if dropped > 0:
            print(f"Dropped {dropped} packets")

    last_seq = state.seq

    latest_state = state
    last_receive_time = time.monotonic()

    print(
        f"SEQ={state.seq} "
        f"JOY=({state.joy_x},{state.joy_y}) "
        f"ENC={state.enc_pos}"
    )


@app.get("/controller")
def controller():
    if latest_state is None:
        return {"connected": False}

    age_ms = (time.monotonic() - last_receive_time) * 1000

    return {
        "connected": age_ms < 500,
        "age_ms": age_ms,

        "state": {
            "seq": latest_state.seq,
            "timestamp": latest_state.timestamp,

            "enc_pos": latest_state.enc_pos,

            "joy_x": latest_state.joy_x,
            "joy_y": latest_state.joy_y,

            "imu_roll": latest_state.imu_roll,
            "imu_yaw": latest_state.imu_yaw,
            "imu_pitch": latest_state.imu_pitch,

            "joy_pressed": latest_state.joy_pressed,
            "enc_pressed": latest_state.enc_pressed,
        }
    }


async def connect_controller():
    """
    Attempt to find and connect to the ESP32 controller.
    Returns only after the BLE connection is established.
    """

    print("Scanning BLE...")

    devices = await BleakScanner.discover(timeout=5)
    device = None

    for d in devices:
        print(d.name, d.address)

        if d.name == DEVICE_NAME:
            device = d
            break

    if device is None:
        raise RuntimeError("SEC Controller not found")

    print("Connecting:", device.address)

    async with BleakClient(device) as client:
        print("BLE connected")
        await client.start_notify(CHAR_UUID, notification_callback)

        while client.is_connected:
            await asyncio.sleep(1)


async def ble_loop():
    while True:
        try:
            await connect_controller()
        except Exception as e:
            print("BLE error:", e)
            print("Retrying in 5 seconds...")
            await asyncio.sleep(5)


async def main():
    ble_task = asyncio.create_task(ble_loop())

    config = uvicorn.Config(
        app,
        host="0.0.0.0",
        port=8000,
        log_level="info"
    )

    server = uvicorn.Server(config)

    await asyncio.gather(ble_task, server.serve())

if __name__ == "__main__":
    asyncio.run(main())