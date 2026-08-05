import struct
from dataclasses import dataclass

PACKET_FORMAT = "<IIihhhhhBB"
PACKET_SIZE = struct.calcsize(PACKET_FORMAT)

@dataclass
class SECState:
    seq: int
    timestamp: int

    enc_pos: int

    joy_x: int
    joy_y: int

    imu_roll: int
    imu_yaw: int
    imu_pitch: int

    joy_pressed: bool
    enc_pressed: bool


def decode_packet(data: bytes) -> SECState:
    if len(data) != PACKET_SIZE:
        raise ValueError(f"Invalid packet size {len(data)}, expected {PACKET_SIZE}")

    values = struct.unpack(PACKET_FORMAT, data)

    return SECState(
        seq=values[0],
        timestamp=values[1],

        enc_pos=values[2],

        joy_x=values[3],
        joy_y=values[4],

        imu_roll=values[5],
        imu_yaw=values[6],
        imu_pitch=values[7],

        joy_pressed=bool(values[8]),
        enc_pressed=bool(values[9]),
    )