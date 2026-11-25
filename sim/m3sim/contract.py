"""Python mirror of lib/contract/m3_contract.{h,c}.

Frame format exactly as Section 2.3; payload layouts PROVISIONAL until M5's
frozen contract module arrives (then import that instead of this file).
Both implementations are checked against contract/test_vectors.json.
"""
from __future__ import annotations

import struct
from dataclasses import dataclass

START = b"\xA5\x5A"
HEADER_LEN = 10

HEIGHT = 0x01
FAN_CMD = 0x10
FAN_CONFIG = 0x11
FAN_STATE = 0x12
GAME_IN = 0x20
GAME_OUT = 0x21

HFLAG_RAW_VALID = 0x01
HFLAG_FILT_VALID = 0x02
HFLAG_BRIDGED = 0x04

FCMD_ENABLE = 0x01
FCMD_REARM = 0x02

M1_OFF, M1_ARMED, M1_RUN, M1_FAULT, M1_ESTOP = range(5)

# little-endian payload layouts (must match the C pack functions)
FMT = {
    HEIGHT: "<hhhBIQH",     # raw_mm filt_mm vel_mm_s flags frame_no capture_us fps_x10
    FAN_CMD: "<HBB",        # bottom_rpm top_level flags
    FAN_CONFIG: "<HHHH",    # kp_x1000 ki_x1000 rate_rpm_s power_limit_pm
    FAN_STATE: "<HHHBBI",   # bottom_rpm bottom_power_pm top_power_pm state fault_code echo_seq
    GAME_IN: "<hhhBB",      # ball_mm range_lo_mm range_hi_mm game_enabled obstacle_speed
    GAME_OUT: "<BBBH",      # target_level hit phase score
}


def crc16(data: bytes) -> int:
    """CRC16/CCITT-FALSE: poly 0x1021, init 0xFFFF, no reflection, xorout 0."""
    crc = 0xFFFF
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def encode_frame(msg_type: int, session: int, seq: int, payload: bytes) -> bytes:
    if len(payload) > 255:
        raise ValueError("payload too long")
    body = struct.pack("<BBII", msg_type, len(payload), session, seq) + payload
    return START + body + struct.pack("<H", crc16(body))


def pack(msg_type: int, *fields) -> bytes:
    return struct.pack(FMT[msg_type], *fields)


def unpack(msg_type: int, payload: bytes) -> tuple:
    return struct.unpack(FMT[msg_type], payload)


@dataclass
class Frame:
    type: int
    session: int
    seq: int
    payload: bytes


class Decoder:
    """Streaming decoder; resynchronises on the start bytes; counts CRC errors."""

    def __init__(self) -> None:
        self.buf = bytearray()
        self.crc_errors = 0
        self.frames_ok = 0

    def feed(self, data: bytes) -> list[Frame]:
        self.buf += data
        out: list[Frame] = []
        while True:
            i = self.buf.find(START)
            if i < 0:
                # keep a trailing 0xA5 that may start the next frame
                self.buf = self.buf[-1:] if self.buf[-1:] == b"\xA5" else bytearray()
                return out
            del self.buf[:i]
            if len(self.buf) < 2 + HEADER_LEN:
                return out
            length = self.buf[3]
            total = 2 + HEADER_LEN + length + 2
            if len(self.buf) < total:
                return out
            body = bytes(self.buf[2 : 2 + HEADER_LEN + length])
            (crc_rx,) = struct.unpack("<H", self.buf[total - 2 : total])
            if crc16(body) != crc_rx:
                self.crc_errors += 1
                del self.buf[:2]  # skip this start, search again
                continue
            t, _, session, seq = struct.unpack("<BBII", body[:HEADER_LEN])
            out.append(Frame(t, session, seq, body[HEADER_LEN:]))
            self.frames_ok += 1
            del self.buf[:total]


class SeqTracker:
    """Accept only newer sequence numbers within a session; a new session restarts."""

    def __init__(self) -> None:
        self.session: int | None = None
        self.last = -1

    def accept(self, session: int, seq: int) -> str:
        if session != self.session:
            self.session, self.last = session, seq
            return "new_session"
        if seq <= self.last:
            return "old"
        self.last = seq
        return "new"
