"""
MTF-01 optical flow + range sensor reader for Raspberry Pi.

Reads Micolink protocol frames over UART and converts optical flow to
ground velocity using the measured height.
"""

from __future__ import annotations

import argparse
import struct
import threading
import time
from dataclasses import dataclass
from typing import Optional

import serial
import numpy as np

MICOLINK_MSG_HEAD = 0xEF
MICOLINK_MAX_PAYLOAD_LEN = 64
MICOLINK_MSG_ID_RANGE_SENSOR = 0x51
MTF01_BAUD = 115200

BAUD_CANDIDATES = (
    115200, 230400, 57600, 460800, 921600, 38400, 19200, 9600,
)

RANGE_SENSOR_FMT = "<IIBBBBhhBBH"
RANGE_SENSOR_SIZE = struct.calcsize(RANGE_SENSOR_FMT)


@dataclass
class SensorReading:
    flow_vx: float = 0.0          # cm/s @ 1 m height
    flow_vy: float = 0.0
    distance_m: float = 0.0
    flow_quality: int = 0
    flow_status: int = 0
    dis_status: int = 0
    time_ms: int = 0
    timestamp: float = 0.0

    @property
    def velocity_m_s(self) -> tuple[float, float]:
        """Ground velocity in m/s from optical flow and measured height."""
        if not self.is_valid:
            return 0.0, 0.0
        scale = self.distance_m / 100.0
        return self.flow_vx * scale, self.flow_vy * scale

    @property
    def is_valid(self) -> bool:
        return (
            self.flow_status > 0
            and self.flow_quality > 0
            and self.distance_m >= 0.01
        )


class MicolinkParser:
    """Byte-stream parser for Micolink frames."""

    def __init__(self) -> None:
        self._last_msg_id = 0
        self._last_payload = b""
        self.reset()

    def reset(self) -> None:
        self._status = 0
        self._payload_cnt = 0
        self.head = 0
        self.dev_id = 0
        self.sys_id = 0
        self.msg_id = 0
        self.seq = 0
        self.length = 0
        self.payload = bytearray(MICOLINK_MAX_PAYLOAD_LEN)
        self.checksum = 0

    def _check_sum(self) -> bool:
        """Sum of head..payload bytes, matching micolink_check_sum()."""
        length = self.length + 6
        frame = bytes([self.head, self.dev_id, self.sys_id, self.msg_id, self.seq, self.length])
        frame += bytes(self.payload[: self.length])
        return (sum(frame[:length]) & 0xFF) == self.checksum

    def feed(self, data: int) -> bool:
        """Parse one byte; return True when a complete valid frame is received."""
        if self._status == 0:
            # print(f"Status: {self._status}, data: {data}, head: {MICOLINK_MSG_HEAD}")
            if data == MICOLINK_MSG_HEAD:
                self.head = data
                self._status = 1
            return False

        if self._status == 1:
            self.dev_id = data
            self._status = 2
        elif self._status == 2:
            self.sys_id = data
            self._status = 3
        elif self._status == 3:
            self.msg_id = data
            self._status = 4
        elif self._status == 4:
            self.seq = data
            self._status = 5
        elif self._status == 5:
            self.length = data
            if self.length == 0:
                self._status = 7
            elif self.length > MICOLINK_MAX_PAYLOAD_LEN:
                self.reset()
            else:
                self._status = 6
        elif self._status == 6:
            self.payload[self._payload_cnt] = data
            self._payload_cnt += 1
            if self._payload_cnt == self.length:
                self._payload_cnt = 0
                self._status = 7
        elif self._status == 7:
            self.checksum = data
            valid = self._check_sum()
            if valid:
                self._last_msg_id = self.msg_id
                self._last_payload = bytes(self.payload[: self.length])
            self.reset()
            return valid
        # print(f"Status: {self._status}, data: {data}")
        return False

    def parse_range_sensor(self) -> Optional[SensorReading]:
        if self._last_msg_id != MICOLINK_MSG_ID_RANGE_SENSOR:
            return None
        if len(self._last_payload) < RANGE_SENSOR_SIZE:
            return None

        payload = self._last_payload[:RANGE_SENSOR_SIZE]
        (
            time_ms,
            distance_mm,
            _strength,
            _precision,
            dis_status,
            _reserved1,
            flow_vel_x,
            flow_vel_y,
            flow_quality,
            flow_status,
            _reserved2,
        ) = struct.unpack(RANGE_SENSOR_FMT, payload)

        return SensorReading(
            flow_vx=float(flow_vel_x),
            flow_vy=float(flow_vel_y),
            distance_m=distance_mm / 1000.0,
            flow_quality=flow_quality,
            flow_status=flow_status,
            dis_status=dis_status,
            time_ms=time_ms,
            timestamp=time.time(),
        )


def detect_baud(port: str, timeout_s: float = 0.8) -> int:
    """Try common baud rates and return the first one with valid Micolink frames."""
    parser = MicolinkParser()
    for baud in BAUD_CANDIDATES:
        frames_ok = 0
        with serial.Serial(port, baud, timeout=0.01) as ser:
            ser.reset_input_buffer()
            deadline = time.time() + timeout_s
            while time.time() < deadline:
                chunk = ser.read(ser.in_waiting or 1)
                for byte in chunk:
                    if parser.feed(byte):
                        frames_ok += 1
                if frames_ok > 0:
                    return baud
    return 0


class MTF01Reader:
    """Background reader for one MTF-01 sensor."""

    def __init__(self, port: str, baud: int = MTF01_BAUD) -> None:
        self.port = port
        self.baud = baud
        self._serial: Optional[serial.Serial] = None
        self._parser = MicolinkParser()
        self._lock = threading.Lock()
        self._latest = SensorReading()
        self._thread: Optional[threading.Thread] = None
        self._running = False

    def connect(self, auto_baud: bool = False) -> None:
        baud = self.baud
        if auto_baud:
            detected = detect_baud(self.port)
            if detected:
                baud = detected
                print(f"{self.port}: detected baud {baud}")
            else:
                print(f"{self.port}: baud auto-detect failed, using {baud}")

        self._serial = serial.Serial(self.port, baud, timeout=0.01)
        self._serial.reset_input_buffer()

    def disconnect(self) -> None:
        self.stop()
        if self._serial is not None:
            self._serial.close()
            self._serial = None

    def start(self) -> None:
        if self._running:
            return
        if self._serial is None:
            raise RuntimeError("Call connect() before start()")
        self._running = True
        self._thread = threading.Thread(target=self._read_loop, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._running = False
        if self._thread is not None:
            self._thread.join(timeout=1.0)
            self._thread = None

    def get_reading(self) -> SensorReading:
        with self._lock:
            return SensorReading(**self._latest.__dict__)

    def _read_loop(self) -> None:
        assert self._serial is not None
        while self._running:
            chunk = self._serial.read(self._serial.in_waiting or 1)
            for byte in chunk:
                if not self._parser.feed(byte):
                    # print(f"Invalid byte: {byte}")
                    continue
                reading = self._parser.parse_range_sensor()
                # print(f"Reading: {reading}")
                if reading is None:
                    continue
                with self._lock:
                    self._latest = reading


def main() -> None:
    logfile = open("mtf01_reader.log", "w")
    parser = argparse.ArgumentParser(description="Read MTF-01 sensor on Raspberry Pi")
    parser.add_argument("--port", default="/dev/serial0", help="Serial port")
    parser.add_argument("--baud", type=int, default=MTF01_BAUD)
    parser.add_argument("--auto-baud", action="store_true")
    parser.add_argument("--rate", type=float, default=100.0, help="Print rate in Hz")
    args = parser.parse_args()

    reader = MTF01Reader(args.port, baud=args.baud)
    reader.connect(auto_baud=args.auto_baud)
    reader.start()

    interval_s = 1.0 / 100
    logfile.write("time_s,vx,vy,height_m,quality,valid\n")
    x = 0
    y = 0
    print_frame = 0
    step = 0
    try:
        r = reader.get_reading()
        last_timestamp = r.timestamp
        while True:
            r = reader.get_reading()
            vx, vy = r.velocity_m_s
            step = r.timestamp - last_timestamp
            last_timestamp = r.timestamp
            if step > 1:
                continue
            # print(f"step: {step}")
            x = x + vx * step 
            y = y + vy * step 
            print_frame += 1
            if print_frame % 10 == 0:
                data_str = f"{r.timestamp:.3f},{x:.6f},{y:.6f},{r.distance_m:.4f},{r.flow_quality},{int(r.is_valid)}\n"
                logfile.write(data_str)
                print(data_str)
                logfile.flush()
            time.sleep(interval_s)
    except KeyboardInterrupt:
        logfile.close()
    finally:
        reader.disconnect()


if __name__ == "__main__":
    main()
