# =============================================================================
# YDLidar X2 Driver
# =============================================================================
PORT           = "/dev/ttyUSB0"
BAUDRATE       = 115200
TIMEOUT        = 1.0
HEADER         = b'\xAA\x55'
DIST_SCALE     = 4.0
ANGLE_SCALE    = 64.0
ANG_CORR_A     = 21.8
ANG_CORR_B     = 155.3
MIN_DIST       = 0.0
MAX_DIST       = float('inf')
READ_BUF_SIZE  = 4096
MAX_ANGLE_STEP = 5.0
# =============================================================================

import math
import struct
import threading
import logging
from dataclasses import dataclass

import serial

log = logging.getLogger(__name__)

_UNPACK_H  = struct.Struct("<H")
_UNPACK_2H = struct.Struct("<HH")


@dataclass
class LidarScan:
    """Represents a single lidar measurement point."""
    angle: float
    distance: float
    is_frame_start: bool


class YDLidarX2:

    def __init__(self,
                 port=PORT, baudrate=BAUDRATE, timeout=TIMEOUT,
                 header=HEADER,
                 dist_scale=DIST_SCALE, angle_scale=ANGLE_SCALE,
                 ang_corr_a=ANG_CORR_A, ang_corr_b=ANG_CORR_B,
                 min_dist=MIN_DIST, max_dist=MAX_DIST,
                 read_buf_size=READ_BUF_SIZE,
                 max_angle_step=MAX_ANGLE_STEP):

        self.header         = header
        self.dist_scale     = dist_scale
        self.angle_scale    = angle_scale
        self.ang_corr_a     = ang_corr_a
        self.ang_corr_b     = ang_corr_b
        self.min_dist       = min_dist
        self.max_dist       = max_dist
        self.read_buf_size  = read_buf_size
        self.max_angle_step = max_angle_step

        self._buf   = bytearray()
        self._stop  = threading.Event()
        self._stats = {
            "packets_ok":      0,
            "packets_bad_cs":  0,
            "packets_rejected": 0,   # added: counts MAX_ANGLE_STEP rejects
            "serial_errors":   0,
        }

        self.ser = serial.Serial(port, baudrate, timeout=timeout)
        log.info("Opened %s @ %d baud", port, baudrate)

    # -------------------------------------------------------------------------
    # Internal buffer
    # -------------------------------------------------------------------------

    def _fill_buf(self):
        waiting = self.ser.in_waiting or self.read_buf_size
        chunk = self.ser.read(min(waiting, self.read_buf_size))
        if not chunk:
            raise TimeoutError("Serial read timed out — device disconnected?")
        self._buf.extend(chunk)

    def _consume(self, n):
        data = bytes(self._buf[:n])
        del self._buf[:n]
        return data

    def _ensure(self, n):
        while len(self._buf) < n:
            self._fill_buf()

    # -------------------------------------------------------------------------
    # Header sync
    # -------------------------------------------------------------------------

    def _sync(self):
        h0, h1 = self.header[0], self.header[1]
        while not self._stop.is_set():
            self._ensure(1)
            if self._buf[0] != h0:
                del self._buf[0]
                continue
            self._ensure(2)
            if self._buf[1] == h1:
                return
            del self._buf[0]

    # -------------------------------------------------------------------------
    # Checksum: XOR of all 16-bit words except CS (bytes 8-9)
    # -------------------------------------------------------------------------

    @staticmethod
    def _checksum(packet):
        cs = 0
        for i in range(0, 8, 2):
            cs ^= _UNPACK_H.unpack_from(packet, i)[0]
        for i in range(10, len(packet), 2):
            cs ^= _UNPACK_H.unpack_from(packet, i)[0]
        return cs

    # -------------------------------------------------------------------------
    # Angle correction  (development manual §Angle analysis, level 2)
    # Returns 0.0 for zero-distance points to avoid atan domain issues.
    # -------------------------------------------------------------------------

    def _angle_correct(self, distance):
        if distance == 0.0:
            return 0.0
        return math.degrees(
            math.atan(
                self.ang_corr_a * (self.ang_corr_b - distance)
                / (self.ang_corr_b * distance)
            )
        )

    # -------------------------------------------------------------------------
    # Read one valid packet
    # -------------------------------------------------------------------------

    def read_packet(self):
        while not self._stop.is_set():
            try:
                self._sync()

                # Fixed header: PH(2) CT(1) LSN(1) FSA(2) LSA(2) CS(2) = 10 bytes
                self._ensure(10)
                header_view = bytes(self._buf[:10])
                lsn = header_view[3]

                # Guard: LSN == 0 is degenerate — skip and resync
                if lsn == 0:
                    log.debug("LSN=0 packet skipped")
                    del self._buf[0]
                    continue

                sample_len = lsn * 2
                self._ensure(10 + sample_len)
                packet = bytes(self._buf[:10 + sample_len])

                cs_expected = _UNPACK_H.unpack_from(packet, 8)[0]
                cs_actual   = self._checksum(packet)

                if cs_actual != cs_expected:
                    self._stats["packets_bad_cs"] += 1
                    log.debug(
                        "Bad checksum (got %04X, expected %04X) — resyncing",
                        cs_actual, cs_expected,
                    )
                    del self._buf[0]
                    continue

                del self._buf[:10 + sample_len]
                self._stats["packets_ok"] += 1
                return packet

            except TimeoutError as e:
                self._stats["serial_errors"] += 1
                log.warning("%s", e)
                raise
            except serial.SerialException as e:
                self._stats["serial_errors"] += 1
                log.error("Serial error: %s", e)
                raise

    # -------------------------------------------------------------------------
    # Parse one packet → list of (angle_deg, distance_mm, is_frame_start)
    # -------------------------------------------------------------------------

    def parse_packet(self, packet):
        ct  = packet[2]
        lsn = packet[3]
        fsa, lsa = _UNPACK_2H.unpack_from(packet, 4)

        start_angle = (fsa >> 1) / self.angle_scale
        end_angle   = (lsa >> 1) / self.angle_scale

        # Handle wraparound: end < start means we crossed 0°
        if end_angle < start_angle:
            end_angle += 360.0

        # With lsn=1 there is no step — treat the single point as start_angle
        angle_step = (end_angle - start_angle) / max(lsn - 1, 1)

        if abs(angle_step) > self.max_angle_step:
            self._stats["packets_rejected"] += 1
            log.debug(
                "Rejected packet: angle_step=%.2f exceeds max %.2f",
                angle_step, self.max_angle_step,
            )
            return []

        is_start     = bool(ct & 0x01)
        min_d, max_d = self.min_dist, self.max_dist
        points       = []

        for i in range(lsn):
            raw_dist = _UNPACK_H.unpack_from(packet, 10 + i * 2)[0]
            distance = raw_dist / self.dist_scale

            # Drop out-of-range — node sees inf for that bin (not a ghost)
            if distance <= min_d or distance > max_d:
                continue

            angle = (
                start_angle
                + angle_step * i
                + self._angle_correct(distance)
            ) % 360.0

            # Only the very first point of the packet carries is_frame_start.
            # All subsequent points in the same packet are normal samples.
            points.append(LidarScan(
                angle=angle,
                distance=distance,
                is_frame_start=is_start and i == 0
            ))

        return points

    # -------------------------------------------------------------------------
    # Convenience generator
    # -------------------------------------------------------------------------

    def scan(self):
        while not self._stop.is_set():
            try:
                yield from self.parse_packet(self.read_packet())
            except (TimeoutError, serial.SerialException):
                break

    # -------------------------------------------------------------------------

    def stop(self):
        self._stop.set()
        self.ser.close()
        log.info("Stopped. Stats: %s", self._stats)

    @property
    def stats(self):
        return dict(self._stats)

    def __enter__(self):
        return self

    def __exit__(self, *_):
        self.stop()


# =============================================================================

if __name__ == "__main__":
    logging.basicConfig(level=logging.DEBUG)
    with YDLidarX2() as lidar:
        try:
            for scan_point in lidar.scan():
                print(
                    f"{'[START] ' if scan_point.is_frame_start else ''}"
                    f"{scan_point.angle:.2f}°  {scan_point.distance:.1f} mm"
                )
        except KeyboardInterrupt:
            pass