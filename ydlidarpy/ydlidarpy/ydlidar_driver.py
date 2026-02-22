# =============================================================================
# YDLidar X2 Driver Configuration
# =============================================================================
PORT          = "/dev/ttyUSB0"
BAUDRATE      = 115200
TIMEOUT       = 1.0          # seconds — serial read timeout
HEADER        = b'\xAA\x55'
DIST_SCALE    = 4.0          # Si / 4  → mm
ANGLE_SCALE   = 64.0         # (FSA|LSA >> 1) / 64 → degrees
ANG_CORR_A    = 21.8         # atan correction coefficient
ANG_CORR_B    = 155.3        # atan correction baseline (mm)
MIN_DIST      = 0.0          # mm, exclusive lower bound (0 drops invalid)
MAX_DIST      = float('inf') # mm, inclusive upper bound
READ_BUF_SIZE = 4096         # internal ring buffer size (bytes)
MAX_ANGLE_STEP = 5.0         # deg/sample sanity cap — rejects corrupt packets
# =============================================================================

import math
import struct
import threading
import logging

import serial

log = logging.getLogger(__name__)

# Pre-compiled for speed inside the hot loop
_UNPACK_H  = struct.Struct("<H")
_UNPACK_2H = struct.Struct("<HH")


class YDLidarX2:
    def __init__(self,
                 port=PORT, baudrate=BAUDRATE, timeout=TIMEOUT,
                 header=HEADER,
                 dist_scale=DIST_SCALE, angle_scale=ANGLE_SCALE,
                 ang_corr_a=ANG_CORR_A, ang_corr_b=ANG_CORR_B,
                 min_dist=MIN_DIST, max_dist=MAX_DIST,
                 read_buf_size=READ_BUF_SIZE,
                 max_angle_step=MAX_ANGLE_STEP):
        self.header        = header
        self.dist_scale    = dist_scale
        self.angle_scale   = angle_scale
        self.ang_corr_a    = ang_corr_a
        self.ang_corr_b    = ang_corr_b
        self.min_dist      = min_dist
        self.max_dist      = max_dist
        self.read_buf_size = read_buf_size
        self.max_angle_step = max_angle_step

        self._buf   = bytearray()
        self._stop  = threading.Event()
        self._stats = {"packets_ok": 0, "packets_bad_cs": 0, "serial_errors": 0}

        self.ser = serial.Serial(port, baudrate, timeout=timeout)
        log.info("Opened %s @ %d baud", port, baudrate)

    # -------------------------------------------------------------------------
    # Internal buffer — single bulk read per iteration, no per-byte syscalls
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
        """Block until at least n bytes are in the buffer."""
        while len(self._buf) < n:
            self._fill_buf()

    # -------------------------------------------------------------------------
    # Sync to next valid header, discarding garbage bytes
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
                return  # header found, leave it in the buffer
            del self._buf[0]  # false h0, keep scanning

    # -------------------------------------------------------------------------
    # Checksum: XOR of 16-bit words PH, CT|LSN, FSA, LSA, S1..SN
    # CS word (bytes 8-9) is excluded per the development manual.
    # -------------------------------------------------------------------------
    @staticmethod
    def _checksum(packet):
        cs = 0
        # XOR all 16-bit words except CS (bytes 8-9)
        for i in range(0, 8, 2):
            cs ^= _UNPACK_H.unpack_from(packet, i)[0]
        for i in range(10, len(packet), 2):
            cs ^= _UNPACK_H.unpack_from(packet, i)[0]
        return cs

    # -------------------------------------------------------------------------
    # Angle correction (development manual §Angle analysis, level 2)
    # -------------------------------------------------------------------------
    def _angle_correct(self, distance):
        if distance == 0.0:
            return 0.0
        return math.degrees(
            math.atan(self.ang_corr_a * (self.ang_corr_b - distance)
                      / (self.ang_corr_b * distance))
        )

    # -------------------------------------------------------------------------
    # Read one valid packet — retries internally on bad checksum or short read
    # -------------------------------------------------------------------------
    def read_packet(self):
        while not self._stop.is_set():
            try:
                self._sync()
                # Read: PH(2) CT(1) LSN(1) FSA(2) LSA(2) CS(2) = 10 bytes
                self._ensure(10)
                header_view = bytes(self._buf[:10])
                lsn = header_view[3]
                sample_len = lsn * 2

                self._ensure(10 + sample_len)
                packet = bytes(self._buf[:10 + sample_len])

                cs_expected = _UNPACK_H.unpack_from(packet, 8)[0]
                cs_actual   = self._checksum(packet)

                if cs_actual != cs_expected:
                    self._stats["packets_bad_cs"] += 1
                    log.debug("Bad checksum (got %04X, expected %04X) — resyncing",
                              cs_actual, cs_expected)
                    del self._buf[0]  # skip one byte and resync
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
    # Single pass: decode + correct + filter in one loop.
    # -------------------------------------------------------------------------
    def parse_packet(self, packet):
        ct  = packet[2]
        lsn = packet[3]
        fsa, lsa = _UNPACK_2H.unpack_from(packet, 4)

        start_angle = (fsa >> 1) / self.angle_scale
        end_angle   = (lsa >> 1) / self.angle_scale
        if end_angle < start_angle:
            end_angle += 360.0

        angle_step = (end_angle - start_angle) / max(lsn - 1, 1)

        # Sanity check — corrupted FSA/LSA produce absurd steps
        if abs(angle_step) > self.max_angle_step:
            log.debug("Rejected packet: angle_step=%.2f exceeds max %.2f",
                      angle_step, self.max_angle_step)
            return []

        is_start    = bool(ct & 0x01)
        min_d, max_d = self.min_dist, self.max_dist
        points      = []

        for i in range(lsn):
            raw_dist = _UNPACK_H.unpack_from(packet, 10 + i * 2)[0]
            distance = raw_dist / self.dist_scale
            if distance <= min_d or distance > max_d:
                continue
            angle = (start_angle + angle_step * i
                     + self._angle_correct(distance)) % 360.0
            points.append((angle, distance, is_start and i == 0))

        return points

    # -------------------------------------------------------------------------
    # Convenience generator — yields (angle_deg, distance_mm, is_frame_start)
    # Handles shutdown cleanly via stop()
    # -------------------------------------------------------------------------
    def scan(self):
        while not self._stop.is_set():
            try:
                yield from self.parse_packet(self.read_packet())
            except (TimeoutError, serial.SerialException):
                break  # device gone — let caller decide what to do

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


if __name__ == "__main__":
    logging.basicConfig(level=logging.DEBUG)
    with YDLidarX2() as lidar:
        try:
            for angle, distance, is_start in lidar.scan():
                print(f"{'[START] ' if is_start else ''}{angle:.2f}°  {distance:.1f} mm")
        except KeyboardInterrupt:
            pass