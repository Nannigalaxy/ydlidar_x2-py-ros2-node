# YDLIDAR X2 Driver & ROS 2 Node

A ROS 2 package for the YDLIDAR X2 2D LiDAR. Includes a pure-Python serial driver and a ROS 2 node.

![Screenshot in Rviz2](viz_screenshot.png)

---

## Hardware Setup

The X2 connects via USB–UART adapter. On Linux it typically enumerates as `/dev/ttyUSB0`.
```bash
# Verify the device is visible
ls /dev/ttyUSB*

# Permanent access (log out and back in after)
sudo usermod -aG dialout $USER

# One-shot for testing
sudo chmod 666 /dev/ttyUSB0
```

The X2 starts ranging and streaming automatically on power-up, no start command required.

---

## Build & Install

### Requirements

- ROS 2 (Tested only on Humble)
- Python 3.10
```bash
pip install -r requirements.txt   # pyserial and colcon
```

## Build
```bash
cd <your_ws>
colcon build --packages-select ydlidarpy
source install/setup.bash
```

---

## Usage

### ROS 2 node
```bash
ros2 run ydlidarpy x2_node
```

#### Parameters

| Parameter | Default | Description |
|---|---|---|
| `port` | `/dev/ttyUSB0` | Serial port |
| `baudrate` | `115200` | Fixed, do not change |
| `frame_id` | `laser` | TF frame for scan messages |
| `topic` | `scan` | Published topic name |
| `min_range` | `0.01` m | Minimum valid range |
| `max_range` | `8.0` m | Maximum valid range |
```bash
ros2 run ydlidarpy x2_node --ros-args -p port:=/dev/ttyUSB1 -p max_range:=6.0
```

### Standalone driver (no ROS)

Useful for verifying serial comms before involving ROS:
```bash
cd ydlidarpy/ydlidarpy
python ydlidar_driver.py
# prints:  angle_deg  distance_mm  per point
```

---

## Published Topics

| Topic | Type | QoS |
|---|---|---|
| `/scan` | `sensor_msgs/LaserScan` | BEST_EFFORT / VOLATILE |

### LaserScan details

- **360 bins**, one per degree (`angle_increment = 2π/360`)
- Published **once per full 360° rotation**, not once per packet
- Buffer is **reset at every rotation boundary** (CT byte LSB); stale points from previous sweeps never bleed into the current scan
- Bins with no valid return in the current rotation are published as `inf`
- The first partial sweep after startup is silently discarded; publishing begins on the second complete rotation

---

## Protocol Reference

### Packet structure
```
Offset  Field  Size  Description
0       PH     2B    Header: 0xAA 0x55
2       CT     1B    bit0=1: start-of-frame; bits7:1: scan freq (F = CT[7:1] / 10)
3       LSN    1B    Sample count in this packet
4       FSA    2B    Start angle (raw)
6       LSA    2B    End angle (raw)
8       CS     2B    XOR checksum (excludes itself)
10      Si…    2B×N  Distance samples (little-endian)
```

### Decoding
```
Distance_i (mm) = Si / 4

Angle_FSA       = (FSA >> 1) / 64
Angle_LSA       = (LSA >> 1) / 64
Angle_i         = diff(Angle) / (LSN - 1) * i + Angle_FSA   (linear interp)

# Per-point angle correction:
if Distance_i == 0:
    AngCorrect = 0
else:
    AngCorrect = atan(21.8 * (155.3 - Distance_i) / (155.3 * Distance_i))

Angle_i_final = Angle_i + AngCorrect
```

### Checksum

XOR of all 16-bit words **except CS** (bytes 8–9): `PH ^ (CT|LSN) ^ FSA ^ LSA ^ S1 ^ ... ^ SN`

---

## Rotation Boundary Detection

The node detects the start of a new 360° sweep by reading the `CT` byte directly from the raw packet header **before** `parse_packet` is called. This is important because `parse_packet` filters out out-of-range points — if all points in a frame-start packet happen to be filtered, the `for` loop over parsed points would never execute and the boundary signal would be missed entirely.

The flow on every packet is:

1. Read `CT` byte from raw packet → check `bit0`
2. If frame-start: publish completed buffer, allocate fresh `[inf] * 360` buffer
3. Call `parse_packet` → accumulate valid points into the fresh buffer

This guarantees ghost-free scans regardless of point filtering outcomes.

---

## Debugging

### No data / silent device
```bash
# Confirm the device is streaming raw bytes
python -c "
import serial, time
s = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
time.sleep(0.5)
print(s.read(64).hex())
"
# Expect packets beginning with: 55aa
```

### No scan messages published

The node discards the first partial rotation and only begins publishing after one complete sweep is received. If `/scan` never appears, confirm the frame-start signal is being received:
```bash
python -c "
import logging; logging.basicConfig(level=logging.DEBUG)
from ydlidarpy.ydlidar_driver import YDLidarX2
with YDLidarX2() as lidar:
    for a, d, s in lidar.scan():
        if s:
            print(f'[START]  {a:.1f}deg  {d:.0f}mm')
"
# You should see [START] lines appearing ~7 times per second
```

### Persistent checksum failures (`packets_bad_cs` climbing)

- Cable quality — try a different USB cable or port
- Any baud rate other than 115200 will cause this
- Enable debug logging to see exact mismatches:
```bash
python -c "
import logging; logging.basicConfig(level=logging.DEBUG)
from ydlidarpy.ydlidar_driver import YDLidarX2
with YDLidarX2() as lidar:
    for a, d, s in lidar.scan():
        print(f'{a:.1f}deg  {d:.0f}mm')
"
```

### Driver stats
```python
print(lidar.stats)
# {'packets_ok': 1482, 'packets_bad_cs': 1, 'packets_rejected': 0, 'serial_errors': 0}
```

`packets_rejected` counts packets dropped due to an implausible angle step (corrupted FSA/LSA fields). A non-zero value alongside a clean `packets_bad_cs` suggests intermittent bit errors on the angle fields specifically — try a shorter or shielded cable.

### Check topic publishing frequency
```bash
ros2 topic hz /scan
# Expected: ~7 Hz (one message per full rotation)
```

### Scan has gaps / missing sectors

Gaps within a single rotation are normal for highly reflective or absorptive surfaces — the driver drops zero-distance returns. If entire angular sectors are consistently missing across multiple rotations, check `packets_rejected` in the driver stats; a high count here means those angle ranges are being dropped due to corrupt FSA/LSA fields.

### Angles look rotated

The X2 has no absolute zero reference. Mounting orientation determines where angle 0 falls. Apply a static TF offset in your URDF or add a yaw rotation to the `map -> laser` transform in `x2_node.py`.

### `spin_scan()` blocks the ROS executor

`spin_scan()` runs a blocking serial loop on the main thread. If you need ROS timers, services, or parameter updates to stay responsive, move it to a thread:
```python
import threading
t = threading.Thread(target=node.spin_scan, daemon=True)
t.start()
rclpy.spin(node)
```

---

## Known Limitations

- `spin_scan()` is blocking — see threading note above
- `scan_time` is hardcoded to `1/7.0 s`; the actual scan frequency encoded in `CT[7:1]` is not forwarded to the `LaserScan` message
- Power-on info packet (`A5 5A 14 00 00 00 04 ...`) is not parsed; the driver syncs past it automatically
- No dynamic reconfigure support
- LSN=0 packets (zero samples) are detected and skipped; they do not trigger a rotation boundary swap