import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import LaserScan
from tf2_ros import StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped
import serial
from pathlib import Path
import yaml
from ydlidar_driver import YDLidarX2

# Load configuration from YAML
_config_path = Path(__file__).parent.parent / "config" / "config.yaml"
with open(_config_path, "r") as f:
    _config = yaml.safe_load(f)

# =============================================================================
TOPIC          = _config["topic"]
FRAME_ID       = _config["frame_id"]
NODE_NAME      = _config["node_name"]
QOS_DEPTH      = _config["qos_depth"]
MIN_RANGE      = _config["min_range"]
MAX_RANGE      = _config["max_range"]
ANGLE_MIN      = _config["angle_min"]
ANGLE_MAX      = _config["angle_max"]
NUM_BINS       = _config["num_bins"]
SCAN_HZ        = _config["scan_hz"]
# =============================================================================


class YDLidarNode(Node):

    def __init__(self):
        super().__init__(NODE_NAME)

        self.declare_parameter("port",      "/dev/ttyUSB0")
        self.declare_parameter("baudrate",  115200)
        self.declare_parameter("frame_id",  FRAME_ID)
        self.declare_parameter("topic",     TOPIC)
        self.declare_parameter("min_range", MIN_RANGE)
        self.declare_parameter("max_range", MAX_RANGE)

        port           = self.get_parameter("port").value
        baudrate       = self.get_parameter("baudrate").value
        self.frame_id  = self.get_parameter("frame_id").value
        self.min_range = self.get_parameter("min_range").value
        self.max_range = self.get_parameter("max_range").value

        qos = QoSProfile(
            depth=QOS_DEPTH,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.pub = self.create_publisher(
            LaserScan,
            self.get_parameter("topic").value,
            qos,
        )

        # Static TF map → laser
        self._tf_broadcaster = StaticTransformBroadcaster(self)
        tf = TransformStamped()
        tf.header.stamp         = self.get_clock().now().to_msg()
        tf.header.frame_id      = "map"
        tf.child_frame_id       = self.frame_id
        tf.transform.rotation.w = 1.0
        self._tf_broadcaster.sendTransform(tf)

        self.lidar = YDLidarX2(
            port=port,
            baudrate=baudrate,
            min_dist=self.min_range * 1000,
            max_dist=self.max_range * 1000,
        )

        # Working buffer for the rotation currently being assembled.
        # Swapped out atomically when a frame-start signal arrives.
        self._pending = [float('inf')] * NUM_BINS

        self._rotation_complete = False

        self.get_logger().info(f"YDLidar X2 node started on {port}")

    # -------------------------------------------------------------------------

    def spin_scan(self):
        while True:
            try:
                packet = self.lidar.read_packet()
            except (TimeoutError, serial.SerialException) as e:
                self.get_logger().error(f"Lidar read failed: {e}")
                break

            # Read the frame-start flag directly from the packet header
            # CT byte is at index 2, LSB = 1 means start of new rotation.
            # This is checked BEFORE parse_packet filters any points out,
            # so we never miss a rotation boundary due to empty point lists.
            is_frame_start = bool(packet[2] & 0x01)

            if is_frame_start:
                if self._rotation_complete:
                    self._publish(self._pending)
                self._pending = [float('inf')] * NUM_BINS
                self._rotation_complete = True

            for scan_point in self.lidar.parse_packet(packet):
                idx = round(scan_point.angle) % NUM_BINS
                self._pending[idx] = scan_point.distance / 1000.0  # Convert mm to meters

    # -------------------------------------------------------------------------

    def _publish(self, ranges):
        msg = LaserScan()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.angle_min       = ANGLE_MIN
        msg.angle_max       = ANGLE_MAX - (ANGLE_MAX / NUM_BINS)  # exclusive end
        msg.angle_increment = ANGLE_MAX / NUM_BINS
        msg.time_increment  = 0.0
        msg.scan_time       = 1.0 / SCAN_HZ
        msg.range_min       = self.min_range
        msg.range_max       = self.max_range
        msg.ranges          = list(ranges) 
        self.pub.publish(msg)


# =============================================================================

def main():
    rclpy.init()
    node = YDLidarNode()
    try:
        node.spin_scan()
    except KeyboardInterrupt:
        pass
    finally:
        node.lidar.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()