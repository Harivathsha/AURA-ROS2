#!/usr/bin/env python3
"""Adapt AURA's existing RGB-D headers and selected odometry for RTAB-Map.

The original camera topics, physical TF tree, raw cloud and EKF are untouched.
RTAB-Map gets an isolated base/optical TF branch with the same physical pose.
Only headers/child-frame names change; pixels, calibration, poses, covariances
and acquisition timestamps retain their original values.
"""

import math
import time


SLAM_BASE = "aura_slam_base"
SLAM_OPTICAL = "aura_slam_optical"


def unit_quaternion(values):
    values = tuple(float(v) for v in values)
    if len(values) != 4 or not all(math.isfinite(v) for v in values):
        raise ValueError("Quaternion must contain four finite xyzw values")
    length = math.sqrt(sum(v * v for v in values))
    if length < 1e-12:
        raise ValueError("Quaternion has zero length")
    return tuple(v / length for v in values)


def quaternion_product(left, right):
    x, y, z, w = unit_quaternion(left)
    a, b, c, d = unit_quaternion(right)
    return unit_quaternion((w*a + x*d + y*c - z*b,
                            w*b - x*c + y*d + z*a,
                            w*c + x*b - y*a + z*d,
                            w*d - x*a - y*b - z*c))


def optical_rotation(camera_rotation):
    # Optical X=physical -Y, optical Y=physical -Z, optical Z=physical +X.
    # This is RPY(-pi/2, 0, -pi/2) relative to the physical camera.
    return quaternion_product(camera_rotation, (-0.5, 0.5, -0.5, 0.5))


def stamp_ns(stamp):
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def main():
    # Keeping ROS imports here permits independent checks of the geometry.
    import rclpy
    from rclpy.clock import Clock
    from rclpy.clock_type import ClockType
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy
    from rclpy.time import Time
    from rclpy.executors import ExternalShutdownException
    from sensor_msgs.msg import CameraInfo, Image
    from nav_msgs.msg import Odometry
    from geometry_msgs.msg import TransformStamped
    from tf2_ros import Buffer, TransformListener
    from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster
    from tf2_ros import TransformException

    class Adapter(Node):
        def __init__(self):
            super().__init__("aura_rgbd_adapter")
            self.declare_parameter("physical_base_frame", "base_footprint")
            self.declare_parameter("physical_camera_frame", "camera")
            self.declare_parameter("expected_camera_header", "camera_optical")
            self.declare_parameter("odom_input_topic", "/aura/odom_fused")
            self.physical_base = self.get_parameter("physical_base_frame").value
            self.physical_camera = self.get_parameter("physical_camera_frame").value
            self.expected_header = self.get_parameter("expected_camera_header").value
            self.odom_input_topic = self.get_parameter("odom_input_topic").value
            if not isinstance(self.odom_input_topic, str) or not self.odom_input_topic.startswith("/"):
                raise ValueError("odom_input_topic must be an absolute ROS topic name")
            if self.odom_input_topic == "/aura_slam/odom":
                raise ValueError("The odometry input cannot be the adapter's own output")
            self.buffer = Buffer()
            self.listener = TransformListener(self.buffer, self)
            self.static_tf = StaticTransformBroadcaster(self)
            self.dynamic_tf = TransformBroadcaster(self)
            self.optical_ready = False
            self.odom_frame = None
            self.last_odom_stamp = None
            self.dimensions = None
            self.counts = {"rgb": 0, "depth": 0, "info": 0, "odom": 0}
            self.last_seen = {}
            self.warning_times = {}
            self.rgb_pub = self.create_publisher(Image, "/aura_slam/rgb/image", 10)
            self.depth_pub = self.create_publisher(Image, "/aura_slam/depth/image", 10)
            self.info_pub = self.create_publisher(CameraInfo, "/aura_slam/rgb/camera_info", 10)
            self.odom_pub = self.create_publisher(Odometry, "/aura_slam/odom", 30)
            self.subscriptions_ = [
                self.create_subscription(Image, "/camera/image", self.rgb, qos_profile_sensor_data),
                self.create_subscription(Image, "/camera/depth_image", self.depth, qos_profile_sensor_data),
                self.create_subscription(CameraInfo, "/camera/camera_info", self.info, qos_profile_sensor_data),
                self.create_subscription(
                    Odometry, self.odom_input_topic, self.odom,
                    QoSProfile(depth=30, reliability=ReliabilityPolicy.BEST_EFFORT)),
            ]
            # Wall timers also diagnose absent /clock or a paused simulation.
            steady = Clock(clock_type=ClockType.STEADY_TIME)
            self.tf_timer = self.create_timer(0.5, self.prepare_optical_tf, clock=steady)
            self.health_timer = self.create_timer(5.0, self.report, clock=steady)
            self.get_logger().info(
                f"AURA adapter: waiting for physical camera TF and {self.odom_input_topic}. "
                "Original topics and TF are unchanged.")

        def warn(self, key, message):
            now = time.monotonic()
            if now - self.warning_times.get(key, -math.inf) >= 5.0:
                self.warning_times[key] = now
                self.get_logger().warning(message)

        def seen(self, kind):
            self.counts[kind] += 1
            self.last_seen[kind] = time.monotonic()

        def prepare_optical_tf(self):
            if self.optical_ready:
                return
            try:
                source = self.buffer.lookup_transform(
                    self.physical_base, self.physical_camera, Time())
            except TransformException as error:
                self.warn("tf", f"Waiting for {self.physical_base} -> {self.physical_camera}: {error}")
                return
            tr = source.transform.translation
            if not all(math.isfinite(v) for v in (tr.x, tr.y, tr.z)):
                raise RuntimeError("Physical camera TF has invalid translation")
            qr = source.transform.rotation
            quaternion = optical_rotation((qr.x, qr.y, qr.z, qr.w))
            alias = TransformStamped()
            alias.header.stamp = self.get_clock().now().to_msg()
            alias.header.frame_id = SLAM_BASE
            alias.child_frame_id = SLAM_OPTICAL
            alias.transform.translation.x = tr.x
            alias.transform.translation.y = tr.y
            alias.transform.translation.z = tr.z
            (alias.transform.rotation.x, alias.transform.rotation.y,
             alias.transform.rotation.z, alias.transform.rotation.w) = quaternion
            self.static_tf.sendTransform(alias)
            self.optical_ready = True
            self.get_logger().info(
                f"Optical TF ready: {SLAM_BASE} -> {SLAM_OPTICAL}; "
                f"mount xyz=({tr.x:.3f}, {tr.y:.3f}, {tr.z:.3f}). "
                "Optical Z forward, X right, Y down.")

        def check_camera(self, msg):
            if not self.optical_ready:
                return False
            if msg.header.frame_id != self.expected_header:
                self.warn("header", f"Unexpected camera header {msg.header.frame_id!r}; "
                          f"expected {self.expected_header!r}. Not relaying it.")
                return False
            if msg.width <= 0 or msg.height <= 0:
                self.warn("size", "Camera message has invalid dimensions.")
                return False
            if self.dimensions is not None and (msg.width, msg.height) != self.dimensions:
                self.warn("size", "Image dimensions disagree with CameraInfo; check registration.")
                return False
            return True

        def rgb(self, msg):
            if not self.check_camera(msg):
                return
            if msg.encoding not in ("rgb8", "bgr8", "rgba8", "bgra8", "mono8"):
                self.warn("rgb", f"Unsupported RGB encoding: {msg.encoding}")
                return
            msg.header.frame_id = SLAM_OPTICAL
            self.rgb_pub.publish(msg)
            self.seen("rgb")

        def depth(self, msg):
            if not self.check_camera(msg):
                return
            if msg.encoding not in ("32FC1", "16UC1"):
                self.warn("depth", f"Unsupported depth encoding: {msg.encoding}")
                return
            msg.header.frame_id = SLAM_OPTICAL
            self.depth_pub.publish(msg)
            self.seen("depth")

        def info(self, msg):
            if not self.check_camera(msg):
                return
            if not all(math.isfinite(float(v)) for v in msg.k) or msg.k[0] <= 0 or msg.k[4] <= 0:
                self.warn("calibration", "CameraInfo has invalid focal lengths.")
                return
            self.dimensions = (msg.width, msg.height)
            msg.header.frame_id = SLAM_OPTICAL
            self.info_pub.publish(msg)
            self.seen("info")

        def odom(self, msg):
            current = stamp_ns(msg.header.stamp)
            if not msg.header.frame_id or msg.child_frame_id != self.physical_base:
                self.warn("odom", "Input Odometry needs a parent frame and child "
                          f"{self.physical_base!r}; got "
                          f"{msg.header.frame_id!r} -> {msg.child_frame_id!r}.")
                return
            if self.odom_frame is not None and msg.header.frame_id != self.odom_frame:
                raise RuntimeError("Input odometry frame changed. Stop this SLAM run and "
                                   "restart it with a new database after the source is stable.")
            if self.last_odom_stamp is not None and current < self.last_odom_stamp:
                raise RuntimeError("Odometry time moved backwards. Restart the SLAM test "
                                   "after resetting Gazebo/odometry, using a new database.")
            position, orientation = msg.pose.pose.position, msg.pose.pose.orientation
            values = (position.x, position.y, position.z, orientation.x,
                      orientation.y, orientation.z, orientation.w)
            if not all(math.isfinite(v) for v in values):
                self.warn("odom", "Ignoring non-finite input pose.")
                return
            rotation = unit_quaternion(values[3:])
            if self.odom_frame is None:
                covariance = msg.pose.covariance
                self.get_logger().info(
                    f"Odometry input active: {self.odom_input_topic}; "
                    f"frames={msg.header.frame_id} -> {msg.child_frame_id}; "
                    f"first xyz=({position.x:.3f}, {position.y:.3f}, {position.z:.3f}); "
                    f"pose covariance xx={covariance[0]:.6g}, "
                    f"yy={covariance[7]:.6g}, yaw={covariance[35]:.6g}. "
                    "Poses, covariances and timestamps are relayed unchanged.")
            self.odom_frame = msg.header.frame_id
            self.last_odom_stamp = current
            transform = TransformStamped()
            transform.header = msg.header
            transform.child_frame_id = SLAM_BASE
            transform.transform.translation.x = position.x
            transform.transform.translation.y = position.y
            transform.transform.translation.z = position.z
            (transform.transform.rotation.x, transform.transform.rotation.y,
             transform.transform.rotation.z, transform.transform.rotation.w) = rotation
            self.dynamic_tf.sendTransform(transform)
            msg.child_frame_id = SLAM_BASE
            self.odom_pub.publish(msg)
            self.seen("odom")

        def report(self):
            missing = [k for k in self.counts if k not in self.last_seen]
            stale = [k for k, stamp in self.last_seen.items() if time.monotonic() - stamp > 5.0]
            self.get_logger().info(
                f"Relayed {self.counts}; input={self.odom_input_topic}; "
                f"optical_ready={self.optical_ready}; "
                f"odom_frame={self.odom_frame}; missing={missing}; stale={stale}")

    rclpy.init()
    node = None
    try:
        node = Adapter()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
