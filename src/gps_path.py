#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Publish a GPS path CSV (time,lat,lon,hdop,quality) to RViz as:
  1) nav_msgs/Path           on /gps_path
  2) visualization_msgs/Marker (POINTS) on /gps_waypoints
  3) visualization_msgs/Marker (TEXT)   on /gps_waypoint_labels  (optional)

lat/lon -> local planar (x,y) with simple approximation (ENU-like).
Good for campus/track scale.

Run:
  ros2 run <your_pkg> gps_csv_path_pub --ros-args -p csv_path:=/abs/path/file.csv

RViz:
  Fixed Frame = map
  Add -> Path   -> /gps_path
  Add -> Marker -> /gps_waypoints
  (optional) Add -> Marker -> /gps_waypoint_labels
"""

import csv
import math
from typing import List, Tuple, Optional

import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Time
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Point
from visualization_msgs.msg import Marker
from sensor_msgs.msg import NavSatFix
from rclpy.qos import qos_profile_sensor_data



def ll2xy_local(lat0: float, lon0: float, lat: float, lon: float) -> Tuple[float, float]:
    """Local tangent plane approximation: x=East[m], y=North[m]."""
    R = 6378137.0
    dlat = math.radians(lat - lat0)
    dlon = math.radians(lon - lon0)
    x = dlon * R * math.cos(math.radians(lat0))
    y = dlat * R
    return x, y


class GPSCsvPathPublisher(Node):
    def __init__(self):
        super().__init__('gps_csv_path_publisher')

        # Params
        self.declare_parameter('csv_path', '/home/nvidia/ros2_ws/src/gps_nav/gps_nav/paths/EntL8-go2-0.75m.txt')
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('publish_hz', 1.0)
        self.declare_parameter('min_quality', 0)
        self.declare_parameter('max_hdop', 999.0)
        self.declare_parameter('z', 0.0)

        # Waypoint marker options
        self.declare_parameter('wp_step', 1)           # plot every Nth point as a waypoint marker
        self.declare_parameter('wp_size', 0.25)        # marker size (m)
        self.declare_parameter('label_enable', False)  # publish text labels
        self.declare_parameter('label_step', 10)       # label every Nth waypoint marker (after wp_step)
        self.declare_parameter('label_z', 0.8)         # text height above ground
        self.declare_parameter('label_scale', 0.35)    # text size
        self.declare_parameter('fix_topic', '/fix')

        self.csv_path: str = self.get_parameter('csv_path').value
        self.frame_id: str = self.get_parameter('frame_id').value
        publish_hz: float = float(self.get_parameter('publish_hz').value)
        self.min_quality: int = int(self.get_parameter('min_quality').value)
        self.max_hdop: float = float(self.get_parameter('max_hdop').value)
        self.z: float = float(self.get_parameter('z').value)

        self.wp_step: int = max(1, int(self.get_parameter('wp_step').value))
        self.wp_size: float = float(self.get_parameter('wp_size').value)
        self.label_enable: bool = bool(self.get_parameter('label_enable').value)
        self.label_step: int = max(1, int(self.get_parameter('label_step').value))
        self.label_z: float = float(self.get_parameter('label_z').value)
        self.label_scale: float = float(self.get_parameter('label_scale').value)
        self.fix_topic = self.get_parameter('fix_topic').value

        if not self.csv_path:
            raise RuntimeError("Parameter 'csv_path' is empty. Provide --ros-args -p csv_path:=/abs/path/file.csv")

        self.path_pub = self.create_publisher(Path, '/gps_path', 1)
        self.wp_pub = self.create_publisher(Marker, '/gps_waypoints', 1)
        self.label_pub = self.create_publisher(Marker, '/gps_waypoint_labels', 1)
        self.current_pub = self.create_publisher(Marker, '/gps_current', 1)
        self.fix_sub = self.create_subscription(NavSatFix, self.fix_topic, self._on_fix, qos_profile_sensor_data)

        self.path_msg: Optional[Path] = None
        self.wp_marker: Optional[Marker] = None
        self.label_marker: Optional[Marker] = None
        self.lat0: Optional[float] = None
        self.lon0: Optional[float] = None
        self.latest_xy: Optional[Tuple[float, float]] = None


        self._load_csv_build_msgs()

        period = 1.0 / max(0.1, publish_hz)
        self.timer = self.create_timer(period, self._on_timer)

        self.get_logger().info(
            f"Loaded path poses={len(self.path_msg.poses) if self.path_msg else 0}, "
            f"waypoints(points)={len(self.wp_marker.points) if self.wp_marker else 0}. "
            f"Publishing @ {publish_hz:.2f} Hz."
        )

    def _float_sec_to_time_msg(self, t_sec: float) -> Time:
        sec = int(t_sec)
        nanosec = int((t_sec - sec) * 1e9)
        msg = Time()
        msg.sec = sec
        msg.nanosec = max(0, min(999_999_999, nanosec))
        return msg

    def _read_csv(self, csv_path: str) -> List[Tuple[float, float, float, float, int]]:
        """
        Return list of (lat, lon, t_sec, hdop, quality) filtered by quality/hdop thresholds.
        CSV expected: time,lat,lon,hdop,quality
        """
        out: List[Tuple[float, float, float, float, int]] = []
        with open(csv_path, 'r', newline='') as f:
            reader = csv.DictReader(f)
            required = {'time', 'lat', 'lon'}
            if not required.issubset(set(reader.fieldnames or [])):
                raise RuntimeError(
                    f"CSV must contain columns {sorted(required)}. Found: {reader.fieldnames}"
                )

            for i, row in enumerate(reader):
                try:
                    t = float(row['time'])
                    lat = float(row['lat'])
                    lon = float(row['lon'])

                    hdop = float(row.get('hdop', '0') or '0')
                    quality = int(float(row.get('quality', '0') or '0'))

                    if quality < self.min_quality:
                        continue
                    if hdop > self.max_hdop:
                        continue

                    out.append((lat, lon, t, hdop, quality))
                except Exception as e:
                    self.get_logger().warn(f"Skip row {i}: parse error: {e}")
                    continue

        return out

    def _load_csv_build_msgs(self) -> None:
        rows = self._read_csv(self.csv_path)
        if not rows:
            raise RuntimeError(f"No valid rows found in CSV: {self.csv_path}")

        lat0, lon0 = rows[0][0], rows[0][1]
        self.lat0 = lat0
        self.lon0 = lon0
        self.get_logger().info(f"Local origin lat0={lat0:.7f}, lon0={lon0:.7f}")

        # 1) Path
        path = Path()
        path.header.frame_id = self.frame_id

        # 2) Waypoint Marker (POINTS)
        wp = Marker()
        wp.header.frame_id = self.frame_id
        wp.ns = "gps_waypoints"
        wp.id = 0
        wp.type = Marker.POINTS
        wp.action = Marker.ADD
        wp.pose.orientation.w = 1.0
        wp.scale.x = self.wp_size
        wp.scale.y = self.wp_size
        wp.color.a = 1.0
        wp.color.g = 1.0  # green
        wp.lifetime.sec = 0  # forever

        # 3) Label Marker (TEXT_VIEW_FACING) - we'll publish as a single MarkerArray-like stream:
        # RViz doesn't accept MarkerArray unless we use MarkerArray msg.
        # So we publish labels as individual markers packed into a single Marker of type TEXT? Not possible.
        # Instead: publish ONE TEXT marker showing the last waypoint index (simple), OR
        # publish many TEXT markers by cycling IDs each publish (heavier).
        #
        # Here we implement the "many TEXT markers" method by building a list and publishing them sequentially
        # is NOT possible with a single Marker topic. So instead we create ONE label marker that shows
        # start/end + count. (Lightweight & robust)
        label = Marker()
        label.header.frame_id = self.frame_id
        label.ns = "gps_waypoint_labels"
        label.id = 0
        label.type = Marker.TEXT_VIEW_FACING
        label.action = Marker.ADD
        label.pose.position.x = 0.0
        label.pose.position.y = 0.0
        label.pose.position.z = self.label_z
        label.pose.orientation.w = 1.0
        label.scale.z = self.label_scale
        label.color.a = 1.0
        label.color.r = 1.0
        label.color.g = 1.0
        label.color.b = 1.0
        label.text = ""  # set below

        # Build
        kept_for_wp = 0
        for idx, (lat, lon, t, hdop, quality) in enumerate(rows):
            x, y = ll2xy_local(lat0, lon0, lat, lon)

            ps = PoseStamped()
            ps.header.frame_id = self.frame_id
            ps.header.stamp = self._float_sec_to_time_msg(t)
            ps.pose.position.x = float(x)
            ps.pose.position.y = float(y)
            ps.pose.position.z = float(self.z)
            ps.pose.orientation.w = 1.0
            path.poses.append(ps)

            if idx % self.wp_step == 0:
                wp.points.append(Point(x=float(x), y=float(y), z=float(self.z)))
                kept_for_wp += 1

        # Put a simple text label at the start point
        sx, sy = ll2xy_local(lat0, lon0, rows[0][0], rows[0][1])
        ex, ey = ll2xy_local(lat0, lon0, rows[-1][0], rows[-1][1])
        label.pose.position.x = float(sx)
        label.pose.position.y = float(sy)
        label.text = f"GPS Path\nposes={len(path.poses)}\nwaypoints={kept_for_wp}\nend=({ex:.1f},{ey:.1f})"

        self.path_msg = path
        self.wp_marker = wp
        self.label_marker = label

    def _make_current_marker(self) -> Marker:
        m = Marker()
        m.header.frame_id = self.frame_id
        m.ns = "gps_current"
        m.id = 0
        m.type = Marker.SPHERE
        m.action = Marker.ADD
        m.pose.orientation.w = 1.0
        m.scale.x = 1.0
        m.scale.y = 1.0
        m.scale.z = 1.0
        m.color.a = 1.0
        m.color.r = 1.0   # red sphere
        m.color.g = 0.0
        m.color.b = 0.0
        m.lifetime.sec = 0
        return m

    def _on_fix(self, msg: NavSatFix):
        if self.lat0 is None or self.lon0 is None:
            # CSV 로드가 아직 안 됐거나 origin이 없으면 무시
            return

        lat = float(msg.latitude)
        lon = float(msg.longitude)
        x, y = ll2xy_local(self.lat0, self.lon0, lat, lon)
        self.latest_xy = (x, y)


    def _on_timer(self):
        now = self.get_clock().now().to_msg()
        if self.path_msg:
            self.path_msg.header.stamp = now
            self.path_pub.publish(self.path_msg)
        if self.wp_marker:
            self.wp_marker.header.stamp = now
            self.wp_pub.publish(self.wp_marker)
        if self.label_enable and self.label_marker:
            self.label_marker.header.stamp = now
            self.label_pub.publish(self.label_marker)
        if self.latest_xy is not None:
            x, y = self.latest_xy
            cur = self._make_current_marker()
            cur.header.stamp = now
            cur.pose.position.x = float(x)
            cur.pose.position.y = float(y)
            cur.pose.position.z = float(self.z)
            self.current_pub.publish(cur)



def main():
    rclpy.init()
    node = None
    try:
        node = GPSCsvPathPublisher()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
