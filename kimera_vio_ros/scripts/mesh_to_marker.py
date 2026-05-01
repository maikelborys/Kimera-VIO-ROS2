#!/usr/bin/env python3
"""mesh_to_marker — Phase 2.5 light-weight visualization helper.

Subscribes to /kimera_vio_ros/mesh (pcl_msgs/PolygonMesh) and republishes a
visualization_msgs/Marker (TRIANGLE_LIST) on /kimera_vio_ros/mesh_marker so
plain `rviz2` can render the Kimera mesh without the abandoned
`mesh_rviz_plugins`. The proper port of that RViz plugin is deferred; this
script is intentionally throwaway.

Note on the wire format:
- `pcl_msgs/PolygonMesh.cloud` is a `sensor_msgs/PointCloud2` carrying the
  vertices. Kimera emits XYZ + custom fields (normals, UV) — we read only
  the (x, y, z) triplets via sensor_msgs_py.point_cloud2.
- `polygons[i].vertices[]` are uint32 indices into the cloud; Kimera emits
  triangles (3 indices each). We tolerate other polygon arities by skipping.
"""

from __future__ import annotations

import sys

import rclpy
from geometry_msgs.msg import Point
from pcl_msgs.msg import PolygonMesh
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs_py import point_cloud2
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker


class MeshToMarker(Node):

    def __init__(self) -> None:
        super().__init__("mesh_to_marker")
        self.declare_parameter("frame_id", "world")
        # Match Kimera's mesh publisher (10 Hz nominal, reliable). Increase
        # depth so we don't drop frames during RViz startup.
        qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE)

        self.sub = self.create_subscription(
            PolygonMesh, "mesh", self._on_mesh, qos)
        self.pub = self.create_publisher(Marker, "mesh_marker", qos)

        self._frame_override = (
            self.get_parameter("frame_id")
            .get_parameter_value()
            .string_value
        )
        self.get_logger().info(
            f"Subscribed to {self.sub.topic_name}; publishing "
            f"{self.pub.topic_name} (frame override: "
            f"{self._frame_override or '<msg header>'})")

    def _on_mesh(self, msg: PolygonMesh) -> None:
        try:
            xyz = list(point_cloud2.read_points(
                msg.cloud, field_names=("x", "y", "z"), skip_nans=False))
        except Exception as e:
            self.get_logger().warn(f"failed to read cloud points: {e}")
            return

        if not xyz or not msg.polygons:
            return

        marker = Marker()
        marker.header = msg.header
        if self._frame_override:
            marker.header.frame_id = self._frame_override
        marker.ns = "kimera_mesh"
        marker.id = 0
        marker.type = Marker.TRIANGLE_LIST
        marker.action = Marker.ADD
        # 1.0 scale = identity for TRIANGLE_LIST; vertex coords are world-frame.
        marker.scale.x = 1.0
        marker.scale.y = 1.0
        marker.scale.z = 1.0
        # Faint, semi-transparent gray — RViz overlays it on the cloud nicely.
        marker.color = ColorRGBA(r=0.7, g=0.7, b=0.75, a=0.5)
        marker.pose.orientation.w = 1.0
        marker.frame_locked = True

        n_pts = len(xyz)
        skipped = 0
        # We iterate triangle by triangle; each polygon should have 3 indices.
        for poly in msg.polygons:
            if len(poly.vertices) != 3:
                skipped += 1
                continue
            i0, i1, i2 = poly.vertices
            if i0 >= n_pts or i1 >= n_pts or i2 >= n_pts:
                skipped += 1
                continue
            for i in (i0, i1, i2):
                p = xyz[i]
                marker.points.append(
                    Point(x=float(p[0]), y=float(p[1]), z=float(p[2])))

        if not marker.points:
            return
        self.pub.publish(marker)
        if skipped:
            self.get_logger().debug(
                f"published {len(marker.points)//3} triangles, "
                f"skipped {skipped} non-triangular polygons")


def main(argv: list[str] | None = None) -> int:
    rclpy.init(args=argv if argv is not None else sys.argv)
    node = MeshToMarker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
