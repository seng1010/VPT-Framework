"""
Raycasting Node
- /cloud_map (RTAB-Map 포인트클라우드) 구독, voxel downsample 후 KDTree 구축
- /gaze_origin, /gaze_direction (vpt_gaze_bridge가 퍼블리시) 구독
- gaze ray와 포인트클라우드의 교차점 G_t(ground-truth gaze point) 계산
- G_t 및 시각화 마커(gaze ray, G_t) 퍼블리시

vpt_gaze_bridge와 분리한 이유: gaze 추정(카메라+GazeTR, 매 프레임 고빈도)과
raycasting(SLAM 맵 갱신 주기에 좌우됨)은 의존성과 갱신 주기가 다른 별개 관심사.
자세한 내용은 docs/decisions.md 참고.
"""

import rclpy
from rclpy.node import Node
import numpy as np
from scipy.spatial import KDTree

from geometry_msgs.msg import Point, PointStamped, Vector3Stamped
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
import message_filters

MAP_FRAME = 'map'
VOXEL_SIZE = 0.05
MAX_RAY_DIST = 5.0
RAY_STEP = 0.05
HIT_THRESHOLD = 0.15


class RaycastingNode(Node):
    def __init__(self):
        super().__init__('raycasting_node')

        self.map_points = None
        self.kdtree = None

        self.gaze_point_pub = self.create_publisher(PointStamped, '/gaze_point', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/head_gaze_markers', 10)

        self.create_subscription(PointCloud2, '/cloud_map', self.cloud_map_callback, 1)

        origin_sub = message_filters.Subscriber(self, PointStamped, '/gaze_origin')
        direction_sub = message_filters.Subscriber(self, Vector3Stamped, '/gaze_direction')
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [origin_sub, direction_sub], queue_size=10, slop=0.1)
        self.sync.registerCallback(self.gaze_callback)

        self.get_logger().info("Raycasting Node 시작! /gaze_origin, /gaze_direction, /cloud_map 구독 중")

    def cloud_map_callback(self, msg):
        if self.kdtree is not None:
            return

        self.get_logger().info("포인트클라우드 로딩 중...")
        points = []
        for p in pc2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True):
            points.append([p[0], p[1], p[2]])

        if len(points) == 0:
            return

        points = np.array(points, dtype=np.float32)

        voxel_indices = np.floor(points / VOXEL_SIZE).astype(int)
        _, unique_idx = np.unique(voxel_indices, axis=0, return_index=True)
        points = points[unique_idx]

        self.map_points = points
        self.kdtree = KDTree(points)
        self.get_logger().info(f"포인트클라우드 로드 완료! {len(points):,} 포인트")

    def ray_cast(self, origin, direction):
        if self.kdtree is None:
            return None
        direction = direction / np.linalg.norm(direction)
        t = 0.1
        while t < MAX_RAY_DIST:
            ray_point = origin + t * direction
            dist, idx = self.kdtree.query(ray_point, k=1)
            if dist < HIT_THRESHOLD:
                return self.map_points[idx]
            t += RAY_STEP
        return None

    def publish_markers(self, origin, G_t):
        markers = MarkerArray()
        now = self.get_clock().now().to_msg()

        ray_marker = Marker()
        ray_marker.header.frame_id = MAP_FRAME
        ray_marker.header.stamp = now
        ray_marker.ns = 'gaze_ray'
        ray_marker.id = 1
        ray_marker.type = Marker.ARROW
        ray_marker.action = Marker.ADD
        p_start = Point(x=float(origin[0]), y=float(origin[1]), z=float(origin[2]))
        p_end = Point(x=float(G_t[0]), y=float(G_t[1]), z=float(G_t[2]))
        ray_marker.points = [p_start, p_end]
        ray_marker.scale.x = 0.03
        ray_marker.scale.y = 0.06
        ray_marker.color = ColorRGBA(r=0.0, g=0.0, b=1.0, a=0.8)
        ray_marker.lifetime.sec = 1
        markers.markers.append(ray_marker)

        gt_marker = Marker()
        gt_marker.header.frame_id = MAP_FRAME
        gt_marker.header.stamp = now
        gt_marker.ns = 'gaze_target'
        gt_marker.id = 2
        gt_marker.type = Marker.SPHERE
        gt_marker.action = Marker.ADD
        gt_marker.pose.position.x = float(G_t[0])
        gt_marker.pose.position.y = float(G_t[1])
        gt_marker.pose.position.z = float(G_t[2])
        gt_marker.pose.orientation.w = 1.0
        gt_marker.scale.x = gt_marker.scale.y = gt_marker.scale.z = 0.15
        gt_marker.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=0.8)
        gt_marker.lifetime.sec = 1
        markers.markers.append(gt_marker)

        self.marker_pub.publish(markers)

    def gaze_callback(self, origin_msg, direction_msg):
        origin = np.array([origin_msg.point.x, origin_msg.point.y, origin_msg.point.z])
        direction = np.array([direction_msg.vector.x, direction_msg.vector.y, direction_msg.vector.z])

        G_t = self.ray_cast(origin, direction)
        if G_t is None:
            return

        gp = PointStamped()
        gp.header.frame_id = MAP_FRAME
        gp.header.stamp = self.get_clock().now().to_msg()
        gp.point.x, gp.point.y, gp.point.z = float(G_t[0]), float(G_t[1]), float(G_t[2])
        self.gaze_point_pub.publish(gp)

        self.get_logger().info(
            f"Origin: [{origin[0]:.2f}, {origin[1]:.2f}, {origin[2]:.2f}] "
            f"G_t: [{G_t[0]:.2f}, {G_t[1]:.2f}, {G_t[2]:.2f}]"
        )

        self.publish_markers(origin, G_t)


def main():
    rclpy.init()
    node = RaycastingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
