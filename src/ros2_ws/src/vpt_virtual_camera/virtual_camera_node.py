#!/usr/bin/env python3
"""
가상 카메라 렌더링 노드.

/cloud_map (PointCloud2) + head_position TF (gaze 방향 포함)를 받아서,
그 시점에서 포인트클라우드를 투영한 2D 이미지를 /virtual_camera/image_raw 로 퍼블리시.

지금까지는 이 토픽에 실제 카메라 raw 영상을 그대로 흘려보내고 있었는데,
그 대신 이 노드가 실제 map-기준 렌더링 이미지를 만들어 대체함.

필요 패키지: pip install open3d
"""

import numpy as np
import open3d as o3d

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import Image, CameraInfo, PointCloud2
from sensor_msgs_py import point_cloud2
from cv_bridge import CvBridge

import tf2_ros


def quaternion_matrix(q):
    """(x, y, z, w) -> 4x4 homogeneous rotation matrix. (tf_transformations 의존성 제거용)"""
    x, y, z, w = q
    n = x * x + y * y + z * z + w * w
    if n < 1e-12:
        return np.identity(4)
    s = 2.0 / n
    X, Y, Z = x * s, y * s, z * s
    wx, wy, wz = w * X, w * Y, w * Z
    xx, xy, xz = x * X, x * Y, x * Z
    yy, yz, zz = y * Y, y * Z, z * Z
    return np.array([
        [1.0 - (yy + zz), xy - wz, xz + wy, 0.0],
        [xy + wz, 1.0 - (xx + zz), yz - wx, 0.0],
        [xz - wy, yz + wx, 1.0 - (xx + yy), 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ])

# ---- 환경에 맞게 수정 ----
MAP_FRAME = "map"
HEAD_FRAME = "head_position"   # gaze 방향 포함된 TF (기존 파이프라인에서 이미 퍼블리시 중)
IMAGE_WIDTH = 640
IMAGE_HEIGHT = 480
FX = 386.3029479980469          # D455 실측 color camera_info 기준
FY = 385.8308410644531
CX = 326.624267578125
CY = 249.3714599609375
POINT_SIZE_PX = 2              # 렌더링 시 포인트 하나가 차지하는 픽셀 반경


class VirtualCameraNode(Node):
    def __init__(self):
        super().__init__("virtual_camera_node")

        self.bridge = CvBridge()
        self.map_points = None  # (N,3) float32, map frame 기준

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.create_subscription(
            PointCloud2, "/cloud_map", self.on_cloud_map, qos
        )

        self.image_pub = self.create_publisher(Image, "/virtual_camera/image_raw", 10)
        self.info_pub = self.create_publisher(CameraInfo, "/virtual_camera/camera_info", 10)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # offscreen renderer는 재사용 (매 프레임 새로 만들면 느림)
        self.renderer = o3d.visualization.rendering.OffscreenRenderer(IMAGE_WIDTH, IMAGE_HEIGHT)
        self.renderer.scene.set_background([0, 0, 0, 1])
        self.material = o3d.visualization.rendering.MaterialRecord()
        self.material.shader = "defaultUnlit"
        self.material.point_size = POINT_SIZE_PX
        self.geometry_added = False

        # ~10Hz로 렌더링 (필요시 조정)
        self.create_timer(0.1, self.render_and_publish)

        self.get_logger().info("virtual_camera_node started")

    def on_cloud_map(self, msg: PointCloud2):
        pts = point_cloud2.read_points_numpy(msg, field_names=("x", "y", "z"), skip_nans=True)
        if pts.shape[0] == 0:
            return
        self.map_points = pts.astype(np.float64)

    def get_head_pose(self):
        """head_position TF -> (4x4 map->head extrinsic)"""
        try:
            tf = self.tf_buffer.lookup_transform(
                MAP_FRAME, HEAD_FRAME, rclpy.time.Time()
            )
        except (tf2_ros.LookupException, tf2_ros.ExtrapolationException,
                tf2_ros.ConnectivityException) as e:
            self.get_logger().warn(f"TF lookup failed: {e}", throttle_duration_sec=2.0)
            return None

        t = tf.transform.translation
        q = tf.transform.rotation
        T_map_head = quaternion_matrix([q.x, q.y, q.z, q.w])
        T_map_head[0:3, 3] = [t.x, t.y, t.z]
        return T_map_head

    def render_and_publish(self):
        if self.map_points is None:
            return

        T_map_head = self.get_head_pose()
        if T_map_head is None:
            return

        if not self.geometry_added:
            pcd = o3d.geometry.PointCloud()
            pcd.points = o3d.utility.Vector3dVector(self.map_points)
            self.renderer.scene.add_geometry("map", pcd, self.material)
            self.geometry_added = True
        else:
            # 맵이 갱신될 수 있으면 매번 지우고 다시 추가 (SLAM이 map을 계속 업데이트하는 경우)
            self.renderer.scene.clear_geometry()
            pcd = o3d.geometry.PointCloud()
            pcd.points = o3d.utility.Vector3dVector(self.map_points)
            self.renderer.scene.add_geometry("map", pcd, self.material)

        intrinsic = o3d.camera.PinholeCameraIntrinsic(
            IMAGE_WIDTH, IMAGE_HEIGHT, FX, FY, CX, CY
        )

        # open3d는 world->camera extrinsic을 요구하므로 head pose(map->head)의 역행렬 사용
        extrinsic = np.linalg.inv(T_map_head)

        self.renderer.setup_camera(intrinsic, extrinsic)
        o3d_img = self.renderer.render_to_image()
        img_np = np.asarray(o3d_img)  # (H, W, 3) uint8, RGB

        img_msg = self.bridge.cv2_to_imgmsg(img_np, encoding="rgb8")
        img_msg.header.stamp = self.get_clock().now().to_msg()
        img_msg.header.frame_id = HEAD_FRAME
        self.image_pub.publish(img_msg)

        info_msg = CameraInfo()
        info_msg.header = img_msg.header
        info_msg.width = IMAGE_WIDTH
        info_msg.height = IMAGE_HEIGHT
        info_msg.k = [FX, 0.0, CX, 0.0, FY, CY, 0.0, 0.0, 1.0]
        self.info_pub.publish(info_msg)


def main():
    rclpy.init()
    node = VirtualCameraNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
