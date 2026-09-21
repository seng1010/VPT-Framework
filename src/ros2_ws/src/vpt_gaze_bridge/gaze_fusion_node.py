"""
Gaze Fusion Node (2026-08-20, #21 VPT 미팅 반영)

교수님 피드백: D455/Kinect는 역할을 나누는(SLAM 전용/얼굴 전용) 게 아니라 두 대가
동시에 같은 역할(얼굴/gaze 검출)을 하고, 그 결과를 합쳐야 한다 — "두 로봇이 같은 사람을
다른 각도에서 보는 것"처럼. gaze_bridge_node.py를 카메라별로(camera_name=kinect/d455)
두 개 띄우면 각각 `/kinect/...`, `/d455/...` 네임스페이스에 head_position/gaze_origin/
gaze_direction을 발행한다. 이 노드는 그 둘을 구독해서 하나로 합친 뒤, vpt_raycasting과
vpt_virtual_camera가 원래 기대하는 정규 토픽(`/head_position`, `/gaze_origin`,
`/gaze_direction`, `/virtual_camera/camera_info`, TF 프레임 `head_position`)으로 다시
발행한다 — 즉 이 두 다운스트림 노드는 전혀 안 건드려도 된다.

융합 방식:
- 각 카메라의 최신 head_position/gaze_direction에 타임스탬프를 붙여 캐시해 둔다.
- STALE_SEC 이내에 갱신된 카메라만 "살아있다"고 보고 융합에 포함한다 (얼굴이 한쪽
  카메라에서만 보이는 경우가 흔함 — 이때는 그 한쪽 값을 그대로 통과시킨다).
- 둘 다 살아있으면 우선 두 카메라의 head_position 거리/gaze_direction 각도 차이를 본다
  (2026-08-25 실측: 같은 사람을 동시에 보고 있는데도 head 48cm, gaze 52° 차이가 난 적
  있음 — Kinect intrinsics가 실측 캘리브레이션이 아니라 근사값인 것과 두 카메라가 각자
  독립적으로 localization하는 데서 오는 오차로 추정). HEAD_DISAGREE_M/GAZE_DISAGREE_DEG를
  넘어서 어긋나면 평균이 오히려 둘 중 어느 카메라 값과도 안 맞는 의미 없는 결과가 되므로,
  평균 대신 한 대만 신뢰해서 그대로 통과시킨다 — 어느 쪽을 고를지는 아래 frontality 기준.
- 어긋나지 않으면 head_position/gaze_direction을 **신뢰도 가중평균**한다(gaze_direction은
  가중합 후 재정규화).

신뢰도 가중치 (2026-09-21 추가):
- 근거: 2026-09-15 고정타겟 파일럿에서 단독 43.03° vs fusion 43.47° — 즉 단순 평균
  fusion이 아무 이득도 못 냈다. 원인은 좋은 추정과 나쁜 추정을 같은 무게로 섞었기 때문이고,
  같은 파일럿에서 오차가 "얼굴이 카메라 광축에서 얼마나 벗어났는가"를 그대로 따라간다는
  것도 확인됐다(편위 22~39°에서 7~28°, 87~89°에서 38~62° — results_summary.md).
  즉 어느 카메라를 덜 믿어야 하는지는 추정 결과를 보기 전에 미리 알 수 있다.
- 신호: gaze_bridge_node.py가 카메라별로 `/{camera}/face_frontality_deg`를 발행한다
  (얼굴 전방축과 '머리→카메라' 방향 사이 각도. 0°=카메라를 똑바로 마주봄).
  가중치 = cos(frontality), 하한 MIN_WEIGHT.
- 불일치 fallback도 이 신호를 쓴다: 예전의 "더 최근에 갱신된 카메라"는 신뢰도 점수가
  없어서 쓰던 임시 대체물이었고, 이제는 "얼굴을 더 정면으로 보는 카메라"를 고른다.
- 이 토픽이 안 들어오면(구버전 bridge 등) 조용히 기존 균등 평균으로 되돌아간다 — WARN 1회.
- 아직 안 쓰는 신호: 얼굴 픽셀 크기(해상도), landmark 가시성. frontality만으로 부족하면 추가.
"""

import math
import time

import rclpy
from rclpy.node import Node
import numpy as np

from geometry_msgs.msg import TransformStamped, PointStamped, Vector3Stamped
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA, Float32
from sensor_msgs.msg import CameraInfo
import tf2_ros

MAP_FRAME = 'map'
STALE_SEC = 1.0          # 이보다 오래된 카메라 데이터는 융합에서 제외
FUSE_RATE_HZ = 10.0       # 발행 주기 (구독 콜백이 아니라 타이머 기반 — 두 카메라 속도가 달라도 안정적)
CAMERAS = ('kinect1', 'kinect2')  # 2026-09-14 ICRA 마감 긴급 파일럿 — Kinect 2대 구성용 임시 변경

# 두 카메라가 동시에 살아있어도 이 이상 어긋나면 평균을 포기하고 한쪽만 쓴다
# (2026-08-25 실측 근거는 모듈 docstring 참고). 값은 첫 실측 기반 1차 추정치 —
# 오탐(정상인데 fallback됨)/누락(어긋났는데 평균됨) 비율 보고 나중에 조정.
HEAD_DISAGREE_M = 0.3
GAZE_DISAGREE_DEG = 30.0
GAZE_DISAGREE_DOT = np.cos(np.radians(GAZE_DISAGREE_DEG))

# 신뢰도 가중치 하한 (2026-09-21). 준측면이라 cos가 0 이하로 떨어져도 완전히 0으로 두면
# 두 카메라 다 준측면일 때 가중치 합이 0이 되어 정규화가 깨진다 — 그때는 사실상 균등 평균이
# 되도록 작은 하한을 준다.
MIN_WEIGHT = 0.01


def quat_from_z_axis(z_axis):
    """gaze_bridge_node.py의 publish_virtual_camera_tf()와 동일한 변환 재사용
    (z축이 gaze 방향을 향하는 회전행렬 -> 쿼터니언)."""
    up = np.array([0, 0, 1])
    if abs(np.dot(z_axis, up)) > 0.99:
        up = np.array([0, 1, 0])
    x_axis = np.cross(up, z_axis)
    x_axis = x_axis / np.linalg.norm(x_axis)
    y_axis = -np.cross(z_axis, x_axis)
    R = np.column_stack([x_axis, y_axis, z_axis])

    trace = R[0, 0] + R[1, 1] + R[2, 2]
    if trace > 0:
        s = 0.5 / np.sqrt(trace + 1.0)
        w = 0.25 / s
        x = (R[2, 1] - R[1, 2]) * s
        y = (R[0, 2] - R[2, 0]) * s
        z = (R[1, 0] - R[0, 1]) * s
    else:
        w, x, y, z = 1.0, 0.0, 0.0, 0.0
    return x, y, z, w


class GazeFusionNode(Node):
    def __init__(self):
        super().__init__('gaze_fusion_node')

        # 카메라별 최신값 캐시: {camera: {'head': np.array3 or None, 'gaze': np.array3 or None,
        #                                  'front': float(도) or None,
        #                                  't_head'/'t_gaze'/'t_front': monotonic}}
        self.cache = {c: {'head': None, 'gaze': None, 'front': None,
                          't_head': 0.0, 't_gaze': 0.0, 't_front': 0.0} for c in CAMERAS}
        self._warned_no_frontality = False
        self.camera_info_cache = {}  # camera -> (CameraInfo, monotonic timestamp)

        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        self.head_pub = self.create_publisher(PointStamped, '/head_position', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/head_gaze_markers', 10)
        self.gaze_origin_pub = self.create_publisher(PointStamped, '/gaze_origin', 10)
        self.gaze_direction_pub = self.create_publisher(Vector3Stamped, '/gaze_direction', 10)
        self.virtual_cam_info_pub = self.create_publisher(CameraInfo, '/virtual_camera/camera_info', 10)

        for cam in CAMERAS:
            self.create_subscription(
                PointStamped, f'/{cam}/head_position',
                self._make_head_cb(cam), 10)
            self.create_subscription(
                Vector3Stamped, f'/{cam}/gaze_direction',
                self._make_gaze_cb(cam), 10)
            self.create_subscription(
                CameraInfo, f'/{cam}/virtual_camera/camera_info',
                self._make_caminfo_cb(cam), 10)
            self.create_subscription(
                Float32, f'/{cam}/face_frontality_deg',
                self._make_frontality_cb(cam), 10)

        self.create_timer(1.0 / FUSE_RATE_HZ, self.fuse_and_publish)

        self.get_logger().info(
            f"Gaze Fusion Node 시작! 구독: {', '.join(f'/{c}/...' for c in CAMERAS)} "
            f"(stale={STALE_SEC}s) -> 정규 토픽(/head_position 등)으로 발행")

    def _make_head_cb(self, cam):
        def cb(msg):
            self.cache[cam]['head'] = np.array([msg.point.x, msg.point.y, msg.point.z])
            self.cache[cam]['t_head'] = time.monotonic()
        return cb

    def _make_gaze_cb(self, cam):
        def cb(msg):
            v = np.array([msg.vector.x, msg.vector.y, msg.vector.z])
            n = np.linalg.norm(v)
            if n > 1e-6:
                self.cache[cam]['gaze'] = v / n
                self.cache[cam]['t_gaze'] = time.monotonic()
        return cb

    def _make_frontality_cb(self, cam):
        def cb(msg):
            self.cache[cam]['front'] = float(msg.data)
            self.cache[cam]['t_front'] = time.monotonic()
        return cb

    def _make_caminfo_cb(self, cam):
        def cb(msg):
            self.camera_info_cache[cam] = (msg, time.monotonic())
        return cb

    def _alive_cameras(self):
        now = time.monotonic()
        alive = []
        for cam in CAMERAS:
            entry = self.cache[cam]
            if (entry['head'] is not None and entry['gaze'] is not None
                    and now - entry['t_head'] < STALE_SEC
                    and now - entry['t_gaze'] < STALE_SEC):
                alive.append(cam)
        return alive

    def _weights(self, cams):
        """카메라별 융합 가중치(합=1)를 face_frontality_deg로부터 계산한다.

        얼굴이 그 카메라를 정면으로 마주볼수록(0°) 눈이 잘 보여 추정이 정확하고,
        준측면(90°에 근접)일수록 무너진다 — cos을 그대로 가중치로 쓰면 이 관계가
        자연스럽게 반영된다(22° -> 0.93, 87° -> 0.05, 약 18배 차이).

        신선한 frontality가 하나라도 없으면 None을 돌려줘서 호출부가 기존 동작(균등
        평균)으로 떨어지게 한다 — 이 토픽을 발행하지 않는 구버전 gaze_bridge_node와
        같이 띄워도 깨지지 않도록.
        """
        now = time.monotonic()
        ws = []
        for c in cams:
            entry = self.cache[c]
            if entry['front'] is None or now - entry['t_front'] >= STALE_SEC:
                return None
            w = math.cos(math.radians(entry['front']))
            ws.append(w if w > MIN_WEIGHT else MIN_WEIGHT)
        total = sum(ws)
        if total < 1e-6:
            return None
        return [w / total for w in ws]

    def fuse_and_publish(self):
        alive = self._alive_cameras()
        if not alive:
            return

        weights = self._weights(alive)
        if weights is None and not self._warned_no_frontality:
            self.get_logger().warn(
                "face_frontality_deg를 못 받아서 신뢰도 가중치 없이 균등 평균으로 동작합니다 "
                "(gaze_bridge_node가 이 토픽을 발행하는 버전인지 확인).")
            self._warned_no_frontality = True

        if len(alive) >= 2:
            cam_a, cam_b = alive[0], alive[1]
            head_a, head_b = self.cache[cam_a]['head'], self.cache[cam_b]['head']
            gaze_a, gaze_b = self.cache[cam_a]['gaze'], self.cache[cam_b]['gaze']
            head_dist = float(np.linalg.norm(head_a - head_b))
            gaze_dot = float(np.clip(np.dot(gaze_a, gaze_b), -1.0, 1.0))

            if head_dist > HEAD_DISAGREE_M or gaze_dot < GAZE_DISAGREE_DOT:
                # 크게 어긋나면 평균이 어느 쪽과도 안 맞으므로 한 대만 믿는다. 예전엔 "더 최근에
                # 갱신된 카메라"를 골랐는데, 그건 신뢰도 신호가 없어서 쓰던 임시 대체물이었다
                # (2026-08-25 주석 참고). 이제 frontality가 있으므로 "얼굴을 더 정면으로 보고
                # 있는 카메라"를 고른다 — 오차가 편위각을 따라간다는 파일럿 결과에 직접 맞는 기준.
                if weights is not None:
                    fallback_cam = alive[int(np.argmax(weights))]
                    reason = f"더 정면, {self.cache[fallback_cam]['front']:.0f}°"
                else:
                    fallback_cam = max(alive, key=lambda c: self.cache[c]['t_head'])
                    reason = "더 최근 갱신"
                alive = [fallback_cam]
                weights = [1.0]
                self.get_logger().warn(
                    f"카메라 간 큰 불일치 감지(head={head_dist:.2f}m, "
                    f"gaze={np.degrees(np.arccos(gaze_dot)):.0f}°) -> "
                    f"평균 대신 {fallback_cam}만 사용({reason})",
                    throttle_duration_sec=3.0)

        heads = np.stack([self.cache[c]['head'] for c in alive])
        gazes = np.stack([self.cache[c]['gaze'] for c in alive])

        if weights is None or len(weights) != len(alive):
            w = np.full(len(alive), 1.0 / len(alive))
        else:
            w = np.asarray(weights, dtype=float)
            w = w / w.sum()

        if len(alive) >= 2:
            # frontality가 없으면(구버전 bridge/stale) 균등 가중이라 각도를 찍을 수 없다 — w만 남긴다.
            self.get_logger().info(
                "가중 융합: " + ", ".join(
                    (f"{c} frontality={self.cache[c]['front']:.0f}° w={wi:.2f}"
                     if self.cache[c]['front'] is not None else f"{c} w={wi:.2f}")
                    for c, wi in zip(alive, w)),
                throttle_duration_sec=5.0)

        head_fused = (heads * w[:, None]).sum(axis=0)
        gaze_fused = (gazes * w[:, None]).sum(axis=0)
        gnorm = np.linalg.norm(gaze_fused)
        if gnorm < 1e-6:
            # 두 카메라의 시선이 정반대라 상쇄된 경우(드묾) — 융합 포기, 가중치가 가장 높은 쪽 사용
            gaze_fused = gazes[int(np.argmax(w))]
        else:
            gaze_fused = gaze_fused / gnorm

        now = self.get_clock().now().to_msg()

        head_msg = PointStamped()
        head_msg.header.frame_id = MAP_FRAME
        head_msg.header.stamp = now
        head_msg.point.x, head_msg.point.y, head_msg.point.z = map(float, head_fused)
        self.head_pub.publish(head_msg)

        origin_msg = PointStamped()
        origin_msg.header.frame_id = MAP_FRAME
        origin_msg.header.stamp = now
        origin_msg.point.x, origin_msg.point.y, origin_msg.point.z = map(float, head_fused)
        self.gaze_origin_pub.publish(origin_msg)

        direction_msg = Vector3Stamped()
        direction_msg.header.frame_id = MAP_FRAME
        direction_msg.header.stamp = now
        direction_msg.vector.x, direction_msg.vector.y, direction_msg.vector.z = map(float, gaze_fused)
        self.gaze_direction_pub.publish(direction_msg)

        self._publish_head_marker(head_fused, now)
        self._publish_virtual_camera_tf(head_fused, gaze_fused, now, alive)

    def _publish_head_marker(self, head_fused, stamp):
        markers = MarkerArray()
        m = Marker()
        m.header.frame_id = MAP_FRAME
        m.header.stamp = stamp
        m.ns = 'head'
        m.id = 0
        m.type = Marker.SPHERE
        m.action = Marker.ADD
        m.pose.position.x, m.pose.position.y, m.pose.position.z = map(float, head_fused)
        m.pose.orientation.w = 1.0
        m.scale.x = m.scale.y = m.scale.z = 0.2
        m.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=0.8)
        m.lifetime.sec = 1
        markers.markers.append(m)
        self.marker_pub.publish(markers)

    def _publish_virtual_camera_tf(self, head_fused, gaze_fused, stamp, alive):
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = MAP_FRAME
        t.child_frame_id = 'head_position'
        t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = (
            map(float, head_fused))
        x, y, z, w = quat_from_z_axis(gaze_fused)
        t.transform.rotation.x = x
        t.transform.rotation.y = y
        t.transform.rotation.z = z
        t.transform.rotation.w = w
        self.tf_broadcaster.sendTransform(t)

        # virtual_camera_node용 CameraInfo — 살아있는 카메라 중 하나의 intrinsics를 그대로 통과
        # (렌더링용 근사 시야각이면 충분, 어느 쪽이든 큰 차이 없음).
        for cam in alive:
            if cam in self.camera_info_cache:
                src_info, _ = self.camera_info_cache[cam]
                cam_info = CameraInfo()
                cam_info.header.stamp = stamp
                cam_info.header.frame_id = 'head_position'
                cam_info.width = src_info.width
                cam_info.height = src_info.height
                cam_info.k = src_info.k
                cam_info.d = src_info.d
                cam_info.r = src_info.r
                cam_info.p = src_info.p
                cam_info.distortion_model = src_info.distortion_model
                self.virtual_cam_info_pub.publish(cam_info)
                break


def main():
    rclpy.init()
    node = GazeFusionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
