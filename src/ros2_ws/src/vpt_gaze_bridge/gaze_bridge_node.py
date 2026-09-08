"""
Gaze Bridge Node
- 카메라 이미지(RGB-D) 구독, MediaPipe로 얼굴/머리 위치 검출
- GazeTR(fallback: head pose 행렬)로 gaze vector 추정
- 머리 위치 + gaze 방향을 map 좌표계로 변환해 퍼블리시
- 머리 위치에 가상 카메라 TF/CameraInfo 퍼블리시 (image_raw는 vpt_virtual_camera 노드가 담당)

Ray casting(포인트클라우드와의 교차점 G_t 계산)은 이 노드가 하지 않는다.
/gaze_origin, /gaze_direction 을 퍼블리시하고, vpt_raycasting 노드가 구독해서
G_t를 계산한다. 이유는 docs/decisions.md 참고.

카메라별 파라미터화 (2026-08-20, #21 VPT 미팅 반영):
교수님 피드백 — D455/Kinect는 역할을 나누는(SLAM 전용/얼굴 전용) 게 아니라 **같은 역할을
동시에** 수행해야 함(두 로봇이 같은 사람을 다른 각도에서 보는 것처럼). D455는 이미
2026-08-06~08-20 세션에서 자체 RTAB-Map localization으로 map 기준 pose를 얻도록 검증됨
(scripts/run_kinect_localization.sh와 대칭되는 D455용 실행이 이미 가능 — extrinsic 실측 불필요).

그래서 이 노드를 카메라 하나에 고정하지 않고 ROS2 파라미터로 카메라를 바꿔가며 두 개
인스턴스(kinect용/d455용)를 동시에 띄울 수 있게 파라미터화했다. 각 인스턴스는 자기 이름이
붙은 토픽(`/{camera_name}/head_position` 등)에만 발행한다 — 기존 정규 토픽
(`/head_position`, `/gaze_origin`, `/gaze_direction`, `/virtual_camera/camera_info`, TF
프레임 `head_position`)은 이 노드가 아니라 새로 추가한 gaze_fusion_node.py가 두 인스턴스의
출력을 합쳐서 발행한다 — vpt_raycasting/vpt_virtual_camera는 전혀 안 건드려도 됨.
"""

import rclpy
from rclpy.node import Node
import numpy as np
import cv2
import mediapipe as mp
from mediapipe.tasks import python
from mediapipe.tasks.python import vision
import os
import sys
import urllib.request
from collections import deque
import torch
# kinect/d455 두 인스턴스가 동시에 떠서 각자 torch 기본 intra-op 스레드풀(코어 수만큼)을
# 잡으면 스레드 오버섭스크립션으로 futex 교착이 걸린 적이 있다(2026-08-21, gaze_bridge_d455가
# 카메라 토픽은 30Hz로 정상인데 콜백 자체가 멈춤 — CPU 116%, futex_do_wait에서 무한 대기).
# 인스턴스당 1스레드로 고정해서 재발 방지.
torch.set_num_threads(1)

# GazeTR 경로 추가 (별도로 clone: https://github.com/yihuacheng/GazeTR)
sys.path.insert(0, os.path.expanduser('~/GazeTR'))
from model import Model

# PureGaze 경로 추가 (별도로 clone: https://github.com/yihuacheng/PureGaze, model/ 하위에 model.py+modules.py)
# GazeTR의 `from model import Model`과 모듈 이름이 겹치므로 별도 함수 스코프에서 늦게 import한다
# (모듈 캐시 오염 방지 — 자세한 이유는 _load_puregaze_model_class() 참고).
PUREGAZE_MODEL_DIR = os.path.expanduser('~/PureGaze/model')
import importlib.util as _il_util


def _load_puregaze_model_class():
    """PureGaze의 model.py도 파일명이 'model.py'라 위에서 이미 실행된
    `from model import Model`(GazeTR)과 sys.modules 이름이 겹친다 — 그냥 import하면
    캐시된 GazeTR의 model 모듈이 재사용되어 PureGaze 클래스를 가져오지 못한다.
    importlib으로 별도 이름('puregaze_model')에 명시적으로 로드해서 충돌을 피한다.
    model.py 내부의 `import modules`가 풀리려면 PUREGAZE_MODEL_DIR이 sys.path에
    있어야 하므로 그것도 여기서 보장한다. (2026-08-27 최초 통합 시엔 카메라 미연결이라
    더미 forward pass까지만 검증했으나, 같은 날 저녁 D455+Kinect 실카메라로
    2분+ 안정 동작·정상 gaze 발행까지 확인 완료 — run_dual_gaze_bridge_puregaze.sh 참고)"""
    if PUREGAZE_MODEL_DIR not in sys.path:
        sys.path.insert(0, PUREGAZE_MODEL_DIR)
    spec = _il_util.spec_from_file_location(
        'puregaze_model', os.path.join(PUREGAZE_MODEL_DIR, 'model.py'))
    module = _il_util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.Model

from geometry_msgs.msg import TransformStamped, PointStamped, Vector3Stamped
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import tf2_ros
import tf2_geometry_msgs
import message_filters

MODEL_PATH = os.path.expanduser("~/face_landmarker.task")
if not os.path.exists(MODEL_PATH):
    print("모델 다운로드 중...")
    urllib.request.urlretrieve(
        "https://storage.googleapis.com/mediapipe-models/face_landmarker/face_landmarker/float16/1/face_landmarker.task",
        MODEL_PATH
    )
    print("다운로드 완료!")

SMOOTH_N = 10
MAP_FRAME = 'map'  # rtabmap map frame (try '/map' if markers don't appear)


class GazeBridgeNode(Node):
    def __init__(self):
        # 노드 이름은 고정('gaze_bridge_node') — 카메라별로 두 개 동시에 띄울 때는
        # 실행 인자로 `--ros-args -r __node:=gaze_bridge_kinect -p camera_name:=kinect`처럼
        # 이름을 리매핑한다 (scripts/run_dual_gaze_bridge.sh 참고).
        super().__init__('gaze_bridge_node')

        self.declare_parameter('camera_name', 'kinect')
        self.declare_parameter('rgb_topic', '/kinect/rgb/image_raw')
        self.declare_parameter('depth_topic', '/kinect/depth/image_raw')
        self.declare_parameter('camera_info_topic', '/kinect/rgb/camera_info')
        self.declare_parameter('tf_frame', 'kinect_rgb_optical_frame')
        self.declare_parameter('show_window', True)
        # 'gazetr'(기존, 기본값) 또는 'puregaze'(2026-08-27 통합, 같은 날 실카메라 검증 완료) —
        # 기본값을 안 건드려서 명시적으로 opt-in 하지 않는 한 기존 동작 그대로 유지.
        self.declare_parameter('gaze_model', 'gazetr')

        self.camera_name = self.get_parameter('camera_name').value
        self.rgb_topic = self.get_parameter('rgb_topic').value
        self.depth_topic = self.get_parameter('depth_topic').value
        self.camera_info_topic = self.get_parameter('camera_info_topic').value
        self.tf_frame = self.get_parameter('tf_frame').value
        self.show_window = self.get_parameter('show_window').value
        self.gaze_model_name = self.get_parameter('gaze_model').value
        self.window_name = f"Gaze Bridge ({self.camera_name})"

        self.bridge = CvBridge()
        self.intrinsics = None
        self.real_camera_info = None

        # GazeTR 모델 로드 (pretrained checkpoint 필수 - 없으면 랜덤 초기화 가중치로 추론하게 됨)
        self.get_logger().info("GazeTR 모델 로딩 중...")
        self.gazetr = Model()
        checkpoint_path = os.path.expanduser('~/Downloads/GazeTR-H-ETH.pt')
        state_dict = torch.load(checkpoint_path, map_location='cpu', weights_only=False)
        self.gazetr.load_state_dict(state_dict)
        self.gazetr.eval()
        self.get_logger().info(f"GazeTR 모델 로드 완료! (checkpoint: {checkpoint_path})")

        # PureGaze 모델 (gaze_model:=puregaze 로 명시했을 때만 로드 — 기본값 'gazetr'일 땐
        # 아예 안 건드려서 기존 파이프라인과 100% 동일하게 유지)
        self.puregaze = None
        if self.gaze_model_name == 'puregaze':
            self.get_logger().info("PureGaze 모델 로딩 중...")
            PureGazeModel = _load_puregaze_model_class()
            self.puregaze = PureGazeModel()
            pg_checkpoint_path = os.path.expanduser('~/Downloads/PureGaze-Res50-ETH.pt')
            pg_state_dict = torch.load(pg_checkpoint_path, map_location='cpu', weights_only=False)
            self.puregaze.load_state_dict(pg_state_dict)  # strict=True 기본값 — 키 472개 완전 일치 확인됨(2026-08-27)
            self.puregaze.eval()
            self.get_logger().info(f"PureGaze 모델 로드 완료! (checkpoint: {pg_checkpoint_path})")
        self.get_logger().info(f"사용할 gaze 모델: {self.gaze_model_name}")

        # 스무딩 버퍼
        self.gaze_buf = deque(maxlen=SMOOTH_N)

        # TF
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # 퍼블리셔 — camera_name으로 네임스페이스 분리 (두 인스턴스 동시 실행 대비).
        # 정규 토픽(/head_position 등, 네임스페이스 없음)은 gaze_fusion_node.py가
        # 이 토픽들을 구독해서 합친 뒤 자기 이름으로 발행한다.
        ns = f"/{self.camera_name}"
        self.head_pub = self.create_publisher(PointStamped, f'{ns}/head_position', 10)
        self.marker_pub = self.create_publisher(MarkerArray, f'{ns}/head_gaze_markers', 10)
        self.gaze_origin_pub = self.create_publisher(PointStamped, f'{ns}/gaze_origin', 10)
        self.gaze_direction_pub = self.create_publisher(Vector3Stamped, f'{ns}/gaze_direction', 10)
        # gaze 모델(GazeTR/PureGaze) 원본 출력(카메라 로컬 프레임, EMA 스무딩/world 변환 전) — 정량 평가용.
        # /gaze_direction은 스무딩+SLAM 변환을 거쳐서 모델 자체 정확도만 분리해서 못 봄.
        self.gaze_raw_pub = self.create_publisher(Vector3Stamped, f'{ns}/gaze_raw', 10)
        # head_position은 map 프레임(SLAM 변환 후)이라 카메라 기준 실측 타겟 위치랑 바로 못
        # 비교함 — gaze_raw와 같은 카메라 로컬 프레임의 머리 위치도 별도로 발행(정량 평가용).
        self.head_position_cam_pub = self.create_publisher(
            PointStamped, f'{ns}/head_position_cam', 10)

        # 가상 카메라 퍼블리셔 (TF/CameraInfo만; image_raw는 vpt_virtual_camera 노드가 렌더링해서 퍼블리시)
        self.virtual_cam_info_pub = self.create_publisher(
            CameraInfo, f'{ns}/virtual_camera/camera_info', 10)

        # MediaPipe
        base_options = python.BaseOptions(model_asset_path=MODEL_PATH)
        options = vision.FaceLandmarkerOptions(
            base_options=base_options,
            output_face_blendshapes=False,
            output_facial_transformation_matrixes=True,
            num_faces=1
        )
        self.detector = vision.FaceLandmarker.create_from_options(options)

        # 구독 — 토픽/프레임은 전부 파라미터화 (2026-08-20, #21 VPT: 두 카메라 동시 운용)
        self.create_subscription(CameraInfo, self.camera_info_topic,
                                  self.camera_info_callback, 10)

        color_sub = message_filters.Subscriber(self, Image, self.rgb_topic)
        depth_sub = message_filters.Subscriber(self, Image, self.depth_topic)
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [color_sub, depth_sub], queue_size=10, slop=0.1)
        self.sync.registerCallback(self.image_callback)

        self.get_logger().info(f"Gaze Bridge Node 시작! (camera_name={self.camera_name}, "
                                f"tf_frame={self.tf_frame})")
        self.get_logger().info(f"구독: {self.rgb_topic}, {self.depth_topic}, {self.camera_info_topic}")
        self.get_logger().info(f"발행: {ns}/head_position, {ns}/gaze_origin, {ns}/gaze_direction "
                                "(fusion은 gaze_fusion_node.py가 담당, ray casting은 vpt_raycasting)")

    def camera_info_callback(self, msg):
        if self.intrinsics is None:
            self.intrinsics = {
                'fx': msg.k[0], 'fy': msg.k[4],
                'cx': msg.k[2], 'cy': msg.k[5],
                'width': msg.width, 'height': msg.height
            }
            self.real_camera_info = msg
            self.get_logger().info("카메라 파라미터 수신!")

    def pixel_to_3d(self, u, v, depth):
        if self.intrinsics is None:
            return None
        x = (u - self.intrinsics['cx']) * depth / self.intrinsics['fx']
        y = (v - self.intrinsics['cy']) * depth / self.intrinsics['fy']
        return np.array([x, y, depth])

    def transform_to_map(self, point, frame_id, stamp=None):
        try:
            transform = self.tf_buffer.lookup_transform(
                MAP_FRAME, frame_id,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.3)
            )
            ps = PointStamped()
            ps.header.frame_id = frame_id
            ps.header.stamp = transform.header.stamp
            ps.point.x, ps.point.y, ps.point.z = float(point[0]), float(point[1]), float(point[2])
            return tf2_geometry_msgs.do_transform_point(ps, transform)
        except Exception as e:
            self.get_logger().warn(f"TF 변환 실패: {e}")
            return None

    def get_gaze_vector_from_matrix(self, transform_matrix):
        """head pose 기반 gaze vector (fallback용)"""
        R = np.array(transform_matrix).reshape(4, 4)[:3, :3]
        gaze = R @ np.array([0, 0, -1])
        return gaze / np.linalg.norm(gaze)

    def get_gaze_vector_gazetr(self, face_img):
        """GazeTR으로 gaze vector 추정 (pitch, yaw → 3D vector)"""
        try:
            face_resized = cv2.resize(face_img, (224, 224))
            face_tensor = torch.from_numpy(face_resized).float()
            face_tensor = face_tensor.permute(2, 0, 1)  # HWC → CHW
            face_tensor = face_tensor / 255.0
            face_tensor = face_tensor.unsqueeze(0)

            with torch.no_grad():
                gaze = self.gazetr({'face': face_tensor})

            pitch = gaze[0][0].item()
            yaw = gaze[0][1].item()

            # pitch, yaw → 3D gaze vector
            gx = np.cos(pitch) * np.sin(yaw)
            gy = -np.sin(pitch)
            gz = np.cos(pitch) * np.cos(yaw)
            gaze_vec = np.array([gx, gy, gz])
            return gaze_vec / np.linalg.norm(gaze_vec)
        except Exception as e:
            self.get_logger().warn(f"GazeTR 추론 실패: {e}")
            return None

    def get_gaze_vector_puregaze(self, face_img):
        """PureGaze로 gaze vector 추정 (pitch, yaw → 3D vector).

        전처리/후처리는 get_gaze_vector_gazetr()과 동일하게 재사용한다 — 두 모델 다
        GazeHub이 배포하는 동일한 ETH-XGaze 정규화 얼굴crop(224x224) 포맷으로 학습됐고,
        PureGaze 공식 gazeto3d()도 "ETH는 [pitch yaw]"라고 명시(GazeTR과 같은 컨벤션).
        입력 텐서 dict key('face')와 리턴 shape([N,2])도 GazeTR과 동일하게 확인함.
        2026-08-27 D455+Kinect 실카메라로 2분+ 검증 — 항상 정상 정규화 벡터 출력,
        예외/NaN 없음. 다만 절대 각도 정확도는 아이트래킹 ground-truth 없이는 판단
        불가 — 방향이 프레임마다 흔들리는 정도가 정상 범위인지는 IRB 참가자 실험으로
        확인해야 함.
        """
        try:
            face_resized = cv2.resize(face_img, (224, 224))
            face_tensor = torch.from_numpy(face_resized).float()
            face_tensor = face_tensor.permute(2, 0, 1)  # HWC → CHW
            face_tensor = face_tensor / 255.0
            face_tensor = face_tensor.unsqueeze(0)

            with torch.no_grad():
                gaze, _ = self.puregaze({'face': face_tensor}, require_img=False)

            pitch = gaze[0][0].item()
            yaw = gaze[0][1].item()

            # pitch, yaw → 3D gaze vector (get_gaze_vector_gazetr()과 동일한 변환식)
            gx = np.cos(pitch) * np.sin(yaw)
            gy = -np.sin(pitch)
            gz = np.cos(pitch) * np.cos(yaw)
            gaze_vec = np.array([gx, gy, gz])
            return gaze_vec / np.linalg.norm(gaze_vec)
        except Exception as e:
            self.get_logger().warn(f"PureGaze 추론 실패: {e}")
            return None

    def publish_virtual_camera_tf(self, head_map, gaze_vec_map):
        """머리 위치에 가상 카메라 TF 퍼블리시"""
        now = self.get_clock().now().to_msg()

        # head_position TF (머리 위치) — camera_name으로 프레임 이름 분리 (두 인스턴스 동시
        # 실행 시 같은 자식 프레임에 서로 다른 값을 발행하는 충돌 방지). 정규 'head_position'
        # 프레임은 gaze_fusion_node.py가 두 인스턴스를 합친 뒤 발행한다.
        t = TransformStamped()
        t.header.stamp = now
        t.header.frame_id = MAP_FRAME
        t.child_frame_id = f'{self.camera_name}_head_position'
        t.transform.translation.x = head_map.point.x
        t.transform.translation.y = head_map.point.y
        t.transform.translation.z = head_map.point.z

        # gaze vector로 회전 계산 (z축이 gaze 방향을 향하도록)
        z_axis = gaze_vec_map
        up = np.array([0, 0, 1])
        if abs(np.dot(z_axis, up)) > 0.99:
            up = np.array([0, 1, 0])
        x_axis = np.cross(up, z_axis)
        x_axis = x_axis / np.linalg.norm(x_axis)
        y_axis = -np.cross(z_axis, x_axis)

        R = np.column_stack([x_axis, y_axis, z_axis])

        # 회전 행렬 → 쿼터니언
        trace = R[0, 0] + R[1, 1] + R[2, 2]
        if trace > 0:
            s = 0.5 / np.sqrt(trace + 1.0)
            w = 0.25 / s
            x = (R[2, 1] - R[1, 2]) * s
            y = (R[0, 2] - R[2, 0]) * s
            z = (R[1, 0] - R[0, 1]) * s
        else:
            w, x, y, z = 1.0, 0.0, 0.0, 0.0

        t.transform.rotation.x = x
        t.transform.rotation.y = y
        t.transform.rotation.z = z
        t.transform.rotation.w = w

        self.tf_broadcaster.sendTransform(t)

        # 가상 카메라 CameraInfo 퍼블리시
        if self.real_camera_info is not None:
            cam_info = CameraInfo()
            cam_info.header.stamp = now
            cam_info.header.frame_id = f'{self.camera_name}_head_position'
            cam_info.width = self.real_camera_info.width
            cam_info.height = self.real_camera_info.height
            cam_info.k = self.real_camera_info.k
            cam_info.d = self.real_camera_info.d
            cam_info.r = self.real_camera_info.r
            cam_info.p = self.real_camera_info.p
            cam_info.distortion_model = self.real_camera_info.distortion_model
            self.virtual_cam_info_pub.publish(cam_info)

    def publish_head_marker(self, head_map):
        markers = MarkerArray()
        now = self.get_clock().now().to_msg()

        head_marker = Marker()
        head_marker.header.frame_id = MAP_FRAME
        head_marker.header.stamp = now
        head_marker.ns = 'head'
        head_marker.id = 0
        head_marker.type = Marker.SPHERE
        head_marker.action = Marker.ADD
        head_marker.pose.position.x = head_map.point.x
        head_marker.pose.position.y = head_map.point.y
        head_marker.pose.position.z = head_map.point.z
        head_marker.pose.orientation.w = 1.0
        head_marker.scale.x = head_marker.scale.y = head_marker.scale.z = 0.2
        head_marker.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=0.8)
        head_marker.lifetime.sec = 1
        markers.markers.append(head_marker)

        self.marker_pub.publish(markers)

    def image_callback(self, color_msg, depth_msg):
        frame = self.bridge.imgmsg_to_cv2(color_msg, 'bgr8')
        depth = self.bridge.imgmsg_to_cv2(depth_msg, '16UC1')
        frame_h, frame_w = frame.shape[:2]

        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb)
        results = self.detector.detect(mp_image)

        if results.face_landmarks and results.facial_transformation_matrixes:
            landmarks = results.face_landmarks[0]
            transform_matrix = results.facial_transformation_matrixes[0]

            nose = landmarks[1]
            u = max(0, min(int(nose.x * frame_w), frame_w - 1))
            v = max(0, min(int(nose.y * frame_h), frame_h - 1))
            d = depth[v, u] / 1000.0

            if 0.1 < d < 5.0:
                head_cam = self.pixel_to_3d(u, v, d)
                head_map = self.transform_to_map(head_cam, self.tf_frame)

                head_cam_msg = PointStamped()
                head_cam_msg.header.frame_id = self.tf_frame
                head_cam_msg.header.stamp = self.get_clock().now().to_msg()
                head_cam_msg.point.x, head_cam_msg.point.y, head_cam_msg.point.z = (
                    float(head_cam[0]), float(head_cam[1]), float(head_cam[2]))
                self.head_position_cam_pub.publish(head_cam_msg)

                if head_map is not None:
                    self.head_pub.publish(head_map)

                    # Gaze vector - gaze_model 파라미터로 GazeTR/PureGaze 중 선택 (fallback: head pose)
                    face_crop = frame[max(0, v - 80):min(frame_h, v + 80), max(0, u - 60):min(frame_w, u + 60)]
                    if face_crop.size > 0:
                        if self.gaze_model_name == 'puregaze':
                            model_vec = self.get_gaze_vector_puregaze(face_crop)
                        else:
                            model_vec = self.get_gaze_vector_gazetr(face_crop)
                    else:
                        model_vec = None

                    if model_vec is not None:
                        raw_gaze = model_vec
                        self.get_logger().info(f"{self.gaze_model_name} gaze: {raw_gaze}")

                        raw_msg = Vector3Stamped()
                        raw_msg.header.frame_id = self.tf_frame
                        raw_msg.header.stamp = self.get_clock().now().to_msg()
                        raw_msg.vector.x, raw_msg.vector.y, raw_msg.vector.z = (
                            float(model_vec[0]), float(model_vec[1]), float(model_vec[2]))
                        self.gaze_raw_pub.publish(raw_msg)
                    else:
                        raw_gaze = self.get_gaze_vector_from_matrix(transform_matrix.data)

                    self.gaze_buf.append(raw_gaze)
                    gaze_vec = np.mean(self.gaze_buf, axis=0)
                    gaze_vec = gaze_vec / np.linalg.norm(gaze_vec)

                    # Gaze vector → map 좌표계 변환
                    try:
                        transform = self.tf_buffer.lookup_transform(
                            MAP_FRAME, self.tf_frame,
                            rclpy.time.Time(),
                            timeout=rclpy.duration.Duration(seconds=0.1)
                        )
                        v3 = Vector3Stamped()
                        v3.header.frame_id = self.tf_frame
                        v3.vector.x = float(gaze_vec[0])
                        v3.vector.y = float(gaze_vec[1])
                        v3.vector.z = float(gaze_vec[2])
                        v3_map = tf2_geometry_msgs.do_transform_vector3(v3, transform)
                        gaze_map = np.array([v3_map.vector.x, v3_map.vector.y, v3_map.vector.z])
                        gaze_map = gaze_map / np.linalg.norm(gaze_map)
                    except Exception:
                        gaze_map = gaze_vec

                    # 가상 카메라 TF + CameraInfo 퍼블리시
                    self.publish_virtual_camera_tf(head_map, gaze_map)
                    self.publish_head_marker(head_map)

                    # ray casting은 vpt_raycasting 노드가 담당 -> origin/direction만 퍼블리시
                    now = self.get_clock().now().to_msg()

                    origin_msg = PointStamped()
                    origin_msg.header.frame_id = MAP_FRAME
                    origin_msg.header.stamp = now
                    origin_msg.point.x, origin_msg.point.y, origin_msg.point.z = (
                        head_map.point.x, head_map.point.y, head_map.point.z)
                    self.gaze_origin_pub.publish(origin_msg)

                    direction_msg = Vector3Stamped()
                    direction_msg.header.frame_id = MAP_FRAME
                    direction_msg.header.stamp = now
                    direction_msg.vector.x, direction_msg.vector.y, direction_msg.vector.z = (
                        float(gaze_map[0]), float(gaze_map[1]), float(gaze_map[2]))
                    self.gaze_direction_pub.publish(direction_msg)

            cv2.circle(frame, (u, v), 10, (0, 255, 0), -1)
        else:
            cv2.putText(frame, "얼굴 감지 안됨", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

        if self.show_window:
            cv2.imshow(self.window_name, frame)
            cv2.waitKey(1)

        # 가상 카메라 이미지는 이 노드가 퍼블리시하지 않는다.
        # /cloud_map 기반 렌더링은 vpt_virtual_camera 노드가 담당 (head_position TF는 위에서 이미 퍼블리시함)

    def destroy_node(self):
        cv2.destroyAllWindows()
        self.detector.close()
        super().destroy_node()


def main():
    rclpy.init()
    node = GazeBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
