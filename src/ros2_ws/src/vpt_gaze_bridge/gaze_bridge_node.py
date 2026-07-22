"""
Gaze Bridge Node
- 카메라 이미지(RGB-D) 구독, MediaPipe로 얼굴/머리 위치 검출
- GazeTR(fallback: head pose 행렬)로 gaze vector 추정
- 머리 위치 + gaze 방향을 map 좌표계로 변환해 퍼블리시
- 머리 위치에 가상 카메라 TF/CameraInfo/Image 퍼블리시

Ray casting(포인트클라우드와의 교차점 G_t 계산)은 이 노드가 하지 않는다.
/gaze_origin, /gaze_direction 을 퍼블리시하고, vpt_raycasting 노드가 구독해서
G_t를 계산한다. 이유는 docs/decisions.md 참고.
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

# GazeTR 경로 추가 (별도로 clone: https://github.com/yihuacheng/GazeTR)
sys.path.insert(0, os.path.expanduser('~/GazeTR'))
from model import Model

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
        super().__init__('gaze_bridge_node')

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

        # 스무딩 버퍼
        self.gaze_buf = deque(maxlen=SMOOTH_N)

        # TF
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # 퍼블리셔
        self.head_pub = self.create_publisher(PointStamped, '/head_position', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/head_gaze_markers', 10)
        self.gaze_origin_pub = self.create_publisher(PointStamped, '/gaze_origin', 10)
        self.gaze_direction_pub = self.create_publisher(Vector3Stamped, '/gaze_direction', 10)

        # 가상 카메라 퍼블리셔
        self.virtual_cam_info_pub = self.create_publisher(
            CameraInfo, '/virtual_camera/camera_info', 10)
        self.virtual_cam_image_pub = self.create_publisher(
            Image, '/virtual_camera/image_raw', 10)

        # MediaPipe
        base_options = python.BaseOptions(model_asset_path=MODEL_PATH)
        options = vision.FaceLandmarkerOptions(
            base_options=base_options,
            output_face_blendshapes=False,
            output_facial_transformation_matrixes=True,
            num_faces=1
        )
        self.detector = vision.FaceLandmarker.create_from_options(options)

        # 구독
        self.create_subscription(CameraInfo, '/camera/camera/color/camera_info',
                                  self.camera_info_callback, 10)

        color_sub = message_filters.Subscriber(self, Image, '/camera/camera/color/image_raw')
        depth_sub = message_filters.Subscriber(self, Image, '/camera/camera/aligned_depth_to_color/image_raw')
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [color_sub, depth_sub], queue_size=10, slop=0.1)
        self.sync.registerCallback(self.image_callback)

        self.get_logger().info("Gaze Bridge Node 시작!")
        self.get_logger().info("가상 카메라 토픽: /virtual_camera/camera_info, /virtual_camera/image_raw")
        self.get_logger().info("gaze 토픽: /gaze_origin, /gaze_direction (ray casting은 vpt_raycasting 노드가 담당)")

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

    def publish_virtual_camera_tf(self, head_map, gaze_vec_map):
        """머리 위치에 가상 카메라 TF 퍼블리시"""
        now = self.get_clock().now().to_msg()

        # head_position TF (머리 위치)
        t = TransformStamped()
        t.header.stamp = now
        t.header.frame_id = MAP_FRAME
        t.child_frame_id = 'head_position'
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
            cam_info.header.frame_id = 'head_position'
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
                head_map = self.transform_to_map(head_cam, 'camera_color_optical_frame')

                if head_map is not None:
                    self.head_pub.publish(head_map)

                    # Gaze vector - GazeTR 사용 (fallback: head pose)
                    face_crop = frame[max(0, v - 80):min(frame_h, v + 80), max(0, u - 60):min(frame_w, u + 60)]
                    gazetr_vec = self.get_gaze_vector_gazetr(face_crop) if face_crop.size > 0 else None

                    if gazetr_vec is not None:
                        raw_gaze = gazetr_vec
                        self.get_logger().info(f"GazeTR gaze: {raw_gaze}")
                    else:
                        raw_gaze = self.get_gaze_vector_from_matrix(transform_matrix.data)

                    self.gaze_buf.append(raw_gaze)
                    gaze_vec = np.mean(self.gaze_buf, axis=0)
                    gaze_vec = gaze_vec / np.linalg.norm(gaze_vec)

                    # Gaze vector → map 좌표계 변환
                    try:
                        transform = self.tf_buffer.lookup_transform(
                            MAP_FRAME, 'camera_color_optical_frame',
                            rclpy.time.Time(),
                            timeout=rclpy.duration.Duration(seconds=0.1)
                        )
                        v3 = Vector3Stamped()
                        v3.header.frame_id = 'camera_color_optical_frame'
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

        cv2.imshow("Gaze Bridge", frame)
        cv2.waitKey(1)

        # 가상 카메라 이미지 퍼블리시 (실제 카메라 이미지 그대로)
        virtual_img_msg = self.bridge.cv2_to_imgmsg(frame, 'bgr8')
        virtual_img_msg.header.stamp = self.get_clock().now().to_msg()
        virtual_img_msg.header.frame_id = 'head_position'
        self.virtual_cam_image_pub.publish(virtual_img_msg)

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
