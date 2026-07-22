"""
Gaze Estimation Prototype
- MediaPipe FaceLandmarker (새 API) 로 head pose 추출
- Head pose 기반 gaze vector 계산
- RealSense D455 사용
- 스무딩 적용
"""

import cv2
import numpy as np
import mediapipe as mp
from mediapipe.tasks import python
from mediapipe.tasks.python import vision
import pyrealsense2 as rs
import urllib.request
import os
from collections import deque

# MediaPipe FaceLandmarker 모델 다운로드
MODEL_PATH = "face_landmarker.task"
if not os.path.exists(MODEL_PATH):
    print("모델 다운로드 중...")
    urllib.request.urlretrieve(
        "https://storage.googleapis.com/mediapipe-models/face_landmarker/face_landmarker/float16/1/face_landmarker.task",
        MODEL_PATH
    )
    print("모델 다운로드 완료!")

FACE_3D_MODEL = np.array([
    [0.0, 0.0, 0.0],
    [0.0, -330.0, -65.0],
    [-225.0, 170.0, -135.0],
    [225.0, 170.0, -135.0],
    [-150.0, -150.0, -125.0],
    [150.0, -150.0, -125.0],
], dtype=np.float64)

FACE_2D_INDICES = [1, 152, 33, 263, 61, 291]

# 스무딩용 버퍼 (최근 N 프레임 평균)
SMOOTH_N = 10
pitch_buf = deque(maxlen=SMOOTH_N)
yaw_buf = deque(maxlen=SMOOTH_N)
roll_buf = deque(maxlen=SMOOTH_N)


def get_head_pose(landmarks, frame_w, frame_h):
    face_2d = []
    for idx in FACE_2D_INDICES:
        lm = landmarks[idx]
        x = lm.x * frame_w
        y = lm.y * frame_h
        face_2d.append([x, y])

    face_2d = np.array(face_2d, dtype=np.float64)

    focal_length = frame_w
    cam_matrix = np.array([
        [focal_length, 0, frame_w / 2],
        [0, focal_length, frame_h / 2],
        [0, 0, 1]
    ], dtype=np.float64)

    dist_matrix = np.zeros((4, 1), dtype=np.float64)

    success, rot_vec, trans_vec = cv2.solvePnP(
        FACE_3D_MODEL, face_2d, cam_matrix, dist_matrix
    )

    if not success:
        return None, None, None

    rmat, _ = cv2.Rodrigues(rot_vec)
    angles, _, _, _, _, _ = cv2.RQDecomp3x3(rmat)
    pitch = angles[0] * 360
    yaw = angles[1] * 360
    roll = angles[2] * 360

    # 스무딩 적용
    pitch_buf.append(pitch)
    yaw_buf.append(yaw)
    roll_buf.append(roll)

    smooth_pitch = np.mean(pitch_buf)
    smooth_yaw = np.mean(yaw_buf)
    smooth_roll = np.mean(roll_buf)

    return smooth_pitch, smooth_yaw, smooth_roll


def get_gaze_vector(pitch, yaw):
    pitch_rad = np.radians(pitch)
    yaw_rad = np.radians(yaw)

    gx = np.cos(pitch_rad) * np.sin(yaw_rad)
    gy = -np.sin(pitch_rad)
    gz = np.cos(pitch_rad) * np.cos(yaw_rad)

    return np.array([gx, gy, gz])


def draw_gaze_arrow(frame, landmarks, gaze_vec, frame_w, frame_h):
    nose = landmarks[1]
    nose_x = int(nose.x * frame_w)
    nose_y = int(nose.y * frame_h)

    arrow_len = 200
    end_x = int(nose_x + gaze_vec[0] * arrow_len)
    end_y = int(nose_y - gaze_vec[1] * arrow_len)

    cv2.arrowedLine(frame, (nose_x, nose_y), (end_x, end_y),
                    (0, 255, 0), 3, tipLength=0.3)


def main():
    base_options = python.BaseOptions(model_asset_path=MODEL_PATH)
    options = vision.FaceLandmarkerOptions(
        base_options=base_options,
        output_face_blendshapes=False,
        output_facial_transformation_matrixes=True,
        num_faces=1
    )
    detector = vision.FaceLandmarker.create_from_options(options)

    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    pipeline.start(config)

    print("=== Gaze Estimation Prototype (RealSense D455) ===")
    print("q 키: 종료")
    print("s 키: 현재 gaze vector 출력")

    try:
        while True:
            frames = pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()
            if not color_frame:
                continue

            frame = np.asanyarray(color_frame.get_data())
            frame_h, frame_w = frame.shape[:2]

            rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb_frame)
            results = detector.detect(mp_image)

            gaze_vec = None

            if results.face_landmarks:
                landmarks = results.face_landmarks[0]

                pitch, yaw, roll = get_head_pose(landmarks, frame_w, frame_h)
                if pitch is not None:
                    gaze_vec = get_gaze_vector(pitch, yaw)
                    draw_gaze_arrow(frame, landmarks, gaze_vec, frame_w, frame_h)

                    cv2.putText(frame, f"Pitch: {pitch:.1f}", (10, 30),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
                    cv2.putText(frame, f"Yaw:   {yaw:.1f}", (10, 60),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
                    cv2.putText(frame, f"Roll:  {roll:.1f}", (10, 90),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
                    cv2.putText(frame, f"Gaze: [{gaze_vec[0]:.2f}, {gaze_vec[1]:.2f}, {gaze_vec[2]:.2f}]",
                                (10, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

            cv2.imshow("Gaze Estimation Prototype", frame)

            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break
            elif key == ord('s') and gaze_vec is not None:
                print(f"\nGaze Vector: {gaze_vec}")
                print(f"Pitch: {pitch:.2f}, Yaw: {yaw:.2f}, Roll: {roll:.2f}")

    finally:
        pipeline.stop()
        cv2.destroyAllWindows()
        detector.close()


if __name__ == "__main__":
    main()
