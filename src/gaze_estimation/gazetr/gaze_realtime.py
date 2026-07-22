"""
GazeTR 실시간 테스트
- RealSense D455 카메라에서 얼굴 이미지 받기
- MediaPipe로 얼굴 crop
- GazeTR에 넣어서 gaze (pitch, yaw) 출력
"""

import sys
import os
sys.path.insert(0, os.path.expanduser('~/GazeTR'))

import cv2
import numpy as np
import torch
import mediapipe as mp
from mediapipe.tasks import python
from mediapipe.tasks.python import vision
import pyrealsense2 as rs

from model import Model

# GazeTR 모델 로드
print("GazeTR 모델 로딩 중...")
gazetr = Model()
gazetr.eval()
print("GazeTR 모델 로드 완료!")

# MediaPipe 얼굴 감지
MODEL_PATH = os.path.expanduser("~/face_landmarker.task")
base_options = python.BaseOptions(model_asset_path=MODEL_PATH)
options = vision.FaceLandmarkerOptions(
    base_options=base_options,
    output_face_blendshapes=False,
    output_facial_transformation_matrixes=False,
    num_faces=1
)
detector = vision.FaceLandmarker.create_from_options(options)

# RealSense 설정
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
pipeline.start(config)

print("실행 중... q 키로 종료")

def crop_face(frame, landmarks, frame_w, frame_h, margin=0.3):
    """얼굴 랜드마크로 얼굴 영역 crop"""
    xs = [lm.x * frame_w for lm in landmarks]
    ys = [lm.y * frame_h for lm in landmarks]
    
    x_min, x_max = int(min(xs)), int(max(xs))
    y_min, y_max = int(min(ys)), int(max(ys))
    
    w = x_max - x_min
    h = y_max - y_min
    
    # margin 추가
    x_min = max(0, int(x_min - w * margin))
    x_max = min(frame_w, int(x_max + w * margin))
    y_min = max(0, int(y_min - h * margin))
    y_max = min(frame_h, int(y_max + h * margin))
    
    face = frame[y_min:y_max, x_min:x_max]
    return face, (x_min, y_min, x_max, y_max)

try:
    while True:
        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        if not color_frame:
            continue
        
        frame = np.asanyarray(color_frame.get_data())
        frame_h, frame_w = frame.shape[:2]
        
        # MediaPipe 얼굴 감지
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb)
        results = detector.detect(mp_image)
        
        if results.face_landmarks:
            landmarks = results.face_landmarks[0]
            
            # 얼굴 crop
            face_img, (x1, y1, x2, y2) = crop_face(frame, landmarks, frame_w, frame_h)
            
            if face_img.size > 0:
                # 224x224 resize
                face_resized = cv2.resize(face_img, (224, 224))
                
                # GazeTR 입력 형식으로 변환
                face_tensor = torch.from_numpy(face_resized).float()
                face_tensor = face_tensor.permute(2, 0, 1)  # HWC → CHW
                face_tensor = face_tensor / 255.0  # 정규화
                face_tensor = face_tensor.unsqueeze(0)  # batch 차원 추가
                
                # GazeTR 추론
                with torch.no_grad():
                    gaze = gazetr({'face': face_tensor})
                
                pitch = gaze[0][0].item()
                yaw = gaze[0][1].item()
                
                # 화면에 표시
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.putText(frame, f"Pitch: {pitch:.3f}", (10, 30),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                cv2.putText(frame, f"Yaw:   {yaw:.3f}", (10, 60),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                
                print(f"Pitch: {pitch:.3f}, Yaw: {yaw:.3f}")
        else:
            cv2.putText(frame, "얼굴 감지 안됨", (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
        
        cv2.imshow("GazeTR Realtime", frame)
        
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

finally:
    pipeline.stop()
    cv2.destroyAllWindows()
    detector.close()
