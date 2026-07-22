# VPT Research

Multi-view 3D Visual Perspective-Taking (VPT) framework for HRI.
듀얼 Intel RealSense D455, RTAB-Map, MediaPipe, GazeTR, eye-tracking glasses(ground-truth) 기반.

## 구조

```
vpt-research/
├── docs/
│   └── decisions.md        # 왜 이렇게 했는지 기록
├── src/
│   ├── ros2_ws/             # ROS2 워크스페이스
│   │   └── src/
│   │       ├── vpt_cameras/       # 듀얼 D455 카메라 노드 (TODO: 아직 구현 안 됨)
│   │       ├── vpt_gaze_bridge/   # GazeTR(conda) <-> ROS2 브릿지, /gaze_origin, /gaze_direction 퍼블리시
│   │       └── vpt_raycasting/    # /cloud_map + gaze origin/direction 구독 -> G_t(ground-truth gaze point) 계산
│   ├── gaze_estimation/
│   │   ├── gazetr/                # GazeTR 관련 스크립트 (conda, Python 3.13). GazeTR 본체는 별도 clone 필요
│   │   └── mediapipe_pipeline/    # head pose / facial landmark 프로토타입
│   └── slam/
│       └── rtabmap_configs/
├── scripts/                # 셋업 및 실행 스크립트
├── configs/
│   └── camera_calibration/ # RealSense 캘리브레이션/프리셋
└── data/                   # git 추적 안 함, 로컬 경로만 안내
```

`vpt_gaze_bridge`와 `vpt_raycasting`은 별개 노드로 분리되어 있다 (근거: `docs/decisions.md`).
`vpt_gaze_bridge`가 `/gaze_origin`, `/gaze_direction`을 퍼블리시하면 `vpt_raycasting`이 `/cloud_map`과
동기화해서 `/gaze_point`(G_t)를 계산한다.

## 환경 셋업

### ROS2 (시스템 Python 3.12)

```bash
cd src/ros2_ws
colcon build
source install/setup.bash
```

### GazeTR (conda, Python 3.13)

```bash
conda env create -f src/gaze_estimation/gazetr/environment.yml
conda activate gazetr
```

> **주의:** GazeTR(conda)과 ROS2(시스템 Python)는 별도 프로세스로 실행.
> ROS2 명령어 실행 전 반드시 `conda deactivate` . 자세한 이유는 `docs/decisions.md` 참고.

## 실행 순서

```bash
# 1. 카메라 노드 실행 (터미널 1, 시스템 Python)
./scripts/run_dual_camera.sh

# 2. GazeTR + ROS2 브릿지 실행 (터미널 2, conda -> deactivate -> ROS2 순서)
./scripts/bridge_gazetr_ros2.sh
```

## 데이터

`data/` 폴더는 git에서 제외됩니다. 카메라 원본 녹화, GazeTR pretrained 가중치 등은 로컬에 별도 보관.
