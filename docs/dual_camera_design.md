# 듀얼 카메라 설계

## 문제

RealSense D455 한 대로 SLAM(환경 매핑)과 얼굴/gaze 검출을 동시에 처리할 수 없다.

**증거 (2026-07-28 재현, 총 3회):**
- 카메라가 얼굴을 클로즈업으로 잡는 순간 `rgbd_odometry`가 특징점 매칭에 실패:
  ```
  OdometryF2M.cpp:622 Registration failed: "Not enough inliers 0/20"
  ```
- odometry quality가 0으로 떨어지고 회복되지 않음 → `map` ↔ `camera_color_optical_frame` TF 트리가 끊김 → `head_position` 계산 불가
- 세 번 모두 같은 패턴: 환경(방)을 보고 있을 때는 quality 250~400으로 정상, 얼굴이 프레임을 채우는 순간 즉시 quality 0

**원인:** SLAM은 환경의 다양한 특징점이 필요하고, 얼굴 클로즈업은 텍스처가 부족하고 프레임 대부분을 차지해 특징점 매칭 기준(방 전체 대비)이 완전히 바뀌어버림. 카메라 1대로는 두 요구사항이 근본적으로 충돌.

기존에도 같은 문제가 있었음 (#18 VPT, 7/22): *"카메라 1대로 SLAM(환경 특징점 필요)과 얼굴 검출(사람 클로즈업)을 동시에 못 함 → odometry tracking 깨짐. 듀얼 D455 설계가 왜 필요한지 실증됨"*

## 제안하는 구조

카메라 역할을 완전히 분리:

| | Camera A (SLAM) | Camera B (Face/Gaze) |
|---|---|---|
| 역할 | 환경 매핑, odometry, localization | 얼굴 검출, GazeTR, head pose |
| 방향 | 방/환경을 향함 (고정 또는 로봇에 장착) | 사람을 향함 |
| 구독 노드 | `rgbd_odometry`, `rtabmap` | `gaze_bridge_node` |
| 퍼블리시 | `/cam_slam/...`, `/rtabmap/cloud_map`, `map` TF | `/cam_face/...`, `/head_position`, `/gaze_origin`, `/gaze_direction` |

이렇게 하면 얼굴이 아무리 클로즈업으로 잡혀도 Camera A의 odometry는 영향을 받지 않는다.

## 구현 계획

### 1. `vpt_cameras` 패키지 (현재 비어있음 — `.gitkeep`만 존재)

두 D455를 시리얼 넘버로 구분해서 독립된 네임스페이스로 띄우는 launch 파일 필요.

```bash
# 시리얼 넘버 확인
rs-enumerate-devices | grep "Serial Number"
```

```python
# vpt_cameras/launch/dual_camera.launch.py (스케치, 아직 미구현)
IncludeLaunchDescription(rs_launch.py,
    launch_arguments={
        'camera_name': 'cam_slam',
        'camera_namespace': 'cam_slam',
        'serial_no': '<SLAM용 D455 시리얼>',
        'align_depth.enable': 'true',
    }.items()),

IncludeLaunchDescription(rs_launch.py,
    launch_arguments={
        'camera_name': 'cam_face',
        'camera_namespace': 'cam_face',
        'serial_no': '<얼굴용 D455 시리얼>',
        'align_depth.enable': 'true',
    }.items()),
```

`serial_no`를 지정하지 않으면 두 장치가 같은 기본 네임스페이스(`camera/camera/...`)로 충돌한다 — 오늘 단일 카메라 테스트에서도 기본 네임스페이스가 이중으로 겹치는 걸 확인함(`/camera/camera/color/...`).

### 2. RTAB-Map은 Camera A만 구독

```
rgb_topic:=/cam_slam/cam_slam/color/image_raw
depth_topic:=/cam_slam/cam_slam/aligned_depth_to_color/image_raw
camera_info_topic:=/cam_slam/cam_slam/color/camera_info
```

### 3. `gaze_bridge_node.py`는 Camera B만 구독

현재 코드(`gaze_bridge_node.py:100-107`)는 `/camera/camera/color/...`를 구독 중 — Camera B 네임스페이스로 바꿔야 함:
```python
self.create_subscription(CameraInfo, '/cam_face/cam_face/color/camera_info', ...)
color_sub = message_filters.Subscriber(self, Image, '/cam_face/cam_face/color/image_raw')
depth_sub = message_filters.Subscriber(self, Image, '/cam_face/cam_face/aligned_depth_to_color/image_raw')
```

### 4. 미해결 — 두 카메라 간 외부 파라미터(extrinsic) 필요

`gaze_bridge_node`는 얼굴 위치를 Camera B의 depth로 구한 뒤, TF로 `map` 프레임까지 변환한다 (`transform_to_map(head_cam, 'camera_color_optical_frame')`). 지금은 이 TF 체인이 Camera A(SLAM 카메라) 자신의 광학 프레임을 통해서만 `map`에 연결된다.

Camera B가 물리적으로 분리되면, `map` → ... → `cam_face_color_optical_frame` 체인이 존재하지 않는다. 두 가지 옵션:

- **(a) 고정 마운트 + static TF**: 두 카메라를 알려진 상대 위치로 리지드하게 고정하고, `cam_slam_link` → `cam_face_link` static_transform_publisher 하나 추가. 가장 간단하지만 두 카메라 상대 위치를 실측/캘리브레이션해야 함.
- **(b) Camera B도 자체 localization**: Camera B도 RTAB-Map의 같은 맵에 대해 별도 localization 세션을 돌려서(카메라 B용 두 번째 `rgbd_odometry`+`rtabmap` localization 인스턴스), 자체적으로 `map` 프레임 기준 pose를 얻음. 얼굴 클로즈업 때문에 이것도 실패할 수 있어서 근본 해결책은 아님 — Camera B가 "가끔 환경을 스치듯 보는" 시나리오가 아니면 이 방법도 불안정할 가능성 높음.

→ **(a)가 더 현실적인 1차 시도**로 보임. 실측 필요.

## 남은 질문 (직접 결정 필요)

1. 실제 카메라 2대 있는지, 물리적으로 어떻게 배치할지 (예: 로봇 몸체에 SLAM용, 사람 마주보는 삼각대에 얼굴용?)
2. 두 카메라를 동시에 USB로 연결했을 때 대역폭 문제 없는지 (D455 두 대 + depth+color 스트림은 USB 대역폭을 많이 씀 — 별도 USB 컨트롤러/허브 필요할 수 있음)
3. static TF 방식으로 갈 경우 상대 위치를 어떻게 잴지 (자로 실측 vs 체커보드 등으로 카메라-카메라 캘리브레이션)

## 참고

- 오늘 재현 로그: `Registration failed: "Not enough inliers 0/20"` 3회 (매핑 세션 2회, localization+gaze 세션 1회)
- 관련 기존 기록: `#18 VPT` (2026-07-22), `docs/decisions.md`
