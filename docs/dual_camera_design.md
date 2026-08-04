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
| 하드웨어 | RealSense D455 | **Kinect v1 (Xbox 360)** — 새로 안 사고 확보한 걸로 대체, 2026-08-04 |
| 방향 | 방/환경을 향함 (고정 또는 로봇에 장착) | 사람을 향함 |
| 구독 노드 | `rgbd_odometry`, `rtabmap` | `gaze_bridge_node` |
| 퍼블리시 | `/camera/camera/...`, `/rtabmap/cloud_map`, `map` TF | `/kinect/rgb/...`, `/kinect/depth/...`, `/head_position`, `/gaze_origin`, `/gaze_direction` |

D455 두 대가 아니라 D455(SLAM) + Kinect(얼굴) 조합으로 바뀜 — 얼굴 검출은 SLAM만큼 정밀한 depth가 필요 없어서 Kinect로 충분하다고 판단 (비용 절감).
이렇게 하면 얼굴이 아무리 클로즈업으로 잡혀도 Camera A의 odometry는 영향을 받지 않는다.

## 구현 완료 (2026-08-04)

### 1. `vpt_cameras` 패키지 — Kinect 드라이버 신규 작성

공식 ROS2 Kinect v1 패키지가 없어서(freenect_stack은 ROS1 전용) `ocams_ros2`와 같은 방식으로
`libfreenect` 동기(sync) API를 감싼 C++ 노드를 새로 작성: `vpt_cameras/src/kinect_face_node.cpp`.

```bash
ros2 run vpt_cameras kinect_face_node
```

퍼블리시:
- `/kinect/rgb/image_raw` (rgb8, ~30fps 실측)
- `/kinect/depth/image_raw` (16UC1, mm, **RGB에 이미 정렬됨** — `FREENECT_DEPTH_REGISTERED` 사용,
  RealSense의 `aligned_depth_to_color`와 동일한 의미라 `gaze_bridge_node`의 depth 룩업 코드를
  안 고쳐도 됨)
- `/kinect/rgb/camera_info` (fx=fy=525, cx=319.5, cy=239.5 — **Kinect v1 RGB의 널리 알려진
  근사값, 이 유닛 실측 캘리브레이션 아님.** 정확도 필요해지면 체커보드로 실측)

**하드웨어 이슈 + 해결**: 처음엔 USB 패킷 손실이 심해서(`Invalid magic ffff`, `Lost packets`,
`resyncing` 반복) 영상이 거의 안 나왔음. `usbfs_memory_mb`가 기본값 16(MB)으로 낮아서 생기는
것으로 잘 알려진 문제 — `echo 1000 | sudo tee /sys/module/usbcore/parameters/usbfs_memory_mb`로
올려서 해결 (재부팅하면 초기화되니 매번 필요할 수 있음, 영구 적용하려면
`/etc/modprobe.d/usbcore.conf`에 `options usbcore usbfs_memory_mb=1000` 추가 고려).

### 2. RTAB-Map은 Camera A(D455)만 구독 — 기존 그대로

D455는 원래 쓰던 `/camera/camera/color/...`, `/camera/camera/aligned_depth_to_color/...`
그대로 사용. 네임스페이스 분리가 필요 없어짐(카메라 종류 자체가 다르니 토픽 이름이 자연히 겹치지
않음 — D455 두 대였으면 필요했던 `serial_no` 기반 네임스페이스 분리는 불필요해짐).

### 3. `gaze_bridge_node.py`를 Kinect 구독으로 변경 — 완료

```python
self.create_subscription(CameraInfo, '/kinect/rgb/camera_info', self.camera_info_callback, 10)
color_sub = message_filters.Subscriber(self, Image, '/kinect/rgb/image_raw')
depth_sub = message_filters.Subscriber(self, Image, '/kinect/depth/image_raw')
```
TF 조회 프레임도 `camera_color_optical_frame` → `kinect_rgb_optical_frame`으로 전부 교체.

### 4. 두 카메라 간 외부 파라미터(extrinsic) — identity placeholder로 임시 해결

옵션 (a)(고정 마운트 + static TF)를 선택. 아직 실측은 안 됨 — MoLBWA의 `IMU.T_b_c1`과 같은
패턴으로, **identity placeholder를 넣어서 일단 파이프라인이 끝까지 돌아가게** 해둠:

```bash
ros2 run tf2_ros static_transform_publisher \
  --frame-id camera_link --child-frame-id kinect_rgb_optical_frame \
  --x 0 --y 0 --z 0 --qx 0 --qy 0 --qz 0 --qw 1
```

`camera_link`는 D455(Camera A)의 base 프레임 — SLAM이 `map`에 연결해주므로, 이 static TF 하나로
`map → ... → camera_link → kinect_rgb_optical_frame` 체인이 완성되어 `gaze_bridge_node`의
`transform_to_map()`이 더 이상 "unconnected trees" 에러 없이 동작한다.

**주의**: 0,0,0/identity는 "두 카메라가 완전히 같은 위치에 같은 방향으로 겹쳐있다"는 뜻이라
명백히 틀렸다 — 실제로는 두 카메라가 물리적으로 떨어져 있고 서로 다른 방향(D455는 환경,
Kinect는 사람)을 보고 있다. **head_position 계산 자체는 되지만 위치가 부정확할 것**. 실측
전까지는 "일단 배선이 끝까지 연결되는지"만 검증하는 용도로 쓸 것.

## 남은 질문 (직접 결정 필요)

1. ~~실제 카메라 2대 있는지~~ → 해결 (D455 + Kinect)
2. 두 카메라 물리적으로 어떻게 배치할지 — **아직 미정, static TF 실측 전에 먼저 정해야 함**
   (D455는 환경 향함, Kinect는 사람 향함 — 서로 반대 방향일 가능성 높은데 그럼 한 리그에
   고정 마운트하는 게 기구적으로 까다로울 수 있음)
3. static TF 실측 방법 — 자로 직접 재는 방법도 있고, 체커보드로 두 카메라가 동시에 보이는 위치에
   놓고 카메라-카메라 캘리브레이션하는 방법도 있음 (Kinect가 SLAM 카메라 쪽을 향하게 임시로
   틀어서 서로를 비추게 하거나, 공통 체커보드를 각자 다른 시점에서 촬영)
4. Kinect USB 대역폭 — D455 + Kinect 동시 연결 시 문제없는지 아직 같이 안 띄워봄 (오늘은 Kinect만
   단독 테스트)

## 참고

- 오늘 재현 로그: `Registration failed: "Not enough inliers 0/20"` 3회 (매핑 세션 2회, localization+gaze 세션 1회)
- 관련 기존 기록: `#18 VPT` (2026-07-22), `docs/decisions.md`
