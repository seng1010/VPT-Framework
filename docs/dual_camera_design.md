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
   → **교수님 제안 (2026-08-06): 실측 대신 Kinect도 자체 localization.** static TF(고정값,
   지금은 identity placeholder)로 D455 pose에서 Kinect 위치를 유추하는 대신, **RTAB-Map을
   D455용/Kinect용 두 개 띄워서** 둘 다 같은 저장된 맵(`~/.ros/rtabmap.db`)에 대해 각자
   localization 하게 만들면 Kinect의 `map` 기준 pose가 실측 없이 자동으로 나옴. 이게 바로 위
   "옵션 (b)"였던 것.

   **스크립트 준비 완료 (2026-08-19), 실카메라 미검증**: `scripts/run_kinect_localization.sh`.
   새 launch 파일을 따로 안 만들고, ROS2에 이미 설치된 범용 `rtabmap_launch/rtabmap.launch.py`
   에 인자만 넘겨서 씀 (`--show-args`로 아래 인자들이 전부 실제로 지원됨을 확인함):
   ```bash
   ros2 launch rtabmap_launch rtabmap.launch.py \
       namespace:=kinect_rtabmap \
       frame_id:=kinect_rgb_optical_frame \
       vo_frame_id:=kinect_odom  odom_topic:=kinect_odom \
       map_frame_id:=map \
       rgb_topic:=/kinect/rgb/image_raw \
       depth_topic:=/kinect/depth/image_raw \
       camera_info_topic:=/kinect/rgb/camera_info \
       approx_sync:=true  localization:=true \
       database_path:=<Kinect 전용 db 사본>
   ```
   - `vo_frame_id`/`odom_topic`을 `kinect_odom`으로 반드시 따로 지정해야 한다 — 기본값이
     둘 다 `odom`이라 D455쪽 visual odometry와 TF 프레임 이름이 겹쳐버림(`map_frame_id`만
     의도적으로 공유, 나머지 프레임은 각자 이름이 달라야 TF 트리가 안 꼬임).
   - **동시 오픈 위험은 회피**: 같은 `rtabmap.db`를 두 프로세스가 동시에 여는 상황 자체를
     테스트하는 대신, 스크립트가 매 실행마다 Kinect 전용 사본(`rtabmap_kinect_localization.db`)
     을 새로 떠서 그걸 읽게 함 — 파일 잠금 충돌 가능성을 원천 차단.
   - 이 스크립트가 잘 동작하면 기존 identity `static_transform_publisher`(camera_link →
     kinect_rgb_optical_frame)는 끄고 이걸로 대체한다. 둘 다 켜두면 static TF가 우선권을 가져가
     Kinect의 실제 localization 결과가 무시될 수 있다.
   - **미검증**: 지금 데스크톱에 D455/Kinect가 물리적으로 연결 안 되어 있어서 `ros2 launch`
     자체를 아직 실행해보지 못함(인자 유효성만 `--show-args`로 확인). 카메라 연결 후 처음
     테스트할 때 확인할 것: (a) Kinect localization이 실제로 붙는지(`ros2 topic echo
     /kinect_rtabmap/localization_pose` 등), (b) `map → kinect_odom → kinect_rgb_optical_frame`
     TF 체인이 끊김 없이 나오는지.
   - **알려진 위험**: Kinect의 원래 역할이 사람 얼굴 클로즈업이라, 이게 오늘까지 계속
     겪었던 "클로즈업하면 SLAM 트래킹 깨짐" 문제를 Kinect 자체에도 그대로 일으킬 수 있음.
     Kinect가 얼굴만이 아니라 배경(방)도 어느 정도 같이 보이는 각도/거리로 배치되면
     완화될 가능성 있음 — 물리적 배치(질문 2번)와도 연결됨.
4. ~~Kinect USB 대역폭~~ → 해결 (2026-08-05 확인). D455+Kinect+RTAB-Map+gaze_bridge_node+
   raycasting_node+virtual_camera_node 전부 동시에 띄워서 여러 차례 테스트 — USB 대역폭 문제
   없음. 병목은 대역폭이 아니라 **CPU/메모리**였음(아래 참고).

## 2026-08-05 — 전체 파이프라인 실측 + 성능 이슈

D455+Kinect+RTAB-Map(localization, 253노드/44만포인트 맵)+gaze_bridge_node+raycasting_node+
virtual_camera_node를 전부 동시에 띄워서 `/head_position`, `/gaze_origin`, `/gaze_direction`,
`/virtual_camera/image_raw`까지 전부 실제로 확인함 (identity static TF라 위치 정확도는 아직 아님,
배선 자체는 끝까지 연결됨).

### `virtual_camera_node` 성능 문제 + 수정
- 매 프레임(10Hz) 44만 포인트를 지웠다 다시 추가 + 렌더링 → CPU 300~500%+, 시스템 load average
  15까지 치솟아서 RViz가 응답 없음 상태로 멈추거나(재현: `ps` STAT이 결국 `Zl`(zombie)로 바뀌며
  크래시) 시스템 전체가 버벅임. `free -h` 확인 결과 메모리도 거의 바닥(804Mi free, swap 사용 중),
  컨텍스트 스위치 초당 22만+ — 리소스 고갈이 원인.
- **수정 3가지**: (1) `map_points`가 실제로 갱신됐을 때만 geometry 재생성(`map_dirty` 플래그),
  (2) 렌더링 주기 10Hz→2Hz, (3) `voxel_down_sample(0.03)`으로 44만 포인트를 다운샘플. CPU
  521%→~50~120%까지 감소.
- **RViz는 이 전체 파이프라인과 동시에 못 씀** (같은 44만 포인트 맵을 RViz도 client-side로 또
  받아서 렌더링하려니 감당이 안 됨) — 대신 `rqt_image_view`(가벼움, CPU 한 자릿수%)로
  `/virtual_camera/image_raw` 확인하는 걸로 대체.
- RTAB-Map의 `/rtabmap/cloud_map`은 localization 모드에서 **한 번만 발행**됨(맵이 안 바뀌므로) —
  `virtual_camera_node`/`raycasting_node`가 그 순간을 놓치면 이후로 데이터가 안 옴. 필요하면
  `ros2 service call /rtabmap/rtabmap/publish_map rtabmap_msgs/srv/PublishMap "{}"`로 강제 재발행.

### 색상 추가
`virtual_camera_node`가 흑백 점만 찍어서 노이즈처럼 보이던 문제 — `/rtabmap/cloud_map`에 이미
`rgb` 필드(PCL 관례, packed float32 = 0x00RRGGBB)가 있어서 디코딩해서 `pcd.colors`에 반영,
점 크기도 2px→6px로 키움. 색 입히니 방 구조가 눈에 띄게 잘 보임.

## 2026-08-19/20 — D455-Kinect extrinsic 실측 대체, 실카메라 검증 완료

`scripts/run_kinect_localization.sh`(교수님 제안, "옵션 (b)")를 D455/Kinect 둘 다 물리
연결된 상태로 실제로 돌려서 검증했다.

- D455: `realsense2_camera` + `rtabmap_launch/rtabmap.launch.py`(localization:=true,
  database_path=`~/.ros/rtabmap.db`)를 그대로 사용.
- Kinect: `scripts/run_kinect_localization.sh` — 같은 db의 전용 사본에 대해 별도
  네임스페이스(`kinect_rtabmap`, `vo_frame_id`/`odom_topic`을 `kinect_odom`으로 분리해
  D455쪽과 TF 프레임 충돌 방지)로 localization.
- 결과: `map -> odom -> camera_color_optical_frame`(D455)와
  `map -> kinect_odom -> kinect_rgb_optical_frame`(Kinect)이 각각 독립적으로, 서로 다른
  값으로(실측 없이) 나옴. **Kinect를 손으로 움직이는 동안 `tf2_echo map
  kinect_rgb_optical_frame` 값이 프레임마다 실시간으로 따라 바뀌는 것까지 확인** —
  저장된 초기값을 그대로 뱉는 게 아니라 매 프레임 다시 계산되고 있다는 확실한 증거.
- 하드웨어 트러블슈팅 기록: D455는 반드시 진짜 USB3(20000M) 버스에 물려야 함 — USB2
  포트에서는 `align_depth` 처리 중 `Failed to grow hash_map` 에러로 즉시 segfault
  (이 머신은 xHCI 컨트롤러 하나가 USB2/USB3 버스를 동시에 노출하는 구조라 포트 겉모습만
  보고는 구분 안 됨, `lsusb -t`로 실제 연결된 bus의 속도를 확인해야 함). Kinect는 별도
  전원 어댑터가 있어야 카메라/오디오 서브장치까지 다 잡힘(모터만 잡히는 건 전형적인
  전력 부족 신호, `journalctl -k`에서 `device not accepting address, error -71` 같은
  protocol error로 나타남) — 그리고 이번엔 데이터 케이블 쪽 PC 포트를 바꾸고서야 해결됨
  (전원 어댑터 문제가 아니라 데이터 포트 자체가 헐거웠던 케이스도 있었음, 원인이 매번
  같다고 단정하지 말 것).

## 2026-08-20 — #21 VPT 미팅: 카메라 역할 재정의 (SLAM/얼굴 분리 → 동시 검출 + fusion)

교수님 피드백: D455/Kinect를 SLAM 전용/얼굴 전용으로 나누는 게 아니라, **두 카메라가 동시에
같은 역할(얼굴/gaze 검출)을 하고 그 결과를 fusion**해야 한다 — "두 로봇이 같은 사람을 다른
각도에서 보는" 것처럼. 이게 원래 연구제안서(M-VPT) 초록의 "multi-view가 가림/사각지대를
완화한다"는 취지와 맞다 — 지금까지의 SLAM/얼굴 역할 분리는 "카메라 1대로 SLAM+얼굴 동시
처리 시 트래킹 깨짐"(2026-07-28) 문제를 피하려던 임시방편이었을 뿐, 최종 목표는 아니었다.
위 섹션의 D455 자체 localization 검증이 정확히 이 전환에 필요한 전제조건이었다(이제 D455도
Kinect처럼 map 기준 pose를 자체적으로 얻으므로, "환경 전용" 역할에 묶여있을 이유가 없다).

### 구현

- `gaze_bridge_node.py`를 파라미터화(`camera_name`, `rgb_topic`, `depth_topic`,
  `camera_info_topic`, `tf_frame`, `show_window`) — 카메라 하나에 고정하지 않고 같은 노드를
  `-p camera_name:=kinect`/`-p camera_name:=d455`로 두 번 띄울 수 있음. 각 인스턴스는 자기
  이름이 붙은 토픽(`/kinect/head_position` 등)에만 발행 — 정규 토픽은 건드리지 않는다.
- `gaze_fusion_node.py` 신규 — 두 인스턴스의 `/{camera}/head_position`,
  `/{camera}/gaze_direction`을 구독해서 융합(1초 이내에 갱신된 카메라만 포함, 하나만
  살아있으면 그대로 통과, 둘 다 있으면 head_position 3D 평균 + gaze_direction 단위벡터
  평균 후 재정규화) 후 **정규 토픽**(`/head_position`, `/gaze_origin`, `/gaze_direction`,
  `/virtual_camera/camera_info`, TF 프레임 `head_position`)으로 재발행. `vpt_raycasting`,
  `vpt_virtual_camera`는 이 노드가 존재한다는 것조차 몰라도 됨 — 인터페이스 그대로 유지.
- `scripts/run_dual_gaze_bridge.sh` 신규 — 위 세 프로세스(kinect용/d455용 gaze_bridge +
  fusion)를 한 번에 띄움. D455/Kinect 각각의 카메라 드라이버+RTAB-Map은 이 스크립트 밖에서
  먼저 띄워져 있어야 함(전제 조건은 스크립트 상단 주석 참고).
- 검증: `gaze_bridge_node.py`를 실제 GazeTR 체크포인트/MediaPipe 모델과 함께
  camera_name=kinect(기본값, 기존 동작과 동일한지)/camera_name=d455(새 파라미터 오버라이드)
  양쪽 다 실제로 생성해봐서 정상 동작 확인. `gaze_fusion_node.py`도 생성 확인. **다만 실제
  두 카메라 앞에 사람을 두고 fusion된 gaze 값이 정확한지는 아직 실측 전** — 다음 세션에서
  `scripts/run_dual_gaze_bridge.sh` 실행 + 두 카메라가 동시에 얼굴을 보는 상태에서 검증할 것.

### 다음

- [ ] 실카메라로 `scripts/run_dual_gaze_bridge.sh` 실행 — 두 카메라 다 얼굴을 보는 상태에서
      fusion된 `/gaze_direction`이 안정적인지, 한쪽 카메라만 얼굴이 보일 때 자연스럽게
      그 카메라로 전환되는지 확인.
- [ ] fusion 방식 고도화 검토 — 지금은 단순 평균. 신뢰도(얼굴 크기, landmark 품질 등) 기반
      가중치나 두 시선 방향이 서로 크게 어긋날 때(둘 중 하나가 오검출일 가능성) 처리 로직 추가.
- [ ] `docs/12_eye_scene_extrinsic_calibration.md`(MoLBWA 쪽 R+t 캘리브레이션 설계)와는
      별개 트랙 — 혼동 주의.
- [ ] GazeTR 대신 PureGaze/L2CS-Net으로 교체하는 것도 `#21 VPT`에 남은 항목 (표 참고).

## 참고

- 오늘 재현 로그: `Registration failed: "Not enough inliers 0/20"` 3회 (매핑 세션 2회, localization+gaze 세션 1회)
- 관련 기존 기록: `#18 VPT` (2026-07-22), `#21 VPT` (2026-08-20), `docs/decisions.md`
