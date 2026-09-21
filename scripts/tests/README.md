# 카메라 없이 돌리는 검증 스크립트

ROS2만 있으면 되고 카메라/모델 체크포인트는 필요 없다 (`test_frontality.py`는
gaze_bridge_node 모듈을 import하므로 `~/GazeTR`, `~/face_landmarker.task`는 있어야 한다).

```bash
export PATH="$(echo "$PATH" | tr ':' '\n' | grep -v anaconda | paste -sd:)"   # conda python이 rclpy를 가림
source /opt/ros/jazzy/setup.bash
/usr/bin/python3 scripts/tests/test_frontality.py
/usr/bin/python3 scripts/tests/test_weighted_fusion.py
```

- `test_frontality.py` — `compute_frontality_deg()`의 각도 계산, 광축이 아닌
  '머리→카메라' 기준을 쓰는 이유, 전방축 부호 자동 판별, 입력 방어.
- `test_weighted_fusion.py` — `gaze_fusion_node`의 가중 융합 8개 시나리오
  (하위호환 균등평균, 가중평균, 대칭, 불일치 fallback 기준 변경, 단일 카메라,
  둘 다 준측면, stale 처리).

실카메라 검증을 대체하지 않는다 — 기하/가중치 계산이 의도대로 도는지만 본다.
