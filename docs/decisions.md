# Decisions Log

이 프로젝트에서 "왜 이렇게 했는지"를 기록합니다. 실험 결과 자체(수치, 그래프)는 Notion 참고.
새 결정이 생기면 아래 형식으로 위에 추가하세요 (최신이 위로).

---

## 템플릿

```markdown
## YYYY-MM-DD 제목

**배경:** 왜 이 결정이 필요했는지
**선택지:** 고려한 대안들
**결정:** 최종 선택
**이유:** 왜 이걸 골랐는지
```

---

## 2026-07-22 gaze 추정과 raycasting 노드 분리

**배경:** 초기 프로토타입(head_gaze_publisher.py)은 카메라 입력 → GazeTR/MediaPipe gaze 추정 → RTAB-Map 포인트클라우드에 ray casting까지 한 노드 안에서 처리했음.
**선택지:**
1. 하나의 노드로 유지 (구현은 간단하지만 관심사가 섞임)
2. gaze 추정 노드(vpt_gaze_bridge)와 raycasting 노드(vpt_raycasting)로 분리, ROS2 토픽(`/gaze_origin`, `/gaze_direction`)으로 연결
**결정:** 2번, 노드 분리
**이유:** gaze 추정은 카메라 프레임마다 고빈도로 도는 반면, raycasting은 SLAM 맵(`/cloud_map`) 갱신 주기에 좌우됨. 의존성(GazeTR/MediaPipe vs 포인트클라우드/KDTree)과 갱신 주기가 다른 두 관심사를 한 노드에 두면 각각 독립적으로 테스트/교체하기 어려움. 예: 맵 소스를 바꾸거나 raycasting 알고리즘만 교체할 때 gaze 추정 쪽을 건드릴 필요가 없어야 함.

---

## 2026-XX-XX GazeTR / ROS2 프로세스 분리

**배경:** GazeTR은 Python 3.13(conda) 환경에서 동작, ROS2는 시스템 Python 3.12 사용. 하나의 프로세스에서 같이 돌릴 수 없음.
**선택지:**
1. ROS2를 conda 환경 안에서 빌드
2. GazeTR을 시스템 Python으로 다운그레이드
3. 두 프로세스로 분리하고 ROS2 토픽으로 브릿지
**결정:** 3번, 프로세스 분리 + 토픽 브릿지
**이유:** ROS2 conda 빌드는 의존성 충돌이 잦고, GazeTR 다운그레이드는 라이브러리 호환성 문제 발생. 프로세스 분리가 가장 안정적.

---

## 2026-XX-XX SLAM: ORB-SLAM3 → RTAB-Map 전환

**배경:** 초기에는 ORB-SLAM3 Stereo-Inertial로 구현 시도.
**선택지:** ORB-SLAM3 유지 vs RTAB-Map 전환
**결정:** RTAB-Map으로 전환
**이유:** ORB-SLAM3의 ROS2 wrapper 호환성 문제로 인해 개발 속도 저하. RTAB-Map이 ROS2 네이티브 지원이 더 안정적.

---

*(위 두 항목은 지금까지 대화 내용 기반 초안입니다. 정확한 날짜와 세부 내용으로 수정해주세요.)*
