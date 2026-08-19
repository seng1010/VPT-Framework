#!/usr/bin/env bash
# Kinect(Camera B)를 D455가 만든 저장된 맵에 대해 독립적으로 localization시켜서,
# D455<->Kinect의 static TF(현재 identity placeholder, docs/dual_camera_design.md
# "남은 질문" 3번)를 실측 없이 대체한다.
#
# 아이디어(교수님 제안, 2026-08-06 docs 기록): 두 카메라의 상대 위치를 직접 재는 대신,
# RTAB-Map을 카메라마다 하나씩(총 2개) 같은 저장 맵에 대해 localization 모드로 띄우면
# 각 카메라의 map 기준 pose가 자동으로 나온다. D455용 인스턴스는 기존 방식(예:
# `ros2 launch rtabmap_examples realsense_d400.launch.py`)대로 계속 띄우고, 이 스크립트는
# 그 옆에서 Kinect용 두 번째 인스턴스만 추가로 띄운다.
#
# 새 launch 파일을 만들 필요 없이 ROS2에 이미 설치된 범용 rtabmap_launch/rtabmap.launch.py
# 를 그대로 쓴다 — namespace/frame_id/토픽 리매핑/localization 모드를 인자로 전부 지원한다.
#
# 이 스크립트가 성공적으로 돌면, gaze_bridge_node.py의
#   tf_buffer.lookup_transform('map', 'kinect_rgb_optical_frame', ...)
# 가 identity static_transform_publisher 대신 이 노드가 발행하는 실제 TF 체인
# (map -> kinect_odom -> kinect_rgb_optical_frame)으로 해석된다. 즉 지금 두고 있는
#   ros2 run tf2_ros static_transform_publisher --frame-id camera_link \
#     --child-frame-id kinect_rgb_optical_frame ...
# 는 이 스크립트를 쓰는 동안은 끄고 실행하지 않는다(둘 다 켜면 static TF가 이겨서
# Kinect의 실제 localization 결과가 무시될 수 있음).
#
# 전제:
#   - D455용 rtabmap이 이미 저장 맵(~/.ros/rtabmap.db)에 대해 localization:=true로
#     돌고 있어야 한다 (map 프레임을 발행하는 쪽).
#   - vpt_cameras/kinect_face_node가 /kinect/rgb/image_raw, /kinect/depth/image_raw,
#     /kinect/rgb/camera_info를 발행 중이어야 한다 (depth는 RGB에 이미 정렬됨,
#     docs/dual_camera_design.md 참고).
#
# 미검증 상태 (2026-08-19): 이 데스크톱에 D455/Kinect가 지금 물리적으로 연결되어 있지
# 않아서 실행 자체는 아직 못 해봤다. 실카메라로 테스트하고 문제 있으면 이 스크립트를
# 고칠 것 — 특히 아래 "알려진 위험"을 확인할 것.
#
# 알려진 위험:
#   1. 같은 rtabmap.db를 D455/Kinect 두 프로세스가 동시에 열어야 하는데, 파일 잠금
#      충돌이 날 수도 있음 — 그래서 이 스크립트는 Kinect 전용 사본을 매번 새로 떠서
#      건드린다(아래 참고). 두 인스턴스가 완전히 별개 파일을 읽으니 이 위험 자체를
#      피해간다.
#   2. Kinect가 얼굴 클로즈업만 보면(배경이 거의 안 보이면) localization 자체가
#      MoLBWA/D455 세션에서 겪었던 것과 같은 이유로 실패할 수 있음 — 물리적 배치
#      (카메라가 사람+배경을 어느 정도 같이 보는 각도/거리)로 완화해야 함.

set -e

if [[ -n "$CONDA_DEFAULT_ENV" ]]; then
    echo "[WARN] conda 환경이 활성화되어 있습니다 ($CONDA_DEFAULT_ENV). 'conda deactivate' 후 다시 실행하세요."
    exit 1
fi

SRC_DB="${1:-$HOME/.ros/rtabmap.db}"
KINECT_DB="$HOME/.ros/rtabmap_kinect_localization.db"

if [[ ! -f "$SRC_DB" ]]; then
    echo "[ERROR] 맵 DB를 못 찾음: $SRC_DB"
    echo "        D455용 rtabmap을 먼저 mapping 모드로 한 번 돌려서 맵을 저장해야 한다."
    exit 1
fi

echo "[kinect-loc] $SRC_DB -> $KINECT_DB 로 복사 (동시-오픈 위험 회피용 Kinect 전용 사본)"
cp "$SRC_DB" "$KINECT_DB"

cd "$(dirname "$0")/../src/ros2_ws"
source install/setup.bash

ros2 launch rtabmap_launch rtabmap.launch.py \
    namespace:=kinect_rtabmap \
    frame_id:=kinect_rgb_optical_frame \
    vo_frame_id:=kinect_odom \
    odom_topic:=kinect_odom \
    map_frame_id:=map \
    rgb_topic:=/kinect/rgb/image_raw \
    depth_topic:=/kinect/depth/image_raw \
    camera_info_topic:=/kinect/rgb/camera_info \
    approx_sync:=true \
    localization:=true \
    database_path:="$KINECT_DB" \
    rtabmap_viz:=false \
    rviz:=false
