#!/usr/bin/env bash
# D455(Camera A) realsense 드라이버 + rtabmap localization을 함께 띄운다.
# run_kinect_localization.sh와 짝을 이룬다 — 이쪽은 저장된 맵(~/.ros/rtabmap.db)의
# "원본"을 직접 연다(Kinect는 동시-오픈 충돌을 피하려고 복사본을 쓴다).
#
# 해상도를 반드시 640x480으로 낮춰서 띄운다 (2026-08-25 확인):
# 이 데스크톱은 USB 컨트롤러가 하나뿐이라(00:14.0 Raptor Lake XHCI, lspci로 확인 —
# 별도 USB 카드 없음) D455 기본 해상도(컬러 1280x720x30)로 스트리밍하면 같은 컨트롤러를
# 타는 마우스/키보드까지 먹통이 될 정도로 USB 대역폭을 잡아먹는다. 640x480x30으로
# 낮추면 재현 안 됨(실측 확인됨) — 이 해상도보다 올리지 말 것.
#
# 전제: D455가 USB3 포트에 물려 있어야 함(lsusb -t에서 Bus 002/5000M로 잡히는지 확인 —
# 이 보드는 포트 색/위치로 구분 안 되고 USB2/USB3 버스가 물리적으로 갈라져 있음).

set -e

if [[ -n "$CONDA_DEFAULT_ENV" ]]; then
    echo "[WARN] conda 환경이 활성화되어 있습니다 ($CONDA_DEFAULT_ENV). 'conda deactivate' 후 다시 실행하세요."
    exit 1
fi

export PATH="$(echo "$PATH" | tr ':' '\n' | grep -v anaconda | paste -sd:)"

DB="${1:-$HOME/.ros/rtabmap.db}"

if [[ ! -f "$DB" ]]; then
    echo "[ERROR] 맵 DB를 못 찾음: $DB"
    echo "        먼저 mapping 모드로 한 번 돌려서 맵을 저장해야 한다."
    exit 1
fi

cd "$(dirname "$0")/../src/ros2_ws"
source install/setup.bash

PIDS=()
cleanup() {
    trap - EXIT INT TERM
    echo "종료 중..."
    kill "${PIDS[@]}" 2>/dev/null
    wait "${PIDS[@]}" 2>/dev/null
}
trap cleanup EXIT INT TERM

echo "[1/2] realsense2_camera (640x480x30) 시작..."
ros2 launch realsense2_camera rs_launch.py \
    align_depth.enable:=true \
    rgb_camera.color_profile:=640x480x30 \
    depth_module.depth_profile:=640x480x30 &
PIDS+=($!)

sleep 5

echo "[2/2] rtabmap (localization) 시작..."
ros2 launch rtabmap_launch rtabmap.launch.py \
    frame_id:=camera_color_optical_frame \
    rgb_topic:=/camera/camera/color/image_raw \
    depth_topic:=/camera/camera/aligned_depth_to_color/image_raw \
    camera_info_topic:=/camera/camera/color/camera_info \
    approx_sync:=true \
    localization:=true \
    database_path:="$DB" \
    rtabmap_viz:=false \
    rviz:=false &
PIDS+=($!)

wait "${PIDS[@]}"
