#!/usr/bin/env bash
# 2026-09-14 ICRA 마감 직전 긴급 파일럿용 — D455 대신 Kinect 2대(kinect1/kinect2)로 듀얼카메라
# gaze 추정. RTAB-Map localization은 시간 관계상 생략하고 static TF(map -> 각 카메라 frame,
# identity)로 대체 — eval_gaze_accuracy.py는 카메라 로컬 프레임 값(gaze_raw, head_position_cam)만
# 쓰기 때문에 map 변환 정확도는 이 파일럿에 영향 없음. RTAB-Map 도입 전 이 프로젝트에서 이미
# 쓰던 방식(scripts/run_kinect_localization.sh 주석 참고)이라 검증된 우회로.
#
# 전제: kinect_face_node가 이미 별도 터미널 2개에서 떠 있어야 함
#   (__ns:=/kinect1 -p device_index:=0, __ns:=/kinect2 -p device_index:=1)
#
# 이 스크립트가 띄우는 것:
#   1. static_transform_publisher: map -> kinect1_rgb_optical_frame (identity)
#   2. static_transform_publisher: map -> kinect2_rgb_optical_frame (identity)
#   3. gaze_bridge_node.py (kinect1, PureGaze)
#   4. gaze_bridge_node.py (kinect2, PureGaze)
#
# Ctrl+C 한 번으로 넷 다 종료됨.

set -e

if [[ -n "$CONDA_DEFAULT_ENV" ]]; then
    echo "[WARN] conda 환경이 활성화되어 있습니다 ($CONDA_DEFAULT_ENV). 'conda deactivate' 후 다시 실행하세요."
    exit 1
fi

export PATH="$(echo "$PATH" | tr ':' '\n' | grep -v anaconda | paste -sd:)"

HERE="$(cd "$(dirname "$0")" && pwd)"
GAZE_BRIDGE_DIR="$HERE/../src/ros2_ws/src/vpt_gaze_bridge"

source /opt/ros/jazzy/setup.bash
source "$HERE/../src/ros2_ws/install/setup.bash"

PIDS=()
cleanup() {
    trap - EXIT INT TERM
    echo "종료 중..."
    kill "${PIDS[@]}" 2>/dev/null
    wait "${PIDS[@]}" 2>/dev/null
}
trap cleanup EXIT INT TERM

echo "[1/4] static TF (map -> kinect1_rgb_optical_frame) 시작..."
ros2 run tf2_ros static_transform_publisher \
    --frame-id map --child-frame-id kinect1_rgb_optical_frame \
    --x 0 --y 0 --z 0 &
PIDS+=($!)

echo "[2/4] static TF (map -> kinect2_rgb_optical_frame) 시작 (실측 캘리브레이션, 2026-09-15 Kabsch 정합, held-out RMS 1.06cm, raw data: kinect_extrinsic_calib_pairs.csv)..."
ros2 run tf2_ros static_transform_publisher \
    --frame-id map --child-frame-id kinect2_rgb_optical_frame \
    --x 0.4836 --y 0.1448 --z 0.1308 \
    --qx -0.0457 --qy -0.1693 --qz 0.1083 --qw 0.9785 &
PIDS+=($!)

sleep 1

echo "[3/4] gaze_bridge_node (kinect1) 시작..."
python3 "$GAZE_BRIDGE_DIR/gaze_bridge_node.py" --ros-args \
    -r __node:=gaze_bridge_kinect1 \
    -p camera_name:=kinect1 \
    -p rgb_topic:=/kinect1/kinect/rgb/image_raw \
    -p depth_topic:=/kinect1/kinect/depth/image_raw \
    -p camera_info_topic:=/kinect1/kinect/rgb/camera_info \
    -p tf_frame:=kinect1_rgb_optical_frame \
    -p gaze_model:=puregaze &
PIDS+=($!)

echo "[4/4] gaze_bridge_node (kinect2) 시작..."
python3 "$GAZE_BRIDGE_DIR/gaze_bridge_node.py" --ros-args \
    -r __node:=gaze_bridge_kinect2 \
    -p camera_name:=kinect2 \
    -p rgb_topic:=/kinect2/kinect/rgb/image_raw \
    -p depth_topic:=/kinect2/kinect/depth/image_raw \
    -p camera_info_topic:=/kinect2/kinect/rgb/camera_info \
    -p tf_frame:=kinect2_rgb_optical_frame \
    -p gaze_model:=puregaze &
PIDS+=($!)

wait "${PIDS[@]}"
