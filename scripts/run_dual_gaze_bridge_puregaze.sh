#!/usr/bin/env bash
# run_dual_gaze_bridge.sh의 PureGaze 버전 (2026-08-27, gaze_model:=puregaze만 추가).
# 2026-08-27 저녁 D455+Kinect 실카메라로 검증 완료 — 2분+ 안정 동작, gaze_bridge_d455/
# gaze_bridge_kinect 둘 다 실시간 puregaze gaze 발행, gaze_fusion_node가 정상적으로
# /head_position·/gaze_direction(map frame)까지 재발행하는 것 확인. 스레드 교착 재발 없음.
# 문제 생기면 그냥 run_dual_gaze_bridge.sh(GazeTR, 검증된 버전)로 돌아가면 됨.
#
# 2026-08-20, #21 VPT 미팅 반영: D455/Kinect가 SLAM전용/얼굴전용으로 나뉘는 게 아니라
# 둘 다 동시에 얼굴/gaze를 검출하고 그 결과를 융합한다 (두 로봇이 같은 사람을 다른 각도에서
# 보는 것처럼). gaze_bridge_node.py를 카메라별로 두 개 띄우고, gaze_fusion_node.py로 합친다.
#
# 전제 (먼저 다른 터미널에서 띄워져 있어야 함):
#   - D455: realsense2_camera + rtabmap(localization, map 발행)
#   - Kinect: vpt_cameras/kinect_face_node + rtabmap(namespace=kinect_rtabmap, localization)
#     -> scripts/run_kinect_localization.sh
#   두 카메라 다 map 기준 TF가 나와야(docs/dual_camera_design.md, 2026-08-19/20 검증됨)
#   gaze_bridge_node.py의 transform_to_map()이 성공한다.
#
# 이 스크립트가 띄우는 것:
#   1. gaze_bridge_node.py --camera-name kinect  (node: gaze_bridge_kinect)
#   2. gaze_bridge_node.py --camera-name d455    (node: gaze_bridge_d455)
#   3. gaze_fusion_node.py                       (정규 /head_position 등으로 재발행)
#
# Ctrl+C 한 번으로 셋 다 종료됨.

set -e

if [[ -n "$CONDA_DEFAULT_ENV" ]]; then
    echo "[WARN] conda 환경이 활성화되어 있습니다 ($CONDA_DEFAULT_ENV). 'conda deactivate' 후 다시 실행하세요."
    exit 1
fi

# anaconda의 python3(3.13)가 PATH 앞쪽에 있으면 시스템 rclpy(3.12용 컴파일된 C 확장)를 못 찾아
# ModuleNotFoundError('rclpy._rclpy_pybind11')로 죽는다 — 이 머신에서 반복적으로 겪은 문제라
# 여기서 아예 PATH에서 anaconda를 제거한다.
export PATH="$(echo "$PATH" | tr ':' '\n' | grep -v anaconda | paste -sd:)"

HERE="$(cd "$(dirname "$0")" && pwd)"
GAZE_BRIDGE_DIR="$HERE/../src/ros2_ws/src/vpt_gaze_bridge"

source /opt/ros/jazzy/setup.bash
source "$HERE/../src/ros2_ws/install/setup.bash"

# PID을 직접 추적해서 그 프로세스들만 종료한다. `kill 0`(프로세스 그룹 전체)은 이 스크립트
# 자신의 쉘도 포함해서 TERM을 다시 받아 cleanup을 재귀 호출하는 무한루프에 빠졌던 적이
# 있어서(2026-08-20) 쓰지 않는다.
PIDS=()
cleanup() {
    trap - EXIT INT TERM  # 재진입 방지
    echo "종료 중..."
    kill "${PIDS[@]}" 2>/dev/null
    wait "${PIDS[@]}" 2>/dev/null
}
trap cleanup EXIT INT TERM

echo "[1/3] gaze_bridge_node (kinect) 시작..."
python3 "$GAZE_BRIDGE_DIR/gaze_bridge_node.py" --ros-args \
    -r __node:=gaze_bridge_kinect \
    -p camera_name:=kinect \
    -p rgb_topic:=/kinect/rgb/image_raw \
    -p depth_topic:=/kinect/depth/image_raw \
    -p camera_info_topic:=/kinect/rgb/camera_info \
    -p tf_frame:=kinect_rgb_optical_frame \
    -p gaze_model:=puregaze &
PIDS+=($!)

echo "[2/3] gaze_bridge_node (d455) 시작..."
python3 "$GAZE_BRIDGE_DIR/gaze_bridge_node.py" --ros-args \
    -r __node:=gaze_bridge_d455 \
    -p camera_name:=d455 \
    -p rgb_topic:=/camera/camera/color/image_raw \
    -p depth_topic:=/camera/camera/aligned_depth_to_color/image_raw \
    -p camera_info_topic:=/camera/camera/color/camera_info \
    -p tf_frame:=camera_color_optical_frame \
    -p gaze_model:=puregaze &
PIDS+=($!)

echo "[3/3] gaze_fusion_node 시작..."
python3 "$GAZE_BRIDGE_DIR/gaze_fusion_node.py" &
PIDS+=($!)

wait "${PIDS[@]}"
