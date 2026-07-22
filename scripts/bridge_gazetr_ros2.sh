#!/usr/bin/env bash
# GazeTR(conda, Python 3.13) 프로세스와 ROS2(시스템 Python 3.12) 브릿지 실행
#
# 순서:
#   1. conda 환경에서 GazeTR 추론 프로세스 시작 (백그라운드)
#   2. conda deactivate
#   3. ROS2 브릿지 노드 실행 (토픽 구독/발행)

set -e

echo "[1/3] GazeTR(conda) 프로세스 시작..."
conda activate gazetr
python "$(dirname "$0")/../src/gaze_estimation/gazetr/run_inference.py" &
GAZETR_PID=$!

echo "[2/3] conda deactivate..."
conda deactivate

echo "[3/3] ROS2 브릿지 노드 실행..."
cd "$(dirname "$0")/../src/ros2_ws"
source install/setup.bash
ros2 run vpt_gaze_bridge bridge_node

# 종료 시 GazeTR 프로세스도 정리
trap "kill $GAZETR_PID" EXIT
