#!/usr/bin/env bash
# 듀얼 RealSense D455 카메라 노드 실행
# 시스템 Python(ROS2) 환경에서 실행할 것. conda 환경이 활성화되어 있으면 안 됨.

set -e

if [[ -n "$CONDA_DEFAULT_ENV" ]]; then
    echo "[WARN] conda 환경이 활성화되어 있습니다 ($CONDA_DEFAULT_ENV). 'conda deactivate' 후 다시 실행하세요."
    exit 1
fi

cd "$(dirname "$0")/../src/ros2_ws"
source install/setup.bash

# TODO: 실제 launch 파일명으로 교체
ros2 launch vpt_cameras dual_camera.launch.py
