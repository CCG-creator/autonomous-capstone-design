#!/bin/bash

# ==========================================
# YOLOv8 + ROS2 + CUDA Fix Script for GTX1650
# ==========================================

set -e

echo "[STEP 1] 현재 torch/ultralytics 완전히 정리 중..."
pip uninstall -y ultralytics torch torchvision torchaudio || true

# 남아있는 의존성도 한번 싹 정리
pip uninstall -y nvidia-cublas-cu12 nvidia-cuda-runtime-cu12 nvidia-cudnn-cu12 || true

# 최신 pip 버전 유지
python3 -m pip install --upgrade pip

echo "[STEP 2] CUDA 12.1 호환 torch 재설치"
pip install torch==2.2.2+cu121 --index-url https://download.pytorch.org/whl/cu121

echo "[STEP 3] ultralytics 최신버전 재설치"
pip install ultralytics --force-reinstall

# 환경 변수 잠시 export (실행할 때마다 필요함)
echo "[STEP 4] CUDA 환경변수 설정 (임시 적용)"
export CUDA_VISIBLE_DEVICES=0
export LD_LIBRARY_PATH=""

# 최종 torch 상태 확인
echo "[STEP 5] torch CUDA 인식 확인"
python3 -c "import torch; print('torch.cuda.is_available():', torch.cuda.is_available()); print('torch version:', torch.__version__); print('device_count:', torch.cuda.device_count())"

# 안내

cat <<EOF

=====================================
 ✅ 설치 완료!
 ✅ ROS2에서 사용할 때는 반드시 아래 환경변수 적용하고 실행하세요:

    export CUDA_VISIBLE_DEVICES=0
    export LD_LIBRARY_PATH=""

 ✅ 이거 하고 ros2 run 하시면 됩니다.
 ✅ ROS2 런처 내부에서 이 export 넣어두면 더 편합니다.

=====================================
EOF
