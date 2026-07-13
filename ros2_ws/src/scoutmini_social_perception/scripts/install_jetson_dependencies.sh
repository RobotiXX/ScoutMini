#!/usr/bin/env bash
set -euo pipefail

if [[ "$(uname -m)" != "aarch64" ]]; then
  echo "This dependency set is only for Jetson aarch64." >&2
  exit 1
fi

cache_dir="${1:-${HOME}/.cache/scoutmini/jetson-wheels}"
mkdir -p "${cache_dir}"

torch_wheel="torch-2.8.0-cp310-cp310-linux_aarch64.whl"
vision_wheel="torchvision-0.23.0-cp310-cp310-linux_aarch64.whl"
torch_url="https://pypi.jetson-ai-lab.io/jp6/cu126/+f/62a/1beee9f2f1470/${torch_wheel}"
vision_url="https://pypi.jetson-ai-lab.io/jp6/cu126/+f/907/c4c1933789645/${vision_wheel}"

wget -c -O "${cache_dir}/${torch_wheel}" "${torch_url}"
wget -c -O "${cache_dir}/${vision_wheel}" "${vision_url}"

(
  cd "${cache_dir}"
  printf '%s  %s\n' \
    '62a1beee9f2f147076a974d2942c90060c12771c94740830327cae705b2595fc' \
    "${torch_wheel}" | sha256sum --check --strict
  printf '%s  %s\n' \
    '907c4c1933789645ebb20dd9181d40f8647978e6bd30086ae7b01febb937d2d1' \
    "${vision_wheel}" | sha256sum --check --strict
)

TMPDIR="${TMPDIR:-${cache_dir}}" python3 -m pip install \
  --user --force-reinstall --no-cache-dir --no-deps \
  "${cache_dir}/${torch_wheel}" "${cache_dir}/${vision_wheel}"

python3 -m pip install --user --no-deps \
  numpy==1.26.4 \
  scipy==1.11.4 \
  lap==0.5.12 \
  onnx==1.17.0 \
  ultralytics==8.4.30

python3 - <<'PY'
import cv2
import lap
import numpy
import onnx
import scipy
import torch
from ultralytics import YOLO

assert torch.cuda.is_available(), 'CUDA PyTorch is not available'
print(f'torch={torch.__version__} cuda={torch.version.cuda}')
print(f'numpy={numpy.__version__} cv2={cv2.__version__}')
print(f'GPU={torch.cuda.get_device_name(0)}')
print('Jetson perception dependencies passed')
PY
