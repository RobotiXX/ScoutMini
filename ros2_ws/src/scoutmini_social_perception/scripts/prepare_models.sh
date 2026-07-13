#!/usr/bin/env bash
set -euo pipefail

model_dir="${1:-${HOME}/models/yolo}"
trtexec="${TRTEXEC:-/usr/src/tensorrt/bin/trtexec}"
mkdir -p "${model_dir}"

download_model() {
  local name="$1"
  local url="$2"
  local expected_sha="$3"
  local path="${model_dir}/${name}"
  if [[ ! -f "${path}" ]]; then
    wget -O "${path}" "${url}"
  fi
  printf '%s  %s\n' "${expected_sha}" "${path}" | \
    sha256sum --check --strict
}

download_model \
  yolo11n.pt \
  https://github.com/ultralytics/assets/releases/download/v8.3.0/yolo11n.pt \
  0ebbc80d4a7680d14987a577cd21342b65ecfd94632bd9a8da63ae6417644ee1
download_model \
  yolo26n-cls.pt \
  https://github.com/ultralytics/assets/releases/download/v8.4.0/yolo26n-cls.pt \
  0dd6f8dbc448870ac98a3cbb7156f923f7ce21fed3755d4019169ffffd279e81

python3 - "${model_dir}/yolo11n.pt" <<'PY'
import sys
from ultralytics import YOLO

YOLO(sys.argv[1], task='detect').export(
    format='onnx',
    imgsz=640,
    opset=17,
    simplify=False,
    dynamic=False,
    device='cpu',
)
PY

"${trtexec}" \
  --onnx="${model_dir}/yolo11n.onnx" \
  --saveEngine="${model_dir}/yolo11n.engine" \
  --fp16 \
  --memPoolSize=workspace:1024 \
  --builderOptimizationLevel=3 \
  --skipInference

"${trtexec}" \
  --loadEngine="${model_dir}/yolo11n.engine" \
  --warmUp=200 \
  --duration=3
